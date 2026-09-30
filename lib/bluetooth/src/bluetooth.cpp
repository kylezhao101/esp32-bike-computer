#include "bluetooth.hpp"

#include <stdio.h>
#include <cstring>
#include <algorithm>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_hs_id.h"
#include "host/ble_gatt.h"

#include "esp_timer.h"


namespace bluetooth
{

static uint8_t own_addr_type;

static ConnectionState connection_state = ConnectionState::Idle;
static constexpr int64_t SEARCH_WINDOW_US = 30000000;
static int64_t search_deadline_us = 0;
static bool host_synced = false;
static bool battery_valid = false;
static bool have_sensor_address = false;
static ble_addr_t sensor_address{};
static ble_addr_t candidate_address{};
static ble_npl_callout search_timer;
static ble_npl_event search_event;
static ble_npl_event cancel_event;
static bool connecting = false;
static bool connected = false;
static bool csc_subscribed = false;

static uint16_t csc_value_handle = 0;
static uint16_t csc_service_end_handle = 0;

static const ble_uuid16_t csc_service_uuid =
    BLE_UUID16_INIT(0x1816);

static const ble_uuid16_t csc_measurement_uuid =
    BLE_UUID16_INIT(0x2A5B);

static const ble_uuid16_t cccd_uuid =
    BLE_UUID16_INIT(0x2902);

static const ble_uuid16_t battery_service_uuid =
    BLE_UUID16_INIT(0x180F);

static const ble_uuid16_t battery_level_uuid =
    BLE_UUID16_INIT(0x2A19);

static bool have_previous_wheel = false;
static bool have_previous_crank = false;
static bool wheel_supported = false;
static bool crank_supported = false;
static uint32_t wheel_generation = 0;
static int64_t previous_wheel_received_us = 0;
static portMUX_TYPE sensor_mux = portMUX_INITIALIZER_UNLOCKED;
static SensorData published{};

static uint32_t previous_wheel_revs = 0;
static uint16_t previous_wheel_time = 0;

static uint16_t previous_crank_revs = 0;
static uint16_t previous_crank_time = 0;

static float current_speed_kmh = 0.0f;
static float current_cadence_rpm = 0.0f;

static uint8_t current_battery_percentage = 0.0f;
static uint32_t current_wheel_revs = 0;

static constexpr float WHEEL_CIRCUMFERENCE_METERS = 2.105f;

static int64_t last_wheel_motion_us = 0;
static int64_t last_crank_motion_us = 0;

static constexpr int64_t MOTION_TIMEOUT_US = 2000000;

// All writers run on the NimBLE task; readers copy under a short lock.
static void publish_sensor_data()
{
    SensorData snapshot{};
    snapshot.connected = connected;
    snapshot.connection_state = connection_state;
    snapshot.battery_valid = battery_valid;
    snapshot.speed_kmh = current_speed_kmh;
    snapshot.cadence_rpm = current_cadence_rpm;
    snapshot.wheel_revs = current_wheel_revs;
    snapshot.battery_percentage = current_battery_percentage;
    snapshot.wheel_valid = connected && have_previous_wheel;
    snapshot.wheel_supported = connected && wheel_supported;
    snapshot.crank_supported = connected && crank_supported;
    snapshot.wheel_generation = wheel_generation;
    snapshot.speed_valid_until_us = last_wheel_motion_us + MOTION_TIMEOUT_US;
    snapshot.cadence_valid_until_us = last_crank_motion_us + MOTION_TIMEOUT_US;
    portENTER_CRITICAL(&sensor_mux);
    published = snapshot;
    portEXIT_CRITICAL(&sensor_mux);
}

static void start_scan();

static int gap_event_handler(
    struct ble_gap_event *event,
    void *arg
);

static int service_discovery_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    const struct ble_gatt_svc *service,
    void *arg
);

static int battery_service_discovery_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    const struct ble_gatt_svc *service,
    void *arg
);

static int subscribe_write_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    struct ble_gatt_attr *attr,
    void *arg
)
{
    if (error->status == 0)
    {
        printf("Subscribed to CSC notifications!\n");
    }
    else
    {
        printf(
            "Failed to subscribe. status=%d\n",
            error->status
        );
    }

    return 0;
}

static int descriptor_discovery_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    uint16_t chr_val_handle,
    const struct ble_gatt_dsc *dsc,
    void *arg
)
{
    if (error->status == 0)
    {
        char uuid_str[BLE_UUID_STR_LEN];

        ble_uuid_to_str(
            &dsc->uuid.u,
            uuid_str
        );

        printf(
            "Descriptor found: handle=%u uuid=%s\n",
            dsc->handle,
            uuid_str
        );

        if (!csc_subscribed &&
            ble_uuid_cmp(
                &dsc->uuid.u,
                &cccd_uuid.u
            ) == 0)
        {
            csc_subscribed = true;

            printf("Found CSC Measurement CCCD at handle %u\n", dsc->handle);

            uint8_t enable_notifications[2] = {
                0x01,
                0x00
            };

            int rc = ble_gattc_write_flat(
                conn_handle,
                dsc->handle,
                enable_notifications,
                sizeof(enable_notifications),
                subscribe_write_cb,
                nullptr
            );

            if (rc != 0)
            {
                printf("Failed to write CCCD: %d\n", rc);
                csc_subscribed = false;
            }
        }

        return 0;
    }

    if (error->status == BLE_HS_EDONE)
    {
        printf("Descriptor discovery complete\n");
        return 0;
    }

    printf(
        "Descriptor discovery failed: %d\n",
        error->status
    );

    return 0;
}

static int characteristic_discovery_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    const struct ble_gatt_chr *chr,
    void *arg
)
{
    if (error->status == 0)
    {
        csc_value_handle = chr->val_handle;

        printf(
            "Found CSC Measurement characteristic\n"
            "  definition handle: %u\n"
            "  value handle: %u\n",
            chr->def_handle,
            chr->val_handle
        );

        int rc = ble_gattc_disc_all_dscs(
            conn_handle,
            chr->val_handle,
            csc_service_end_handle,
            descriptor_discovery_cb,
            nullptr
        );

        if (rc != 0)
        {
            printf(
                "Failed to start descriptor discovery: %d\n",
                rc
            );
        }

        return 0;
    }

    if (error->status == BLE_HS_EDONE)
    {
        printf("CSC characteristic discovery complete\n");
        printf("Starting battery service discovery...\n");

        int rc = ble_gattc_disc_svc_by_uuid(
            conn_handle,
            &battery_service_uuid.u,
            battery_service_discovery_cb,
            nullptr
        );

        if (rc != 0)
        {
            printf(
                "Failed to start battery service discovery: %d\n",
                rc
            );
        }

        return 0;
    }

    printf(
        "Characteristic discovery failed: %d\n",
        error->status
    );

    return 0;
}

static int battery_read_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    struct ble_gatt_attr *attr,
    void *arg
)
{
    if (error->status != 0)
    {
        printf(
            "Battery read failed: %d\n",
            error->status
        );
        return 0;
    }

    if (attr == nullptr || attr->om == nullptr)
    {
        printf("Battery read returned no data\n");
        return 0;
    }

    uint8_t battery_level = 0;


    if (os_mbuf_copydata(
        attr->om,
        0,
        sizeof(battery_level),
        &battery_level
    ) != 0)
    {
        printf("Failed to copy battery level\n");
        return 0;
    }

    current_battery_percentage = battery_level;
    battery_valid = battery_level <= 100;
    publish_sensor_data();

    printf(
        "BK805 battery: %u%%\n",
        battery_level
    );

    return 0;
}


static int battery_characteristic_discovery_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    const struct ble_gatt_chr *chr,
    void *arg
)
{
    if (error->status == 0)
    {
        printf(
            "Found Battery Level characteristic\n"
            "  definition handle: %u\n"
            "  value handle: %u\n",
            chr->def_handle,
            chr->val_handle
        );

        int rc = ble_gattc_read(
            conn_handle,
            chr->val_handle,
            battery_read_cb,
            nullptr
        );

        if (rc != 0)
        {
            printf(
                "Failed to start battery read: %d\n",
                rc
            );
        }

        return 0;
    }

   if (error->status == BLE_HS_EDONE)
    {
        printf("Battery characteristic discovery complete\n");
        return 0;
    }

    printf(
        "Battery characteristic discovery failed: %d\n",
        error->status
    );

    return 0;
}

static int battery_service_discovery_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    const struct ble_gatt_svc *service,
    void *arg
)
{
    if (error->status == 0)
    {
        printf(
            "Found Battery service\n"
            "  start handle: %u\n"
            "  end handle: %u\n",
            service->start_handle,
            service->end_handle
        );

        int rc = ble_gattc_disc_chrs_by_uuid(
            conn_handle,
            service->start_handle,
            service->end_handle,
            &battery_level_uuid.u,
            battery_characteristic_discovery_cb,
            nullptr
        );

        if (rc != 0)
        {
            printf(
                "Failed to start Battery Level characteristic discovery: %d\n",
                rc
            );
        }

        return 0;
    }

    if (error->status == BLE_HS_EDONE)
    {
        printf("Battery service discovery complete\n");
        return 0;
    }

    printf(
        "Battery service discovery failed: %d\n",
        error->status
    );

    return 0;
}

static int service_discovery_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    const struct ble_gatt_svc *service,
    void *arg
)
{
    if (error->status == 0)
    {
        printf(
            "Found CSC service\n"
            "  start handle: %u\n"
            "  end handle: %u\n",
            service->start_handle,
            service->end_handle
        );

        csc_service_end_handle = service->end_handle;

        int rc = ble_gattc_disc_chrs_by_uuid(
            conn_handle,
            service->start_handle,
            service->end_handle,
            &csc_measurement_uuid.u,
            characteristic_discovery_cb,
            nullptr
        );

        if (rc != 0)
        {
            printf(
                "Failed to start characteristic discovery: %d\n",
                rc
            );
        }

        return 0;
    }

    if (error->status == BLE_HS_EDONE)
    {
        printf("Service discovery complete\n");
        return 0;
    }

    printf(
        "Service discovery failed: %d\n",
        error->status
    );

    return 0;
}

static void handle_csc_measurement(struct os_mbuf *om)
{
    uint8_t data[11];

    int len = OS_MBUF_PKTLEN(om);

    if (len < 1)
        return;

    if (len > sizeof(data))
        len = sizeof(data);

    if (os_mbuf_copydata(om, 0, len, data) != 0)
    {
        printf("Failed to copy CSC packet\n");
        return;
    }

    uint8_t flags = data[0];

    bool has_wheel = flags & 0x01;
    bool has_crank = flags & 0x02;

    int index = 1;

    uint32_t wheel_revs = 0;
    uint16_t wheel_time = 0;

    uint16_t crank_revs = 0;
    uint16_t crank_time = 0;

    if (has_wheel)
    {
        if (len < index + 6)
            return;

        wheel_revs =
            ((uint32_t)data[index]) |
            ((uint32_t)data[index + 1] << 8) |
            ((uint32_t)data[index + 2] << 16) |
            ((uint32_t)data[index + 3] << 24);

        index += 4;

        wheel_time =
            ((uint16_t)data[index]) |
            ((uint16_t)data[index + 1] << 8);



        index += 2;
    }

    if (has_crank)
    {
        if (len < index + 4)
            return;

        crank_revs =
            ((uint16_t)data[index]) |
            ((uint16_t)data[index + 1] << 8);

        index += 2;

        crank_time =
            ((uint16_t)data[index]) |
            ((uint16_t)data[index + 1] << 8);
    }

    const int64_t now = esp_timer_get_time();
    // CSC Measurement flags identify which cumulative revolution fields this
    // sensor provides. Keep the capability for the connection lifetime so a
    // stopped wheel/crank can be recorded as zero instead of missing data.
    wheel_supported = wheel_supported || has_wheel;
    crank_supported = crank_supported || has_crank;
    if (has_wheel)
    {
        if (have_previous_wheel)
        {
            const uint32_t delta = wheel_revs - previous_wheel_revs;
            const uint16_t ticks = wheel_time - previous_wheel_time;
            const double elapsed_s = (now - previous_wheel_received_us) / 1000000.0;
            // Generous 150 km/h bound plus two revolutions for packet jitter.
            // Reject counter resets/jumps and rebaseline rather than adding distance.
            const double max_revs = elapsed_s * (150.0 / 3.6) /
                                    WHEEL_CIRCUMFERENCE_METERS + 2;
            const float speed = ticks ? delta * WHEEL_CIRCUMFERENCE_METERS *
                                        3.6f * 1024.0f / ticks : 0;
            if (delta > max_revs || (delta > 0 && (ticks == 0 || speed > 150))) {
                ++wheel_generation;
                current_speed_kmh = 0;
                last_wheel_motion_us = 0;
            } else if (delta > 0) {
                current_speed_kmh = speed;
                last_wheel_motion_us = now;
            }
        }
        current_wheel_revs = previous_wheel_revs = wheel_revs;
        previous_wheel_time = wheel_time;
        previous_wheel_received_us = now;
        have_previous_wheel = true;
    }
    if (has_crank)
    {
        if (have_previous_crank)
        {
            const uint16_t delta = crank_revs - previous_crank_revs;
            const uint16_t ticks = crank_time - previous_crank_time;
            const float cadence = ticks ? delta * 60.0f * 1024.0f / ticks : 0;
            if (delta > 0 && (ticks == 0 || cadence > 250)) {
                current_cadence_rpm = 0;
                last_crank_motion_us = 0;
            } else if (delta > 0) {
                current_cadence_rpm = cadence;
                last_crank_motion_us = now;
            }
        }
        previous_crank_revs = crank_revs;
        previous_crank_time = crank_time;
        have_previous_crank = true;
    }
    publish_sensor_data();
}

// These operations and their timer callbacks run on the NimBLE host queue.
static void end_search()
{
    search_deadline_us = 0;
    ble_npl_callout_stop(&search_timer);
    connection_state = ConnectionState::Idle;
    if (ble_gap_disc_active()) ble_gap_disc_cancel();
    if (connecting) ble_gap_conn_cancel();
    publish_sensor_data();
}

static void start_scan()
{
    const int64_t remaining_us = search_deadline_us - esp_timer_get_time();
    if (remaining_us <= 0) { end_search(); return; }
    if (connected || connecting || ble_gap_disc_active()) return;
    if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
        end_search();
        return;
    }
    ble_gap_disc_params params{};
    params.filter_duplicates = 1;
    params.passive = 1;
    const int rc = ble_gap_disc(own_addr_type,
        static_cast<int32_t>((remaining_us + 999) / 1000),
        &params, gap_event_handler, nullptr);
    connection_state = ConnectionState::Searching;
    publish_sensor_data();
    if (rc != 0) printf("Scan start failed: %d\n", rc);
}

static void search_tick(ble_npl_event*)
{
    if (search_deadline_us == 0 || connected) return;
    if (esp_timer_get_time() >= search_deadline_us) {
        end_search();
        return;
    }
    start_scan(); // Retry failed attempts without extending the deadline.
    if (search_deadline_us != 0) {
        const int64_t remaining_ms = std::max<int64_t>(1,
            (search_deadline_us - esp_timer_get_time() + 999) / 1000);
        const auto ticks = ble_npl_time_ms_to_ticks32(
            static_cast<uint32_t>(std::min<int64_t>(250, remaining_ms)));
        if (ble_npl_callout_reset(&search_timer, ticks) != 0) end_search();
    }
}

static void begin_search(ble_npl_event* = nullptr)
{
    // Let a cancelled connection finish before accepting a new request.
    if (!host_synced || connected || connecting || search_deadline_us != 0) return;
    search_deadline_us = esp_timer_get_time() + SEARCH_WINDOW_US;
    connection_state = ConnectionState::Searching;
    publish_sensor_data();
    search_tick(nullptr);
}

static void cancel_search_cb(ble_npl_event*)
{
    if (!connected) end_search();
}

void search()
{
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &search_event);
}

void cancel_search()
{
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &cancel_event);
}

static int gap_event_handler(
    struct ble_gap_event *event,
    void *arg
)
{
    switch (event->type)
    {
        case BLE_GAP_EVENT_DISC:
        {
            if (connecting || connected || search_deadline_us == 0 ||
                esp_timer_get_time() >= search_deadline_us)
                return 0;

            const struct ble_gap_disc_desc *disc = &event->disc;

            struct ble_hs_adv_fields fields;

            int rc = ble_hs_adv_parse_fields(
                &fields,
                disc->data,
                disc->length_data
            );

            if (rc != 0)
                return 0;

            const bool known_sensor = have_sensor_address &&
                disc->addr.type == sensor_address.type &&
                std::memcmp(disc->addr.val, sensor_address.val, 6) == 0;
            if (known_sensor || (!have_sensor_address && fields.name != nullptr &&
                fields.name_len >= 3 &&
                fields.name[0] == 'B' &&
                fields.name[1] == 'K' &&
                fields.name[2] == '8'))
            {
                printf(
                    "Found BK8: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    disc->addr.val[5],
                    disc->addr.val[4],
                    disc->addr.val[3],
                    disc->addr.val[2],
                    disc->addr.val[1],
                    disc->addr.val[0]
                );

                connecting = true;
                connection_state = ConnectionState::Connecting;
                candidate_address = disc->addr;
                publish_sensor_data();

                ble_addr_t sensor_addr = disc->addr;

                ble_gap_disc_cancel();

                printf("Connecting...\n");

                rc = ble_gap_connect(
                    own_addr_type,
                    &sensor_addr,
                    static_cast<int32_t>(std::max<int64_t>(1,
                        std::min<int64_t>(10000,
                            (search_deadline_us - esp_timer_get_time()) / 1000))),
                    nullptr,
                    gap_event_handler,
                    nullptr
                );

                if (rc != 0)
                {
                    printf("Failed to start connection: %d\n", rc);
                    connecting = false;
                }
            }

            break;
        }

        case BLE_GAP_EVENT_DISC_COMPLETE:
            if (search_deadline_us != 0 && !connecting && !connected &&
                esp_timer_get_time() >= search_deadline_us) end_search();
            break;

        case BLE_GAP_EVENT_CONNECT:
        {
            connecting = false;
            if (event->connect.status == 0)
                {
                    printf("Connected to BK805!\n");

                    // A success can race cancellation or the recovery deadline.
                    if (search_deadline_us == 0 ||
                        esp_timer_get_time() >= search_deadline_us) {
                        ble_gap_terminate(event->connect.conn_handle,
                                          BLE_ERR_REM_USER_CONN_TERM);
                        end_search();
                        break;
                    }
                    search_deadline_us = 0;
                    ble_npl_callout_stop(&search_timer);
                    sensor_address = candidate_address;
                    have_sensor_address = true;
                    connected = true;
                    battery_valid = false;
                    wheel_supported = false;
                    crank_supported = false;
                    connection_state = ConnectionState::Connected;
                    publish_sensor_data();

                    uint16_t conn_handle = event->connect.conn_handle;

                    int rc = ble_gattc_disc_svc_by_uuid(
                        conn_handle,
                        &csc_service_uuid.u,
                        service_discovery_cb,
                        nullptr
                    );

                    if (rc != 0)
                    {
                        printf(
                            "Failed to start CSC service discovery: %d\n",
                            rc
                        );
                    }
                }
            else
            {
                printf(
                    "Connection failed. status=%d\n",
                    event->connect.status
                );

                connecting = false;
            }

            break;
        }

        case BLE_GAP_EVENT_NOTIFY_RX:
        {
            if (event->notify_rx.attr_handle == csc_value_handle)
            {
                handle_csc_measurement(event->notify_rx.om);
            }

            break;
        }

        case BLE_GAP_EVENT_DISCONNECT:
        {
            printf(
                "Disconnected. reason=%d\n",
                event->disconnect.reason
            );

            const bool was_connected = connected;
            connecting = false;
            connected = false;
            battery_valid = false;
            connection_state = ConnectionState::Idle;
            csc_subscribed = false;

            csc_value_handle = 0;
            csc_service_end_handle = 0;

            have_previous_wheel = false;
            have_previous_crank = false;
            wheel_supported = false;
            crank_supported = false;
            ++wheel_generation;
            last_wheel_motion_us = last_crank_motion_us = 0;

            current_speed_kmh = 0.0f;
            current_cadence_rpm = 0.0f;
            publish_sensor_data();

            if (was_connected) begin_search();

            break;
        }

        default:
            break;
    }

    return 0;
}


static void on_ble_sync()
{
    printf("NimBLE synced\n");
    host_synced = true;
    begin_search();
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}


void init()
{
    esp_err_t ret = nvs_flash_init();

    if (
        ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND
    )
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else
    {
        ESP_ERROR_CHECK(ret);
    }

    ESP_ERROR_CHECK(nimble_port_init());

    ble_npl_event_init(&search_event, begin_search, nullptr);
    ble_npl_event_init(&cancel_event, cancel_search_cb, nullptr);
    ESP_ERROR_CHECK(ble_npl_callout_init(&search_timer,
        nimble_port_get_dflt_eventq(), search_tick, nullptr));
    ble_hs_cfg.sync_cb = on_ble_sync;

    nimble_port_freertos_init(nimble_host_task);

    printf("NimBLE initialized\n");
}


SensorData get_sensor_data()
{
    portENTER_CRITICAL(&sensor_mux);
    SensorData snapshot = published;
    portEXIT_CRITICAL(&sensor_mux);
    const int64_t now = esp_timer_get_time();
    if (!snapshot.connected || now >= snapshot.speed_valid_until_us)
        snapshot.speed_kmh = 0;
    if (!snapshot.connected || now >= snapshot.cadence_valid_until_us)
        snapshot.cadence_rpm = 0;
    return snapshot;
}

}
