#include "fit_recorder.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sd_access.hpp"
#include "esp_log.h"
#include "esp_timer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <sys/stat.h>
#include <unistd.h>

namespace fit_recorder {
namespace {
constexpr char tag[] = "fit_recorder";
constexpr uint32_t fit_epoch_offset = 631065600U;
// Garmin FIT SDK profile version 21.217 (major * 1000 + minor).
constexpr uint16_t fit_profile_version = 21217;
constexpr size_t queue_length = 34;
constexpr uint8_t header_size = 14;
enum class Kind : uint8_t { Start, Sample, Stop };
struct Item {
    Kind kind;
    ride::RecordingSample sample;
    ride::RideData totals;
};
QueueHandle_t queue = nullptr;
TaskHandle_t task = nullptr;
portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
Status current_status = Status::Unavailable;

uint16_t crc_byte(uint16_t crc, uint8_t byte)
{
    static constexpr uint16_t table[16] = {0x0000,0xCC01,0xD801,0x1400,0xF001,0x3C00,0x2800,0xE401,0xA001,0x6C00,0x7800,0xB401,0x5000,0x9C01,0x8801,0x4400};
    uint16_t tmp = table[crc & 0x0f];
    crc = (crc >> 4) & 0x0fff;
    crc = crc ^ tmp ^ table[byte & 0x0f];
    tmp = table[crc & 0x0f];
    crc = (crc >> 4) & 0x0fff;
    crc = crc ^ tmp ^ table[(byte >> 4) & 0x0f];
    return crc;
}

struct Writer {
    FILE* file = nullptr;
    uint32_t data_size = 0;
    int64_t last_sync_us = 0;
    int64_t start_utc = 0;
    ride::RecordingSample first{};
    ride::RecordingSample last{};
    ride::RideData totals{};
    char part_path[96]{};
    char final_path[96]{};

    bool raw(const void* bytes, size_t size, bool data)
    {
        if (std::fwrite(bytes, 1, size, file) != size) return false;
        if (data) {
            data_size += size;
        }
        return true;
    }
    bool u8(uint8_t v) { return raw(&v, 1, true); }
    bool u16(uint16_t v) { uint8_t b[2] = {uint8_t(v),uint8_t(v >> 8)}; return raw(b,2,true); }
    bool u32(uint32_t v) { uint8_t b[4] = {uint8_t(v),uint8_t(v >> 8),uint8_t(v >> 16),uint8_t(v >> 24)}; return raw(b,4,true); }
    bool definition(uint8_t local, uint16_t global, const uint8_t* fields, uint8_t count)
    {
        const uint8_t head[6] = {uint8_t(0x40 | local),0,0,uint8_t(global),uint8_t(global >> 8),count};
        return raw(head,sizeof(head),true) && raw(fields,size_t(count)*3,true);
    }
    bool message(uint8_t local) { return u8(local); }
    bool open(const ride::RecordingSample& sample)
    {
        start_utc = sample.utc_seconds - sample.elapsed_seconds;
        std::tm tm{};
        const time_t t = static_cast<time_t>(start_utc);
        if (!gmtime_r(&t, &tm)) return false;
        char name[48];
        if (!std::strftime(name, sizeof(name), "ride_%Y%m%d_%H%M%S", &tm)) return false;
        for (unsigned suffix = 0; suffix < 100; ++suffix) {
            char base[64];
            if (suffix) std::snprintf(base,sizeof(base),"%s_%02u",name,suffix);
            else std::snprintf(base,sizeof(base),"%s",name);
            std::snprintf(part_path,sizeof(part_path),"/sdcard/%s.fit.part",base);
            std::snprintf(final_path,sizeof(final_path),"/sdcard/%s.fit",base);
            struct stat st{};
            if (stat(part_path,&st) != 0 && stat(final_path,&st) != 0) break;
            if (suffix == 99) return false;
        }
        file = std::fopen(part_path,"w+b");
        if (!file) return false;
        uint8_t header[header_size] = {header_size,0x20,uint8_t(fit_profile_version),uint8_t(fit_profile_version>>8),0,0,0,0,'.','F','I','T',0,0};
        const bool ok = raw(header,sizeof(header),false);
        if (!ok) return false;
        first = last = sample;
        last_sync_us = esp_timer_get_time();
        return file_id();
    }
    bool file_id()
    {
        const uint8_t fields[] = {0,1,0,1,2,0x84,2,2,0x84,4,4,0x86};
        return definition(0,0,fields,4) && message(0) && u8(4) && u16(255) && u16(1) && u32(fit_time(start_utc));
    }
    uint32_t fit_time(int64_t utc) const { return utc < fit_epoch_offset ? 0 : static_cast<uint32_t>(utc-fit_epoch_offset); }
    bool event(uint32_t time, uint8_t type)
    {
        const uint8_t f[] = {253,4,0x86,0,1,0,1,1,0};
        return definition(1,21,f,3) && message(1) && u32(time) && u8(0) && u8(type);
    }
    bool record(const ride::RecordingSample& s)
    {
        const uint8_t f[] = {253,4,0x86,0,4,0x85,1,4,0x85,5,4,0x86,6,2,0x84,4,1,2};
        if (!definition(2,20,f,6) || !message(2)) return false;
        const int32_t lat = s.gps_fix_valid ? static_cast<int32_t>(s.latitude * (2147483648.0/180.0)) : INT32_MAX;
        const int32_t lon = s.gps_fix_valid ? static_cast<int32_t>(s.longitude * (2147483648.0/180.0)) : INT32_MAX;
        const uint32_t dist = std::isfinite(s.distance_km) && s.distance_km >= 0 ? static_cast<uint32_t>(std::min<double>(s.distance_km*100000.0,4294967294.0)) : UINT32_MAX;
        const uint16_t speed = s.speed_valid && std::isfinite(s.speed_kmh) && s.speed_kmh >= 0 ? static_cast<uint16_t>(std::min<float>(s.speed_kmh/3.6f*1000.0f,65534.0f)) : UINT16_MAX;
        const uint8_t cadence = s.cadence_valid && s.cadence_rpm >= 0 ? static_cast<uint8_t>(std::min<float>(s.cadence_rpm,254.0f)) : UINT8_MAX;
        return u32(fit_time(start_utc+s.elapsed_seconds)) && u32(static_cast<uint32_t>(lat)) && u32(static_cast<uint32_t>(lon)) && u32(dist) && u16(speed) && u8(cadence);
    }
    bool lap_or_session(uint16_t global, uint8_t local, const ride::RecordingSample& begin, const ride::RecordingSample& end, bool session)
    {
        const uint8_t lap_fields[] = {253,4,0x86,2,4,0x86,7,4,0x86,8,4,0x86,9,4,0x86,14,2,0x84,15,2,0x84,18,1,2,0,1,0,1,1,0};
        const uint8_t session_fields[] = {253,4,0x86,2,4,0x86,7,4,0x86,8,4,0x86,9,4,0x86,14,2,0x84,15,2,0x84,18,1,2,5,1,0,6,1,0,0,1,0,1,1,0};
        const uint8_t* fields = session ? session_fields : lap_fields;
        const uint8_t count = session ? 12 : 10;
        if (!definition(local,global,fields,count) || !message(local)) return false;
        const float avg = totals.average_speed_kmh / 3.6f;
        const float max = totals.max_speed_kmh / 3.6f;
        const uint32_t elapsed = totals.elapsed_seconds * 1000U;
        const uint32_t dist = static_cast<uint32_t>(std::max(0.0f,totals.distance_km)*100000.0f);
        bool ok = u32(fit_time(start_utc+end.elapsed_seconds)) && u32(fit_time(start_utc)) && u32(elapsed) && u32(elapsed) && u32(dist) && u16(static_cast<uint16_t>(std::min(avg*1000.0f,65534.0f))) && u16(static_cast<uint16_t>(std::min(max*1000.0f,65534.0f))) && u8(static_cast<uint8_t>(std::min(totals.average_cadence_rpm,254.0f)));
        if (session) ok = ok && u8(2) && u8(0) && u8(0) && u8(1);
        else ok = ok && u8(9) && u8(1);
        (void)begin;
        return ok;
    }
    bool summaries()
    {
        if (!event(fit_time(start_utc+last.elapsed_seconds),1)) return false;
        if (!lap_or_session(19,3,first,last,false)) return false;
        if (!lap_or_session(18,4,first,last,true)) return false;
        const uint8_t f[] = {253,4,0x86,0,4,0x86,1,2,0x84,2,1,0,3,1,0};
        if (!definition(5,34,f,5) || !message(5)) return false;
        return u32(fit_time(start_utc+last.elapsed_seconds)) && u32(fit_time(start_utc+last.elapsed_seconds)) && u16(1) && u8(0) && u8(1);
    }
    bool finalize()
    {
        if (!file) return false;
        const bool content_ok = summaries();
        if (std::fflush(file) != 0 || fsync(fileno(file)) != 0) return false;
        if (!content_ok) return false;
        uint8_t header[header_size] = {header_size,0x20,uint8_t(fit_profile_version),uint8_t(fit_profile_version>>8),uint8_t(data_size),uint8_t(data_size>>8),uint8_t(data_size>>16),uint8_t(data_size>>24),'.','F','I','T',0,0};
        uint16_t header_crc = 0;
        for (size_t i=0; i<12; ++i) header_crc=crc_byte(header_crc,header[i]);
        header[12]=uint8_t(header_crc);
        header[13]=uint8_t(header_crc>>8);
        if (fseek(file,0,SEEK_SET) != 0 || std::fwrite(header,1,sizeof(header),file) != sizeof(header)) return false;
        if (std::fflush(file) != 0 || fseek(file,0,SEEK_SET) != 0) return false;
        uint16_t crc = 0;
        uint8_t block[512];
        size_t n;
        while ((n=std::fread(block,1,sizeof(block),file)) != 0)
            for (size_t i=0;i<n;++i) crc=crc_byte(crc,block[i]);
        if (std::ferror(file)) return false;
        if (fseek(file,0,SEEK_END) != 0) return false;
        uint8_t crc_bytes[2] = {uint8_t(crc),uint8_t(crc>>8)};
        if (std::fwrite(crc_bytes,1,2,file) != 2 || std::fflush(file) != 0 || fsync(fileno(file)) != 0) return false;
        if (fclose(file) != 0) { file=nullptr; return false; }
        file=nullptr;
        return rename(part_path,final_path)==0;
    }
};

void set_status(Status value) { portENTER_CRITICAL(&state_mux); current_status=value; portEXIT_CRITICAL(&state_mux); }
// status() is exported below; all accesses use the state spinlock.

void writer_task(void*)
{
    Writer writer;
    bool active=false, waiting=false;
    Item item{};
    for (;;) {
        if (xQueueReceive(queue,&item,active?pdMS_TO_TICKS(5000):portMAX_DELAY)!=pdTRUE) {
            if (active && !waiting && writer.file && sd_access::lock(pdMS_TO_TICKS(5000))) {
                const bool ok=std::fflush(writer.file)==0 && fsync(fileno(writer.file))==0;
                sd_access::unlock();
                if (!ok) { set_status(Status::Error); active=false; }
            }
            continue;
        }
        if (item.kind==Kind::Start) { writer=Writer{}; active=true; waiting=true; set_status(Status::WaitingForTime); continue; }
        if (!active) continue;
        if (item.kind==Kind::Sample) {
            if (waiting) {
                if (!item.sample.utc_valid) continue;
                if (!sd_access::lock(pdMS_TO_TICKS(5000))) { set_status(Status::Error); active=false; continue; }
                const bool ok=writer.open(item.sample);
                sd_access::unlock();
                if (!ok) { ESP_LOGE(tag,"Could not create ride FIT file"); set_status(Status::Error); active=false; continue; }
                waiting=false; set_status(Status::Recording);
                if (!sd_access::lock(pdMS_TO_TICKS(5000))) { set_status(Status::Error); active=false; continue; }
                const bool began=writer.event(writer.fit_time(writer.start_utc),0) && writer.record(item.sample);
                sd_access::unlock();
                if (!began) { set_status(Status::Error); active=false; continue; }
            } else {
                if (!sd_access::lock(pdMS_TO_TICKS(5000))) { set_status(Status::Error); active=false; continue; }
                const bool ok=writer.record(item.sample);
                sd_access::unlock();
                if (!ok) { set_status(Status::Error); active=false; continue; }
            }
            writer.last=item.sample;
            if (esp_timer_get_time()-writer.last_sync_us >= 5000000) {
                if (!sd_access::lock(pdMS_TO_TICKS(5000))) { set_status(Status::Error); active=false; continue; }
                const bool synced=std::fflush(writer.file)==0 && fsync(fileno(writer.file))==0;
                sd_access::unlock();
                if (!synced) { set_status(Status::Error); active=false; }
                else writer.last_sync_us=esp_timer_get_time();
            }
        } else if (item.kind==Kind::Stop) {
            writer.totals=item.totals;
            if (waiting) { ESP_LOGE(tag,"No valid GPS UTC time; ride FIT was not created"); active=false; set_status(Status::Error); continue; }
            if (!sd_access::lock(pdMS_TO_TICKS(5000))) { set_status(Status::Error); active=false; continue; }
            const bool ok=writer.finalize();
            sd_access::unlock();
            if (ok) { ESP_LOGI(tag,"Saved %s",writer.final_path); set_status(Status::Idle); }
            else {
                if (writer.file) { std::fclose(writer.file); writer.file=nullptr; }
                ESP_LOGE(tag,"FIT finalization failed; partial file retained at %s",writer.part_path);
                set_status(Status::Error);
            }
            active=false;
        }
    }
}
} // namespace

bool init()
{
    if (queue) return true;
    queue=xQueueCreate(queue_length,sizeof(Item));
    if (!queue || !sd_access::init() || xTaskCreate(writer_task,"fit_writer",6144,nullptr,6,&task)!=pdPASS) {
        set_status(Status::Unavailable); return false;
    }
    set_status(Status::Idle);
    return true;
}

bool start() { Item i{}; i.kind=Kind::Start; return task && queue && xQueueSend(queue,&i,0)==pdTRUE; }
bool submit(const ride::RecordingSample& sample)
{
    if (!queue || status()==Status::Error) return false;
    if (uxQueueSpacesAvailable(queue)<=2) { set_status(Status::Error); return false; }
    Item i{}; i.kind=Kind::Sample; i.sample=sample;
    if (xQueueSend(queue,&i,0)!=pdTRUE) { set_status(Status::Error); return false; }
    return true;
}
bool stop(const ride::RideData& totals) { Item i{}; i.kind=Kind::Stop; i.totals=totals; return queue && xQueueSend(queue,&i,0)==pdTRUE; }
Status status() { portENTER_CRITICAL(&state_mux); auto s=current_status; portEXIT_CRITICAL(&state_mux); return s; }
}
