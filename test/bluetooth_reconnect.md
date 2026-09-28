# Manual reconnect checks

Build and upload manually, then check on the ESP32:

- Boot with the sensor asleep: Searching appears, then Sensor not connected / Search after about 30 seconds. Leave it for a minute; no new scan should start.
- Wake the sensor and tap Search: Searching, Connecting, then Sensor connected. Battery percentage appears only after a successful battery read.
- Let a connected sensor sleep: disconnect starts one 30-second recovery window. Wake it inside the window to reconnect automatically; otherwise wake it and tap Search.
- Tap Cancel while searching and while connecting: return to Search and do not restart automatically. A late connection must not remain connected.
- Cause connection failures: retries must share the original 30-second deadline, rather than each getting another 30 seconds.
- Keep the sensor connected but stop the magnets: readings reach zero without starting a new scan.
- During a ride, disconnect and reconnect: elapsed time continues, distance and stats remain, disconnected speed/cadence show --, and reconnection does not add a distance jump.
- Check that the sensor status and Search/Cancel button fit above distance/time without overlapping.

The last connected sensor address is remembered only until reboot. Sensors that change their Bluetooth address may require a reboot to rediscover. Initial selection still uses the BK8 name prefix.
