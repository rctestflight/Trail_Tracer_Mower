# Trail Tracer merge

The active `src` uses `src_trail_tracer` as its application base. The source reference folder is unchanged.

Preserved from the previous active firmware:
- Native TWAI CAN, GPIO36 TX / GPIO38 RX, 1 Mbit/s, ODrive nodes 0-5.
- RC differential velocity mixing, deadband/expo/filtering, per-node current limits, signal-age validation, axis-error handling, bus-off recovery, and encoder polling.
- MPU6050 on GPIO39 SDA / GPIO40 SCL, startup offset calibration, and Z gyro output every 20 ms. Keep the sensor still during startup. Initialization failure leaves outputs neutral and halts startup.
- Corrected RMT allocation, HIGH pulse capture, and signal freshness tracking.

Newer USB MSC configuration, ESP-NOW telemetry, drive modes, front/rear PID steering, BTx behavior, charging, and STM32 boot handshake are retained. USB support headers and the flash partition table came from the local Trail-Tracer project because they were absent from the supplied reference folder.

Integration details:
- GPIO39/40 auxiliary creep/stop inputs are disabled because those pins now carry I2C.
- CAN servicing and button debounce run independently of STM32 data. Incomplete UART packets do not block the main loop waiting for bytes.
- ODrive commands retain the existing raw RC control path. Trail Tracer wire-follow steering and autonomous throttle still control PWM outputs; they do not drive the ODrive CAN setpoints.
- The newer STM32 protocol uses a packed 122-byte packet with CRC16, charge_code, and vesp_3v3. It requires compatible STM32 firmware; the old 116-byte/checksum protocol is not compatible.
- USB configuration requires the new partitions.csv layout. No firmware was uploaded during this merge.

The previous active main.cpp is saved in tmp/main_before_trail_merge.cpp.
