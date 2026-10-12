// Monitor role (USB command "monitor", one boot only): the board runs as the PC-tethered ESP-NOW monitor
// (Rx-B role) instead of as a vehicle. The same image and the same bootloader, partition table,
// update path (lib/usb_ota), USB guard and "dld" serve both roles; only the main application
// differs. src/monitor_role.cpp has the line protocol.
#pragma once

namespace monitor_role {
void setup(const char* version);  // from setup(), after the settings load; no vehicle hardware starts
void loop();                      // from loop(), instead of the vehicle loop
}  // namespace monitor_role
