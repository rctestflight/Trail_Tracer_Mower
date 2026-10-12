# Rover test guide: branch feature/rx-v2-firmware

This guide tells you how to load this branch on a rover with the PlatformIO Upload button and a USB cable, and how to use it in the field.

> **Quick start**
> 1. Check out the branch `feature/rx-v2-firmware` (VS Code: the branch name in the status bar, or Appendix A).
> 2. Connect the rover by USB. Push the PlatformIO **Upload** button (the arrow in the status bar).
> 3. Do what the terminal asks. Over V1.0 it asks for the 0 button once (section 3).
> 4. If the terminal shows `Full power-off now`: USB and vehicle power off, 5 s, on.
> 5. Open the PlatformIO serial monitor (the plug in the status bar). Type `boot`: the answer starts with `BOOT rover-firmware 0.5.56`.
> 6. Later updates: push **Upload** again. Details: sections [3](#3-what-the-upload-button-does), [6](#6-field-workflow-laptop-rover-usb-cable), [7](#7-back-to-v10).

## 1. What changes, what stays

- The ESP32 firmware changes: the main branch, plus rover mode (off by default), telemetry, a flash log, settings commands over USB, updates over USB with rollback, and the monitor role. `README.md` and `docs/rover_guide.md` list each change.
- The STM32 app on the board stays as it is. The firmware runs the v1 STM32 app on the v1 link. The STM32 v2 app comes in a later step.
- Outside rover mode, the rover moves as on `main`. The ODrive CAN code is the main-branch code, with no change.

## 2. Get the branch

In VS Code, click the branch name in the status bar and select `origin/feature/rx-v2-firmware`. The command lines are in Appendix A.

## 3. What the Upload button does

The Upload button builds the firmware, then looks at the board on the USB port (`tools/upload_auto.py`):

| Board | What Upload does | Your part |
|---|---|---|
| This firmware | Update over USB. The settings and the log stay. A new image that gets no confirm within 120 s returns to the previous image. | Nothing |
| V1.0 or any other firmware | Full install: download mode, erase of the full flash, write of all images. The settings and the files on the rover's USB drive are lost. | The 0 button once, then a full power-off |

> **Caution:** a full install erases config.json on the rover's USB drive. If you changed a setting on V1.0, copy config.json from the rover's USB drive to the PC first. Section 6.2 tells you how to set the values again.

### 3.1 Full install over V1.0

1. Push **Upload**.
2. V1.0 does not go into download mode by itself. The terminal shows the button steps and waits 120 s:
   1. Disconnect the USB cable.
   2. Switch off the vehicle power.
   3. Hold the 0 button.
   4. Connect the USB cable.
   5. Release the 0 button.
3. The erase and the write take about 30 s.
4. The terminal shows `Full power-off now: USB and vehicle power off, 5 s, on.` Do that. A reset does not start the new firmware after a full install.

## 4. Confirm the load

Open the PlatformIO serial monitor. Type `boot` and push Enter. The answer starts with `BOOT rover-firmware 0.5.56`. Each second the board sends a `HB` line. `packets=100` in that line shows that the STM32 app sends 100 packets a second. The start lines show `STM link v1` for a v1 STM32 app.

Close the monitor before you push Upload or use a tool from section 6.

## 5. Defaults

After a full install, the firmware starts with the main-branch values:

| Setting | Value |
|---|---|
| `kp`, `ki`, `kd` | 5, 0, 0 |
| `steering_rate_kp`, `steering_rate_ki`, `steering_rate_kd` | 0.02, 0.01, 0 |
| `steering_rate_max_dps` | 90 |
| `CRUISE_THROTTLE`, `CREEP_THROTTLE` | 1650, 1620 |
| `vehicleID`, `rx_channel` | 3, 1 |
| `rover_enabled` | false (rover mode off) |
| `imu_required` | true: if the IMU fails, the motors stay locked |

`docs/settings_def.json` lists every setting with its unit, limits and default.

## 6. Field workflow (laptop, rover, USB cable)

### 6.1 Live lines

Open the serial monitor. The board sends the main-branch status lines, the `HB` line each second and its events.

### 6.2 Settings

In the serial monitor, type `list`, `get <key>`, `set <key> <value>` or `save`. A `set` acts at once. `save` keeps the values over a power-off.

### 6.3 Log dump

The rover keeps a log in its flash. Close the monitor, then dump and decode the log with the command in Appendix A.

### 6.4 Later updates

Push **Upload**. A board with this firmware updates over USB with no download mode and no power-off.

## 7. Back to V1.0

1. Check out the branch `main`.
2. Do the button steps of section 3.1 (this firmware ignores the USB reset of the main-branch upload).
3. Push **Upload**.
4. Do a full power-off.

## 8. Known limits

- The motor command goes out on the 12 ms timer of the main branch. On a bench board, 99 % of commands left within 10.7 ms of the sensor frame.
- The log erases flash only while the axes are disarmed or the rover charges. The erased flash holds about 25 min of armed run at the default log rate of 20 Hz. After that, the log stops until the axes disarm.
- Verified on bench boards: the update over USB with the Upload button, and the full install with the Upload button from this firmware. The full install also ran from a main-branch build with no button, on a Linux PC. Not verified: the button steps, and the way back to V1.0 (section 7).
- The shortest update period setting is 3.6 ms (`dsp_frame_period_us` 3600).
- This build did not drive an ODrive or a real rover before this test. The bench has no ODrive.

## Appendix A: command lines

**PowerShell**
```powershell
git fetch origin
```

**PowerShell**
```powershell
git checkout feature/rx-v2-firmware
```

The Upload button:

**PowerShell**
```powershell
pio run -t upload
```

A full install on a board with this firmware (the settings and the log are lost):

**PowerShell**
```powershell
$env:FULL_INSTALL = "1"; pio run -t upload; Remove-Item Env:FULL_INSTALL
```

The serial monitor:

**PowerShell**
```powershell
pio device monitor -b 115200
```

The tools need Python 3.8 or later and pyserial. Replace `COM7` with the rover's port.

**PowerShell**
```powershell
python -m pip install pyserial
```

**PowerShell**
```powershell
python tools/rover_settings.py COM7 list
```

**PowerShell**
```powershell
python tools/rover_settings.py COM7 set kp 5
```

**PowerShell**
```powershell
python tools/rover_settings.py COM7 save
```

**PowerShell**
```powershell
python tools/rover_log.py usbdump COM7 --out run1.jsonl
```
