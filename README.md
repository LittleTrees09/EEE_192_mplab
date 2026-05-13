# EEE_192 Robot Firmware — HC-05 Quick Connect

Password for pairing: 1234

This file contains simple, focused steps to connect to the robot over an HC-05 Bluetooth serial adapter on Linux and Windows. The firmware accepts commands over a serial interface (common default baud: 38400; some HC-05 modules use 9600).

Serial settings (typical)
- Baud: `38400` (or try `9600` if you see no response)
- Data: `8` | Parity: `N` | Stop: `1` | Flow control: `None`

Common keys sent over serial
- `M` — Manual mode
- `U` — IR follow auto mode
- `O` — Ultrasonic avoid auto mode
- `SPACE` — Stop / pause auto
- `X` — Clear SAFE latch
- Manual drive keys: `W`/`S`/`A`/`D`/`Q`/`E` and arrow keys

Linux (quick)
1. Turn on Bluetooth (system settings or `bluetoothctl`).
2. In your desktop Bluetooth UI, find `HC-05`, right-click it and choose `Connect` → `Serial` (or similar "Connect Serial" action).
3. Pairing PIN: enter `1234` when prompted.
4. Open a terminal on your PC.
5. Connect with `screen` (choose the baud that works for your module):

```bash
screen /dev/rfcomm0 38400
# or, if your module uses 9600:
screen /dev/rfcomm0 9600
```

Notes for Linux
- If your desktop does not create `/dev/rfcomm0` automatically, use `rfcomm` or `bluetoothctl` to bind the device.
- If `screen` complains, ensure the Bluetooth connection shows as "connected" in your Bluetooth settings.

Windows (quick)
1. Open Settings → Bluetooth & devices and make sure Bluetooth is on.
2. Put the HC-05 into pairing mode and pair it; PIN: `1234`.
3. Ensure the device shows as "Connected" (not just paired).
4. Open Device Manager → Ports (COM & LPT) and note the assigned COM port (e.g., COM8).
5. Open `PuTTY` (or `Tera Term`) → choose Serial, set Port=`COM8` (your COM), Speed=`38400` (or `9600`), Data=`8`, Parity=`None`, Stop=`1`, Flow control=`None`, then Open.

Using the console
- After opening the serial connection, press the robot's enable button if required, then send `M` for manual drive or `U`/`O` for autos.
- If the robot enters SAFE mode due to comms loss, send `X` to clear and resume.

