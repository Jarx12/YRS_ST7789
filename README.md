# YRS_ST7789

Temperature monitor for an ESP32-C3 with an ST7789 LCD and two thermistor inputs.

## Features

- Displays engine and A/C temperatures and calculated resistances.
- Shows a 0–120 °C engine temperature bar with markers at 95 °C and 110 °C.
- Shows a full-screen alert when the engine reaches 110 °C.
- Keeps the alert active until the engine temperature falls to 105 °C or lower.
- Updates at approximately 100 ms intervals.
- Joins a WiFi network in the background and serves an HTTP page that installs a new firmware over the air.
- Exposes a JSON status endpoint with the live readings, IP, signal strength and free heap.

## Hardware and configuration

The current PlatformIO environment is `esp32-c3-devkitm-1` using the ESP32 Arduino framework.

| Function              | GPIO ## |
| Engine thermistor ADC | GPIO 00 |
| A/C thermistor ADC    | GPIO 02 |
| LCD SPI MOSI          | GPIO 03 |
| LCD SPI SCLK          | GPIO 04 |
| LCD chip select       | GPIO 07 |
| LCD data/command      | GPIO 01 |
| LCD reset             | GPIO 10 |
| LCD backlight         | GPIO 21 |

LCD resolution = 172 × 320 (rotated to 320 × 172 landscape)
SPI frequency = 27 MHz


The display uses the tracked TFT_eSPI 2.5.43 copy in `lib/TFT_eSPI-2.5.43`

## Temperature model and calibration

Both channels use Steinhart–Hart conversion from the measured divider voltage. The engine channel applies an empirical `+3 °C` correction because the remote thermistor location loses heat relative to the heat source; this is an installation-specific correction, not a correction to the Steinhart–Hart coefficients.

The calibration constants and divider values are kept in:

- `src/YRS_ST7789.ino`
- `Docs/SH FOR BLUE THERMISTOR CAR 110C.png`
- `Docs/SH FOR HW503.JPG`

## Build and upload

From the project directory, with PlatformIO installed:

```text
pio run
pio run -t upload
pio device monitor
```

The serial monitor speed configured in `platformio.ini` is 115200 baud.

## WiFi and HTTP OTA update

The board joins a WiFi network in the background and serves a small web server on port 80. The network is only used for the update page: no dashboard feature depends on it, and a missing or locked-out access point does not stop the display, the sensors or the alert.

### Credentials

Credentials are **not** in the repository. Copy the template and fill it in:

```text
copy include\secrets.h.example include\secrets.h
```

| Macro            | Used for                                  |
| ---------------- | ----------------------------------------- |
| `WIFI_SSID`      | Network to join                           |
| `WIFI_PASSWORD`  | Network password                          |
| `WIFI_HOSTNAME`  | mDNS/hostname, also reported by `/info`   |
| `OTA_USER`       | HTTP Basic user for the update page       |
| `OTA_PASS`       | HTTP Basic password for the update page   |

`include/secrets.h` is listed in `.gitignore`, so it never reaches the repository. The build fails with a clear `#error` if the file is missing rather than silently joining nothing.

### Endpoints

All routes are behind HTTP Basic auth using `OTA_USER` / `OTA_PASS`.

| Route      | Method | Purpose                                                       |
| ---------- | ------ | ------------------------------------------------------------- |
| `/`        | GET    | Upload page                                                   |
| `/info`    | GET    | JSON status: firmware version, IP, RSSI, uptime, free heap, and the last reading of each channel |
| `/update`  | POST   | Firmware upload (`multipart/form-data`)                       |

### Installing a firmware over the air

1. Build the image: `pio run` (this produces `.pio/build/esp32-c3-devkitm-1/firmware.bin`).
2. Find the board's IP — the serial monitor prints it on join, or check the router's client list.
3. Open `http://<board-ip>/` and sign in with the OTA credentials.
4. Select `firmware.bin` and submit. The board shows an `ACTUALIZANDO` screen, writes the flash and reboots into the new image.

### Requirements and limits

- **Two app slots are required.** OTA writes into the app partition the bootloader is not running from, so the partition table must keep both `ota_0` and `ota_1`. The `default.csv` table already does (`platformio.ini` sets `board_build.partitions = default.csv`); switching to a single-slot table such as `huge_app.csv` would silently remove OTA capability.
- **The image must fit one slot.** Each slot is 0x140000 (1,310,720) bytes. The current build is about 894 kB (66 % of a slot) — WiFi, `WebServer` and `Update` together cost roughly 520 kB of flash, so check the size reported by `pio run` before adding more dependencies.
- **The display freezes during the upload.** The Arduino `WebServer` serves an upload synchronously, so the temperature dashboard stops refreshing for the duration. The `ACTUALIZANDO` screen is the only feedback, and nothing is sampled while it runs — treat the gap as missing data, not a stable temperature.
- **Plain HTTP, not HTTPS.** Basic auth only base64-encodes the credentials; they are readable by anything on the path. Use the update page on a trusted network only, and change the defaults in `secrets.h` before first use.
- **Do not power-cycle mid-upload.** An interrupted write leaves the target slot incomplete; the running slot is untouched, so the current firmware still boots, but the next update attempt is needed.

## Project layout

- `src/YRS_ST7789.ino` — firmware, display layout, sensor conversion, filtering, and alert state.
- `platformio.ini` — ESP32-C3 board, TFT_eSPI, display, and SPI settings.
- `lib/TFT_eSPI-2.5.43/` — tracked TFT_eSPI library copy.
- `Docs/` — pinout and thermistor calibration references.
- `test/` — PlatformIO test directory; no project-specific tests are currently included.

## Known limitations

### Open thermistors may look valid

The `SENSOR ERROR` state is not a reliable open-thermistor detector. If a thermistor or its wiring opens, the ESP32 ADC input may float and read ambient noise or another plausible intermediate voltage. That value can be converted into a plausible resistance and temperature.

`SENSOR ERROR` only reports readings rejected by the current ADC-range and calculation checks; it does not guarantee that a disconnected or open sensor will be detected. Verify the wiring and sensor integrity independently when a false reading could be unsafe.
