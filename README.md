# YRS_ST7789

Temperature monitor for an ESP32-C3 with an ST7789 LCD and two thermistor inputs.

## Features

- Displays engine and A/C temperatures and calculated resistances.
- Shows a compact 3-second trend arrow and temperature delta for each channel.
- Shows a 0–120 °C engine temperature bar with markers at 95 °C and 110 °C.
- Shows a full-screen alert when the engine reaches 110 °C.
- Keeps the alert active until the engine temperature falls to 105 °C or lower.
- Updates at approximately 100 ms intervals.

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

## Project layout

- `src/YRS_ST7789.ino` — firmware, display layout, sensor conversion, filtering, trends, and alert state.
- `platformio.ini` — ESP32-C3 board, TFT_eSPI, display, and SPI settings.
- `lib/TFT_eSPI-2.5.43/` — tracked TFT_eSPI library copy.
- `Docs/` — pinout and thermistor calibration references.
- `test/` — PlatformIO test directory; no project-specific tests are currently included.

## Known limitations

### Open thermistors may look valid

The `SENSOR ERROR` state is not a reliable open-thermistor detector. If a thermistor or its wiring opens, the ESP32 ADC input may float and read ambient noise or another plausible intermediate voltage. That value can be converted into a plausible resistance and temperature.

`SENSOR ERROR` only reports readings rejected by the current ADC-range and calculation checks; it does not guarantee that a disconnected or open sensor will be detected. Verify the wiring and sensor integrity independently when a false reading could be unsafe.
