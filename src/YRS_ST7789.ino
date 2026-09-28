#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite img = TFT_eSprite(&tft); // Create the Sprite object

bool overheat_alert_active = false;
// The dashboard currently on screen.
enum DashboardView { VIEW_DASHBOARD = 0, VIEW_TEMP_GRAPH = 1, VIEW_COUNT = 2 };
// Default view
DashboardView current_view = VIEW_DASHBOARD;

unsigned long last_display_update = 0;

// Setup variables for the temperature thresholds and display parameters
const int TEMP_WARNING = 95;
const int TEMP_ALERT = 110;
const int TEMP_ALERT_CLEAR = 105;
// Measured 3.3V rail in mV
const int SUPPLY_MV = 3329;
const unsigned long DISPLAY_PERIOD_MS = 100;

// ==================== VIEW BUTTON (GPIO6, momentary) ====================
const uint8_t BUTTON_PIN = 6;
const unsigned long BUTTON_DEBOUNCE_MS = 30;

// ==================== DASHBOARD LAYOUT (320x172 landscape) ====================
// The bottom half of the screen, top to bottom: gauge bar, then the status
// badge. These are named constants because the bands have to be tuned against
// each other - the numbers below leave clear air between them and around the
// badge, so neither element can collide with what sits above it.
const int GAUGE_X = 20, GAUGE_Y = 86, GAUGE_W = 280, GAUGE_H = 12;
const int BADGE_X = 20, BADGE_Y = 116, BADGE_W = 280, BADGE_H = 40;

// ==================== GRAPH DASHBOARD LAYOUT (320x172 landscape) ====================
// Title band, plot rectangle with its axis gutters, then the read-out band: the
// same top/middle/bottom rhythm as the dashboard layout above. PLOT_X leaves a
// 36px left gutter, wide enough for a "-120" scale label.
const int PLOT_X = 40, PLOT_Y = 22, PLOT_W = 272, PLOT_H = 96;
const int PLOT_DIVISIONS = 4;    // 4 cells each way => 5 gridlines, 5 labels
const int PLOT_X_LABEL_Y = 133;  // "seconds ago" row under the plot
const int PLOT_FOOT_Y = 145;     // divider above the read-out band
const int PLOT_FOOT_TEXT_Y = 157;
const uint16_t PLOT_BG = 0x0841;    // near black, lifts the plot off the screen
const uint16_t PLOT_GRID = 0x4208;  // same dim grey as the existing dividers
const uint16_t MOTOR_TRACE = TFT_CYAN; // trace colour, matched to the read-out text
const uint16_t AC_TRACE = TFT_GREEN;

// One logged point every GRAPH_SAMPLE_MS, so the buffer covers a full minute.
// The history is filled on every display tick, graph or not, so the plot is
// already populated the instant the button is pressed.
const unsigned long GRAPH_SAMPLE_MS = 200;
const int GRAPH_SAMPLES = 300;            // 300 * 200ms = 60s of history
const float GRAPH_MIN_SPAN_C = 20.0f;     // floor on the auto scale: a flat trace must not rescale itself
const int GRAPH_SCALE_STEP_C = 5;         // scale values snap outwards to this step

// Vertical scale of the temp vs time dashboard. The scale is derived from the
// samples actually inside the window rather than fixed, so a 6C climb fills the
// plot instead of hiding on a 0..120 gauge; GRAPH_MIN_SPAN_C and the outward
// snap to GRAPH_SCALE_STEP_C keep a steady engine from redrawing its own axis
// on every frame. Declared up here, with the other types, because the Arduino
// pre-processor emits the function prototypes above the definitions.
struct GraphScale { int lo; int hi; bool has_data; };

struct SensorData {
    long temperature;
    long resistance;
    bool valid;
  };

// ==================== THERMISTOR CHANNELS ====================
// The engine and the A/C probe do not differ by a pin and a couple of numbers:
// they sit behind different resistor networks, so the upper leg of each divider
// is a different circuit. The A/C node has the board's 10k fitted across it and
// 150R in series with its 20k, so its upper leg is a parallel network that
// equals 6.683k - treating that as "20k" silently halves the resistance the
// Steinhart-Hart curve is handed. UpperLeg keeps that structure visible instead
// of collapsing it into one pre-computed number someone can edit into the wrong
// value; everything else in the chain is genuinely shared.
struct UpperLeg {
    float pullup_ohms;   // fitted pull-up from the rail down to the sense node
    float series_ohms;   // extra resistance in series with that pull-up
    float parallel_ohms; // another resistor from the rail to the same node, 0 if none

    // Equivalent resistance of the whole upper leg, which is what the divider
    // equation needs. With no second branch this is just pullup + series; with
    // one it is the parallel sum, written in the same reciprocal form the A/C
    // code used so the resolved value is bit for bit what it was before.
    float effectiveOhms() const
    {
        float branch = pullup_ohms + series_ohms;
        if (parallel_ohms <= 0.0f) return branch;
        return 1.0f / ((1.0f / branch) + (1.0f / parallel_ohms));
    }
};

// Read -> range check -> low-pass -> divide by this channel's network ->
// Steinhart-Hart. Every step is identical for every channel; the only
// per-channel inputs are the ADC pin, the upper leg, the curve, the filter
// strength and the calibration offset.
class ThermistorChannel {
private:
    uint8_t adc_pin;
    UpperLeg upper_leg;
    float sh_a, sh_b, sh_c; // Steinhart-Hart coefficients for this probe
    float alpha;            // low-pass strength, 0.01..1.0
    float temp_offset_c;    // calibration trim in degrees C
    int offset_mv;          // trim on the sense node itself
    float filtered_mv = -1.0f; // negative until the first sample primes it

public:
    ThermistorChannel(uint8_t pin, UpperLeg leg, float a, float b, float c,
                      float filter_alpha, float temp_offset = 0.0f, int mv_offset = 0)
        : adc_pin(pin), upper_leg(leg), sh_a(a), sh_b(b), sh_c(c), alpha(filter_alpha),
          temp_offset_c(temp_offset), offset_mv(mv_offset) {}

    // One sample. Any failed reading comes back with valid == false and -1s, and
    // the filter state is left untouched so a single glitch cannot poison the
    // samples that follow it.
    SensorData read()
    {
        SensorData data = {};
        data.resistance = -1;
        data.temperature = -1;
        data.valid = false;

        // 1. Sample the sense node.
        int vout_raw = analogReadMilliVolts(adc_pin) - offset_mv;

        // 2. Range check. A short, a loose wire or an open thermistor all land
        //    outside the window a working divider can reach, and dividing by
        //    them would turn a wiring fault into a plausible looking reading.
        if (vout_raw <= 0 || vout_raw >= SUPPLY_MV) return data;

        // 3. Prime the low-pass on the first sample instead of filtering up from
        //    zero, which would fake a cold engine for the first few seconds.
        if (filtered_mv < 0.0f) filtered_mv = (float)vout_raw;
        filtered_mv = (alpha * (float)vout_raw) + ((1.0f - alpha) * filtered_mv);

        // 4. Divider -> this thermistor's own resistance, at full precision.
        // The old engine code held this in a long and handed the curve that
        // truncated value, while the A/C code handed it the float, so the two
        // probes quietly disagreed. Both now get the undivided value; the
        // read-out still reports whole ohms because that is what the display
        // shows.
        float r_eff = upper_leg.effectiveOhms();
        float ohms = (filtered_mv * r_eff) / (SUPPLY_MV - filtered_mv);
        data.resistance = (long)ohms;
        if (ohms <= 0.0f) return data;

        // 5. Steinhart-Hart: 1/T = A + B*ln(R) + C*ln(R)^3
        float ln_r = logf(ohms);
        float inv_t = sh_a + (sh_b * ln_r) + (sh_c * ln_r * ln_r * ln_r);
        if (inv_t == 0.0f) return data;
        data.temperature = (long)((1.0f / inv_t) - 273.15f + temp_offset_c);
        data.valid = true;

        // --- Serial Output para Debug ---
        // Serial.print("VOUT_Filt: "); Serial.print(filtered_mv); Serial.print(" mV | ");
        // Serial.print("R_therm: "); Serial.print(ohms); Serial.print(" ohms | ");
        // Serial.print("Temp: "); Serial.print(data.temperature); Serial.println(" C");
        return data;
    }
};

// Engine probe: one 46.85k pull-up to the rail, the thermistor to ground.
ThermistorChannel motor_channel(
    0,                                    // ADC1 channel on GPIO0
    UpperLeg{46850.0f, 0.0f, 0.0f},        // pull-up, no series, no second branch
    0.2276068653e-3f, 3.085959593e-4f, -1.752477535893581e-7f,
    0.15f,                                // heavy smoothing
    3.0f);                                // +3C calibration trim on this probe

// A/C probe: the board's 10k sits across the same node, so the upper leg is
// (20k + 150) || 10k = 6683.3, not 20k. Faster filter, different curve, no trim.
ThermistorChannel ac_channel(
    2,                                    // ADC1 channel on GPIO2
    UpperLeg{20000.0f, 150.0f, 10000.0f}, // pull-up, series, board parallel branch
    2.725132873e-3f, -0.3077755504e-4f, 11.79553855e-7f,
    0.40f,                                // reacts faster than the engine probe
    0.0f);                                // no calibration trim on this probe

// Momentary contact on a plain GPIO, read by polling. The debounce is time
// based rather than a sample count because loop() is a free running millis()
// walk: a raw level only counts once it has held still for BUTTON_DEBOUNCE_MS.
// A press is latched until the contact is released, so holding the button down
// steps the dashboards once and not once per loop.
class ViewButton {
private:
    uint8_t pin;
    bool raw_pressed = false;
    bool press_latched = false;
    unsigned long last_change_ms = 0;

public:
    void attach(uint8_t button_pin)
    {
        pin = button_pin;
        pinMode(pin, INPUT_PULLUP);
        // Seed from the pin instead of assuming a level, otherwise a button
        // held down while the board boots would fire a phantom step.
        raw_pressed = (digitalRead(pin) == LOW);
        press_latched = raw_pressed;
        last_change_ms = millis();
    }

    // Returns true exactly once per press. Call it on every pass of loop(),
    // not only on the display tick, otherwise a tap is not seen for up to
    // DISPLAY_PERIOD_MS.
    bool consumePress()
    {
        bool reading = (digitalRead(pin) == LOW);
        unsigned long now = millis();

        if (reading != raw_pressed) {
            raw_pressed = reading;
            last_change_ms = now;
        }

        // Re-arm on release
        if (!raw_pressed) {
            press_latched = false;
            return false;
        }

        if (!press_latched && (now - last_change_ms) >= BUTTON_DEBOUNCE_MS) {
            press_latched = true;
            return true;
        }
        return false;
    }
};

// Rolling temperature log. each sample carries is timestamped
class TempHistory {
private:
    static const int CAPACITY = GRAPH_SAMPLES;
    float motor_value[CAPACITY];
    float ac_value[CAPACITY];
    unsigned long stamp_ms[CAPACITY];
    uint8_t valid_bits[CAPACITY]; // bit 0 = motor readable, bit 1 = A/C readable
    int count = 0;                // samples held, saturates at CAPACITY
    int head = 0;                 // next write slot, which is also the oldest sample once full
    unsigned long last_sample_ms = 0;
    bool started = false;

    int slot(int age) const { return (head - 1 - age + CAPACITY) % CAPACITY; }

public:
    void tick(const SensorData &motor, const SensorData &ac)
    {
        unsigned long now = millis();
        if (!started) {
            started = true; // first call lands a sample straight away
        } else if ((now - last_sample_ms) < GRAPH_SAMPLE_MS) {
            return;
        }
        last_sample_ms = now;

        motor_value[head] = motor.temperature;
        ac_value[head] = ac.temperature;
        stamp_ms[head] = now;
        valid_bits[head] = (motor.valid ? 0x01 : 0x00) | (ac.valid ? 0x02 : 0x00);

        head = (head + 1) % CAPACITY;
        if (count < CAPACITY) count++;
    }

    int samples() const { return count; }

    // age = 0 is the newest sample, age = 1 the one before it, and so on.
    float ageSeconds(int age) const { return (millis() - stamp_ms[slot(age)]) / 1000.0f; }
    float valueAt(int age, bool ac_channel) const { return ac_channel ? ac_value[slot(age)] : motor_value[slot(age)]; }
    bool validAt(int age, bool ac_channel) const { return (valid_bits[slot(age)] & (ac_channel ? 0x02 : 0x01)) != 0; }
};

ViewButton view_button;
TempHistory temp_history;


void setup() {
    tft.init();
    tft.setRotation(1); // Landscape 320x172
    tft.fillScreen(TFT_BLACK);
    analogSetAttenuation(ADC_11db); // Esto permite leer hasta ~3.1V - 3.3V
    pinMode(0, ANALOG);
    pinMode(2, ANALOG);
    view_button.attach(BUTTON_PIN); // pull-up interno: pressed == pulled to GND
    analogWrite(TFT_BL, 716); // 70% Brightness
    tft.fillScreen(TFT_BLACK);
    if (img.createSprite(320, 172) == nullptr) {
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(TFT_WHITE);
        tft.drawString("LCD INIT ERROR", 160, 86, 2);
        while (true) {
            delay(1000);
        }
    }
}

void loop()
{
    //Polling view button: a press is latched until the contact is released
    if (view_button.consumePress()) {
        current_view = (DashboardView)((current_view + 1) % VIEW_COUNT);
    }
    // Display update is throttled to DISPLAY_PERIOD_MS, even so the button is polled on every loop()
    unsigned long now = millis();
    if (now - last_display_update >= DISPLAY_PERIOD_MS) {
        last_display_update = now;
        updateDisplay();
    }
}

// ==================== DISPLAY HUB ====================
// Read the sensors, update the shared states (history), then hands off to one view function
// =====================================================
void updateDisplay() {
    SensorData motor = motor_channel.read();
    SensorData ac    = ac_channel.read();

    // Log both channels on every tick, whichever dashboard is on screen. The
    // graph is one button press away at any moment, so its history has to be
    // warm already rather than starting empty when the view changes.
    temp_history.tick(motor, ac);

    // The engine alert is latched at 110 C and clears at 105 C.
    if (motor.valid) {
        if (!overheat_alert_active && motor.temperature >= TEMP_ALERT) {
            overheat_alert_active = true;
        } else if (overheat_alert_active && motor.temperature <= TEMP_ALERT_CLEAR) {
            overheat_alert_active = false;
        }
    }

    // Priority 1 - Critical alert
    if (!motor.valid || overheat_alert_active) {
        alert_in_display(motor, !motor.valid);
        return;
    }

    // Priority 2 - The selected view
    switch (current_view) {
        case VIEW_TEMP_GRAPH:
            graph_in_display(motor, ac);
            break;
        case VIEW_DASHBOARD:
        default:
            dashboard_in_display(motor, ac);
            break;
    }
}

// ==================== OVERRIDE VIEW: ALERT / SENSOR FAULT ====================
void alert_in_display(const SensorData &motor, bool sensorFault)
{
    uint16_t bg = sensorFault ? TFT_DARKGREY : TFT_RED;
    img.fillSprite(bg);
    img.setTextColor(TFT_WHITE, bg);

    // --- Title ---
    img.setTextDatum(MC_DATUM);
    img.setFreeFont(&FreeSansBold12pt7b);
    img.drawString(sensorFault ? "SENSOR ERROR" : "RECALENTADO", 160, 30);

    // A failed reading has no trustworthy temperature or bar to show,
    // and the engine may still be genuinely overheating off the last good sample.
    if (sensorFault || !motor.valid) {
        img.setFreeFont(&FreeSansBold9pt7b);
        img.drawString("LECTURA INVALIDA", 160, 120);
        img.pushSprite(0, 0);
        return;
    }

    // --- Current engine temperature, the number and its unit as one block ---
    // textWidth() measures the active free font, so each width is taken with its
    // own font already selected, and the pair is then placed by their total.
    // That keeps the read-out on the centre line with no magic offset to tune.
    img.setFreeFont(&FreeSansBold18pt7b);
    String value_text = String((int)motor.temperature);
    int value_w = img.textWidth(value_text);

    img.setFreeFont(&FreeSans9pt7b);
    const int UNIT_GAP = 4;
    int block_x = 160 - (value_w + UNIT_GAP + img.textWidth("C")) / 2;

    img.setTextDatum(TC_DATUM);
    img.setFreeFont(&FreeSansBold18pt7b);
    img.drawString(value_text, block_x + value_w, 95);

    img.setFreeFont(&FreeSans9pt7b);
    img.setTextDatum(TL_DATUM);
    img.drawString("C", block_x + value_w + UNIT_GAP, 95);

    // --- How close to the limit ---
    drawTemperatureBar((int)motor.temperature, 40, 148, 240, 10);

    img.pushSprite(0, 0);
}

// Helper function to format resistance cleanly (e.g., 46850 -> "46.8k ohm" or "850 ohm")
String formatResistance(long r) {
    if (r < 0) return "SENSOR ERROR";
    if (r >= 10000) {
        return String(r / 1000.0f, 1) + "k ohm"; // e.g. 46.8k ohm
    }
    return String(r) + " ohm";                   // e.g. 850 ohm
}

// ==================== VIEW: MAIN TEMPERATURE DASHBOARD ====================
void dashboard_in_display(const SensorData &motor, const SensorData &ac) {
    img.fillSprite(TFT_BLACK);

    // ==================== 1. DIVIDERS & GRID ====================
    // Vertical line separating Motor & A/C columns. It stops at the horizontal
    // divider, so it never runs through the gauge bar or the status badge.
    img.drawFastVLine(160, 0, 80, TFT_DARKGREY);
    // Horizontal divider separating sensor columns from the gauge bar
    img.drawFastHLine(10, 80, 300, 0x4208);

    // ==================== 2. LEFT COLUMN (MOTOR) ====================
    uint16_t leftCenterX = 80;

    // Header: channel name only, so it is free to be centred in its column and
    // can never touch a neighbouring glyph. With MC_DATUM a FreeSansBold9pt7b
    // line occupies rows y-6 .. y+7.
    img.setTextDatum(MC_DATUM);
    img.setTextColor(TFT_WHITE, TFT_BLACK);
    img.setFreeFont(&FreeSansBold9pt7b);
    img.drawString("MOTOR", leftCenterX, 10);

    // Motor Temperature Number
    uint16_t motorColor = TFT_CYAN;
    if (motor.temperature >= TEMP_WARNING && motor.temperature < TEMP_ALERT)
        motorColor = TFT_YELLOW;
    if (motor.temperature >= TEMP_ALERT)
        motorColor = TFT_ORANGE;

    img.setTextDatum(TC_DATUM);
    img.setTextColor(motorColor, TFT_BLACK);
    img.setFreeFont(&FreeSansBold18pt7b);
    img.drawNumber((int)motor.temperature, leftCenterX - 10, 22);

    // Unit '°C'
    img.setFreeFont(&FreeSans9pt7b);
    img.setTextDatum(TL_DATUM);
    img.drawString("C", leftCenterX + 24, 22);

    // Motor Resistance
    img.setTextDatum(TC_DATUM);
    img.setTextColor(TFT_WHITE, TFT_BLACK);
    img.drawString(formatResistance(motor.resistance), leftCenterX, 58);


    // ==================== 3. RIGHT COLUMN (A/C) ====================
    uint16_t rightCenterX = 240;

    // Header: "A/C" only - see the MOTOR header note above for the rationale.
    img.setTextDatum(MC_DATUM);
    img.setTextColor(TFT_WHITE, TFT_BLACK);
    img.setFreeFont(&FreeSansBold9pt7b);
    img.drawString("A/C", rightCenterX, 10);

    // A/C Temperature Number
    img.setTextDatum(TC_DATUM);
    if (ac.valid) {
        img.setTextColor(TFT_GREEN, TFT_BLACK);
        img.setFreeFont(&FreeSansBold18pt7b);
        img.drawNumber((int)ac.temperature, rightCenterX - 10, 22);

        // Unit '°C'
        img.setFreeFont(&FreeSans9pt7b);
        img.setTextDatum(TL_DATUM);
        img.drawString("C", rightCenterX + 24, 22);
    } else {
        img.setTextColor(TFT_RED, TFT_BLACK);
        img.setFreeFont(&FreeSansBold9pt7b);
        img.setTextDatum(TC_DATUM);
        img.drawString("ERROR", rightCenterX, 22);
    }

    // A/C Resistance
    img.setTextDatum(TC_DATUM);
    img.setTextColor(ac.valid ? TFT_WHITE : TFT_RED, TFT_BLACK);
    img.setFreeFont(&FreeSans9pt7b);
    img.drawString(ac.valid ? formatResistance(ac.resistance) : "SENSOR ERROR", rightCenterX, 58);

    // ==================== 4. TEMPERATURE BAR ====================
    drawTemperatureBar((int)motor.temperature, GAUGE_X, GAUGE_Y, GAUGE_W, GAUGE_H);

    // ==================== 5. BOTTOM STATUS BANNER ====================
    uint16_t statusBgColor = TFT_DARKGREEN;
    uint16_t statusTextColor = TFT_WHITE;
    String statusText = "ESTADO: NORMAL";

    if (motor.temperature >= TEMP_WARNING) {
        statusBgColor = 0x8400; // Dark Yellow / Amber
        statusTextColor = TFT_BLACK;
        statusText = "ESTADO: CALENTADO";
    }

    // Draw solid status badge
    img.fillRoundRect(BADGE_X, BADGE_Y, BADGE_W, BADGE_H, 6, statusBgColor);
    img.drawRoundRect(BADGE_X, BADGE_Y, BADGE_W, BADGE_H, 6, TFT_WHITE); // White border accent

    // Render Status Text inside badge
    img.setTextDatum(MC_DATUM); // Middle-Center Alignment
    img.setTextColor(statusTextColor, statusBgColor);
    img.setFreeFont(&FreeSansBold9pt7b);
    img.drawString(statusText, 160, BADGE_Y + BADGE_H / 2);

    // Push frame to ST7789 display
    img.pushSprite(0, 0);
}

void drawTemperatureBar(int temp, int x, int y, int w, int h) 
{
    // 1. Constrain temperature
    int safeTemp = constrain(temp, 0, 120);
    
    // 2. Define inner usable area (1px padding inside border)
    int innerX = x + 1;
    int innerY = y + 1;
    int innerW = w - 2;
    int innerH = h - 2;

    // 3. Calculate fill width based on INNER width
    int fillW = map(safeTemp, 0, 120, 0, innerW);

    // 4. Determine color
    uint16_t barColor = TFT_GREEN;
    if (safeTemp >= TEMP_WARNING && safeTemp < TEMP_ALERT) barColor = TFT_YELLOW;
    if (safeTemp >= TEMP_ALERT)                  barColor = TFT_ORANGE;

    // 5. Draw Outer Frame
    img.drawRect(x, y, w, h, TFT_WHITE); 

    // 6. Draw Filled Active Bar (only if fillW > 0 to prevent underflow)
    if (fillW > 0) {
        img.fillRect(innerX, innerY, fillW, innerH, barColor);
    }

    // 7. Draw Empty Background (Clears remaining space perfectly without touching frame)
    int emptyW = innerW - fillW;
    if (emptyW > 0) {
        img.fillRect(innerX + fillW, innerY, emptyW, innerH, TFT_BLACK);
    }

    // 8. Add scale marks, plus the two engine warning/alert markers.
    for (int i = 0; i <= 120; i += 30) {
        int markX = x + map(i, 0, 120, 0, w - 1); // w - 1 keeps tick 120 within bounds
        img.drawFastVLine(markX, y + h, 4, TFT_LIGHTGREY);
    }

    int warningX = x + map(TEMP_WARNING, 0, 120, 0, w - 1);
    int alertX = x + map(TEMP_ALERT, 0, 120, 0, w - 1);
    img.drawFastVLine(warningX, y - 2, h + 6, TFT_WHITE);
    img.drawFastVLine(alertX, y - 2, h + 6, TFT_WHITE);
}

// ==================== VIEW: TEMP vs TIME ====================
void graph_in_display(const SensorData &motor, const SensorData &ac)
{
    float window_s = graph_window_seconds();
    GraphScale scale = graph_auto_scale(window_s);

    img.fillSprite(TFT_BLACK);

    // --- Title ---
    img.setTextDatum(MC_DATUM);
    img.setTextColor(TFT_WHITE, TFT_BLACK);
    img.setFreeFont(&FreeSansBold9pt7b);
    img.drawString("TEMP vs TIEMPO", 160, 10);

    // --- Plot background, grid and frame ---
    img.fillRect(PLOT_X, PLOT_Y, PLOT_W, PLOT_H, PLOT_BG);
    for (int i = 1; i < PLOT_DIVISIONS; i++) {
        img.drawFastHLine(PLOT_X, PLOT_Y + (PLOT_H * i) / PLOT_DIVISIONS, PLOT_W, PLOT_GRID);
        img.drawFastVLine(PLOT_X + (PLOT_W * i) / PLOT_DIVISIONS, PLOT_Y, PLOT_H, PLOT_GRID);
    }
    img.drawRect(PLOT_X, PLOT_Y, PLOT_W, PLOT_H, TFT_DARKGREY);

    // --- Engine warning level, dashed, and only while it is inside the scale ---
    if (scale.has_data && TEMP_WARNING <= scale.hi && TEMP_WARNING >= scale.lo) {
        int warn_y = plot_y(TEMP_WARNING, scale);
        for (int x = PLOT_X + 1; x < PLOT_X + PLOT_W - 1; x += 6) {
            img.drawFastHLine(x, warn_y, 3, TFT_DARKGREY);
        }
    }

    // --- Traces ---
    draw_trace(false, MOTOR_TRACE, scale, window_s);
    draw_trace(true, AC_TRACE, scale, window_s);

    if (!scale.has_data) {
        img.setTextColor(TFT_RED, PLOT_BG);
        img.setFreeFont(&FreeSansBold9pt7b);
        img.setTextDatum(MC_DATUM);
        img.drawString("SIN DATOS", PLOT_X + PLOT_W / 2, PLOT_Y + PLOT_H / 2);
    }

    // --- Axes: temperature on the left, seconds ago along the bottom ---
    // Both rows are measured with textWidth() and re-centred by hand, so a
    // two digit scale, a "-120" scale or a "-60s" tick can never run into the
    // plot or off the right edge of the screen.
    img.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    img.setFreeFont(&FreeSans9pt7b);
    img.setTextDatum(MC_DATUM);

    for (int i = 0; i <= PLOT_DIVISIONS; i++) {
        int value = scale.hi - (scale.hi - scale.lo) * i / PLOT_DIVISIONS;
        String label = String(value);
        int label_x = PLOT_X - 4 - img.textWidth(label) / 2;
        img.drawString(label, label_x, PLOT_Y + (PLOT_H * i) / PLOT_DIVISIONS);
    }

    for (int i = 0; i <= PLOT_DIVISIONS; i++) {
        int seconds = (int)((window_s * (PLOT_DIVISIONS - i)) / PLOT_DIVISIONS + 0.5f);
        String label = (seconds > 0) ? (String("-") + String(seconds) + "s") : String("0s");
        int grid_x = PLOT_X + (PLOT_W * i) / PLOT_DIVISIONS;
        int label_w = img.textWidth(label);
        img.drawString(label, constrain(grid_x - label_w / 2, 1, tft.width() - label_w - 1), PLOT_X_LABEL_Y);
    }

    draw_graph_readout(motor, ac);

    img.pushSprite(0, 0);
}

float graph_window_seconds()
{
    float nominal = (GRAPH_SAMPLE_MS * GRAPH_SAMPLES) / 1000.0f;
    if (temp_history.samples() < 2) return nominal;
    float span = temp_history.ageSeconds(temp_history.samples() - 1);
    if (span < 2.0f) return nominal; // one or two samples: keep the full frame
    return (span < nominal) ? span : nominal;
}

GraphScale graph_auto_scale(float window_s)
{
    GraphScale scale = { 0, 120, false };
    float lo = 0.0f, hi = 0.0f;
    bool any = false;

    for (int age = 0; age < temp_history.samples(); age++) {
        if (temp_history.ageSeconds(age) > window_s) break; // history is newest first
        for (int channel = 0; channel < 2; channel++) {
            bool ac_channel = (channel == 1);
            if (!temp_history.validAt(age, ac_channel)) continue;
            float value = temp_history.valueAt(age, ac_channel);
            if (!any) { lo = hi = value; any = true; }
            else { if (value < lo) lo = value; if (value > hi) hi = value; }
        }
    }
    if (!any) return scale; // nothing readable yet: fall back to the gauge range

    float center = (lo + hi) / 2.0f;
    if (hi - lo < GRAPH_MIN_SPAN_C) {
        lo = center - GRAPH_MIN_SPAN_C / 2.0f;
        hi = center + GRAPH_MIN_SPAN_C / 2.0f;
    }
    lo = floorf(lo / GRAPH_SCALE_STEP_C) * GRAPH_SCALE_STEP_C;
    hi = ceilf(hi / GRAPH_SCALE_STEP_C) * GRAPH_SCALE_STEP_C;

    // A broken thermistor must not be able to drag the axis off the screen.
    if (lo < -40.0f) lo = -40.0f;
    if (hi > 200.0f) hi = 200.0f;
    if (hi - lo < GRAPH_SCALE_STEP_C) hi = lo + GRAPH_SCALE_STEP_C;

    scale.lo = (int)lo;
    scale.hi = (int)hi;
    scale.has_data = true;
    return scale;
}

// Mapped with MC_DATUM semantics on both axes, so both helpers clamp to the plot
// rectangle: a value outside the scale, or a sample a hair older than the
// window, can never draw outside the frame.
int plot_x(float age_s, float window_s)
{
    int x = PLOT_X + PLOT_W - 1 - (int)((age_s / window_s) * (PLOT_W - 1)); // 0s ago at the right edge
    return constrain(x, PLOT_X, PLOT_X + PLOT_W - 1);
}

int plot_y(float temp, const GraphScale &scale)
{
    int y = PLOT_Y + PLOT_H - 1 - (int)(((temp - scale.lo) / (float)(scale.hi - scale.lo)) * (PLOT_H - 1));
    return constrain(y, PLOT_Y, PLOT_Y + PLOT_H - 1);
}

void draw_trace(bool ac_channel, uint16_t color, const GraphScale &scale, float window_s)
{
    bool pen_down = false;
    int prev_x = 0, prev_y = 0;

    for (int age = 0; age < temp_history.samples(); age++) {
        float age_s = temp_history.ageSeconds(age);
        if (age_s > window_s) break;

        // A failed reading is a hole in the trace, not a zero: lift the pen so
        // the line cannot jump the gap and invent a temperature.
        if (!temp_history.validAt(age, ac_channel)) {
            pen_down = false;
            continue;
        }

        int x = plot_x(age_s, window_s);
        int y = plot_y(temp_history.valueAt(age, ac_channel), scale);
        if (pen_down) img.drawLine(prev_x, prev_y, x, y, color);
        prev_x = x;
        prev_y = y;
        pen_down = true;
    }
}

// Read-out under the plot: the two live numbers, colour matched to their trace
// so the legend and the plot can never disagree about which line is which.
void draw_graph_readout(const SensorData &motor, const SensorData &ac)
{
    img.drawFastHLine(10, PLOT_FOOT_Y, 300, PLOT_GRID);
    img.drawFastVLine(160, PLOT_FOOT_Y + 3, 22, PLOT_GRID);

    img.setTextDatum(MC_DATUM);
    img.setFreeFont(&FreeSansBold9pt7b);

    String motor_text = motor.valid ? (String("MOTOR ") + String((int)motor.temperature) + "C") : String("MOTOR ERR");
    img.setTextColor(MOTOR_TRACE, TFT_BLACK);
    img.drawString(motor_text, 80, PLOT_FOOT_TEXT_Y);

    String ac_text = ac.valid ? (String("A/C ") + String((int)ac.temperature) + "C") : String("A/C ERR");
    img.setTextColor(ac.valid ? AC_TRACE : TFT_RED, TFT_BLACK);
    img.drawString(ac_text, 240, PLOT_FOOT_TEXT_Y);
}