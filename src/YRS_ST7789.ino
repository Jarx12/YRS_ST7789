#include <Arduino.h>
#include <TFT_eSPI.h>


TFT_eSPI tft = TFT_eSPI();
TFT_eSprite img = TFT_eSprite(&tft); // Create the Sprite object

bool overheat_alert_active = false;
unsigned long last_display_update = 0;

const int TEMP_WARNING = 95;
const int TEMP_ALERT = 110;
const int TEMP_ALERT_CLEAR = 105;
const unsigned long DISPLAY_PERIOD_MS = 100;
// Minimum temperature change over the trend window to be treated as a real rise/fall.
// Shared by the TrendTracker instances and the trend text so they can never disagree:
// anything inside this band is sensor noise and is reported as steady.
const float TREND_NOISE_THRESHOLD = 0.60f; // degrees C

// ==================== DASHBOARD LAYOUT (320x172 landscape) ====================
// The bottom half of the screen, top to bottom: gauge bar, trend strip, status
// badge. These are named constants because the bands have to be tuned against
// each other - the numbers below leave an empty gap of 5-7px between any two of
// them, which is what stopped the title/delta collision in the first place.
const int GAUGE_X = 20, GAUGE_Y = 86, GAUGE_W = 280, GAUGE_H = 12;
const int TREND_X = 20, TREND_Y = 106, TREND_W = 280, TREND_H = 26;
const int BADGE_X = 20, BADGE_Y = 138, BADGE_W = 280, BADGE_H = 28;
const uint16_t TREND_BG = 0x0841;   // near black, just lifts the band off the screen
const uint16_t TREND_LINE = 0x4208; // same dim grey as the existing dividers

enum TempTrend { ESTABLE = 0, SUBIENDO = 1, BAJANDO = -1 };

struct SensorData {
    long temperature;
    long resistance;
    float temp_change_3s;
    TempTrend trend;
    bool valid;
  };

class TrendTracker {
private:
    static const int BUFFER_SIZE = 32; // Covers a 3s window at 100ms intervals
    float temp_history[BUFFER_SIZE];
    unsigned long time_history[BUFFER_SIZE];
    int head = 0;
    bool buffer_filled = false;
    unsigned long window_ms;
    float noise_threshold;

public:
    // Constructor: customize window time and sensitivity per instance
    TrendTracker(unsigned long window_ms = 3000, float noise_threshold = 0.25f)
        : window_ms(window_ms), noise_threshold(noise_threshold) {}

    void update(float current_temp, TempTrend &trend_out, float &delta_out) 
    {
        unsigned long now = millis();

        // 1. Store sample
        temp_history[head] = current_temp;
        time_history[head] = now;

        // 2. Find the oldest sample that is at least one window old
        int oldest_idx = head;
        int sample_count = buffer_filled ? BUFFER_SIZE : (head + 1);

        for (int i = 0; i < sample_count; i++) {
            int check_idx = (head - i + BUFFER_SIZE) % BUFFER_SIZE;
            if (now - time_history[check_idx] >= window_ms) {
                oldest_idx = check_idx;
                break;
            }
            oldest_idx = check_idx; 
        }

        // 3. Calculate Delta T
        float dt = (now - time_history[oldest_idx]) / 1000.0f;
        
        if (dt >= 1.0f) {
            float delta_T = current_temp - temp_history[oldest_idx];
            delta_out = delta_T;

            if (delta_T > noise_threshold) {
                trend_out = SUBIENDO;
            } else if (delta_T < -noise_threshold) {
                trend_out = BAJANDO;
            } else {
                trend_out = ESTABLE;
            }
        } else {
            trend_out = ESTABLE;
            delta_out = 0.0f;
        }

        // 4. Advance ring buffer index
        head = (head + 1) % BUFFER_SIZE;
        if (head == 0) buffer_filled = true;
    }
};


void setup() {
    tft.init();
    tft.setRotation(1); // Landscape 320x172
    tft.fillScreen(TFT_BLACK);
    analogSetAttenuation(ADC_11db); // Esto permite leer hasta ~3.1V - 3.3V
    pinMode(0, ANALOG);
    pinMode(2, ANALOG);
    analogWrite(TFT_BL, 70); // 70% Brightness
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
    unsigned long now = millis();
    if (now - last_display_update >= DISPLAY_PERIOD_MS) {
        last_display_update = now;
        updateDisplay();
    }
}

void alert_in_display(const SensorData &motor, bool sensorFault)
{
    uint16_t bg = sensorFault ? TFT_DARKGREY : TFT_RED;
    img.fillSprite(bg);
    img.setTextColor(TFT_WHITE, bg);

    // --- Title ---
    img.setTextDatum(MC_DATUM);
    img.setFreeFont(&FreeSansBold12pt7b);
    img.drawString(sensorFault ? "SENSOR ERROR" : "RECALENTADO", 160, 30);

    // A failed reading has no trustworthy temperature, trend or bar to show,
    // and the engine may still be genuinely overheating off the last good sample.
    if (sensorFault || !motor.valid) {
        img.setFreeFont(&FreeSansBold9pt7b);
        img.drawString("LECTURA INVALIDA", 160, 120);
        img.pushSprite(0, 0);
        return;
    }

    // --- Current engine temperature (same offset trick as the dashboard) ---
    img.setTextDatum(TC_DATUM);
    img.setFreeFont(&FreeSansBold18pt7b);
    img.drawNumber((int)motor.temperature, 145, 95);

    img.setFreeFont(&FreeSans9pt7b);
    img.setTextDatum(TL_DATUM);
    img.drawString("C", 169, 95);

    // --- Climbing, falling or steady? ---
    drawTrendIndicator(motor.trend, 192, 84, 22, TFT_WHITE);

    img.setTextDatum(MC_DATUM);
    img.setFreeFont(&FreeSansBold9pt7b);
    img.drawString(formatTrendDelta(motor), 160, 128);

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

String formatTrendDelta(const SensorData &data) {
    if (!data.valid) return "ERR";
    // The TrendTracker already rejects anything below TREND_NOISE_THRESHOLD, so
    // only print a number while the engine is genuinely moving. The old +/-0.05
    // dead-band let sensor noise through and made this flicker between "+0.1",
    // "+0.2" and "0.0" while nothing was actually happening.
    if (data.trend == ESTABLE) return "0.0";

    String result = String(data.temp_change_3s, 1);
    if (data.temp_change_3s > 0) {
        result = String("+") + result;
    }
    return result;
}

void updateDisplay() {
    SensorData ValoresMotor = leer_termistor_motor();
    SensorData ValoresAC = leer_termistor_ac();

    // The engine alert is latched at 110 C and clears at 105 C.
    if (ValoresMotor.valid) {
        if (!overheat_alert_active && ValoresMotor.temperature >= TEMP_ALERT) {
            overheat_alert_active = true;
        } else if (overheat_alert_active && ValoresMotor.temperature <= TEMP_ALERT_CLEAR) {
            overheat_alert_active = false;
        }
    }

    // A failed engine sensor takes priority over the normal dashboard.
    // An invalid reading never clears a previously latched overheat alert.
    if (!ValoresMotor.valid || overheat_alert_active) {
        alert_in_display(ValoresMotor, !ValoresMotor.valid);
        return;
    }

    img.fillSprite(TFT_BLACK);

    // ==================== 1. DIVIDERS & GRID ====================
    // Vertical line separating Motor & A/C columns. It stops at the gauge bar;
    // inside the trend strip below, the divider is redrawn in the strip colour.
    img.drawFastVLine(160, 0, 80, TFT_DARKGREY);
    // Horizontal divider separating sensor columns from the gauge bar
    img.drawFastHLine(10, 80, 300, 0x4208);

    // ==================== 2. LEFT COLUMN (MOTOR) ====================
    uint16_t leftCenterX = 80;

    // Header: channel name only. The trend arrow and its 3s delta now live in
    // the trend strip between the gauge and the badge, so the title is free to
    // be centred in its column and can never touch a neighbouring glyph.
    // With MC_DATUM a FreeSansBold9pt7b line occupies rows y-6 .. y+7.
    img.setTextDatum(MC_DATUM);
    img.setTextColor(TFT_WHITE, TFT_BLACK);
    img.setFreeFont(&FreeSansBold9pt7b);
    img.drawString("MOTOR", leftCenterX, 10);

    // Motor Temperature Number
    uint16_t motorColor = TFT_CYAN;
    if (ValoresMotor.temperature >= TEMP_WARNING && ValoresMotor.temperature < TEMP_ALERT)
        motorColor = TFT_YELLOW;
    if (ValoresMotor.temperature >= TEMP_ALERT)
        motorColor = TFT_ORANGE;

    img.setTextDatum(TC_DATUM);
    img.setTextColor(motorColor, TFT_BLACK);
    img.setFreeFont(&FreeSansBold18pt7b);
    img.drawNumber((int)ValoresMotor.temperature, leftCenterX - 10, 22);

    // Unit '°C'
    img.setFreeFont(&FreeSans9pt7b);
    img.setTextDatum(TL_DATUM);
    img.drawString("C", leftCenterX + 24, 22);

    // Motor Resistance
    img.setTextDatum(TC_DATUM);
    img.setTextColor(TFT_WHITE, TFT_BLACK);
    img.drawString(formatResistance(ValoresMotor.resistance), leftCenterX, 58);


    // ==================== 3. RIGHT COLUMN (A/C) ====================
    uint16_t rightCenterX = 240;

    // Header: "A/C" only - see the MOTOR header note above for the rationale.
    img.setTextDatum(MC_DATUM);
    img.setTextColor(TFT_WHITE, TFT_BLACK);
    img.setFreeFont(&FreeSansBold9pt7b);
    img.drawString("A/C", rightCenterX, 10);

    // A/C Temperature Number
    img.setTextDatum(TC_DATUM);
    if (ValoresAC.valid) {
        img.setTextColor(TFT_GREEN, TFT_BLACK);
        img.setFreeFont(&FreeSansBold18pt7b);
        img.drawNumber((int)ValoresAC.temperature, rightCenterX - 10, 22);

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
    img.setTextColor(ValoresAC.valid ? TFT_WHITE : TFT_RED, TFT_BLACK);
    img.setFreeFont(&FreeSans9pt7b);
    img.drawString(ValoresAC.valid ? formatResistance(ValoresAC.resistance) : "SENSOR ERROR", rightCenterX, 58);

    // ==================== 4. TEMPERATURE BAR ====================
    drawTemperatureBar((int)ValoresMotor.temperature, GAUGE_X, GAUGE_Y, GAUGE_W, GAUGE_H);

    // ==================== 5. TREND STRIP ====================
    // The arrow + 3s delta of both channels now live in their own band between
    // the gauge bar and the status badge. That band is the space the slimmer
    // badge hands over, and it is the only place the trend is now drawn.
    img.fillRect(TREND_X, TREND_Y, TREND_W, TREND_H, TREND_BG);
    img.drawRect(TREND_X, TREND_Y, TREND_W, TREND_H, TREND_LINE);
    // The column divider continues through the band, so each chip reads as
    // belonging to the column directly above it.
    img.drawFastVLine(160, TREND_Y, TREND_H, TREND_LINE);

    int trendCenterY = TREND_Y + TREND_H / 2;
    drawTrendChip(ValoresMotor, leftCenterX, trendCenterY, TFT_WHITE, TREND_BG);
    drawTrendChip(ValoresAC, rightCenterX, trendCenterY, ValoresAC.valid ? TFT_WHITE : TFT_RED, TREND_BG);

    // ==================== 6. BOTTOM STATUS BANNER ====================
    // Slimmer badge (28px instead of 40px). The 12px it gives up is what pays
    // for the trend strip above, which is why the band below stays a clean 6px.
    uint16_t statusBgColor = TFT_DARKGREEN;
    uint16_t statusTextColor = TFT_WHITE;
    String statusText = "ESTADO: NORMAL";

    if (ValoresMotor.temperature >= TEMP_WARNING) {
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

void drawTrendIndicator(TempTrend trend, int x, int y, int s, uint16_t color) 
{
    
    // Prevent drawing if size is too small
    if (s < 5) return;

    switch (trend) {
        case SUBIENDO: {
            // A triangle pointing up (top, bottom-left, bottom-right)
            // img.fillTriangle(x + s/2, y, x, y + s - 1, x + s - 1, y + s - 1, color);
            
            // Or an open arrow for a sleeker look
            img.drawLine(x + s/2, y, x, y + s - 1, color); // Left slanted line
            img.drawLine(x + s/2, y, x + s - 1, y + s - 1, color); // Right slanted line
            img.drawFastHLine(x, y + s - 1, s, color); // Bottom base
            break;
        }

        case BAJANDO: {
            // A triangle pointing down (bottom, top-left, top-right)
            // img.fillTriangle(x + s/2, y + s - 1, x, y, x + s - 1, y, color);

            // Or an open arrow
            img.drawLine(x + s/2, y + s - 1, x, y, color); // Left slanted line
            img.drawLine(x + s/2, y + s - 1, x + s - 1, y, color); // Right slanted line
            img.drawFastHLine(x, y, s, color); // Top base
            break;
        }

        case ESTABLE: {
            // Equal sign (two horizontal lines)
            int thickness = s / 5; // Simple way to scale thickness with size
            if (thickness < 1) thickness = 1;

            int upperY = y + (s / 3);
            int lowerY = y + (2 * s / 3);

            // Draw two distinct lines
            img.fillRect(x, upperY, s, thickness, color);
            img.fillRect(x, lowerY, s, thickness, color);
            break;
        }
    }
}

// One trend read-out for the trend strip: the hand drawn arrow plus the 3s
// delta, laid out as a single block optically centred on (centerX, centerY).
// The block is measured rather than hardcoded, so a longer delta ("-12.3") or a
// shorter one ("0.0") stays centred and never runs into the column divider.
void drawTrendChip(const SensorData &data, int centerX, int centerY, uint16_t color, uint16_t bg)
{
    const int arrowSize = 16;
    const int gap = 6;

    // Set the font before measuring: textWidth() measures the active free font.
    img.setFreeFont(&FreeSansBold9pt7b);
    img.setTextColor(color, bg);
    img.setTextDatum(MC_DATUM);

    if (!data.valid) {
        img.drawString("ERR", centerX, centerY);
        return;
    }

    String delta = formatTrendDelta(data);
    int blockW = arrowSize + gap + img.textWidth(delta);
    int x = centerX - blockW / 2;

    // With MC_DATUM a FreeSansBold9pt7b line occupies rows y-6 .. y+7, so
    // centring the arrow on centerY lines it up with the digits' optical middle.
    drawTrendIndicator(data.trend, x, centerY - arrowSize / 2, arrowSize, color);
    img.drawString(delta, x + arrowSize + gap, centerY);
}

SensorData leer_termistor_ac() 
{
    SensorData datosAC = {};
    datosAC.resistance = -1;
    datosAC.temperature = -1;
    datosAC.temp_change_3s = 0.0f;
    datosAC.trend = ESTABLE;
    datosAC.valid = false;
    static TrendTracker ACTrendTracker(3000, TREND_NOISE_THRESHOLD);  // 3s window
    // Valores Conocidos
    int Vin = 3329; 
    int R1 = 20000; 
    int R2 = 150;
    int R_pullup_board = 10000; // Resistor de 10k fisico en GPIO 2 
    const float R_eff = 1.0f / ((1.0f / (R1 + R2)) + (1.0f / (float)R_pullup_board));

    // Coeficientes de Steinhart-Hart                                                                 
    float Lnr = 0;
    float InvT = 0;
    float A = 2.725132873e-3, B = -0.3077755504e-4, C = 11.79553855e-7;
    
    const int offset_calibracion = 0;
    const float alpha = 0.40; // Alpha de 0.40 para reaccionar rápido
    static float Vout_filtrado = -1.0f; 

    // 1. Tomar una muestra instantánea de voltaje
    int Vout_raw = analogReadMilliVolts(2) - offset_calibracion; 
    // Validacion: Evitar cortocircuitos o cables sueltos que causen división por cero
    if (Vout_raw <= 0 || Vout_raw >= Vin) 
    {
        return datosAC; // Retorna -1 si la lectura física es fallida o fuera de rango, evitando cálculos erróneos
    }
    // 2. Inicializar el filtro en el primer ciclo del programa
    if (Vout_filtrado < 0) 
    {
        Vout_filtrado = (float)Vout_raw;
    }
    // 3. Aplicar el filtro digital al voltaje
    Vout_filtrado = (alpha * (float)Vout_raw) + ((1.0f - alpha) * Vout_filtrado);
    // 4. Calcular R3 utilizando el voltaje ya filtrado y libre de ruido eléctrico
    // R3 = (Vout * R_eff) / (Vin - Vout)
    float R3_final = (Vout_filtrado * R_eff) / ((float)Vin - Vout_filtrado);
    datosAC.resistance = R3_final;
    // 5. Aplicar la ecuación de Steinhart-Hart
    if (R3_final > 0) 
    {
        Lnr = logf(R3_final);
        InvT = A + (B * Lnr) + (C * Lnr * Lnr * Lnr);
        if (InvT != 0.0f) {
            datosAC.temperature = (1.0f / InvT) - 273.15f;
            datosAC.valid = true;
        }
    } 
    // --- Serial Output para Debug ---
    // Serial.print("VOUT_Filt: "); Serial.print(Vout_filtrado); Serial.print(" mV | ");
    // Serial.print("R3: "); Serial.print(datosAC.resistance); Serial.print(" ohms | ");
    // Serial.print("Temp: "); Serial.print(datosAC.temperature); Serial.println(" °C");
    if (datosAC.valid) {
        ACTrendTracker.update(datosAC.temperature, datosAC.trend, datosAC.temp_change_3s);
    }
    return datosAC;
}

SensorData leer_termistor_motor() 
{
    SensorData datosMotor = {};
    datosMotor.resistance=-1;
    datosMotor.temperature=-1;
    datosMotor.temp_change_3s = 0.0f;
    datosMotor.trend = ESTABLE;
    datosMotor.valid = false;
    static TrendTracker motorTrendTracker(3000, TREND_NOISE_THRESHOLD); // 3s window
    //Ecuacion Steinhart-Hart: 1/T = A + B * (ln(R)) + C(ln(R)^3 (Donde R es la Resistencia en Ohmios del Termisor(R2) y T la temperatura en Kelvin)   
    //Valores Conocidos
    const int Vin = 3329; const int R1 = 46850; long R2 = 0;
    //Coeficientes de Steinhart-Hart                                                      
    float Vout=0;
    float Lnr=0;
    float InvT=0;
    float A = 0.2276068653e-3, B = 3.085959593e-4, C = -1.752477535893581e-7;
    const int offset_calibracion = 0;
    const float alpha = 0.15; // Factor de suavizado (entre 0.01 y 1.0). Menor = más filtrado.
    static float Vout_filtrado = -1.0; //Static para que conserve su valor entre llamadas y permita el filtrado digital low-pass

    // 1. Leer la muestra instantánea actual
    int Vout_inst = analogReadMilliVolts(0) - offset_calibracion;
    // Validacion: Evitar divisiones por cero o valores absurdos antes de filtrar
    if (Vout_inst <= 0 || Vout_inst >= Vin) {
        return datosMotor; // Devuelve -1 si hay un fallo de lectura
    }
    // 2. Inicializar el filtro en el primer arranque para evitar transitorios desde cero
    if (Vout_filtrado < 0) {
        Vout_filtrado = (float)Vout_inst;
    }
    // 3. Aplicar la ecuación del filtro digital low-pass
    Vout_filtrado = (alpha * Vout_inst) + ((1.0f - alpha) * Vout_filtrado);
    // 4. Calcular la resistencia R2 basándonos en el voltaje ya filtrado
    R2 = (long)((Vout_filtrado * R1) / (Vin - Vout_filtrado));
    datosMotor.resistance = R2;
    // 5. Ecuación Steinhart-Hart
    if (R2 > 0) {
        float Lnr = logf((float)R2);
        float InvT = A + B * Lnr + C * Lnr * Lnr * Lnr;
        if (InvT != 0.0f) {
            // Conversión a Celsius y ajuste por calibración (+3)
            datosMotor.temperature = (1.0f / InvT) - 273.15f + 3.0f;
            datosMotor.valid = true;
        }
    } 
    if (datosMotor.valid) {
        motorTrendTracker.update(datosMotor.temperature, datosMotor.trend, datosMotor.temp_change_3s);
    }
    return datosMotor;
}