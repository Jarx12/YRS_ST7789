#include <Arduino.h>
#include <TFT_eSPI.h>


TFT_eSPI tft = TFT_eSPI();
TFT_eSprite img = TFT_eSprite(&tft); // Create the Sprite object

bool flash_alert_flag=false;
enum TempTrend { ESTABLE = 0, SUBIENDO = 1, BAJANDO = -1 };

struct SensorData {
    long temperature;
    long resistance;
    float temp_change_3s;
    TempTrend trend;
  };

class TrendTracker {
private:
    static const int BUFFER_SIZE = 25; // Enough for 3.5s at 100ms intervals
    float temp_history[BUFFER_SIZE];
    unsigned long time_history[BUFFER_SIZE];
    int head = 0;
    bool buffer_filled = false;
    unsigned long window_ms;
    float noise_threshold;

public:
    // Constructor: customize window time and sensitivity per instance
    TrendTracker(unsigned long window_ms = 3500, float noise_threshold = 0.25f) 
        : window_ms(window_ms), noise_threshold(noise_threshold) {}

    void update(float current_temp, TempTrend &trend_out, float &delta_out) 
    {
        unsigned long now = millis();

        // 1. Store sample
        temp_history[head] = current_temp;
        time_history[head] = now;

        // 2. Find sample closest to window_ms ago
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
    //Serial.begin(115200);
    delay(50);
    tft.init();
    tft.setRotation(1); // Landscape 320x172
    tft.fillScreen(TFT_BLACK); 
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Kernel C3 Estable!", 10, 30, 2);
    delay(30);
    tft.drawString("LCD Inicializado!", 10, 45, 2);
    analogSetAttenuation(ADC_11db); // Esto permite leer hasta ~3.1V - 3.3V
    delay(50);
    tft.drawString("ADC Atenuado a 11dB!", 10, 60, 2);
    pinMode(0, ANALOG);
    pinMode(2, ANALOG);
    delay(50);
    tft.drawString("GPIO 0 y 2 configurados como ANALOG!", 10, 75, 2);
    delay(50);
    tft.drawString("Iniciando Monitor!", 10, 90, 2);
    delay(300);
    analogWrite(TFT_BL, 45); // 70% Brightness
    tft.fillScreen(TFT_BLACK); 
    img.createSprite(320, 172);
    
}
void loop() 
{
    updateDisplay();
    delay(100);
    if (flash_alert_flag)
        alert_in_display();
}

void alert_in_display()
{
    flash_alert_flag=false;
    img.fillSprite(TFT_RED);
    img.setFreeFont(&FreeSans12pt7b);
    img.setTextColor(TFT_WHITE, TFT_BLACK);
    img.drawString("RECALENTANDO", 160, 90);
    img.pushSprite(0, 0);
    delay(250); 
}

// Helper function to format resistance cleanly (e.g., 46850 -> "46.8k ohm" or "850 ohm")
String formatResistance(long r) {
    if (r < 0) return "ERROR";
    if (r >= 10000) {
        return String(r / 1000.0f, 1) + "k ohm"; // e.g. 46.8k ohm
    }
    return String(r) + " ohm";                   // e.g. 850 ohm
}

void updateDisplay() {
    SensorData ValoresMotor = leer_termistor_motor();
    SensorData ValoresAC = leer_termistor_ac(); 
    const int temp_threshold_warn = 95;
    const int temp_threshold_overheat = 105;

    if (ValoresMotor.temperature > temp_threshold_overheat)
        flash_alert_flag = true;

    img.fillSprite(TFT_BLACK);

    // ==================== 1. DIVIDERS & GRID ====================
    // Vertical line separating Motor & A/C columns
    img.drawFastVLine(160, 0, 78, TFT_DARKGREY); 
    // Horizontal divider separating sensor columns from the gauge bar
    img.drawFastHLine(10, 80, 300, 0x4208); 


    // ==================== 2. LEFT COLUMN (MOTOR) ====================
    uint16_t leftCenterX = 80;
    
    // Header: "MOTOR" + Trend Arrow
    img.setTextDatum(TC_DATUM);
    img.setTextColor(TFT_WHITE, TFT_BLACK);
    img.setFreeFont(&FreeSansBold9pt7b);
    img.drawString("MOTOR", leftCenterX + 10, 4);
    drawTrendIndicator(ValoresMotor.trend, 18, 4, 14, TFT_WHITE);

    // Motor Temperature Number
    uint16_t motorColor = TFT_CYAN;
    if (ValoresMotor.temperature >= temp_threshold_warn && ValoresMotor.temperature < temp_threshold_overheat) 
        motorColor = TFT_YELLOW;
    if (ValoresMotor.temperature >= temp_threshold_overheat) 
        motorColor = TFT_ORANGE;

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

    // Header: "A/C" + Trend Arrow
    img.setTextDatum(TC_DATUM);
    img.setTextColor(TFT_WHITE, TFT_BLACK);
    img.setFreeFont(&FreeSansBold9pt7b);
    img.drawString("A/C", rightCenterX + 10, 4);
    drawTrendIndicator(ValoresAC.trend, 178, 4, 14, TFT_WHITE);

    // A/C Temperature Number
    img.setTextColor(TFT_GREEN, TFT_BLACK);
    img.setFreeFont(&FreeSansBold18pt7b);
    img.drawNumber((int)ValoresAC.temperature, rightCenterX - 10, 22);

    // Unit '°C'
    img.setFreeFont(&FreeSans9pt7b);
    img.setTextDatum(TL_DATUM);
    img.drawString("C", rightCenterX + 24, 22);

    // A/C Resistance
    img.setTextDatum(TC_DATUM);
    img.setTextColor(TFT_WHITE, TFT_BLACK);
    img.drawString(formatResistance(ValoresAC.resistance), rightCenterX, 58);


    // ==================== 4. TEMPERATURE BAR ====================
    // Positioned cleanly in the middle (Y = 86, Height = 12)
    drawTemperatureBar((int)ValoresMotor.temperature, 20, 86, 280, 12);


    // ==================== 5. BOTTOM STATUS BANNER ====================
    // Colored Status Badge Box (Y = 120 to 162)
    uint16_t statusBgColor = TFT_DARKGREEN;
    uint16_t statusTextColor = TFT_WHITE;
    String statusText = "ESTATUS: NORMAL";

    if (ValoresMotor.temperature >= temp_threshold_warn && ValoresMotor.temperature < temp_threshold_overheat) {
        statusBgColor = 0x8400; // Dark Yellow / Amber
        statusTextColor = TFT_BLACK;
        statusText = "ESTATUS: CALENTADO";
    } 
    else if (ValoresMotor.temperature >= temp_threshold_overheat) {
        statusBgColor = TFT_RED;
        statusTextColor = TFT_WHITE;
        statusText = "ESTATUS: SOBRECALENTADO";
    }

    // Draw solid status badge
    img.fillRoundRect(20, 120, 280, 40, 6, statusBgColor);
    img.drawRoundRect(20, 120, 280, 40, 6, TFT_WHITE); // White border accent

    // Render Status Text inside badge
    img.setTextDatum(MC_DATUM); // Middle-Center Alignment
    img.setTextColor(statusTextColor, statusBgColor);
    img.setFreeFont(&FreeSansBold9pt7b);
    img.drawString(statusText, 160, 140);

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
    if (safeTemp >= 95 && safeTemp < 105) barColor = TFT_YELLOW;
    if (safeTemp >= 105)                  barColor = TFT_ORANGE;

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

    // 8. Add Scale Markers (Ticks drawn underneath, constrained inside w bounds)
    for (int i = 0; i <= 120; i += 30) {
        int markX = x + map(i, 0, 120, 0, w - 1); // w - 1 keeps tick 120 within bounds
        img.drawFastVLine(markX, y + h, 4, TFT_LIGHTGREY);
    }
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

SensorData leer_termistor_ac() 
{
    SensorData datosAC;
    datosAC.resistance = -1;
    datosAC.temperature = -1;
    static TrendTracker ACTrendTracker(3500, 0.60f);  // 3.5s window, 0.60°C threshold for trend detection
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
        datosAC.temperature = (1.0f / InvT) - 273.15f;
    } 
    // --- Serial Output para Debug ---
    // Serial.print("VOUT_Filt: "); Serial.print(Vout_filtrado); Serial.print(" mV | ");
    // Serial.print("R3: "); Serial.print(datosAC.resistance); Serial.print(" ohms | ");
    // Serial.print("Temp: "); Serial.print(datosAC.temperature); Serial.println(" °C");
    ACTrendTracker.update(datosAC.temperature, datosAC.trend, datosAC.temp_change_3s);
    return datosAC;
}

SensorData leer_termistor_motor() 
{
    SensorData datosMotor;
    datosMotor.resistance=-1;
    datosMotor.temperature=-1;
    static TrendTracker motorTrendTracker(3500, 0.60f); ///3.5s window 0.60°C threshold for trend detection
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

    //Buffer de Lecturas para calcular la tendencia de temperatura en los últimos 3.5 segundos
    const unsigned long WINDOW_MS = 3500; // Target window: 3.5 seconds
    const int BUFFER_SIZE = 20;            // Keeps up to 20 historical points
    static float temp_history[BUFFER_SIZE];
    static unsigned long time_history[BUFFER_SIZE];
    static int head = 0;
    static bool buffer_filled = false;


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
        
        // Conversión a Celsius y ajuste por calibración (+3)
        datosMotor.temperature = (1.0f / InvT) - 273.15f + 3.0f;
    } 
    motorTrendTracker.update(datosMotor.temperature, datosMotor.trend, datosMotor.temp_change_3s);
    return datosMotor;
}
