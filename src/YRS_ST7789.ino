#include <Arduino.h>
#include <TFT_eSPI.h>


TFT_eSPI tft = TFT_eSPI();
TFT_eSprite img = TFT_eSprite(&tft); // Create the Sprite object

struct SensorData {
    long temperature;
    long resistance;
  };


void setup() {
    //Serial.begin(115200);
    delay(50);
    tft.init();
    tft.setRotation(1); // Landscape 320x172
    tft.fillScreen(TFT_BLACK); 
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Kernel C3 Estable!", 10, 30, 2);
    delay(60);
    tft.drawString("LCD Inicializado!", 10, 45, 2);
    analogSetAttenuation(ADC_11db); // Esto permite leer hasta ~3.1V - 3.3V
    delay(60);
    tft.drawString("ADC Atenuado a 11dB!", 10, 60, 2);
    pinMode(0, ANALOG);
    pinMode(2, ANALOG);
    delay(60);
    tft.drawString("GPIO 0 y 2 configurados como ANALOG!", 10, 75, 2);
    delay(60);
    tft.drawString("Iniciando Monitor!", 10, 90, 2);
    delay(500);
    analogWrite(TFT_BL, 50); // 70% Brightness
    tft.fillScreen(TFT_BLACK); 
    img.createSprite(320, 172);
    
}
void loop() 
{
    updateDisplay();
    delay(200);
}

void updateDisplay() {
    SensorData ValoresMotor = leer_termistor_motor();
    SensorData ValoresAC = leer_termistor_ac(); 
    img.fillSprite(TFT_BLACK);

    // --- LEFT COLUMN (MOTOR) ---
    uint16_t leftCenterX = 80; // Center of the first half (0-160)
    img.setTextDatum(TC_DATUM); // Set all coordinates to start from the TOP-CENTER of the text
    img.setTextColor(TFT_WHITE, TFT_BLACK);
    img.setFreeFont(&FreeSans9pt7b);
    img.drawString("MOTOR", leftCenterX, 10, 1);
    img.setFreeFont(NULL);
    img.drawFloat(ValoresMotor.resistance,0,30,160);
    img.drawFloat(ValoresAC.resistance,0,290,160);
    // Dynamic Color for Motor
    uint16_t motorColor = TFT_CYAN;
    if (ValoresMotor.temperature >= 94 && ValoresMotor.temperature < 105)  motorColor = TFT_YELLOW;
    if (ValoresMotor.temperature >= 105) motorColor = TFT_ORANGE;
    img.setTextColor(motorColor, TFT_BLACK);
    // Draw Large Temp
    img.setFreeFont(&FreeSansBold24pt7b);
    img.drawNumber(ValoresMotor.temperature, leftCenterX - 10, 45); // Offset slightly left for the 'C'
    // Draw Unit 'C' relative to the number
    img.setFreeFont(&FreeSans12pt7b);
    img.setTextDatum(TL_DATUM); // Switch to Top-Left for the 'C' unit
    img.drawString("C", leftCenterX + 35, 40); 
    // --- RIGHT COLUMN (A/C) ---
    uint16_t rightCenterX = 240; // Center of the second half (160-320)
    img.setTextDatum(TC_DATUM);
    img.setTextColor(TFT_WHITE, TFT_BLACK);
    img.setFreeFont(&FreeSans9pt7b);
    img.drawString("A/C", rightCenterX, 10, 1);
    img.setFreeFont(NULL);
    img.setTextColor(TFT_GREEN, TFT_BLACK);
    img.setFreeFont(&FreeSansBold18pt7b);
    img.drawNumber(ValoresAC.temperature, rightCenterX - 10, 50);

    img.setFreeFont(&FreeSansBold9pt7b);
    img.setTextDatum(TL_DATUM);
    img.drawString("C", rightCenterX + 25, 48);

    // --- BOTTOM STATUS BAR (CENTERED) ---
    img.drawFastHLine(20, 120, 280, 0x4208); // Divider
    img.drawFastVLine(160, 0, 120, TFT_DARKGREY); // Vertical divider in the status bar
    img.setTextDatum(BC_DATUM); // Bottom-Center Datum
    img.setFreeFont(&FreeSansBold9pt7b);
    
    drawTemperatureBar(ValoresMotor.temperature, 20, 105, 280, 15);
    if (ValoresMotor.temperature < 95) {
        img.setTextColor(TFT_GREEN, TFT_BLACK);
        img.drawString("ESTATUS: NORMAL", 160, 160); // 160 is horizontal center
    } 
    else if (ValoresMotor.temperature >= 95 && ValoresMotor.temperature < 105) 
    {
        img.setTextColor(TFT_YELLOW, TFT_BLACK);
        img.drawString("ESTATUS: CALENTADO", 160, 160); // 160 is horizontal center
    }
    else {
        img.setTextColor(TFT_ORANGE, TFT_BLACK);
        img.drawString("ESTATUS: SOBRECALENTADO", 160, 160);
    }
    
    img.pushSprite(0, 0);
}

void drawTemperatureBar(int temp, int x, int y, int w, int h) {
    // 1. Constrain temperature to our gauge limits
    int safeTemp = constrain(temp, 0, 120);
    
    // 2. Calculate fill width
    int fillWidth = map(safeTemp, 0, 120, 0, w);
    
    // 3. Determine color based on temperature
    uint16_t barColor = TFT_GREEN;
    if (safeTemp >= 95 && safeTemp < 105)  barColor = TFT_YELLOW;
    if (safeTemp >= 105) barColor = TFT_ORANGE;
    
    // 4. Draw the Gauge Frame (The "Empty" Part)
    img.drawRect(x, y, w, h, TFT_DARKGREY); 
    
    // 5. Draw the Fill
    // We fill a slightly smaller area to leave a 1px border
    img.fillRect(x + 2, y + 2, fillWidth - 4, h - 4, barColor);
    
    // 6. Draw the "Empty" background (to clear previous higher readings)
    img.fillRect(x + fillWidth, y + 2, w - fillWidth - 2, h - 4, TFT_BLACK);

    // 7. Add Scale Markers (Optional - every 30 degrees)
    for (int i = 0; i <= 120; i += 30) {
        int markX = x + map(i, 0, 120, 0, w);
        img.drawFastVLine(markX, y + h, 5, TFT_LIGHTGREY);
    }
}

SensorData leer_termistor_ac() 
{
    SensorData datosAC;
    datosAC.resistance = -1;
    datosAC.temperature = -1;

    // Valores Conocidos
    int Vin = 3329; 
    int R1 = 20000; 
    int R2 = 150;
    int R_pullup_board = 10000; // Resistor de 10k fisico en GPIO 2 
    float R_eff = 1.0 / ((1.0 / (R1+R2)) + (1.0 / R_pullup_board));
    float R3_sum = 0;
    //float Vout_avg = 0;
    // Coeficientes de Steinhart-Hart                                                                 
    float Lnr = 0;
    float InvT = 0;
    float A = 2.725132873e-3, B = -0.3077755504e-4, C = 11.79553855e-7;

    const int num_muestras = 32;
    for (int x = 0; x < num_muestras; x++)
    { 
        int Vout_raw = analogReadMilliVolts(2); 
        //Vout_avg += Vout_raw;
        // Calculation for R3 based on the formula: Vout = Vin * (R3 / (R1 + R2 + R3))
        // R3 = (Vout * R_eff) / (Vin - Vout)
        R3_sum += (Vout_raw * (R_eff)) / (Vin - Vout_raw);
    }
    //Vout_avg /= num_muestras;
    float R3_final = R3_sum / num_muestras;
    datosAC.resistance = R3_final;

    if (R3_final > 0) 
    {
        Lnr = logf(R3_final);
        InvT = A + (B * Lnr) + (C * Lnr * Lnr * Lnr);
        datosAC.temperature = (1.0 / InvT) - 273.15;
    } 

    // --- Serial Output ---
    //Serial.print("VOUT(avg): "); Serial.print(Vout_avg); Serial.print(" mV | ");
    //Serial.print("R3: "); Serial.print(datosAC.resistance); Serial.print(" ohms | ");
    //Serial.print("Temp: "); Serial.print(datosAC.temperature); Serial.println(" °C");
    
    return datosAC;
}

SensorData leer_termistor_motor() {
    SensorData datosMotor;
    datosMotor.resistance=-1;
    datosMotor.temperature=-1;
    //Ecuacion Steinhart-Hart: 1/T = A + B * (ln(R)) + C(ln(R)^3 (Donde R es la Resistencia en Ohmios del Termisor(R2) y T la temperatura en Kelvin)   
    //Valores Conocidos
    const int Vin = 3329; const int R1 = 46850; long R2 = 0;
    //Coeficientes de Steinhart-Hart                                                      
    float Vout=0;
    float Lnr=0;
    float InvT=0;
    float A = 0.2276068653e-3, B = 3.085959593e-4, C = -1.752477535893581e-7;
    const int num_muestras = 32;
    const int offset_calibracion = 0; //Offset de calibracion para el ADC
    for (int x = 0; x < num_muestras; x++)
    { //Numero de muestras
        int Vout = analogReadMilliVolts(0) - offset_calibracion; // Lee el voltaje en milivoltios
        R2 += ((Vout * R1) / (Vin-Vout)); //R2 despejada del Divisor de Voltaje VOUT = VIN * (R2/(R1+R2))        
    }
    R2 /= num_muestras; //Valor promedio de las muestras
    datosMotor.resistance=R2;
    if (R2 > 0) {
        Lnr = logf(R2);
        InvT = A + B * Lnr + C * Lnr*Lnr*Lnr;
        datosMotor.temperature=(1 / InvT) - 273.15 + 3; //Inv T es el inverso de la temperatura en Kelvin la cual convertimos a Celsius usando C = T - 273,15, sumamos 3 por un factor de calibracion no contemplado
    } 
    return datosMotor;
}
