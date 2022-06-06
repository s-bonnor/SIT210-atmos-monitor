// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_DHT.h>

// This #include statement was automatically added by the Particle IDE.
#include "CCS811.h" // Sparkfun libary for CCS811 sensor board, modified for particle boards, then modified by me to work with the clone board im using

SYSTEM_THREAD(ENABLED);

LEDStatus LED_COMM_ERROR(RGB_COLOR_RED, LED_PATTERN_BLINK, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);
LEDStatus LED_COMM_READY(RGB_COLOR_GREEN, LED_PATTERN_BLINK, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);

#define CCS811_ADDR 0x5A // I2C Address of CCS811 Clone
CCS811 mySensor(CCS811_ADDR);

int RESET_PIN = D2;

#define DHTTYPE  DHT11
#define DHTPIN   5

int DHT11_GND = D3;
int DHT11_VCC = D4;
int DHT11_SIG = D5;
DHT dht(DHT11_SIG, DHTTYPE);


#define TIME_PER_READING_MS     5000
#define READINGS_FOR_AVERAGE  60
uint16_t tVOC;
uint16_t CO2;
float temperature;
float humidity;
float tempHisto[READINGS_FOR_AVERAGE];
uint16_t CO2Histo[READINGS_FOR_AVERAGE];
float CO2Avg;
float temperatureAvg;
int historyIndex = 0;
unsigned long last_update = 0;

bool tempInit = false;
bool co2Init = false;
bool alertSent = false;

// Alert Levels configured here
float CO2AlertLevel = 7000;
float TemperatureAlertLevel = 35;

void setup() {
    // Setup pins used as VCC & Ground for DHT11
    pinMode(DHT11_GND, OUTPUT);
    pinMode(DHT11_VCC, OUTPUT);
    pinResetFast(DHT11_GND);
    pinSetFast(DHT11_VCC);
    
    Serial.begin(9600);
    waitFor(Serial.isConnected, 30000);
    Serial.println("Atmospheric Monitor Sam Bonnor");
    
    // board must be reset after power on to initilize it
    pinMode(RESET_PIN, OUTPUT);
    digitalWrite(RESET_PIN, LOW);
    delay(200);
    digitalWrite(RESET_PIN, HIGH);
    delay(200);
    
    Wire.begin(); // Initialize I2C Hardware
    
    if (mySensor.begin() == false) {
        Serial.print("CCS811 error. Please check wiring. Freezing...");
        LED_COMM_ERROR.setActive(true);
        while (1);
    }

    // Initialize dht11    
	dht.begin();

    for (int tempIndex = 0; tempIndex < READINGS_FOR_AVERAGE; tempIndex += 1) {
        tempHisto[tempIndex] = 0;
        CO2Histo[tempIndex] = 0;
    }

    LED_COMM_READY.setActive(true);
    delay(1000);
    LED_COMM_READY.setActive(false);
    last_update = millis();
}

void loop() {
    update();
    logToSerial();
    logToWeb();
    checkAlert();
    delay(TIME_PER_READING_MS - (millis() - last_update));
    last_update = millis();
}

// Main update function
void update() {
    updateAirData();
    updateTemperature();
    updateHumidity();
    updateHistory();
}

// Publish data to thingspeak
void logToWeb() {
    Particle.publish("atmos_monitor", String::format("{ \
        \"temperature\":\"%f\", \
        \"humidity\":\"%f\", \
        \"eco2\":\"%f\", \
        \"tvoc\":\"%f\", \
        \"temperature_avg\":\"%f\", \
        \"eco2_avg\":\"%f\"}",
        temperature,
        humidity,
        float(CO2),
        float(tVOC),
        temperatureAvg,
        CO2Avg));
}

// Log data to serial port for debugging
void logToSerial() {
    Serial.print("AvgTemp[");
    Serial.print(temperatureAvg);
    Serial.print("] TempC[");
    Serial.print(temperature);
    Serial.print("] Humidity[");
    Serial.print(humidity);
    Serial.print("] CO2[");
    Serial.print(CO2);
    Serial.print("] CO2Avg[");
    Serial.print(CO2Avg);
    Serial.print("] tVOC[");
    Serial.print(tVOC);
    Serial.print("] millis[");
    Serial.print(millis());
    Serial.print("]");
    Serial.println();

}

// Check if an alert needs to be sent
void checkAlert() {
    if (alertSent) return;
    if (!Particle.connected()) return;
    bool sendAlert = false;
    if (CO2Avg > CO2AlertLevel) sendAlert = true;
    if (temperatureAvg > TemperatureAlertLevel) sendAlert = true;
    if (!sendAlert) return;
    Particle.publish("amp_alert", String::format("{\"temp_avg\":\"%f\",\"eco2_avg\":\"%f\"}", temperatureAvg, CO2Avg));
    alertSent = true;
}

// Read eCO2 & tVOC data
void updateAirData() {
    while (!mySensor.dataAvailable()) {
        delay(10);
    }
    mySensor.readAlgorithmResults();
    CO2 = mySensor.getCO2();
    tVOC = mySensor.getTVOC();
}

// Read temperature data
void updateTemperature() {
    float tmp = dht.getTempCelcius();
    while (isnan(tmp)) {
        delay(10);
        tmp = dht.getTempCelcius();
    }
    temperature = tmp;
}

// Read humidity data
void updateHumidity() {
    float tmp = dht.getHumidity();
    while (isnan(tmp)) {
        delay(10);
        tmp = dht.getTempCelcius();
    }
    humidity = tmp;
}

// Update Historical data and averages
void updateHistory() {
    tempHisto[historyIndex] = temperature;
    CO2Histo[historyIndex] = CO2;
    if (!tempInit) {
        for (int tempIndex = 0; tempIndex < READINGS_FOR_AVERAGE; tempIndex += 1) {
            tempHisto[tempIndex] = temperature;
        }
        tempInit = true;
    }
    if (!co2Init && (CO2 > 50)) {
        for (int tempIndex = 0; tempIndex < READINGS_FOR_AVERAGE; tempIndex += 1) {
            CO2Histo[tempIndex] = CO2;
        }
        co2Init = true;
    }
    
    historyIndex = (historyIndex + 1) % READINGS_FOR_AVERAGE;
    CO2Avg = 0;
    temperatureAvg = 0;
    for (int tempIndex = 0; tempIndex < READINGS_FOR_AVERAGE; tempIndex += 1) {
        temperatureAvg += tempHisto[tempIndex];
        CO2Avg += float(CO2Histo[tempIndex]);
    }
    temperatureAvg = temperatureAvg / READINGS_FOR_AVERAGE;
    CO2Avg = CO2Avg / READINGS_FOR_AVERAGE;
}
