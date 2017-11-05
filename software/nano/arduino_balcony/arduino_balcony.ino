/* ==== Includes ==== */
#include <BME280I2C.h>
#include <M2M_LM75A.h>
#include <INA219.h>
#include <Wire.h>             // Needed for legacy versions of Arduino.
#include <FreqCount.h>
/* ====  END Includes ==== */

/* ==== Defines ==== */
#define SERIAL_BAUD 9600
/* ==== END Defines ==== */

/* ==== Global Variables ==== */
BME280I2C bme;                   // Default : forced mode, standby time = 1000 ms
// Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

M2M_LM75A lm75a;

#define SENSOR_ADDR  (0x90 >> 1)

bool metric = true;
/* ==== END Global Variables ==== */

/*
   INA219
*/

// Current sensor and shunt used
INA219 ina219_input(0x45);
INA219 ina219_system(0x40);

#define R_SHUNT 0.002
#define V_SHUNT_MAX 0.04
#define V_BUS_MAX 32
#define I_MAX_EXPECTED 20
#define EXTERNAL_IO 8
#define MUX_A 11
#define MUX_B 12
#define MUX_C 10
#define MUX_D 9

// current and voltage readings
float shuntvoltage_input = 0;
float busvoltage_input= 0;
float current_A_input = 0;
float batvoltage_input = 0;
float power_input = 0;

/* ==== Prototypes ==== */
/* === Print a message to stream with the temp, humidity and pressure. === */
void printBME280Data(Stream * client);
/* === Print a message to stream with the altitude, and dew point. === */
void printBME280CalculatedData(Stream* client);
/* ==== END Prototypes ==== */

/* ==== Setup ==== */
void setup() {
  //save some power
  ADCSRA = ADCSRA & B01111111;
  ACSR = B10000000;
  //setup
  ina219_input.begin();
  ina219_input.configure(INA219::RANGE_32V, INA219::GAIN_1_40MV, INA219::ADC_64SAMP, INA219::ADC_64SAMP, INA219::CONT_SH_BUS);
  ina219_input.calibrate(R_SHUNT, V_SHUNT_MAX, V_BUS_MAX, I_MAX_EXPECTED);
  ina219_system.begin();
  ina219_system.configure(INA219::RANGE_16V, INA219::GAIN_1_40MV, INA219::ADC_64SAMP, INA219::ADC_64SAMP, INA219::CONT_SH_BUS);
  ina219_system.calibrate(R_SHUNT, V_SHUNT_MAX, V_BUS_MAX, I_MAX_EXPECTED);
  lm75a.begin();
  lm75a.shutdown();
  //external power control
  pinMode(EXTERNAL_IO, OUTPUT);
  digitalWrite(EXTERNAL_IO, 1);
  //setup Mux I/O
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  pinMode(MUX_C, OUTPUT);
  pinMode(MUX_D, OUTPUT);
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {} // Wait
  while (!bme.begin()) {
    delay(1000);
  }
}
/* ==== END Setup ==== */

void getSensorData() {
  float temp(NAN), hum(NAN), pres(NAN);
  uint8_t pressureUnit(3);
  float altitude = bme.alt(metric);
  float dewPoint = bme.dew(metric);
  bme.read(pres, temp, hum, metric, pressureUnit);
  Serial.println(String(temp, 2) + ";" + String(hum, 2) + ";" + String(pres, 2) + ";" + String(altitude, 1) + ";" + String(dewPoint, 2));
}

void getSystemPower() {
  Serial.print(ina219_system.shuntCurrent() * 1000, 4);
  Serial.print(";");  
  Serial.print(ina219_system.busVoltage(), 4);
  Serial.print(";");
  Serial.print(ina219_system.busPower() * 1000, 4);
  Serial.println();
}

void getSolarPower() {
  Serial.print(ina219_input.shuntCurrent() * 1000, 4);
  Serial.print(";");  
  Serial.print(ina219_input.busVoltage(), 4);
  Serial.print(";");
  Serial.print(ina219_input.busPower() * 1000, 4);
  Serial.println();
}

void getSystemTemp() {
  lm75a.wakeup();
  Serial.println(lm75a.getTemperature());
  lm75a.shutdown();
}

void debugIIC() {
  Serial.println("Scanning...");

  byte error, address;
  int nDevices;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void debugExternal(boolean state){
  Serial.println("Debug external\n");
  Serial.println(state);
  if(state){
    digitalWrite(EXTERNAL_IO, 1);
  }else{
    digitalWrite(EXTERNAL_IO, 0);
  }
}

void debugFrequency(){
  Serial.println("Debug frequency");
  digitalWrite(MUX_A, 1);
  digitalWrite(MUX_B, 1);
  digitalWrite(MUX_C, 1);
  digitalWrite(MUX_D, 0);
  delay(100);
  FreqCount.begin(1000);
  while(!FreqCount.available()) {
    Serial.println("Waiting for freq read!");
  }
  unsigned long count = FreqCount.read();
  Serial.println(count);
  FreqCount.end();
}

/* ==== Loop ==== */
void loop() {
  if (Serial.available() > 0) {
    String mockString = Serial.readString();
    if (mockString.equals("getHydroData\r\n")) {
      Serial.println("111;222;333;444;555;666;777;888;999;000;123;234;345;456;567;678");
    }
    if (mockString.equals("getSensorData\r\n")) {
      getSensorData();
    }
    if (mockString.equals("debugIIC\r\n")) {
      debugIIC();
    }
    if (mockString.equals("getSystemTemp\r\n")) {
      getSystemTemp();
    }
    if (mockString.equals("getSolarPower\r\n")) {
      getSolarPower();
    }
    if (mockString.equals("getSystemPower\r\n")) {
      getSystemPower();
    }
    if (mockString.equals("debugExternalOn\r\n")) {
      debugExternal(false);
    }
    if (mockString.equals("debugExternalOff\r\n")) {
      debugExternal(true);
    }
    if (mockString.equals("debugFrequencyEight\r\n")) {
      debugFrequency();
    }
  }
}
/* ==== End Loop ==== */
