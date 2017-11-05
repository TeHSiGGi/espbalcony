/* ==== Includes ==== */
#include <BME280I2C.h>
#include <M2M_LM75A.h>
#include <INA219.h>
#include <Wire.h>
#include <FreqCount.h>
/* ====  END Includes ==== */

/* ==== Defines ==== */
// BAUD RATE FOR COMMUNICATION
#define SERIAL_BAUD 9600
// INA219 SPECIFICATIONS, R_SHUNT IS 0.002 Ohms, Max expected voltage over R_SHUNT IS 40mV, Max BUS Voltage is 32V, Max Current 20A
#define R_SHUNT 0.002
#define V_SHUNT_MAX 0.04
#define V_BUS_MAX 32
#define I_MAX_EXPECTED 20
// GPIO Pin 8 is for providing +5V to external I/O via MOSFET. Output is inverted GPIO8: HIGH <> +5V External LOW
#define EXTERNAL_IO 8
// MUX Control Line mappings for input selection
#define MUX_A 11
#define MUX_B 12
#define MUX_C 10
#define MUX_D 9
// Sensor Address for LM75, results in 0x48
#define SENSOR_ADDR  (0x90 >> 1)
/* ==== END Defines ==== */

/* ==== Global Variables ==== */
// BME Sensor
BME280I2C bme;                 
// LM75 Sensor
M2M_LM75A lm75a;
// Input Power Measurement INA219
INA219 ina219_input(0x45);
// System Power Measurement INA219
INA219 ina219_system(0x40);
// Usage of metric values for BME Sensor
bool metric = true;
/* ==== END Global Variables ==== */

/* ==== Setup ==== */
void setup() {
  //save some power
  ADCSRA = ADCSRA & B01111111;
  ACSR = B10000000;
  //INA 219 for Input Measurement
  ina219_input.begin();
  ina219_input.configure(INA219::RANGE_32V, INA219::GAIN_1_40MV, INA219::ADC_64SAMP, INA219::ADC_64SAMP, INA219::CONT_SH_BUS);
  ina219_input.calibrate(R_SHUNT, V_SHUNT_MAX, V_BUS_MAX, I_MAX_EXPECTED);
  //INA 219 for System Measurement
  ina219_system.begin();
  ina219_system.configure(INA219::RANGE_16V, INA219::GAIN_1_40MV, INA219::ADC_64SAMP, INA219::ADC_64SAMP, INA219::CONT_SH_BUS);
  ina219_system.calibrate(R_SHUNT, V_SHUNT_MAX, V_BUS_MAX, I_MAX_EXPECTED);
  //LM75 for mainboard temperature
  lm75a.begin();
  //Shutdown LM75 until needed
  lm75a.shutdown();
  //external power control GPIO
  pinMode(EXTERNAL_IO, OUTPUT);
  //disable external power until needed
  digitalWrite(EXTERNAL_IO, 1);
  //setup Mux I/O for input choosing
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  pinMode(MUX_C, OUTPUT);
  pinMode(MUX_D, OUTPUT);
  //init UART for communication
  Serial.begin(SERIAL_BAUD);
  //wait until UART is available
  while (!Serial) {}
  //wait until BME is available
  while (!bme.begin()) {
    delay(1000);
  }
}
/* ==== END Setup ==== */

/* getSensorData - retrieve BME Sensor Data and Output via UART*/
void getSensorData() {
  float temp(NAN), hum(NAN), pres(NAN);
  uint8_t pressureUnit(3);
  float altitude = bme.alt(metric);
  float dewPoint = bme.dew(metric);
  bme.read(pres, temp, hum, metric, pressureUnit);
  Serial.println(String(temp, 2) + ";" + String(hum, 2) + ";" + String(pres, 2) + ";" + String(altitude, 1) + ";" + String(dewPoint, 2));
}

/* getSystemPower - retrieve INA219 data and output via UART */
void getSystemPower() {
  Serial.print(ina219_system.shuntCurrent() * 1000, 4);
  Serial.print(";");  
  Serial.print(ina219_system.busVoltage(), 4);
  Serial.print(";");
  Serial.print(ina219_system.busPower() * 1000, 4);
  Serial.println();
}

/* getSolarPower - retrieve INA219 data and output via UART */
void getSolarPower() {
  Serial.print(ina219_input.shuntCurrent() * 1000, 4);
  Serial.print(";");  
  Serial.print(ina219_input.busVoltage(), 4);
  Serial.print(";");
  Serial.print(ina219_input.busPower() * 1000, 4);
  Serial.println();
}

/* getSystemTemp - retrieve LM75 temperaturevalue and output via UART */
void getSystemTemp() {
  lm75a.wakeup();
  Serial.println(lm75a.getTemperature());
  lm75a.shutdown();
}

/* debugIIC - scan I²C bus for existing devices, used for debugging of hardware in development state. */
void debugIIC() {
  Serial.println("Scanning...");
  byte error, address;
  int nDevices;
  for (address = 1; address < 127; address++ )
  {
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

/* debugExternal - used to manually set the power of the external I/O for debugging of hardware in development state. */
void debugExternal(boolean state){
  Serial.println("Debug external\n");
  Serial.println(state);
  if(state){
    digitalWrite(EXTERNAL_IO, 1);
  }else{
    digitalWrite(EXTERNAL_IO, 0);
  }
}

/* debugFrequency - used to test channel 7 (sensor 8) using MUX for debugging of hardware in development state. */
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
