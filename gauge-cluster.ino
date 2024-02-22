#include <ArduinoJson.h>
#include <DFL168A.h>
#include <Ewma.h>
#include <Adafruit_AHTX0.h>
#include <Wire.h>
#include <Adafruit_MS8607.h>
#include <Adafruit_Sensor.h>

/*
  Reset the Arduino board.
  Declare reset fuction at address 0.
*/
void(* resetFunc) (void) = 0;
/*
  Setup the Dafulai board to read the J1708 ECM data.
*/
DFL168A myDFL168A(&Serial1, J1708_PROTOCOL, 5000, 500000, 9600, 500);

/* 
  Exponentially Weighted Moving Average filter is used for smoothing data series readings.
  Ewma adcFilter1(0.1); //Less smoothing - faster to detect changes, but more prone to noise
  Ewma adcFilter2(0.01);  // More smoothing - less prone to noise, but slower to detect changes
*/
Ewma adcFilter(0.1);
Ewma adcFilterOilPressure(0.1);



/*
  Constant integer to set the baud rate for serial monitor
*/
const int BAUD_RATE = 9600;
/* 
  Analog to Digital resolution for Arduino
*/
const float A_2_D_RESOLUTION = 1024.0;
/* 
  Dafulai board J1708
*/
bool Vehicle_OK;
/* 
  Analog input pin for fuel tank sensor.
*/
const int ANALOG_PIN_FUEL_TANK = A2;
/* 
  Analog input pin for engine oil pressure sensor.
*/
const int ANALOG_PIN_ENGINE_OIL_PRESSURE = A3;


/** AIR PRESSURE TANKS - START **/
/*
  Primary Tank (green). Analog input pin for the pressure transducer 1.
*/
const int ANALOG_PIN_PRIMARY_AIR_TANK = A0;
/*
  Secondary Tank (red). Analog input pin for the pressure transducer 2.
*/
const int ANALOG_PIN_SECONDARY_AIR_TANK = A1;
/*
  Analog reading of pressure transducer at 0psi
*/
const int pressureZero = 102.4;
/*
  Analog reading of pressure transducer at pressureTransducerMaxPSI
*/
const int pressureMax = 921.6;
/*
  Max PSI read by transducer.
*/
const int pressureTransducerMaxPSI = 150;
/** AIR PRESSURE TANKS - END **/

/*
  Variable to account for non blocking delays.
*/
unsigned long previousMillis = 0;
/*
  Constant to execute loop using this interval in millis.
*/
const long WAIT_INTERVAL = 1000;

/* 
  External Temp/Humidity.
  Adafruit AM2315C Sensor (AHT20).
*/
Adafruit_AHTX0 adafruitHumidityTempExternal;
/* 
  Adafruit AM2315C Sensor (AHT20)
*/
bool adafruitSensorAHTX0_OK;
/* 
  Internal Temp/Humidity/Pressure
  Adafruit MS8607 Sensor.
*/
Adafruit_MS8607 adafruitHumidityTempInternal;
/* 
  Adafruit MS8607 Sensor.
*/
bool adafruitSensorMS8607_OK;
/* 
  Adafruit Sensors Enum.
*/
sensors_event_t humidity, temp, pressure;

void setup() {
  Serial.begin(BAUD_RATE);
  //Setup Dafulai ECM board pin.
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  // Start Dafulai board
  Vehicle_OK = myDFL168A.begin();
  if (Vehicle_OK) {
    digitalWrite(13, HIGH);
  } else {
    // Delay 3 seconds before reset the Arduino board.
    // Serial.println("ECM module not ready.");
    // Serial.println("Calling reset.");
    // delay(3000);
    // resetFunc();
  }
  // Setup external Humidity and Temperature sensors
  adafruitSensorAHTX0_OK = adafruitHumidityTempExternal.begin();
  adafruitSensorMS8607_OK = adafruitHumidityTempInternal.begin();
}

void loop() {
  // Get the current millis to execute the loop only every WAIT_INTERVAL period.
  unsigned long currentMillis = millis();
  // If WAIT_INTERVAL period is passed execute calls.
  if (currentMillis - previousMillis >= WAIT_INTERVAL) {
    // Update previousMillis variable.
    previousMillis = currentMillis;
    //Initiate Json Document
    JsonDocument docJson;
    // Read all data
    readAirTanks(docJson);
    readFuelLevel(docJson);
    readEngineOilPressure(docJson);
    // Only read external Temperature and Humidity if module is connected.
    if (adafruitSensorAHTX0_OK) {
      docJson["externalTempHumidity"] = true;
      readExternalTempHumidity(docJson);
    } else {
      docJson["externalTempHumidity"] = false;
    }
    // Only read internal Temperature, Humidity and Pressure if module is connected.
    if (adafruitSensorMS8607_OK) {
      docJson["internalTempHumidityPressure"] = true;
      readInternalTempHumidityPressure(docJson);
    } else {
      docJson["internalTempHumidityPressure"] = false;
    }

    // Only read J1708 data if module is connected to vehicle.    
    if (Vehicle_OK) {
      docJson["ecmData"] = true;
      readJ1708Data(docJson);
    } else {
      docJson["ecmData"] = false;
    }
    // Send Json over Serial.
    serializeJson(docJson, Serial);
    Serial.println("");
  }
}

/* Read Internal Temperature, Humidity and Pressure from Sensors */
void readInternalTempHumidityPressure(JsonDocument& doc) {
  adafruitHumidityTempInternal.getEvent(&pressure, &temp, &humidity);

  int t = celciusToFahrenheit(int(temp.temperature));
  doc["internalTemperature"] = t;
  doc["internalHumidity"] = int(humidity.relative_humidity);
  doc["internalPressure"] = int(pressure.pressure);
}

/* Read External Temperature and Humidity from Sensors */
void readExternalTempHumidity(JsonDocument& doc) {
//  sensors_event_t humidity, temp;
  adafruitHumidityTempExternal.getEvent(&humidity, &temp);

  int t = celciusToFahrenheit(int(temp.temperature));
  doc["externalTemperature"] = t;
  doc["externalHumidity"] = int(humidity.relative_humidity);
}

/* Read Engine Oil Pressure */
void readEngineOilPressure(JsonDocument& doc) {
  int analogOilPressureRead = 0;
  float oilFilteredRead = 0.0;
  // Clean analogRead();
  int dummy = analogRead(ANALOG_PIN_ENGINE_OIL_PRESSURE);
  // Read the analog data from sensor.
  for(int i=1; i <=10; i++) {
    analogOilPressureRead = analogRead(ANALOG_PIN_ENGINE_OIL_PRESSURE);
    // Serial.print("analogOilPressureRead ");
    // Serial.println(analogOilPressureRead);
    // Apply the filter to smooth the data and minimize the noise.
    oilFilteredRead = adcFilterOilPressure.filter(analogOilPressureRead);
    // Serial.print("oilFilteredRead ");
    // Serial.println(oilFilteredRead);
  }
  // Voltage from Arduino
  int oilVin = 5;
  //Voltage to calculate from unkown resistor R2
  float oilVout = 0.0;
  //Known fuel resistor (ohms)
  float oilR1 = 180.0;
  // Unkown resistor from fuel sender (ohms)
  float oilR2 = 0.0;

  // Calculate real voltage from digital conversion
  oilVout = (oilVin * oilFilteredRead)/A_2_D_RESOLUTION;
  // Serial.print("oilVout ");
  // Serial.println(oilVout);
  // Calculate Unknow resistance R2 
  oilR2 = (oilVin / oilVout - 1) * oilR1;
  // Serial.print("oilR2 ");
  // Serial.println(oilR2);

  // Mapped R2 value from 10-184ohms to 0-100PSI
  long m = map(oilR2, 10, 184, 0, 100);
  m = constrain(m, 0, 100);
  
  // Write Oil Pressure 0-100PSI to Json Document.
  doc["engineOilPressure"] = m;
}

/* Read Fuel Level */
void readFuelLevel(JsonDocument& doc) {
  int analogFuelRead = 0;
  float filteredRead = 0.0;
  // Clean analogRead();
  int dummy = analogRead(ANALOG_PIN_FUEL_TANK);
  // Serial.print("dummy fuel ");
  // Serial.println(dummy);
  // Read the analog data from sensor.
  for(int i=1; i <=10; i++) {
    analogFuelRead = analogRead(ANALOG_PIN_FUEL_TANK);
    // Serial.print("analogFuelRead ");
    // Serial.println(analogFuelRead);
    // Apply the filter to smooth the data and minimize the noise.
    filteredRead = adcFilter.filter(analogFuelRead);
    // Serial.print("filteredRead ");
    // Serial.println(filteredRead);
  }
  // analogFuelRead = analogRead(ANALOG_PIN_FUEL_TANK);
  // Serial.print("analogFuelRead ");
  // Serial.println(analogFuelRead);
  // // Apply the filter to smooth the data and minimize the noise.
  // filteredRead = adcFilter.filter(analogFuelRead);
  // Serial.print("filteredRead ");
  // Serial.println(filteredRead);

   // Voltage from Arduino
  int fuelVin = 5;
  //Voltage to calculate from unkown resistor R2
  float fuelVout = 0.0;
  //Known fuel resistor (ohms)
  float fuelR1 = 90.0;
  // Unkown resistor from fuel sender (ohms)
  float fuelR2 = 0.0;

  // Calculate real voltage from digital conversion
  fuelVout = (fuelVin * filteredRead)/A_2_D_RESOLUTION;
  // Serial.print("fuelVout ");
  // Serial.println(fuelVout);
  // Calculate Unknow resistance R2 
  fuelR2 = (fuelVin / fuelVout - 1) * fuelR1;
  // Serial.print("fuelR2 ");
  // Serial.println(fuelR2);

  // Mapped R2 value from 0-90ohms to 0%-100%
  long m = map(fuelR2, 0, 90, 0, 100);
  m = constrain(m, 0, 100);
  
  // Write fuel level 0-100% to Json Document.
  doc["fuelLevel"] = m;
}

/* Read Primary and Secondary Air Tank pressure from analog sensors */
void readAirTanks(JsonDocument& doc) {
  // Primary tank
  // Clean the analoRead()
  int dummy = analogRead(ANALOG_PIN_PRIMARY_AIR_TANK);
  int tempTankRead = 0;
  for(int i = 0; i < 10; i++ ) {
    tempTankRead += analogRead(ANALOG_PIN_PRIMARY_AIR_TANK);
  }
  // Allows rounding for integer math.
  tempTankRead += 5;
  // Get the average for the 10 readings.
  float pressureValue1 = tempTankRead / 10;
  //conversion equation to convert analog reading to psi
  pressureValue1 = ((pressureValue1-pressureZero)*pressureTransducerMaxPSI)/(pressureMax-pressureZero);
  // Add tank PSI to Json Document
  doc["primaryAirTank"] = int(pressureValue1);

  // Secondary tank
  // Clean the analoRead()
  dummy = analogRead(ANALOG_PIN_SECONDARY_AIR_TANK);
  tempTankRead = 0;
  for(int x = 0; x < 10; x++ ) {
    tempTankRead += analogRead(ANALOG_PIN_SECONDARY_AIR_TANK);
  }
  // Allows rounding for integer math.
  tempTankRead += 5;
  // Get the average for the 10 readings.
  float pressureValue2 = tempTankRead / 10;
  //conversion equation to convert analog reading to psi
  pressureValue2 = ((pressureValue2-pressureZero)*pressureTransducerMaxPSI)/(pressureMax-pressureZero);
  // Add tank PSI to Json Document
  doc["secondaryAirTank"] = int(pressureValue2);
}

/* Read all data from J1708 module and stores into JsonDocument */
void readJ1708Data(JsonDocument& doc) {
  //Current vehicle speed km/h.
  float VehicleSpeed;
  //It is the ratio of current output torque to maximum torque available at the current engine speed. This is percentage.
  float EngineLoad;
  //It is the temperature of liquid found in engine cooling system in Celsius degree.
  int  CoolantTemperature;
  //It is the current fuel economy at current vehicle velocity in Km/L.
  float InstantFuelEconomy;
  //It is the Average of instantaneous fuel economy for that segment of vehicle operation of interest in Km/L.
  float AvgFuelEconomy;
  //It is the net brake power that the engine will deliver continuously, specified for a given application at a rated speed in KW.
  float RatedEnginePower;
  //It is the measured electrical potential of the battery in Volts.
  float BatteryVoltage;
  //It is the temperature of air surrounding vehicle(engine bay?) in Celsius degree.
  int AmbientTemp;
  //It is the rotational velocity of crankshaft in RPM.
  int EngineSpeed;
  //It is the temperature of precombustion air found in intake manifold of engine air supply system in Celsius degree.
  int IntakeManifoldTemp;
  //It is the ratio of actual accelerator pedal position to maximum pedal position. This is percentage.
  float AccelPedalPosi1;
  //It is the amount of fuel consumed by engine per unit of time in L/s.
  float FuelRate;

  if (myDFL168A.J1708.getVehicleSpeed(VehicleSpeed)) {
    int mph = kmhToMph(VehicleSpeed);
    doc["vehicleSpeed"] = mph;
  } else {
    doc["vehicleSpeed"] = NULL;
  }

  if (myDFL168A.J1708.getEngineLoad(EngineLoad)) {
    doc["engineLoad"] = int(EngineLoad);
  } else {
    doc["engineLoad"] = NULL;
  }

    if (myDFL168A.J1708.getCoolantTemperature(CoolantTemperature)) {
    int far = celciusToFahrenheit(CoolantTemperature);
    doc["coolantTemperature"] = far;
  } else {
    doc["coolantTemperature"] = NULL;
  }

  if (myDFL168A.J1708.getInstantFuelEconomy(InstantFuelEconomy)) {
    doc["instantFuelEconomy"] = InstantFuelEconomy;
  } else {
    doc["instantFuelEconomy"] = NULL;
  }

  if (myDFL168A.J1708.getAvgFuelEconomy(AvgFuelEconomy)) {
    float mpg = kmlToMpg(AvgFuelEconomy);
    doc["avgFuelEconomy"] = mpg;
  } else {
    doc["avgFuelEconomy"] = NULL;
  }

  if (myDFL168A.J1708.getRatedEnginePower(RatedEnginePower)) {
    float hp = kwToHp(RatedEnginePower);
    doc["ratedEnginePower"] = hp;
  } else {
    doc["ratedEnginePower"] = NULL;
  }

  if (myDFL168A.J1708.getBatteryVoltage(BatteryVoltage)) {
    doc["batteryVoltage"] = BatteryVoltage;
  } else {
    doc["batteryVoltage"] = NULL;
  }

  if (myDFL168A.J1708.getAmbientTemp(AmbientTemp)) {
    int fah = celciusToFahrenheit(AmbientTemp);
    doc["ambientTempEngineBay"] = fah;
  } else {
    doc["ambientTempEngineBay"] = NULL;
  }

  if (myDFL168A.J1708.getEngineSpeed(EngineSpeed)) {
    doc["rpm"] = EngineSpeed;
  } else {
    doc["rpm"] = NULL;
  }

  if (myDFL168A.J1708.getIntakeManifoldTemp(IntakeManifoldTemp)) {
    int fah = celciusToFahrenheit(IntakeManifoldTemp);
    doc["intakeManifoldAirTemp"] = fah;
  } else {
    doc["intakeManifoldAirTemp"] = NULL;
  }

  if (myDFL168A.J1708.getAccelPedalPosi1(AccelPedalPosi1)) {
    doc["accelPedalPosi1"] = int(AccelPedalPosi1);
  } else {
    doc["accelPedalPosi1"] = NULL;
  }

  if (myDFL168A.J1708.getFuelRate(FuelRate)) {
    doc["fuelRate"] = FuelRate;
  } else {
    doc["fuelRate"] = NULL;
  }
}

// Convert Kilometer per Hour to Miles per Hour
int kmhToMph(float kmh) {
  int mph = kmh * 0.6213711922;
  return mph;
}
// Convert Kilometer per Liter to Miles per Gallon
float kmlToMpg(float kml) {
  float mpg = kml * 2.35215;
  return mpg;
}
// Convert Celcius to Fahrenheit
int celciusToFahrenheit(int celsius) {
  int fahrenheit = ((celsius * 9) / 5 + 32);
  return fahrenheit;
}
// Converts Kilowatt to Mechanical Horsepower
float kwToHp(float kw) {
  float hp = kw * 1.341;
  return hp;
}
