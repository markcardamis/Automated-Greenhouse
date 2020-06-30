#include <DHT.h>
#include <Servo.h>
#include <SandTimer.h>

#define TEMPINTPIN 0      // Analog pin 0 connected to the Temperature Sensor Interior
#define TEMPEXTPIN 1      // Analog pin 1 connected to the Temperature Sensor Exterior
#define SOILPIN 2         // Analog pin 2 connected to the Soil Moisture Sensor
#define LUMPIN 3          // Analog pin 3 connected to the Light Sensor
#define SOILPINVCC 2      // Digital pin 2 connected to the 5V pin of Soil Moisture 
#define FANPIN 3          // Digital pin 3 connected to the Fan
#define DHTINTPIN 4       // Digital pin 4 connected to the DHT Sensor Interior
#define DHTEXTPIN 5       // Digital pin 5 connected to the DHT Sensor Exterior
#define PUMPPIN 6         // Digital pin 6 connected to the Water Solenoid Pump (Relay 1)
#define LAMP1PIN 7        // Digital pin 7 connected to the Lights Left (Relay 2)
#define LAMP2PIN 8        // Digital pin 8 connected to the Lights Right (Relay 3)
#define SERVOPIN 10       // Digital pin 10 connected to the ServoMotor
#define DHTINTVCC 12      // Digital pin 12 connected to the 5V pin of Interior DHT Sensor 

// Function declarations
void readSensors(bool);   // Read sensor with print to Serial set to false
void command (int, bool);
void pwm_command (int, bool);
double Thermister(int);
int readSoil(int, int);
void window (bool);
void pump (bool);

// Constant variables
const int kSerialBaudrate = 9600;               // baud rate
const int kCycleTime = 100;                     // cycle time of the arduino loop
const int kServomotorOpened = 0;                // Angle which servo opens window completely
const int kServomotorClosed = 88;               // Angle which servo closes window completely
const uint32_t kFanLowSpeed = 50;               // Duty cycle of fan at low speed
const uint32_t kFanHighSpeed = 200;             // Duty cycle of fan at high speed
const int pumpTimeThreshold = 5000;             // time (ms) water pump is activated
const int kSensorPolling = 10000;               // Time (ms) between sensor polls
const uint32_t kSensorSlowPolling = 600000;     // Time (ms) between slow sensor polls
const uint32_t kSensorBiDailyPolling = 43200000;// Time (ms) between bi-daily sensor polls


// Global objects
Servo servomotor;                  // create servo object to control a servo
DHT dhtInterior(DHTINTPIN, DHT22); // Create an Interior DHT sensor object
DHT dhtExterior(DHTEXTPIN, DHT11); // Create an Exterior DHT sensor object
SandTimer sensorTimer;             // Create a general Timer to limit sensor polling
SandTimer soilMoistureTimer;       // Create a slow-polling Timer for the soil sensor
SandTimer pumpTimer;               // Create a daily Timer for the pump solenoid

// Initialize variables
bool booleanCoolingOn = false;
bool debugMode = false;

unsigned long startTime = 0; // start time of loop in millis
unsigned long nowTime = 0; // current time of loop in millis

char serialChar = 0; // reading the serial port
String query = ""; // save the command from the serial port

float temp_int = 0; // Indoor greenhouse temperature
float humidity_int = 0; // Indoor greenhouse humidity
float temp_ext = 0; // Outdoor greenhouse temperature
float humidity_ext = 0; // Indoor greenhouse humidity
int soil_moisture = 0; // Soil moisture level
int luminosity = 0; // Light level

int humidity_limit_low = 20; // low humidity limit
int humidity_limit_high = 60; // high humidity limit
String humitidy_temp_var; // temporary variable for humidity

// temperature regulation
int temperature_setpoint = 23; // average temperature setpoint
int temperature_delta = 5; // from T% to the setpoint
int temperature_state = 0; // switch mode from cooling to heating

// logic command of the arduino
bool pumpState = 0; // pump
bool fanState = 0; // fan
bool lampState = 0; // lamp
bool servoState = 0; // servomotor

void setup () 
{  
  cli();                               // disable global interrupts
  TCCR2B = TCCR2B & 0b11111000 | 0x01; // Set pin 3 & 11 to fastPWM 31372.55 Hz
  sei();                               // enable global interrupts:

  Serial.begin(kSerialBaudrate); // start serial

  dhtInterior.begin(); // Start Interior DHT sensor object
  dhtExterior.begin(); // Start Exterior DHT sensor object

  sensorTimer.start(kSensorPolling);       // Poll general input sensors every 10 seconds
  soilMoistureTimer.start(kSensorSlowPolling); // Poll soil Moisture sensor every 10 min
  pumpTimer.start(kSensorBiDailyPolling);  // Poll the pump solenoid state every 12 hours
  
  // Initialize the digital inputs/outputs
  pinMode(SOILPINVCC, OUTPUT);    // Pin to control the Soil Moisture VCC
  pinMode(DHTINTVCC, OUTPUT);    // Pin to control the Interior DHT VCC
  pinMode(PUMPPIN, OUTPUT);       // Water Solenoid Pump
  pinMode(LAMP1PIN, OUTPUT);      // Lights Left Ouput
  pinMode(LAMP2PIN, OUTPUT);      // Lights Right Output
  pinMode(FANPIN, OUTPUT);        // Fan Output
  pinMode(FANPIN, OUTPUT);        // Fan Output
  command(SOILPINVCC, LOW);       // Set the Soil Moisture output low on startup
  command(DHTINTVCC, HIGH);       // Set the Interior DHT output high on startup
  command(PUMPPIN, LOW);          // Set Pump Relay Output Low
  command(LAMP1PIN, LOW);         // Set Lamp1 Relay Output Low
  command(LAMP2PIN, LOW);         // Set Lamp2 Relay Output Low
  pwm_command(FANPIN, LOW);       // Reset fan speed
  window(0);                      // Reset servo motor
  
  delay(2000);
}

void loop () 
{

  readSensors(debugMode);

  // ************************************************ ******** Series reading part
  if (Serial.available() > 0)
  {
    serialChar = Serial.read();
    if (serialChar != 10)
    {
      query.concat (serialChar);
    }
  }

  // ************************************************ ******** Interpretation of the order
    if (query == "PUMP")
    {
      Serial.println (pumpState);
      query = "";
    }
    else if (query == "FAN")
    {
      Serial.println (fanState);
      query = "";
    }
    else if (query == "LAMP")
    {
      Serial.println (lampState);
      query = "";
    }
    else if (query == "SERVO")
    {
      Serial.println (servoState);
      query = "";
    }
    else if (query == "TPINT")
    {
      Serial.println (temp_int);
      query = "";
    }
    else if (query == "TPEXT")
    {
      Serial.println (temp_ext);
      query = "";
    }
    else if (query == "HMINT")
    {
      Serial.println (humidity_int);
      query = "";
    }
    else if (query == "HMEXT")
    {
      Serial.println (humidity_ext);
      query = "";
    }
    else if (query == "MOIST")
    {
      Serial.println (soil_moisture);
      query = "";
    }
    else if (query == "LUMIN")
    {
      Serial.println (luminosity);
      query = "";
    }
    else if (query == "RESET")
    {
      pumpState = 0;
      fanState = 0;
      lampState = 0;
      servoState = 0;
      query = "";
    }
    else if (query == "DBGON")
    {
      debugMode = true;
      query = "";
    }
    else if (query == "DBGOF")
    {
      debugMode = false;
      query = "";
    }
    else if (query == "WINON")
    {
      window(1); 
      query = "";
    }
    else if (query == "WINOF")
    {
      window(0);
      query = "";
    }
    else if (query == "WINRD")
    {
      Serial.println (servomotor.read()); 
      query = "";
    }
    else
    {
    if (Serial.available () == 0)
      {
        query = "";
      }
    }
 
  // ************************ logic cmd temperature regulation (lamp / fan)
  switch (temperature_state)
  {
    case 0:
      if (temp_int <= (temperature_setpoint - temperature_delta)) // heating requirement
      {
        temperature_state = 1;
      }
      else if (temp_int >= (temperature_setpoint + temperature_delta)) // need for cooling
      {
        temperature_state = 2;
      }
      fanState = 0;
      lampState = 0;
      servoState = 0;
      
      break;
    case 1:
      if (temp_int >= temperature_setpoint)
      {
        temperature_state = 0;
      }
      fanState = 0;
      lampState = 1;
      servoState = 0;
      break;
    case 2:
      if (temp_int <= temperature_setpoint)
      {
        temperature_state = 0;
      }
      fanState = 1;
      lampState = 0;
      servoState = 1;
      break;
    default: break;
  }


  // ************************ Command part outputs via forced or logic command

  pwm_command(FANPIN, fanState);
  command(LAMP1PIN, lampState);
  delay(100);
  command(LAMP2PIN, lampState);

    
  // ************************************************ ******** servomotor management
  window (servoState);

  pump (pumpState);

  // ************************************************ ******** cycle time
  delay (kCycleTime);

}

// ************************************************ ******** Functions ***************************************** ********************

void readSensors(bool debugPrintMode)
{
  if (sensorTimer.finished())
  {
    temp_ext = Thermister(analogRead(TEMPEXTPIN));
    soil_moisture = readSoil(SOILPIN, SOILPINVCC);
    luminosity = (((float) analogRead(LUMPIN)) / 10.24);
    temp_int = dhtInterior.readTemperature();
    humidity_int = dhtInterior.readHumidity();
    humidity_ext = dhtExterior.readHumidity();

    if (isnan(temp_int))
    {
      DHT_restart();
    }
    
    if (debugPrintMode)
    {
      Serial.print(F("Temperature Analog Exterior: "));
      Serial.print(temp_ext);
      Serial.println(F("°C ")); 
      Serial.print(F("Moisture Analog Scale: "));
      Serial.println(soil_moisture);
      Serial.print(F("Luminosity Analog Interior: "));
      Serial.print(luminosity);
      Serial.println(F("%"));
      Serial.print(F("Temperature Digital Interior: "));
      Serial.print(temp_int);
      Serial.println(F("°C "));
      Serial.print(F("Humidity Digital Interior: "));
      Serial.print(humidity_int);
      Serial.println(F("%"));
      Serial.print(F("Humidity Digital Exterior: "));
      Serial.print(humidity_ext);
      Serial.println(F("%"));
    } else {
      Serial.print(temp_int);
      Serial.print(" ");
      Serial.print(temp_ext);
      Serial.print(" ");
      Serial.println((temperature_setpoint-temperature_delta) + (lampState*temperature_delta));
    }
    
    sensorTimer.startOver();
  }
}

// **************************** function of the output control
void command (int pin, bool resultcmd)
{
  if (resultcmd == 1)
  {
    digitalWrite (pin, LOW);
  }
  else
  {
    digitalWrite (pin, HIGH);
  }
}

void pwm_command (int pin, bool resultcmd)
{
  if (resultcmd == 1)
  {
    analogWrite (pin, kFanHighSpeed);
  }
  else
  {
    analogWrite (pin, kFanLowSpeed);
  }
}

// ****************************   Thermister conversion function
double Thermister (int RawADC) 
{
  double Temp;
  Temp = log(((10240000/RawADC) - 10000));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
  Temp = Temp - 273.15; // Convert Kelvin to Celcius
  return Temp;
}

// ****************************   Soil read function
int readSoil (int Pin, int PinVcc) 
{
  if (soilMoistureTimer.finished())
  {
    float temp_raw_adc;
    command(PinVcc, HIGH);
    delay(200);
    temp_raw_adc = analogRead(Pin);
    command(PinVcc, LOW);
    soilMoistureTimer.startOver();
    return (((temp_raw_adc / 950) * 100)); // 0-300 dry, 300-700 humit, 700-950 water
  }
  else
  {
    return soil_moisture;
  }
}

// ****************************   Window control function
void window (bool resultcmd)
{
  if (resultcmd == 1) // opening request
  { 
    int delayTime= (abs(servomotor.read()-kServomotorOpened))*15;
    servomotor.attach(SERVOPIN);
    servomotor.write(kServomotorOpened);
    delay(delayTime);
    servomotor.detach();
  }
  else if (resultcmd == 0) // closing request
  { 
    int delayTime= (abs(servomotor.read()-kServomotorClosed))*15;
    servomotor.attach(SERVOPIN);
    servomotor.write(kServomotorClosed);
    delay(delayTime);
    servomotor.detach();
  }
}

// ****************************   Pump control function
void pump (bool resultcmd)
{
  if ((resultcmd == true) || (pumpTimer.finished())) // opening request
  { 
      command(PUMPPIN, HIGH);
      delay(pumpTimeThreshold);
      command(PUMPPIN, LOW);
      pumpTimer.startOver();
  } 
  else
  {
    command(PUMPPIN, LOW);
  }  

}

// Physically power recycle the DHT using a relay/GPIO
void DHT_restart()
{
  command(DHTINTVCC, LOW);   
  delay(1000);
  command(DHTINTVCC, HIGH);  
  delay(1000);
}