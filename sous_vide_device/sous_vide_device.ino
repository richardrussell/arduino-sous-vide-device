#include <math.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

// LCD is connected to pins as follows
#define LCD_RS_PIN 7
#define LCD_ENABLE_PIN 6
#define LCD_DATA4_PIN 5
#define LCD_DATA5_PIN 4
#define LCD_DATA6_PIN 3
#define LCD_DATA7_PIN 2

// Buttons are connected (top to bottom) as follows:
#define BUTTON1_PIN 12
#define BUTTON2_PIN 11
#define BUTTON3_PIN 10

// Relay
#define RELAY_PIN 9

// DS18B20 Temperature sensor
#define DS18B20_PIN 8

// Thermistor is on Analog 1 - this should likely be set to 3.3V
#define THERMISTOR_PIN 1

// TMP36 (ambient) is on Analog 2 - this may need resetting to 3.3V
#define TMP36_PIN 2

// Thermistor definitions
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      

// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   

// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5

// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950

// the value of the 'other' resistor
#define SERIESRESISTOR 10000    

int  resolution = 12;
unsigned long lastTempRequest = 0;
int  delayInMillis = 0;
float temperature = 0.0;
int  idle = 0;
DeviceAddress tempDeviceAddress;


// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS_PIN, LCD_ENABLE_PIN, LCD_DATA4_PIN, LCD_DATA5_PIN, LCD_DATA6_PIN, LCD_DATA7_PIN);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(DS18B20_PIN);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

int relayState=LOW;


//Define Variables we'll be connecting to
double Setpoint = 55.0, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 20, 5, 1, DIRECT);

int WindowSize = 5000; // PWM lentgh for relay (in ms)

unsigned long windowStartTime;



float getSensorTemp() {
  if (millis() - lastTempRequest >= delayInMillis) // waited long enough??
  {
    temperature = sensors.getTempCByIndex(0);
    sensors.requestTemperatures(); 
    delayInMillis = 750 / (1 << (12 - resolution));
    lastTempRequest = millis(); 
  }
  
  
//  sensors.requestTemperatures();  // Send the command to get temperatures
  return temperature;
//  return sensors.getTempCByIndex(0); // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
}

float getThermistorTemp() {
  uint8_t i;
  int samples[NUMSAMPLES];  
  float average;
  
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTOR_PIN);
   delay(10);
  }
 
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
 
//  Serial.print("Average analog reading "); 
//  Serial.println(average);
 
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
 // Serial.print("Thermistor resistance "); 
 // Serial.println(average);
 
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
 
 // Serial.print("Temperature "); 
 // Serial.print(steinhart);
 // Serial.println(" *C");
  return steinhart;                                      // Return the Temperature
}

float getAmbientTemp() {
  float volts = analogRead(TMP36_PIN) * 0.004882814;
  return (volts - 0.5) *100;
}



void setup() {

  // Start up the onewire library
  sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, resolution);
  
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  delayInMillis = 750 / (1 << (12 - resolution)); 
  lastTempRequest = millis(); 

  Serial.begin(9600);
  Serial.println("hello serial monitor!");
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(BUTTON3_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  windowStartTime = millis();


  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.println("hello, world!");
//  lcd.println(getThermistorTemp());
  delay(1000);
  lcd.clear();
}


void loop() {
  float thermistorTemp = getThermistorTemp(); 
  float sensorTemp = getSensorTemp();
  float ambientTemp = getAmbientTemp();
  
  int button1_state = digitalRead(BUTTON1_PIN);
  int button2_state = digitalRead(BUTTON2_PIN);
  int button3_state = digitalRead(BUTTON3_PIN);
  
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 0);
  // print the number of seconds since reset:
  //lcd.print(millis()/1000);
  lcd.print(thermistorTemp, 1); 
  lcd.print(" ");
  lcd.print(sensorTemp, 1); 
  lcd.print("       ");
  lcd.setCursor(0, 1);
  lcd.print(ambientTemp, 1);
  lcd.print(" ");
  lcd.print(Setpoint, 1);
  lcd.print(" ");
  lcd.print(relayState);
  lcd.print(" ");
  lcd.print(button1_state);
  lcd.print(button2_state);
  lcd.print(button3_state);
  lcd.print("    ");

//  Input = (sensorTemp+thermistorTemp)/2;
  Input = thermistorTemp;
  myPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  unsigned long now = millis();
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(Output > now - windowStartTime) relayState=HIGH;
  else relayState=LOW;
  digitalWrite(RELAY_PIN,relayState);
}


