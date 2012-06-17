#include <math.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
//#include <WProgram.h>
//#include <Arduino.h>
//#include <MemoryFree.h>



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

// how many thermistor samples to take and average
#define NUMSAMPLES 5

// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950

// the value of the 'other' resistor
#define SERIESRESISTOR 10000    

PROGMEM uint8_t resolution = 12;

unsigned long lastTempRequest = 0;
unsigned int  delayInMillis = 0;
float temperature = 0.0;
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
PID myPID(&Input, &Output, &Setpoint, 2, 1, 1, DIRECT);

PROGMEM unsigned int WindowSize = 5000; // PWM lentgh for relay (in ms)

unsigned long windowStartTime;

uint8_t SampleCursor = 0;
int samples[NUMSAMPLES];

float getAccurateTemp() {
  float t1, t2;
  t1 = getSensorTemp();
  t2 = getThermistorTemp();
  if (t2 - t1 > 2.0) {
    return t1;
  } else {
    return t2;
  }
}

float getSensorTemp() {
  if (millis() - lastTempRequest >= delayInMillis) // waited long enough??
  {
    temperature = sensors.getTempCByIndex(0);
    sensors.requestTemperatures(); 
    delayInMillis = 750 / (1 << (12 - resolution));
    lastTempRequest = millis(); 
  }
  return temperature;
}

float getThermistorTemp() {
  float average;
  samples[SampleCursor] = analogRead(THERMISTOR_PIN);
  SampleCursor++;
  SampleCursor % NUMSAMPLES;
  
  average = 0;
  for (uint8_t i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
 
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
 
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

  // setup thermistor sample ring
  for (uint8_t i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTOR_PIN);
   delay(10);
  }

  Serial.begin(57600);
  Serial.println("hello!");
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
  lcd.clear();
  lcd.println("hello");
//  lcd.println(getThermistorTemp());
  delay(1000);
  lcd.clear();
}

boolean paused = true;
boolean button1_pressed = false;
boolean changeRelay = false;
int loopCounter = 0;
unsigned long lastLoop = millis();


void loop() {
  unsigned long now = millis();
  delay(min(now-lastLoop, 100));
  lastLoop=now;
  loopCounter++;
  if (loopCounter % 10 == 0) {  
    loopCounter = 0;
    Serial.println(now);
  }
  
  float thermistorTemp = getThermistorTemp(); 
  float sensorTemp = getSensorTemp();
  float ambientTemp = getAmbientTemp();
  //  Input = (sensorTemp+thermistorTemp)/2;
  // Input = getAccurateTemp();
  Input = thermistorTemp;
  
  int button1_state = digitalRead(BUTTON1_PIN);
  int button2_state = digitalRead(BUTTON2_PIN);
  int button3_state = digitalRead(BUTTON3_PIN);
  
//  lcd.clear();
  lcd.setCursor(0, 0);
  // print the number of seconds since reset:
  //lcd.print(millis()/1000);
  lcd.print(thermistorTemp, 1); 
  lcd.setCursor(5, 0);

//  lcd.print(" ");
  lcd.print(sensorTemp, 1); 
  lcd.setCursor(10, 0);
  if (paused) {
    lcd.print("Stop");
  } else {
    lcd.print("Go  ");
  }
  lcd.setCursor(0, 1);
  lcd.print(Input, 1);
  lcd.setCursor(5, 1);
  lcd.print(Setpoint, 1);
  lcd.print(" ");
  lcd.print(Output, 0);
  lcd.print(" ");
//  lcd.print(button1_state);
//  lcd.print(button2_state);
//  lcd.print(button3_state);

  
//  Serial.print("freeMemory() reports ");
//  Serial.println( freeMemory() );

  if (!button2_state) {
    Serial.println("B2");
    Setpoint+=0.1;
    delay(100);
  } else if (!button3_state) {
    Serial.println("B3");
    Setpoint-=0.1;
    delay(100);
  } 
  
  if (!button1_state) {
    if (!button1_pressed) {
      Serial.println("B1");
      button1_pressed = true;
      if(paused) {
        paused = false;
      } else {
        paused = true;
      }
    }
  } else {
    button1_pressed = false;
  }
  
//  lcd.print("    ");
  
  if (paused) {
//    if (loopCounter % 10 == 0) {  
//      Serial.print("windowStartTime: ");
//      Serial.println(windowStartTime);
//    }
    digitalWrite(RELAY_PIN,LOW);
  } else {
    myPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
    while (now - windowStartTime>WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    
//    if (loopCounter % 10 == 0) {  
//      Serial.print("windowStartTime: ");
//      Serial.println(windowStartTime);
//      Serial.print("now:             ");
//      Serial.println(now);
//      Serial.print("paused:          ");
//      Serial.println(paused);
//    }  
    if(Output > now - windowStartTime) {
//       Serial.println("Relay On");
      if (relayState == LOW) {
        relayState=HIGH;
        Serial.println("On");
        changeRelay=true;
      }
    } else {
//      Serial.println("Relay Off");
      if (relayState == HIGH) {
        relayState=LOW;
        Serial.println("Off");
        changeRelay=true;
      }
      if (changeRelay == true) {
        digitalWrite(RELAY_PIN,relayState);
        delay(100);
        changeRelay = false;
      }
    }
  }
}


