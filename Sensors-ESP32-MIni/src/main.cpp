#include <Arduino.h>
#include <Wire.h>

//Libraries for BMP280 sensor.
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>

//Libraries for Heart Rate sensor.
#include <driver/adc.h>

//Libraries for Oximeter sensor.
#include "MAX30105.h"
#include "spo2_algorithm.h"

// I2C pins for BMP280 sensor
#define I2C_SCL 37
#define I2C_SDA 39
// I2C object for BMP280 sensor Corresponding to line 182 in Adafruit_BMP280.h
TwoWire I2CBMP = TwoWire(0);
Adafruit_BMP280 bmp(&I2CBMP); // Interact with the BMP280 sensor. &I2CBMP tells the bmp object to use the custom I2C bus
// Altimeter Calibration
#define SEALEVELPRESSURE_HPA (1017)
unsigned long delayTime;

// Oximeter sensor variables

MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255
uint32_t irBuffer[100]; // store infrared LED sensor data
uint32_t redBuffer[100];  //store red LED sensor data
int32_t bufferLength; //data length
int32_t spo2; //store calculated SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //store calculated heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
byte pulseLED = 11; // control LED indicator for visual feedback (not necessary in our case). Must be on PWM pin
byte readLED = 13; // control LED indicator for visual feedback (not necessary in our case). Blinks with each data read

// Function to read BMP280 sensor
void readBMP280() {
  // Temperature ("Cº")
  int Tmp = bmp.readTemperature();
  Serial.print("Temperature: ");
  Serial.println(Tmp);
  Serial.println("Cº");

  // Pressure ("hPa")
  Serial.print("Pressure: ");
  Serial.println(bmp.readPressure() / 100.0F);
  Serial.println(" nPa");
  
  // Altitude ("m")
  Serial.print("Altitude: ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
}

// Function to read ECG
void ECG() {
  char characterReceived = 's'; //s -> start 
  do 
  {
    if (analogRead(ADC1_CHANNEL_7) >= 0 & analogRead(ADC1_CHANNEL_7) < 4096) {
      Serial.print(">");          // This delete 
      Serial.print("HeartRate:"); // This delete
      Serial.println(analogRead(ADC1_CHANNEL_7));
      delay(10);
    }
    
    if (Serial.available() > 0) {
      characterReceived = Serial.read();
    }
    //Serial.println(characterReceived);
  } while (characterReceived != 'd'); //d -> end to finalise the loop. 
}

// Function to read Oximeter sensor
void readOximeter() {
  // Read the Oximeter sensor and send the data to serial monitor
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  char characterReceived = 'o'; //o -> oxymeter start
  while (characterReceived != 'e') //e -> oxymeter end
  {  if (Serial.available() > 0) {
      characterReceived = Serial.read();
    }
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }
    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}

void setup() {
  Serial.begin(115200); // Usually 9600, 115200 for plotting ECG
  delay (5000);
  Serial.println(F("Setup Sensors")); // Print to know that we are initializing the sensors
  
  // Setup for BMP280 sensor
  // I2C object for BMP280 sensor begin. Initialized pins I2C.
  I2CBMP.begin(I2C_SDA, I2C_SCL, 100000); // 100000 for BMP280
  // Check if the sensor is connected 
  unsigned status;
  // Try initialize the BMP280 sensor. 0x76 I2C address of the sensor,. 0x60 is the Chip ID -> identifies the sensor.
  status = bmp.begin(0x76, 0x60);//ADDRESS, CHIP_ID. Corresponding with line 187 de Adafruit_BMP280.h   
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  Serial.println("BMP280 Sensor OK");

  // Setup for EGC sensor
  analogReadResolution(12); // Establece la resolución a 12 bits
  adcAttachPin(ADC1_CHANNEL_7); // Pin 7 associated with ADC1

  // Setup for Oximeter sensor
  Serial.println(I2C_SPEED_FAST); // Print the clock spee of the I2C
  pinMode(pulseLED, OUTPUT); // This is not really necessary
  pinMode(readLED, OUTPUT); // This is not really necessary 
  // Initialize Oximeter sensor
  //Wire.begin(39,37); // SDA connected to pin 39. SCL connected to pin 37. 
  Serial.println("Scanning for I2C devices");

  if (!particleSensor.begin(I2CBMP, I2C_SPEED_STANDARD, 0x57)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  Serial.println("Oximeter initalized successfully");
  
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  
}

void loop() { 
  if (Serial.available() > 0) {
    char characterReceived = Serial.read(); // Read the character received
    Serial.print("Received: "); 
    Serial.println(characterReceived); // Prints the character received

    if (characterReceived == 'b') {
      readBMP280(); //Read de BMP280 sensor and sent to serial monitor 
      Serial.println("BMP280 Read");
    } else if (characterReceived == 'h') {
      ECG();  // Start de EGC read and sent the data to serial monitor
      Serial.println("ECG Done");
    } else if (characterReceived == 'o') {
      readOximeter();  // Read the Oximeter sensor and sent the data to serial monitor
      Serial.println("Oximeter Read");
    }
  }
}



