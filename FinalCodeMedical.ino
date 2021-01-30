  //  HealthConnect device 
  // Developed by Manivannan S for HealthConnect device 
  // Github : https://github.com/Manivannan-maker?tab=repositories
  
  
  #include <Wire.h>
  #include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Nano  doesn't have enough SRAM to store 75 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[75]; //infrared LED sensor data
uint16_t redBuffer[75];  //red LED sensor data
#else
uint32_t irBuffer[75]; //infrared LED sensor data
uint32_t redBuffer[75];  //red LED sensor data
#endif

//pulse sensor MAX30102
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

boolean startup=1;

//Body temperature
  const int sensor=A1; // Assigning analog pin A1 to variable 'sensor' 

  const byte interruptPin = 3;

  //SOS
  byte SOSEnable=0;
  byte startupSOS=1;
  //float tempc; //variable to store temperature in degree Celsius
  float tempf;
  float vout; //temporary variable to hold sensor reading

  //Active status
  float Yout=0;
  float YoutPrev=0;
  const int YAxis=A2; // Assigning analog pin A1 to variable 'sensor' 

  
  byte bodyTemperature=0;
  byte active=0;
  byte heartRatedata=70;
  byte SpO2=90;
  byte I2CData[4]={0};
  void setup() {
Wire.begin(0x07);                // join i2c bus with address #7
    Wire.onRequest(requestEvent); // register event
    
    pinMode(sensor,INPUT); // Configuring sensor pin as input
    
    pinMode(YAxis,INPUT); // Configuring sensor pin as input
    
    
Serial.begin(9600); // initialize serial communication at 115200 bits per second:

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

   //I2C in 0 bus

    TCA9548A(0);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  //////Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
 // while (Serial.available() == 0) ; //wait until user presses a key
 // Serial.read();
 
delay(2000);
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
Wire.begin(0x07);                // join i2c bus with address #7
    Wire.onRequest(requestEvent); // register event
 
delay(2000);

  pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), SOS, FALLING);
    
  }
  
  void loop() {

    //I2C in 0 bus
    TCA9548A(0);
    
if(startup==1)
{
 bufferLength = 75; //buffer length of 75 stores 4 seconds of samples running at 25sps

  //read the first 75 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
    delay(10);
 
  }

  //calculate heart rate and SpO2 after first 75 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  if(validSPO2==1)
    {
    SpO2=abs(spo2);
    }
    else
    {
     SpO2=0;      
 
    }

    if(validHeartRate==1)
    {
          heartRatedata=abs(heartRate);
              ////Serial.println("HR samples completed ");
        ////Serial.println(heartRatedata,DEC);


    }
    else
    {
      heartRatedata=0;
    }
    startup=0; // startup call ends

}


    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 75; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 50; i < 75; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
      delay(10);

   
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    if(validSPO2==1)
    {
    SpO2=abs(spo2);
    
    }
    else
    {
     SpO2=0;      
    }

    if(validHeartRate==1)
    {
      
      
          heartRatedata=abs(heartRate);
   
    }
    else
    {
      heartRatedata=0;
    }

    ////Serial.println("25 samples completed");
    // Body temperature measurement
    vout=analogRead(sensor); //Reading the value from sensor
  vout=(vout*500)/1023;
 
  tempf=(vout*1.8)+32; // Converting to Fahrenheit

  bodyTemperature=abs(tempf);


  //Y-Axis Status
  
  Yout=analogRead(YAxis); //Reading the value from sensor
  Yout=(Yout*500)/1023;
  if(Yout>YoutPrev+50 || Yout<YoutPrev-50)
  {
active=1;
YoutPrev=Yout;
    
  }
  else
  {
    active=0;
    YoutPrev=Yout;

  }
 
  if(SOSEnable==1)
  {

    Serial.write("1 \n");
    delay(100);    
  
SOSEnable=0; // disable it for next interrupt
  }

    //I2C in 1 bus

    TCA9548A(1);
    
    delay(10000);

  //I2C in 0 bus

    TCA9548A(0);
    
  }
  
  // function that executes whenever data is requested by master
  // this function is registered as an event, see setup()
  void requestEvent() {
  
    
    I2CData[0]=bodyTemperature;
    I2CData[1]=active;
    I2CData[2]=heartRatedata;
    I2CData[3]=SpO2;
  
    
    Wire.write(I2CData,4); // respond with message of 4 bytes
    // as expected by master
  }



  // SOS call

  
void SOS() {
  if(startupSOS==1)
  {
    SOSEnable=0;
    startupSOS=0;

  }
  else
  {
SOSEnable=1;
  }
}





//I2c multiplexer
void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}
