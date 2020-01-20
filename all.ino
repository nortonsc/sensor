#include <dht.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "GravityTDS.h"
#include <avr/pgmspace.h>

#define ONE_WIRE_BUS 2
#define DHT11_PIN 7

#define TdsSensorPin A2
GravityTDS gravityTds;

#define DoSensorPin  A1    //dissolved oxygen sensor analog output pin to arduino mainboard
#define VREF 5000    //for arduino uno, the ADC reference is the AVCC, that is 5000mV(TYP)
float doValue;      //current dissolved oxygen value, unit; mg/L
float temperature = 25;    //default temperature is 25^C, you can use a temperature sensor to read it


#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    //store the analog value in the array, readed from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;

float SaturationDoVoltage,SaturationDoTemperature;
float averageVoltage;

const float SaturationValueTab[41] PROGMEM = {      //saturation dissolved oxygen concentrations at various temperatures
14.46, 14.22, 13.82, 13.44, 13.09,
12.74, 12.42, 12.11, 11.81, 11.53,
11.26, 11.01, 10.77, 10.53, 10.30,
10.08, 9.86,  9.66,  9.46,  9.27,
9.08,  8.90,  8.73,  8.57,  8.41,
8.25,  8.11,  7.96,  7.82,  7.69,
7.56,  7.43,  7.30,  7.18,  7.07,
6.95,  6.84,  6.73,  6.63,  6.53,
6.41,
};

byte sensorInterrupt = 1;  // 0 = digital pin 3
byte waterSensorPin       = 3;

// The hall-effect flow sensor outputs approximately 4.5 pulses per second per
// litre/minute of flow.
float calibrationFactor = 4.5;

volatile byte pulseCount;  

float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;

unsigned long oldTime;


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
dht DHT;

int sensorPin = A5;
float volt;
float ntu;
float temperaturee = 25,tdsValue = 0;
int ecPin = A4; //The analog input pin used by the EC sensor 
int phPin = A3; //Thw analog input pin used by the PH sensor 

//Variables to store the processed values 
float finalPhValue = 0; 
float finalEcValue = 0; 

//Define variables used in methods here 
int rawPhReading = 0, rawEcReading = 0; 
char dat[10];
char sensor1[10],sensor2[10],sensor3[10],sensor4[10],sensor5[10],sensor6[10],sensor7[10],sensor8[10],sensor9[10];
String data;
int len;
char stat;

void receiveEvent(int howMany) {
 while (0 <Wire.available()) {
    stat = Wire.read();      /* receive byte as a character */
    Serial.print(stat);           /* print the character */
  }
 Serial.println();             /* to newline */
}

// function that executes whenever data is requested from master
void requestEvent() {
  if(stat == '1'){
    Wire.write(sensor1);
    }else if(stat == '2'){
      Wire.write(sensor2);
      }else if(stat == '3'){
        Wire.write(sensor3);
        }else if(stat == '4'){
          Wire.write(sensor4);
          }else if(stat == '5'){
            Wire.write(sensor5);
            }else if(stat == '6'){
              Wire.write(sensor6);
              }else if(stat == '7'){
                Wire.write(sensor7);
                }else if(stat == '8'){
                  Wire.write(sensor8);
                  }else if(stat == '9'){
                  Wire.write(sensor9);
                  }else{}
  
}
void setup(){
  Serial.begin(9600);
  Wire.begin(8);
  Wire.onReceive(receiveEvent); /* register receive event */
   Wire.onRequest(requestEvent); /* register request event */
  sensors.begin();
  gravityTds.setPin(TdsSensorPin);
    gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
    gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
    gravityTds.begin();  //initialization
    readDoCharacteristicValues();
     pinMode(DoSensorPin,INPUT);
     pinMode(waterSensorPin, INPUT);
  digitalWrite(waterSensorPin, HIGH);

  pulseCount        = 0;
  flowRate          = 0.0;
  flowMilliLitres   = 0;
  totalMilliLitres  = 0;
  oldTime           = 0;
   attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
}

void loop()
{
  ///////////////////////////////////////////////////////////////
  int chk = DHT.read11(DHT11_PIN);
  Serial.println("Sensor 1 Temp/Hum");
  Serial.print("Temperature = ");
  Serial.println(DHT.temperature);
  Serial.print("Humidity = ");
  Serial.println(DHT.humidity);
  data = String(DHT.temperature);
  len = data.length()+1;
  data.toCharArray(sensor1, len);
  data = String(DHT.humidity);
  len = data.length()+1;
  data.toCharArray(sensor2, len);
  
  /////////////////////////////////////////////////////////////////////////
  Serial.println("Sensor 2");
   volt = 0;
    for(int i=0; i<800; i++)
    {
        volt += ((float)analogRead(sensorPin)/1023)*5;
    }
    volt = volt/800;
    volt = round_to_dp(volt,1);
    if(volt < 2.5){
      ntu = 3000;
    }else{
      ntu = -1120.4*square(volt)+5742.3*volt-4353.8; 
    }
    Serial.print("Volts = ");
    Serial.println(volt);
    Serial.print("NTU = ");
    Serial.println(ntu);
    data = String(ntu);
    len = data.length()+1;
     data.toCharArray(sensor3, len);
    /////////////////////////////////////////////////
    Serial.println("Sensor 3");
    sensors.requestTemperatures();
    Serial.print("Temperature for DS is: ");
    Serial.println(sensors.getTempCByIndex(0));
    data = String(sensors.getTempCByIndex(0));
    len = data.length()+1;
    data.toCharArray(sensor4, len);
    
    ///////////////////////////////////////////////////////////////////////
    Serial.println("Sensor 4 Total Dissolved Solids");
    gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
    gravityTds.update();  //sample and calculate
    tdsValue = gravityTds.getTdsValue();  // then get the value
    Serial.print(tdsValue,0);
    Serial.println("ppm");
    data = String(tdsValue);
    len = data.length()+1;
   data.toCharArray(sensor5, len);
    ////////////////////////////////////////////////////////////
    Serial.println("Sensor 5 PH/EC");
    rawPhReading = analogRead(phPin);
    finalPhValue = rawPhReading*14.00/1024;
    rawEcReading = analogRead(ecPin);
    finalEcValue = rawEcReading*3200.00 /1024; 
    Serial.print("ph = ");
    Serial.println(finalPhValue);
    Serial.print("ec = ");
    Serial.println(finalEcValue);
    data = String(finalPhValue);
    len = data.length()+1;
   data.toCharArray(sensor6, len);
   data = String(finalEcValue);
    len = data.length()+1;
   data.toCharArray(sensor7, len);
    ////////////////////////////////////////////////
    Serial.println("Sensor 6 Disolved Oxygen");
    static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 30U)     //every 30 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(DoSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT)
         analogBufferIndex = 0;
   }

   static unsigned long tempSampleTimepoint = millis();
   if(millis()-tempSampleTimepoint > 500U)  // every 500 milliseconds, read the temperature
   {
      tempSampleTimepoint = millis();
      //temperature = readTemperature();  // add your temperature codes here to read the temperature, unit:^C
   }

   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 1000U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
      {
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      }
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the value more stable by the median filtering algorithm
      Serial.print(F("Temperature:"));
      Serial.print(temperature,1);
      Serial.print(F("^C"));
      doValue = pgm_read_float_near( &SaturationValueTab[0] + (int)(SaturationDoTemperature+0.5) ) * averageVoltage / SaturationDoVoltage;  //calculate the do value, doValue = Voltage / SaturationDoVoltage * SaturationDoValue(with temperature compensation)
      Serial.print(F(",  DO Value:"));
      Serial.print(doValue,2);
      Serial.println(F("mg/L"));
      data = String(doValue);
      len = data.length()+1;
      data.toCharArray(sensor8, len);
   }
   //////////////////////////////////////////////////////////
   while((millis() - oldTime < 1000)){}
   oldTime = millis();
   Serial.println("Sensor 7 Flow Rate");
   detachInterrupt(sensorInterrupt);
   flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
   flowMilliLitres = (flowRate / 60) * 1000;
   totalMilliLitres += flowMilliLitres;
unsigned int frac;
    
    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate: ");
    Serial.print(int(flowRate));  // Print the integer part of the variable
    Serial.print("L/min");
    Serial.print("\t");       // Print tab space

    // Print the cumulative total of litres flowed since starting
    Serial.print("Output Liquid Quantity: ");        
    Serial.print(totalMilliLitres);
    Serial.println("mL"); 
    Serial.print("\t");       // Print tab space
  Serial.print(totalMilliLitres/1000);
  Serial.print("L");
  data = String(totalMilliLitres);
    len = data.length()+1;
   data.toCharArray(sensor8, len);
    

    // Reset the pulse counter so we can start incrementing again
    pulseCount = 0;
  attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
  delay(5000);
  
}
void pulseCounter()
{
  // Increment the pulse counter
  pulseCount++;
}
float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}
int getMedianNum(int bArray[], int iFilterLen)
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      {
      bTab[i] = bArray[i];
      }
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++)
      {
      for (i = 0; i < iFilterLen - j - 1; i++)
          {
        if (bTab[i] > bTab[i + 1])
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void readDoCharacteristicValues(void)
{

      SaturationDoVoltage = 1127.6;   //default voltage:1127.6mv
      SaturationDoTemperature = 25.0;   //default temperature is 25^C

}
