#include <XBee.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>

// Create XBee object for data transmission
XBee xbee = XBee();

// Define BMP085 daughter board
Adafruit_BMP085 bmp;

// Pin definitions
#define ANEMOMETER_PIN 2
#define ANEMOMETER_INT 0
#define RAINFALL_PIN 3
#define RAINFALL_INT 1
#define VANE_PIN A0
#define VANE_POWER 4
#define STATUS_LED 13
#define ERROR_LED 13

// Global consant definitions
#define TEST_PERIOD 60000        // time to measure wind speed
#define WIND_RATE 0.746          // one-click per second windspeed
#define RAIN_FACTOR 0.011        // inches of rain per switch activation
#define DEFAULT_SEALEVEL_HPA 101600 // default value to use for sea level mean pressure
// Payload to send to brain
uint8_t payload[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// SH + SL Address of receiving XBee
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x4076456C);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

/* vaneValues are voltage readings from the input pin.
   vaneDirections correspond to the voltage readings.
   Note that vaneDirections do not necessarily reflect actual
   cardinal directions, and calibration may be needed to 
   match vaneDirection values with true compass directions.
*/
static int vaneValues[] = {66,84,92,127,184,244,287,406,461,600,631,702,786,827,889,946};
static int vaneDirections[] = {1125,675,900,1575,1350,2025,1800,225,450,2475,2250,3375,0,2925,3150,2700};

// Global variable definitions
volatile int wind_counter = 0;
volatile int rain_counter = 0;
volatile unsigned long rain_last = 0;
volatile unsigned long anem_last = 0;
volatile unsigned long anem_min = 0xffffffff;

volatile unsigned int windSpeed = 0;
volatile unsigned int windGust = 0;
volatile unsigned int windDirection = 0;
volatile unsigned int rain = 0;
volatile unsigned int temp = 0;
volatile unsigned int humidity = 0;
volatile unsigned int baroPressure = 0;
volatile unsigned int altitude = 0;
volatile float seaLevelPressure = DEFAULT_SEALEVEL_HPA;

void setup() {
  // Set pin states and enable 20k pull-up resistors
  pinMode(ANEMOMETER_PIN, INPUT);
  digitalWrite(ANEMOMETER_PIN, HIGH);
  pinMode(RAINFALL_PIN, INPUT);
  digitalWrite(RAINFALL_PIN, HIGH);
  pinMode(VANE_POWER, OUTPUT);
  digitalWrite(VANE_POWER, LOW);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(ERROR_LED, OUTPUT);
  
  attachInterrupt(RAINFALL_INT, rainCupDump, FALLING);
  attachInterrupt(ANEMOMETER_INT, anemometerCount, FALLING);
  
  Serial.begin(9600);
  xbee.setSerial(Serial);
  
  char announcement[42] = "XBee radios are communicating\r\n";
  ZBTxRequest zbTxAnnounce = ZBTxRequest(addr64, (uint8_t*) (announcement), strlen(announcement));
  xbee.send(zbTxAnnounce);
  
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    char bmpError[50] = "No BMP085 detected... barometer unavailable";
  }
}

void loop() {
  /*Serial.println("Weather data...");
  Serial.println();
  Serial.println("Wind Velocity (mph):");
  Serial.println(getWindVelocity());
  Serial.println("Wind Gust (mph): ");
  Serial.println(getWindGust());
  Serial.println("Wind Direction: ");
  Serial.println(getWindDirection());
  Serial.println("Rainfall (inches):");
  Serial.println(getRain());*/
 
  baroPressure = bmp.readPressure();
    
  temp = bmp.readTemperature();
    
  float seaLevelPressure = DEFAULT_SEALEVEL_HPA;
  altitude = bmp.readAltitude(seaLevelPressure);
  
  getWindSpeed();
  getWindGust();
  windDirection = getWindDirection();
  getRain();
  
  // Set payload wind speed bytes
  payload[0] = windSpeed >> 8 & 0xff;
  payload[1] = windSpeed & 0xff;
  
  // Set payload wind gust bytes
  payload[2] = windGust >> 8 & 0xff;
  payload[3] = windGust & 0xff;
  
  // Set payload wind direction bytes
  payload[4] = windDirection >> 8 & 0xff;
  payload[5] = windDirection & 0xff;
  
  // Set payload rainfall bytes
  payload[6] = rain >> 8 & 0xff;
  payload[7] = rain & 0xff;
  
  // Set payload temperature bytes
  payload[8] = temp >> 8 & 0xff;
  payload[9] = temp & 0xff;
  
  // Set payload humidity bytes
  payload[10] = humidity >> 8 & 0xff;
  payload[11] = humidity & 0xff;
  
  // Set payload barometric pressure bytes
  payload[12] = baroPressure >> 8 & 0xff;
  payload[13] = baroPressure & 0xff;
  
  xbee.send(zbTx);


  // flash TX indicator
  flashLed(STATUS_LED, 1, 100);

  // after sending a tx request, we expect a status response
  // wait up to half second for the status response
  if (xbee.readPacket(500)) {
    // got a response!

    // should be a znet tx status            	
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);

      // get the delivery status, the fifth byte
      if (txStatus.getDeliveryStatus() == SUCCESS) {
        // success.  time to celebrate
        flashLed(STATUS_LED, 5, 50);
      } else {
        // the remote XBee did not receive our packet. is it powered on?
        flashLed(ERROR_LED, 3, 500);
      }
    }
  } else if (xbee.getResponse().isError()) {
    //nss.print("Error reading packet.  Error code: ");  
    //nss.println(xbee.getResponse().getErrorCode());
  } else {
    // local XBee did not provide a timely TX Status Response -- should not happen
    flashLed(ERROR_LED, 2, 50);
  }
  *
  // delay 2 minutes
  delay(120000);
}

void getWindSpeed(){
  unsigned long reading = wind_counter;
  wind_counter = 0;
  windSpeed = ((WIND_RATE * reading)/(TEST_PERIOD / 1000)) * 100;
}

void getWindGust(){
  unsigned long reading = anem_min;
  anem_min = 0xffffffff;
  double time = reading / 1000000.0;
  
  windGust = ((1/(time)) * WIND_RATE) * 100;
}

unsigned int getWindDirection(){
  analogReference(DEFAULT);
  digitalWrite(VANE_POWER, HIGH);
  delay(100);
  for(int n=0;n<10;n++)
  {
    analogRead(VANE_PIN);
  }
 
  unsigned int reading=analogRead(VANE_PIN);
  digitalWrite(VANE_POWER,LOW);
  unsigned int lastDiff=2048;
 
  for (int n=0;n<16;n++)
  {
    int diff=reading-vaneValues[n];
    diff=abs(diff);
    if(diff==0)
      return vaneDirections[n];
 
    if(diff>lastDiff)
    {
      return vaneDirections[n-1];
    }
 
    lastDiff=diff;
 }
 
  return vaneDirections[15];
}

void getRain() {
  unsigned long reading=rain_counter;
  rain_counter=0;
 
  rain = (reading * RAIN_FACTOR) * 100;
}

void rainCupDump(){
  long thisTime=micros()-rain_last;
    rain_last=micros();
    if(thisTime>500)
    {
      rain_counter++;
    }
}

void anemometerCount(){
  long thisTime = micros() - anem_last;
  anem_last = micros();
  if (thisTime > 500){
    wind_counter++;
    if (thisTime < anem_min) {
      anem_min = thisTime;
    }
  }
}

void flashLed(int pin, int times, int wait) {

  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);

    if (i + 1 < times) {
      delay(wait);
    }
  }
}
