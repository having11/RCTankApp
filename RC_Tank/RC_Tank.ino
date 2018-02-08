// Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
//
// This code shows how to listen to the GPS module via polling. Best used with
// Feathers or Flora where you have hardware Serial and no interrupt
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
// 
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada
     
#include <Adafruit_GPS.h>
#include <Servo.h>

Servo pan;

// what's the name of the hardware serial port?
#define GPSSerial Serial1
#define BT Serial3

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

int8_t pins[7] = {38,37,35,36,16,17,23};
int8_t pingPin = 16;
int8_t echoPin = 17;
     
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

bool automatic = false;
const int safe_distance = 18; //How many cm away is safe

uint32_t timer = millis();


void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  BT.begin(9600);
  Serial.println("Adafruit GPS library basic test!");
  for(int i=0;i<4;i++){
    pinMode(pins[i],OUTPUT);
  }
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pan.attach(pins[6]);
  pan.write(90);
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

void motors(int m1Val, int m2Val){ //0: stop, 1: forward, 2: reverse
  switch(m1Val){
    case 0:
      analogWrite(pins[0], 0);
      analogWrite(pins[1], 0);
      break;
    case 1:
      analogWrite(pins[0], 150);
      analogWrite(pins[1], 0);
      break;
    case 2:
      analogWrite(pins[0], 0);
      analogWrite(pins[1], 150);
      break;
  }
  switch(m2Val){
    case 0:
      analogWrite(pins[2], 0);
      analogWrite(pins[3], 0);
      break;
    case 1:
      analogWrite(pins[2], 150);
      analogWrite(pins[3], 0);
      break;
    case 2:
      analogWrite(pins[2], 0);
      analogWrite(pins[3], 150);
      break;
  }
}

void loop() // run over and over again
{
  if(automatic){
    motors(1,1);
    delay(10);
    int distance = ping();
    if (distance==0) distance = safe_distance + 2; 
    if(distance < safe_distance){
      Serial.println(distance);
      motors(0,0);
      motors(2,2);
      delay(400);
      motors(0,0);
      pan.write(30);
      delay(500);
      int distance_r = ping();
      pan.write(150);
      delay(500);
      int distance_l = ping();
      pan.write(90);
      if(distance_r > safe_distance|| distance_l>safe_distance){
      if(distance_r < distance_l){
        motors(1,2);
        delay(200);
        motors(0,0);
      }
      else if(distance_l < distance_r){
        motors(2,1);
        delay(200);
        motors(0,0);
      }
      }
      else{
        motors(2,2);
        delay(300);
        motors(0,0);
      }
    }
  }
  if(BT.available()>0){
    int cmd = BT.parseInt();
    Serial.println(cmd);
    switch(cmd){
      case 1: //Forward
        motors(1,1);
        delay(400);
        motors(0,0);
        break;
      case 2: //Reverse
        motors(2,2);
        delay(400);
        motors(0,0);
        break;
      case 3: //Right
        motors(2,1);
        delay(400);
        motors(0,0);
        break;
      case 4: //Left
        motors(1,2);
        delay(400);
        motors(0,0);
        break;
      case 5:
        automatic = false;
        motors(0,0);
        break;
      case 6:
        automatic = true;
    }
  }
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
     
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    print_gps();
  }
    
}

void print_gps(){
  timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude/100, 5); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude/100, 5); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
    /*BT.print("\nTime: ");
    BT.print(GPS.hour, DEC); BT.print(':');
    BT.print(GPS.minute, DEC); BT.print(':');
    BT.print(GPS.seconds, DEC); BT.print('.');
    BT.println(GPS.milliseconds);
    BT.print("Date: ");
    BT.print(GPS.day, DEC); BT.print('/');
    BT.print(GPS.month, DEC); BT.print("/20");
    BT.println(GPS.year, DEC);
    BT.print("Fix: "); BT.print((int)GPS.fix);
    BT.print(" quality: "); BT.println((int)GPS.fixquality);
    if (GPS.fix) {
      BT.print("Location: ");
      BT.print(GPS.latitude/100, 5); BT.print(GPS.lat);
      BT.print(", ");
      BT.print(GPS.longitude/100, 5); BT.println(GPS.lon);
      BT.print("Speed (knots): "); BT.println(GPS.speed);
      BT.print("Angle: "); BT.println(GPS.angle);
      BT.print("Altitude: "); BT.println(GPS.altitude);
      BT.print("Satellites: "); BT.println((int)GPS.satellites);
    }*/
    if(GPS.fix){
      BT.println(String((GPS.latitude/100)+.2752,5)+",-"+String((GPS.longitude/100)+.13715,5));
    }
    
}

int ping(){
  long duration, inches, cm;
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  cm = microsecondsToCentimeters(duration);
  return cm;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

