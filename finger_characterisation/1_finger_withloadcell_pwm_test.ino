#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor
#include "Adafruit_HX711.h"

// define pins for finger directions
// NOTE: ONLY ELECTRICALLY CONNECT ONE FINGER 
int Finger1_o = 7; //inside
int Finger1_cc = 8; //outside
int Finger1_cw = 9; //clockwise
int Finger1_i = 10; //counter-clock

const int num_Actives = 9;
int Active_cur_idx = 0;
int Actives_to_test[num_Actives] = {25, 38, 51, 64, 76, 89, 102, 115, 127}; // duty cycle = {0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5}
int Active = Actives_to_test[Active_cur_idx]; // PWM amount

// store incoming data
float y_1_new = 0;
float x_1_new = 0;

ADS flexSensor1; // Create object of the ADS class
byte deviceType; // hope this helps bc sensor starter code is horrible

// timing data collection
uint32_t start_time;
uint32_t cur_time;
uint32_t duration = 120*1000000; // duration of data collection in microseconds 

// for load cell
// Define the pins for the HX711 communication
const uint8_t DATA_PIN = 2; 
const uint8_t CLOCK_PIN = 3; 
Adafruit_HX711 hx711(DATA_PIN, CLOCK_PIN);
long one_kg_weight = 1016323;


void setup() {
  Serial.begin(2000000);

  Wire.begin();

  initialize_sensors();
  initialize_actuators();
  initialize_loadcell();

  start_time = micros();
  Serial.println("time(us),power(pwm/255),position(deg),force(grams)");

}

void loop() {
  cur_time = micros();
  if (flexSensor1.available() == true) {
    y_1_new = flexSensor1.getY();
    x_1_new = flexSensor1.getX();
  }
  else {
    Serial.println("Sensor not connected!");
    delay(1000);
    start_time = micros();
    return;
  }
  if((start_time + duration) > cur_time && x_1_new < target_x && x_1_new < target_x + 5) {
    actuate(Active, 50000); // actuation time in microseconds
    int32_t weightA128 = hx711.readChannelBlocking(CHAN_A_GAIN_128); // read load cell
    float weight_grams = float(weightA128) / float(one_kg_weight) * 1000; // convrt load cell reading to grams

    String output = String(cur_time) + "," + String(Active) + "," + String(x_1_new) + "," + String(y_1_new) + "," + String(weight_grams);
    Serial.println(output);
  }
  else {
    delay(60000);
    start_time = micros();
    Active_cur_idx++;
    if (Active_cur_idx >= num_Actives) {delay(100000000000);}
    else {Active = Actives_to_test[Active_cur_idx];}
  }




}

void initialize_actuators() {
    pinMode(Finger1_o, OUTPUT);
    pinMode(Finger1_i, OUTPUT);
    pinMode(Finger1_cc, OUTPUT);
    pinMode(Finger1_cw, OUTPUT);

    return;
}

void initialize_sensors() {
  if (flexSensor1.begin() == false) {
      Serial.println(F("Sensor 1 not detected. Check wiring. Freezing..."));
      while (1);
    }
  deviceType = flexSensor1.getDeviceType();
  if (deviceType == ADS_ONE_AXIS)
    Serial.println(F("One axis displacement sensor detected"));
  else if (deviceType == ADS_TWO_AXIS)
    Serial.println(F("Two axis displacement sensor detected"));
}

void initialize_loadcell() {
  // Initialize the HX711
  hx711.begin();

  // read and toss 3 values each
  Serial.println("Tareing....");
  for (uint8_t t=0; t<3; t++) {
    hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
    hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
  }
}

void actuate(int pwm, float duration) {
    long start_time = micros();
    while ((micros() - duration) < start_time){
      analogWrite(Finger1_i, pwm);
      analogWrite(Finger1_o, pwm);
      analogWrite(Finger1_cc, pwm);
      analogWrite(Finger1_cw, pwm);
    }
    analogWrite(Finger1_i, 0);
    analogWrite(Finger1_o, 0);
    analogWrite(Finger1_cc, 0);
    analogWrite(Finger1_cw, 0);
}
