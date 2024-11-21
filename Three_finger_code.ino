#include "Wire.h"
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor

#define TCAADDR 0x70 // Multiplexer I2C address

#define sensor_1_port 4 // ports on multiplexer
#define sensor_2_port 3
#define sensor_3_port 5


///////////////////////
// NOTE: YOU NEED TO UPDATE THE FINGER MAPPINGS (A few lines down) AND SENSOR MAPPINGS (get_sensor_readings()) BEFORE RUNNING THIS CODE


ADS flexSensor1; // Create object of the ADS class
ADS flexSensor2; // Create object of the ADS class
ADS flexSensor3; // Create object of the ADS class

// Finger Mappings
int Finger3_i = 3;//inside
int Finger3_o = 4;//outside
int Finger3_cw = 2;//clockwise
int Finger3_cc = 5;//counter-clock

int Finger1_i = 7;//inside
int Finger1_o = 9;//outside
int Finger1_cw = 8;//clockwise
int Finger1_cc = 6;//counter-clock

int Finger2_i = 12;//inside
int Finger2_o = 10;//outside
int Finger2_cw = 44;//clockwise
int Finger2_cc = 11;//counter-clock

const int Active = 80; // PWM amount
const int Active_close = 40; // PWM amount when close to goal
int Active_cur_1_x = 0; // update this based on position error
int Active_cur_1_y = 0; // update this based on position error
int Active_cur_2_x = 0; // update this based on position error
int Active_cur_2_y = 0; // update this based on position error
int Active_cur_3_x = 0; // update this based on position error
int Active_cur_3_y = 0; // update this based on position error
const int close_bound = 3; // degrees to be considered close

float x_1_new;
float y_1_new;
float x_2_new;
float y_2_new;
float x_3_new;
float y_3_new;

float x_1_filt;
float x_2_filt;
float x_3_filt;

float x_1_filt_prev = 0;
float x_2_filt_prev = 0;
float x_3_filt_prev = 0;

float v_1 = 0;
float v_2 = 0;
float v_3 = 0;

float v_1_prev = 0;
float v_2_prev = 0;
float v_3_prev = 0;

float a_1 = 0;
float a_2 = 0;
float a_3 = 0;

bool contact_1 = false;
bool contact_2 = false;
bool contact_3 = false;

bool target_updated_1 = false;
bool target_updated_2 = false;
bool target_updated_3 = false;




float target_1_x = -30;
float target_1_y = 0;
float target_2_x = -30;
float target_2_y = 0;
float target_3_x = -30;
float target_3_y = 0;
int targets_reached = 0; // zero initialized

float x_1_err = 0;
float y_1_err = 0;
float x_2_err = 0;
float y_2_err = 0;
float x_3_err = 0;
float y_3_err = 0;


// FOR Gaussian filter

const int numPoints = 15;       // Number of points for the Gaussian mean
float sigma = 3.0; 
float data_x1[numPoints] = {0};    // Array to store the last 15 data points
float data_x2[numPoints] = {0};    // Array to store the last 15 data points
float data_x3[numPoints] = {0};    // Array to store the last 15 data points
float gaussianWeights[numPoints];  // Array to store Gaussian weights
int currentIndex = 0;           // Index to keep track of the most recent data point



void setup() {

    Wire.begin();
    Serial.begin(2000000);
    // Serial.println("Starting");
    initialize_sensors();
    initialize_actuators();
    compute_gaussian_weights();

}

void loop() {

  int in = 12;
  int out = -12;
  int left = 30;
  int right = -30;

  // change the waypoints for different motions!
  int pos1[9] = {out, left, out, left, out, left, 0, 0, 0}; // x1, y1, x2, y2, x3, y3, hold time (ms), check for contant [bool], wait time (ms)
  int pos2[9] = {in, left, in, left, in, left, 0, 0, 0}; // x1, y1, x2, y2, x3, y3, hold time (ms), check for contant [bool], wait time (ms)
  int pos3[9] = {in, right, in, right, in, right, 0, 0, 0}; // x1, y1, x2, y2, x3, y3, hold time (ms), check for contant [bool], wait time (ms)
  int pos4[9] = {out, right, out, right, out, right, 0, 0, 15000}; // x1, y1, x2, y2, x3, y3, hold time (ms), check for contant [bool], wait time (ms)


  int num_waypoints = 4;
  int waypoints[num_waypoints] = {pos1, pos2, pos3, pos4};

  for (int i=0; i<num_waypoints; i++){
    targets_reached = 0;
    int* wp = waypoints[i];
    target_1_x = wp[0];
    target_1_y = wp[1];
    target_2_x = wp[2];
    target_2_y = wp[3];
    target_3_x = wp[4];
    target_3_y = wp[5];
    long hold_ms = wp[6];
    bool check_contact = wp[7];
    long wait_ms = wp[8];
    while(targets_reached < 6) {
      sense_and_move();
      calculate_filtered_vals();
      calculate_derivatives();
      // String output = String(x_1_new) + "," + String(y_1_new) + "," + String(x_2_new) + "," + String(y_2_new) + "," + String(x_3_new) + "," + String(y_3_new) + "," + String(targets_reached);
      // Serial.println(output);
      // // String output = String(micros()) + "," + String(x_2_new) + ", " + String(y_2_new) + ", " + String(target_2_x) + ", " + String(target_2_y); // + ", " + String(x_2_filt)+ ", " + String(v_2)+ ", " + String(a_2)+ ", " + String(contact_2);
      // // Serial.print(output);

      if (check_contact) {
        check_for_contact();
        update_targets_for_contact();
      }
    }
    if (hold_ms > 0) { // code to hold position 
      long start = millis();
      while ((millis()-start) < hold_ms) {
        sense_and_move();
        calculate_filtered_vals();
        calculate_derivatives();
        String output = String(micros()) + "," + String(x_2_new) + ", " + String(y_2_new) + ", " + String(target_2_x) + ", " + String(target_2_y); // + ", " + String(x_2_filt)+ ", " + String(v_2)+ ", " + String(a_2)+ ", " + String(contact_2);
        Serial.print(output);
      }
  
    }
    delay(wait_ms);
  }


}

void tcaselect(uint8_t i) { // multiplexer channel switching
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  

  return;
}

void initialize_sensors() {
    tcaselect(sensor_1_port);
    if (flexSensor1.begin() == false) {
      Serial.println(F("Sensor 1 not detected. Check wiring. Freezing..."));
      while (1);
    }

    tcaselect(sensor_2_port);
    if (flexSensor2.begin() == false) {
      Serial.println(F("Sensor 2 not detected. Check wiring. Freezing..."));
      while (1);
    }

    tcaselect(sensor_3_port);
    if (flexSensor3.begin() == false) {
      Serial.println(F("Sensor 3 not detected. Check wiring. Freezing..."));
      while (1);
    }

    return;
}

void initialize_actuators() {
    pinMode(Finger1_o, OUTPUT);
    pinMode(Finger1_i, OUTPUT);
    pinMode(Finger1_cc, OUTPUT);
    pinMode(Finger1_cw, OUTPUT);

    pinMode(Finger2_o, OUTPUT);
    pinMode(Finger2_i, OUTPUT);
    pinMode(Finger2_cc, OUTPUT);
    pinMode(Finger2_cw, OUTPUT);
    
    pinMode(Finger3_o, OUTPUT);
    pinMode(Finger3_i, OUTPUT);
    pinMode(Finger3_cc, OUTPUT);
    pinMode(Finger3_cw, OUTPUT);

    return;
}


void sense_and_move() {
    get_sensor_readings(); // ~3.5 ms

    calculate_errors(); // update position errors // ~ 0.06 ms


    check_target_reached();
    set_movement_speeds(); // ~ 0.06 ms
    
    actuate(); // ~0.33 ms in addition to actuation time
}

void get_sensor_readings() {

  
  tcaselect(sensor_1_port);
  if (flexSensor1.available())
  {
    x_3_new = flexSensor1.getX(); 
    y_3_new = -flexSensor1.getY();
  }

  tcaselect(sensor_2_port);
  if (flexSensor2.available())
  {
    x_1_new = -flexSensor2.getX();
    y_1_new = -flexSensor2.getY();
    }

  tcaselect(sensor_3_port);
  if (flexSensor3.available())
  {
    x_2_new = flexSensor3.getX();
    y_2_new = -flexSensor3.getY();
  }

  return;
}

void calculate_errors() {
  x_1_err = x_1_new - target_1_x;
  y_1_err = y_1_new - target_1_y;
  x_2_err = x_2_new - target_2_x;
  y_2_err = y_2_new - target_2_y;
  x_3_err = x_3_new - target_3_x;
  y_3_err = y_3_new - target_3_y;

  return;
}


void check_target_reached() {
  targets_reached = 0;
  if (abs(x_1_err) < close_bound) {targets_reached++;}
  if (abs(y_1_err) < close_bound) {targets_reached++;}
  if (abs(x_2_err) < close_bound) {targets_reached++;}
  if (abs(y_2_err) < close_bound) {targets_reached++;}
  if (abs(x_3_err) < close_bound) {targets_reached++;}
  if (abs(y_3_err) < close_bound) {targets_reached++;}
  return;

}

void set_movement_speeds() {
    Active_cur_1_x = (abs(x_1_err) < close_bound) ? Active_close : Active;
    Active_cur_1_y = (abs(y_1_err) < close_bound) ? Active_close : Active;
    Active_cur_2_x = (abs(x_2_err) < close_bound) ? Active_close : Active;
    Active_cur_2_y = (abs(y_2_err) < close_bound) ? Active_close : Active;
    Active_cur_3_x = (abs(x_3_err) < close_bound) ? Active_close : Active;
    Active_cur_3_y = (abs(y_3_err) < close_bound) ? Active_close : Active;
    return;
}

void actuate() {
    byte set_1_i = (x_1_new < target_1_x) ? Active_cur_1_x : 0;
    byte set_1_o = (x_1_new > target_1_x) ? Active_cur_1_x : 0;
    byte set_1_cc = (y_1_new > target_1_y) ? Active_cur_1_y : 0;
    byte set_1_cw = (y_1_new < target_1_y) ? Active_cur_1_y : 0;

    byte set_2_i = (x_2_new < target_2_x) ? Active_cur_2_x : 0;
    byte set_2_o = (x_2_new > target_2_x) ? Active_cur_2_x : 0;
    byte set_2_cc = (y_2_new > target_2_y) ? Active_cur_2_y : 0;
    byte set_2_cw = (y_2_new < target_2_y) ? Active_cur_2_y : 0;

    byte set_3_i = (x_3_new < target_3_x) ? Active_cur_3_x : 0;
    byte set_3_o = (x_3_new > target_3_x) ? Active_cur_3_x : 0;
    byte set_3_cc = (y_3_new > target_3_y) ? Active_cur_3_y : 0;
    byte set_3_cw = (y_3_new < target_3_y) ? Active_cur_3_y : 0;




    analogWrite(Finger1_i, set_1_i);
    analogWrite(Finger1_o, set_1_o);
    analogWrite(Finger1_cc, set_1_cc);
    analogWrite(Finger1_cw, set_1_cw);
    analogWrite(Finger2_i, set_2_i);
    analogWrite(Finger2_o, set_2_o);
    analogWrite(Finger2_cc, set_2_cc);
    analogWrite(Finger2_cw, set_2_cw);
    analogWrite(Finger3_i, set_3_i);
    analogWrite(Finger3_o, set_3_o);
    analogWrite(Finger3_cc, set_3_cc);
    analogWrite(Finger3_cw, set_3_cw);
    delay(50);
    analogWrite(Finger1_i, 0);
    analogWrite(Finger1_o, 0);
    analogWrite(Finger1_cc, 0);
    analogWrite(Finger1_cw, 0);
    analogWrite(Finger2_i, 0);
    analogWrite(Finger2_o, 0);
    analogWrite(Finger2_cc, 0);
    analogWrite(Finger2_cw, 0);
    analogWrite(Finger3_i, 0);
    analogWrite(Finger3_o, 0);
    analogWrite(Finger3_cc, 0);
    analogWrite(Finger3_cw, 0); 
}


void print_data() {
    String output = "x1:" + String(x_1_new) + ", " +
                    "y1:" + String(y_1_new) + ", " +
                    "x2:" + String(x_2_new) + ", " +
                    "y2:" + String(y_2_new) + ", " +
                    "x3:" + String(x_3_new) + ", " +
                    "y3:" + String(y_3_new) + ", ";
                    // "T_1x:" + String(target_1_x) + ", " +
                    // "T_1y:" + String(target_1_y) + ", " +
                    // "T_2x:" + String(target_2_x) + ", " +
                    // "T_2y:" + String(target_2_y) + ", " +
                    // "T_3x:" + String(target_3_x) + ", " +
                    // "T_3y:" + String(target_3_y) + ", ";
                    // "x1_d:" + String(x_1_deriv) + ", " +
                    // "y1_d:" + String(y_1_deriv) + ", " +
                    // "x2_d:" + String(x_2_deriv) + ", " +
                    // "y2_d:" + String(y_2_deriv) + ", ";
                    // "x3_d:" + String(x_3_deriv) + ", " +
                    // "y3_d:" + String(y_3_deriv);
    Serial.println(output);
}


void compute_gaussian_weights() {
  float sum = 0.0;

  for (int i = 0; i < numPoints; i++) {
    float x = (float)(i - (numPoints - 1) / 2);
    gaussianWeights[i] = exp(-0.5 * sq(x / sigma));
    sum += gaussianWeights[i];
  }

  // Normalize the weights so that they sum to 1
  for (int i = 0; i < numPoints; i++) {
    gaussianWeights[i] /= sum;
  }
}

// Update the data buffer with new data point
void updateBuffers() {
  data_x1[currentIndex] = x_1_new;
  data_x2[currentIndex] = x_2_new;
  data_x3[currentIndex] = x_3_new;
  currentIndex = (currentIndex + 1) % numPoints;

}

void calculate_filtered_vals() {
  updateBuffers();
  x_1_filt = 0.0;
  x_2_filt = 0.0;
  x_3_filt = 0.0;

  for (int i = 0; i < numPoints; i++) {
    int index = (currentIndex + i) % numPoints;
    x_1_filt += data_x1[index] * gaussianWeights[i];
    x_2_filt += data_x2[index] * gaussianWeights[i];
    x_3_filt += data_x3[index] * gaussianWeights[i];
  }


}

void calculate_derivatives() {

  float d_time = 0.055; // approx time between readings in seconds

  v_1 = (x_1_filt - x_1_filt_prev) / d_time;
  v_2 = (x_2_filt - x_2_filt_prev) / d_time;
  v_3 = (x_3_filt - x_3_filt_prev) / d_time;

  a_1 = (v_1 - v_1_prev) / d_time;
  a_2 = (v_2 - v_2_prev) / d_time;
  a_3 = (v_3 - v_3_prev) / d_time;


  x_1_filt_prev = x_1_filt;
  x_2_filt_prev = x_2_filt;
  x_3_filt_prev = x_3_filt;
  
  v_1_prev = v_1;
  v_2_prev = v_2;
  v_3_prev = v_3;
}

void check_for_contact() {
  float accel_contact_thresh = -10;
  float pos_contact_thresh = 0;
  if (a_1 < accel_contact_thresh && x_1_filt > pos_contact_thresh) {
    contact_1 = true;
    // Serial.println("CONTACT 1");
  }
  else {contact_1 = false;}
  if (a_2 < accel_contact_thresh && x_2_filt > pos_contact_thresh) {
    contact_2 = true;
    // Serial.println("CONTACT 2");
  }
  else {contact_2 = false;}
  if (a_3 < accel_contact_thresh && x_3_filt > pos_contact_thresh) {
    contact_3 = true;
    // Serial.println("CONTACT 3");
  }
  else {contact_3 = false;}
}

void update_targets_for_contact() {
  if (contact_1 && !target_updated_1) {
    target_1_x = x_1_new+10;
    target_updated_1 = true;
    Serial.println("new target_1_x = " + String(target_1_x));
  }
  if (contact_2 && !target_updated_2) {
    target_2_x = x_2_new+10;
    target_updated_2 = true;
    Serial.println("new target_2_x = " + String(target_2_x));

  }
  if (contact_3 && !target_updated_3) {
    target_3_x = x_3_new+10;
    target_updated_3 = true;
    Serial.println("new target_3_x = " + String(target_3_x));

  }
}
