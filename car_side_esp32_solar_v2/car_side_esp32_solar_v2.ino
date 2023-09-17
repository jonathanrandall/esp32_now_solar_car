

#include <esp_now.h>
#include <WiFi.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */


// RTC_DATA_ATTR int bootCount = 0;
RTC_NOINIT_ATTR int servo_top_persistent;
RTC_NOINIT_ATTR  int servo_bottom_persistent; //these are the persistent servo values

unsigned long t_last_com; //time since last communication

bool first_update = true;

int pwmA_pin = 13; //white
int pwmB_pin = 33; //yellow
int max_spd = 255;
int min_spd = 60;

int rmI1 = 12; //green
int rmI2 = 27; //blue
int lmI1 = 25; // H out1 = H | L out1 = L | H short break | L stop purple
int lmI2 = 26; // L out2 = L | H out2 = H | H short break | L stop brown
int STDBY = 19; //Low is off (current save) //not connected so far

// setting PWM properties

const int freq = 2000;
const int ledChannel_l = 2;
const int ledChannel_r = 3;
const int resolution = 8;

//previous values to check if we need to update.
int rs_prev;
int ls_prev;
int dl_prev;
int dr_prev;

//tracker stuff
SemaphoreHandle_t Semaphore_Controls = NULL;
int counter;

int top_pin = 18; //this is the pin that is held high to get a 3.3v signal at the top of the 2.2k resistor.
                  //alternatively  you can connect the 2.2k to the 3.3v rail, but you won't be able to shut 
                  // down the current (which is small but still non-zero) if you want to put the eps32 to sleep
                  //while you charge the cell.

const int top_servo = 16; // Top servo 
const int bottom_servo = 4; //Bottomservo
const int channel_top = 5;
const int channel_b = 4;
const int servoFrequency = 50; // PWM frequency in Hz
int micros_top = 1650;
int micros_bottom = 1500;

int micros_top_min = 1400;
int micros_bottom_min = 800;
int micros_top_max = 2200;
int micros_bottom_max = 2200;

//analogue inputs for photoresistors
const int photo_rb = 34;
const int photo_rt = 35;
const int photo_lb = 36;
const int photo_lt = 39;

uint16_t val_rb, val_rt, val_lb, val_lt;


typedef struct stuct_message {

//  int vals[5];
//  uint8_t a1;
  uint16_t t1;
  uint16_t t2; //int
//  uint16_t t3;
  uint16_t speedval;
  uint16_t breakval;
//  int a2;
} struct_message;

struct_message control_data;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : 
      Serial.println("Wakeup caused by timer"); 
      micros_top=servo_top_persistent;
      micros_bottom=servo_bottom_persistent;  
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void robot_stop(){
  digitalWrite(rmI1,LOW);
  digitalWrite(rmI2,LOW);
  digitalWrite(lmI1,LOW);
  digitalWrite(lmI2,LOW);
}

void robot_setup(){
  pinMode(rmI1,OUTPUT);
  pinMode(rmI2,OUTPUT);
  pinMode(lmI1,OUTPUT);
  pinMode(lmI2,OUTPUT);
  
  ledcSetup(ledChannel_l, freq, resolution);
  ledcSetup(ledChannel_r, freq, resolution);

  ledcAttachPin(pwmA_pin, ledChannel_l);
  ledcAttachPin(pwmB_pin, ledChannel_r);


}

void serovs_setup(){
  ledcSetup(channel_b, servoFrequency, 16);
  ledcAttachPin(bottom_servo, channel_b);

  // Configure LED Controller Channel 5 for Servo 2
  ledcSetup(channel_top, servoFrequency, 16);
  ledcAttachPin(top_servo, channel_top);
}
void writeServoMicros(int pin, int micros) {
  // Map the desired pulse width (micros) to the PWM range (0-65535)
  int dutyCycle = map(micros, 0, 20000, 0, 65535);

  // Set the PWM duty cycle to move the servo
  ledcWrite(pin, dutyCycle);
}

void left_motor_drive(int spd, int dir){
//  Serial.print("in left motor drive ");
//  Serial.println(spd);
  ls_prev = spd; dl_prev = dir;
  if(dir>0){
    digitalWrite(lmI1, HIGH);
    digitalWrite(lmI2, LOW);
  } else {
    digitalWrite(lmI2, HIGH);
    digitalWrite(lmI1, LOW);
   }

  ledcWrite(ledChannel_l, spd);
}

void right_motor_drive(int spd, int dir){
  rs_prev = spd; dr_prev = dir;
 if(dir>0){
  digitalWrite(rmI1, HIGH);
  digitalWrite(rmI2, LOW);
 } else {
  digitalWrite(rmI2, HIGH);
  digitalWrite(rmI1, LOW);
 }

 ledcWrite(ledChannel_r, spd);
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.println("incoming");
  memcpy(&control_data, incomingData, sizeof(control_data));
  Serial.println(sizeof(control_data));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("xpin = ");
  Serial.println(control_data.t1);
  Serial.print("ypin = ");
  Serial.println(control_data.t2);
  Serial.print("speedval = ");
  Serial.println(control_data.speedval);
  Serial.print("breakval = ");
  Serial.println(control_data.breakval);
  t_last_com  = millis();
  if (control_data.breakval==0) {
    robot_stop();
  } else { //change speed
    update_robot();
  }
} //void on

void update_robot(){
  int dir;
  int fwd_spd;
  int right_spd;
  int left_spd;
  int dir_lft;
  int dir_rgt;
  
  if(control_data.t2>1850){ //bias a little bit forward
    //forward
    dir = 1;
    fwd_spd = map(control_data.t2, 2000, 4095, 0,max_spd+10);
  } else {
    dir = -1;
    fwd_spd = map(control_data.t2, 2000, 0, 0,max_spd+10);    
  }

  dir_lft = dir;
  dir_rgt = dir;
  
  if(control_data.t1>1960){
    //left
    right_spd = fwd_spd + map(control_data.t1, 2000, 4095, 0,max_spd+10);
    left_spd = fwd_spd - map(control_data.t1, 2000, 4095, 0,max_spd+10);
    
  } else {
    right_spd = fwd_spd - map(control_data.t1, 2000, 0, 0,max_spd+10);
    left_spd = fwd_spd + map(control_data.t1, 2000, 0, 0,max_spd+10);
  }
  if(right_spd < 0){
    right_spd = -right_spd;
    dir_rgt = -dir;
  }
  if(left_spd < 0){
    left_spd = -left_spd;
    dir_lft = -dir;
  }
  if(right_spd > max_spd) right_spd = max_spd;
  if(left_spd > max_spd) left_spd = max_spd;

  
  //we need to stop the motor if speed less that certain value.
  if (right_spd < min_spd && left_spd < min_spd){
    robot_stop();
  } else {
    if(first_update){
      right_motor_drive(right_spd, dir_rgt);
      left_motor_drive(left_spd, dir_lft);
      first_update = false;
    } else {
      left_motor_drive(left_spd, dir_lft);//dir_lft);
      right_motor_drive(right_spd, dir_rgt);
//      if(abs(right_spd - rs_prev)>10 || dr_prev != dir_rgt) {
//        right_motor_drive(right_spd, dir_rgt);
//      }
//      if(abs(left_spd - ls_prev) > 10 || dl_prev != dir_lft){
//         left_motor_drive(left_spd, dir_lft);
//      }
    }  
  }
} //update robot

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable   detector
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  
  
  // Initialize Serial Monitor
  Serial.begin(115200);
  print_wakeup_reason();
  Semaphore_Controls = xSemaphoreCreateMutex();

  //stdby is the standby for the motor drivers.
  pinMode(STDBY,OUTPUT);
  digitalWrite(STDBY,HIGH);
  digitalWrite(2,LOW);

  //topp pin is for 3.3v rail for Analogue inputs
  pinMode(top_pin, OUTPUT);
  digitalWrite(top_pin, HIGH);

  robot_setup(); 
//  left_motor_drive(255, 1);
//  delay(200);
  robot_stop();

  //testing 1... 2... 3... 
  // right_motor_drive(255, 1);
  // left_motor_drive(255, 1);
  // delay(2000);

  
  // right_motor_drive(255, 0);
  // left_motor_drive(255, 0);
  // Serial.println("ok0");
  // delay(2000);
  // Set device as a Wi-Fi Station

  serovs_setup();

  WiFi.mode(WIFI_STA);
 

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  
  // delay(5);
  writeServoMicros(channel_top, micros_top);
  writeServoMicros(channel_b, micros_bottom);
  delay(500);
  writeServoMicros(channel_top, 2000);
  writeServoMicros(channel_b, 2000);
  delay(500);
  writeServoMicros(channel_top, micros_top);
  writeServoMicros(channel_b, micros_bottom);
  delay(20);
  val_rb = 0x0000 | analogRead(photo_rb); //  
  val_rt = 0x0000 | analogRead(photo_rt);
  val_lb = 0x0000 | analogRead(photo_lb);
  val_lt = 0x0000 | analogRead(photo_lt);
  delay(2);
  
  t_last_com = millis();

}

// void update_servos(void *params){
void loop(){
  double alpha;
  
  alpha = 0.85;
  
    for (int i = 0; i < 5; i++){ 
    //use an exponentially weighted moving average to get rid of noise.
    val_rb = (uint16_t) (alpha*((double) val_rb) + (1.0-alpha)*((double) (0x0000 | analogRead(photo_rb)))); //
    val_rt = (uint16_t) (alpha*((double) val_rt) + (1.0-alpha)*((double) (0x0000 | analogRead(photo_rt))));
    val_lb = (uint16_t) (alpha*((double) val_lb) + (1.0-alpha)*((double) (0x0000 | analogRead(photo_lb))));
    val_lt = (uint16_t) (alpha*((double) val_lt) + (1.0-alpha)*((double) (0x0000 | analogRead(photo_lt))));
    
      }
  //this is for debuggin. In general, I try to avoid Serial.println in freertos.
  if (counter > 800){
  Serial.println("values");
  Serial.println(val_rb);
  Serial.println(val_rt);
  Serial.println(val_lb);
  Serial.println(val_lt);
  Serial.println(micros_top);
  Serial.println(micros_bottom);
  counter=0;

  }
  counter++;



  if(val_rb < val_rt-10 && val_lb < val_lt-10){
    //move forward
    micros_top++;
    if(micros_top > micros_top_max){micros_top = micros_top_max;}
    writeServoMicros(channel_top, micros_top);
  } else if(val_rb > val_rt+10 && val_lb > val_lt+10){
    //move backward
    micros_top--;
    if(micros_top < micros_top_min){micros_top = micros_top_min;}
    writeServoMicros(channel_top, micros_top);
  }

  if(val_rb > val_lb && val_rt > val_lt){
    //move left
    micros_bottom++;
    if(micros_bottom > micros_bottom_max){micros_bottom = micros_bottom_max;}
    writeServoMicros(channel_b, micros_bottom);
  } else if(val_rb < val_lb && val_rt < val_lt){
    //move right
    micros_bottom--;
    if(micros_bottom < micros_bottom_min){micros_bottom = micros_bottom_min;}
    writeServoMicros(channel_b, micros_bottom);
  }

  if((millis()-t_last_com) > 500){
      //if more than 500 ms since last communication stop robot.
      //this will make sure robot doesn't run away if communication signal is broken.
      robot_stop(); 
      // digitalWrite(STDBY,LOW);
    }

    if((millis()-t_last_com) > 8000){
      
      //this will put the robot into deep sleep if it doesn't hear from the remote control in 4s.
      digitalWrite(STDBY,LOW);
      
      servo_top_persistent=micros_top;
      servo_bottom_persistent=micros_bottom;
      
      Serial.println("going to sleep");
      esp_deep_sleep_start();
     
    }
    

  
}

