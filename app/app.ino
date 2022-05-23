#include "Wire.h"
#include <VL53L1X.h>
#include <Servo.h>

#define TCAADDR 0x70
// #define S_DEBUG
//#define TOF_DEBUG
//#define PRINT_STATE

// current state
char curry_state = 'l';

// servo vars
const int open_angle = 40;

Servo servos[4];
double servos_angle[4] = {165, open_angle, 300-open_angle, 180-0}; // claw_arm up => 165*
const double servos_max_angle[4] = {180, 300, 300, 180};
const double servos_pin[4] = {9, 4, 5, 6};
bool servo_change = false;
namespace Servos {
  enum Servos { ARM, LEFT, RIGHT, COMPART };
}
unsigned long servo_time_start;

// switch vars
const int inPin = A0;

// motor vars
/**set control port**/
const int E1Pin = 10;
const int M1Pin = 12;
const int E2Pin = 11;
const int M2Pin = 13;

/**inner definition**/
typedef struct {
  byte enPin;
  byte directionPin;
} MotorContrl;

const int M1 = 0;
const int M2 = 1;
const int MotorNum = 2;

const MotorContrl MotorPin[] = { {E1Pin, M1Pin}, {E2Pin, M2Pin} } ;

const int Forward = LOW;
const int Backward = HIGH;
const float k_p = -0.05;
const float k_d = 0.001;

// serial communication + movement vars
double rotation = 0;
double speed = 0;
char buffer[100];
int buff_size = 0;

// time of flight + obstacle vars
VL53L1X r_sensor;
VL53L1X l_sensor;
VL53L1X f_b_sensor;

int r_side_dist = 0;
int l_side_dist = 0;
int front_b_dist = 0;
bool see_line = 0;
unsigned long obs_time_start;
double o_rotation;

enum class Turn { RIGHT, LEFT };
Turn turn = Turn::LEFT;

int obs_state = 0;

bool see_out = false;

bool see_silver_tape = false;
bool pickup = false;
bool leave_evac = false;
bool see_ball = false;
double evac_speed = 0;
double evac_rotation = 0;
double evac_prev_rotation = 0;
unsigned long evac_time_start;
unsigned long evac_turn_time;
unsigned long evac_forward_timer;
unsigned long pickup_timer;
Turn evac_turn = Turn::LEFT;
long rand_time;

int pickup_state = 0;

bool see_deposit = false;
double deposit_dev = 0;
unsigned long deposit_timer;
int deposit_state = 0;

double r_k_p = 0.005;
double s_k_p = 0.010;

int ball_counter = 0;

const int trigPin = 2;
const int echoPin = 3;
volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
volatile bool newPulseDurationAvailable = true;
struct Ultrasound_r {
  int curr;
  bool out_of_range;
};
struct Ultrasound_r front_dist={0, true};

/**program**/
void setup() {
  initMotor();
  Serial.begin(9600);
  Serial.setTimeout(100);
  Serial.flush();

  for (int i = Servos::ARM; i != (Servos::COMPART + 1); i++) {
    servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));  
    servos[i].attach(servos_pin[i]);
  }
  Wire.begin();
  tofinit(r_sensor, 2);
  tofinit(l_sensor, 1);
//  tofinit(f_sensor, 3); 
  tofinit(f_b_sensor, 3);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(echoPin), echoInterrupt, CHANGE);
  
  delay(500);
  evac_time_start = millis();
}

void serialEvent() {
  while (Serial.available()) {
    int data = Serial.read();
    if (data != '\n') { // a bit cringe
      buffer[buff_size] = data;
      buff_size++;
    } else {
      // do something with data
      switch (buffer[0]) {
        case 'r': 
          {
            char * r = (char*) &rotation;
            for (int i = 1; i <= buff_size; i++) {
              r[i - 1] = buffer[i];
            }
          }
          break;
        
        case 's': 
          {
            char * s = (char*) &speed;
            for (int i = 1; i <= buff_size; i++) {
              s[i - 1] = buffer[i];
            }
          }
          break;

        case 'l':
          {
            see_line = true ? (uint8_t)buffer[1] == 1 : false;
          }
          break;

        case 'm':
          {
            servo_change = true;
            char * s = (char*) &(servos_angle[(uint8_t)buffer[1]]);
            for (int i = 2; i <= buff_size; i++) {
              s[i - 2] = buffer[i];
            }
//            for (int i = 0; i < 4; i++) {
//              Serial.print(servos_angle[i]);
//              Serial.print(" ");
//            }
//            Serial.println((uint8_t)buffer[1]);
          }
          break;

        case 't':
          {
            see_silver_tape = true ? (uint8_t)buffer[1] == 1 : false;
          }
          break;

        case 'p':
          {
            pickup = true ? (uint8_t)buffer[1] == 1 : false;
          }
          break;

        case 'e':
          {
            leave_evac = true ? (uint8_t)buffer[1] == 1 : false;
          }
          break;

        case 'b':
          {
            see_ball = true ? (uint8_t)buffer[1] == 1 : false;
          }
          break;

        case 'd':
          {
            see_deposit = true ? (uint8_t)buffer[1] == 1 : false;
          }
          break;

        case 'D':
          {
            char * D = (char*) &deposit_dev;
            for (int i = 1; i <= buff_size; i++) {
              D[i - 1] = buffer[i];
            }
          }
          break;
      }
      buff_size = 0;
    }
  }
}

#define async_read(var, sen, port) tcaselect(port); if (sen.dataReady()) var = sen.read(false)
#define limit_dist(var, reading, max_val) var = reading > max_val ? 0 : reading

void loop() {
  tcaselect(2);
  if (r_sensor.dataReady()) r_side_dist = r_sensor.read(false);
  tcaselect(1);
  if (l_sensor.dataReady()) l_side_dist = l_sensor.read(false);
  tcaselect(3);
  if (f_b_sensor.dataReady()) front_b_dist = f_b_sensor.read(false);

  if (newPulseDurationAvailable) {
    newPulseDurationAvailable = false;
    int d = (pulseInTimeEnd - pulseInTimeBegin) / 58.2;
    if (d < 200) front_dist.curr = d;
//    else front_dist.out_of_range = true;
    triggerPulse();
  }
  
  #ifdef TOF_DEBUG
  Serial.print("f: ");
  Serial.print(front_dist.out_of_range);
  Serial.print(" ");
  Serial.print(front_dist.curr);
  Serial.print(" r: ");
  Serial.print(r_side_dist);
  Serial.print(" l: ");
  Serial.print(l_side_dist);
  Serial.print(" fb: ");
  Serial.println(front_b_dist);
  return;
  #endif
//  Serial.println(curry_state);
  if (digitalRead(inPin)) {
    differentialSteer(0, 0);
  } else {
    #ifdef PRINT_STATE
    Serial.print(see_line);
    Serial.println(curry_state);
    #endif
    #ifdef S_DEBUG
    // Serial.println(rotation);
    differentialSteer(1, 0);
    #else
    if (servo_change && ((curry_state == 'A'||curry_state == 'B'||curry_state == 'P') ? front_dist.curr > 10 : true)) {
      for (int i = Servos::ARM; i != (Servos::COMPART + 1); i++) {
        servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));  
//        Serial.print(servos_angle[i]); Serial.print(" "); Serial.println();
      }
      servo_time_start = millis();
      servo_change = false;
    }
    
    
    switch (curry_state) {
      
      case 'l': 
      {
//        if (front_dist < 110) {
//          if (millis() - servo_time_start > 1000) {
//            curry_state = 's';
////            turn = r_side_dist > l_side_dist ? Turn::RIGHT : Turn::LEFT;
//            turn = Turn::LEFT;
//          }
//        } else 
        if (see_silver_tape) {
          curry_state = 'A';
          see_silver_tape = false;
        } else {
          limit_s_r(speed, rotation);
          differentialSteer(speed, rotation);
        }
      }
      break;

      case 's': // Obstacle Start
      {
        int side_dist;
        if (turn == Turn::RIGHT) {
          side_dist = l_side_dist;
          o_rotation = 1;
        } else {
          side_dist = r_side_dist;
          o_rotation = -1;
        }
        differentialSteer(0.75, o_rotation);
        if (fabs(side_dist) < 100) {
          differentialSteer(0, 0);
          obs_state = 0;
          curry_state = 'o';
          obs_time_start = millis();
        }
      }
      break;

      case 'o': // Obstacle logic
      {
        int side_dist;
        if (turn == Turn::RIGHT) {
          side_dist = l_side_dist;
          o_rotation = -0.9;
        } else {
          side_dist = r_side_dist;
          o_rotation = 0.9;
        }

        switch (obs_state) {
          case 0: 
          {
            differentialSteer(0.7, 0);
            if (side_dist < 150) obs_state++;
          }
          break;

          case 1: 
          {
            differentialSteer(0.7, 0);
            if (side_dist > 150) obs_state++;
          }
          break;

          case 2: 
          {
            differentialSteer(0.7, o_rotation);
            if (side_dist < 150) obs_state++;
          }
          break;

          case 3: 
          {
            differentialSteer(0.7, o_rotation);
            if (side_dist > 150) obs_state = 0;
          }
          break;
        }

        if (see_line && (millis() - obs_time_start) > 6000) {
          differentialSteer(0, 0);
          curry_state = 'b';
          obs_time_start = millis();
          see_line = false;
        }
      }
      break;

      case 'b': // Turning back onto the line
      {
        differentialSteer(0.8, (turn == Turn::RIGHT ? 1 : -1)*0.5);
        if (millis() - obs_time_start > 1000) {
          differentialSteer(0, 0);
          curry_state = 'l';
        }
      }
      break;

     case 'A':
     {
//      Serial.println("hi");
      if ((front_dist.curr > 10 && !front_dist.out_of_range) && front_b_dist < 90) {
        curry_state = 'P';
        pickup_timer = millis();
      }
      
      if (front_dist.curr < 19 || front_dist.out_of_range || (millis() - evac_forward_timer) > 18000) {
        if (front_dist.out_of_range) see_out = true; // if you just use 'out_of_range' then it transitions from B to A like instantaneously
//        Serial.println("bruh");
        evac_time_start = millis();
        curry_state = 'B';
        evac_turn = r_side_dist > l_side_dist ? Turn::RIGHT : Turn::LEFT;
        rand_time = random(500, 1200);
        evac_forward_timer = millis();
        evac_turn_time = rand_time;
        evac_prev_rotation = rotation;
        see_silver_tape = false;
        differentialSteer(0, 0);
      }
      // Detecting the proximity + avoid rubbing the wall (Proportional)
      if (r_side_dist < 80) {
        double l_t = -constrain(20/(double)r_side_dist, 0, 1);
        differentialSteer(0.9, l_t);
      } else if (l_side_dist < 80) {
        double r_t = constrain(20/(double)l_side_dist, 0, 1);
        differentialSteer(0.9, r_t);
      } else {
        differentialSteer(0.85, rotation);
      }
      
      if (ball_counter >= 3) {
        curry_state = 'G';
        claw_up();
      }
      
     }
     break;

     case 'B':
     {
      differentialSteer(1, 1 * (evac_turn == Turn::RIGHT ? 1 : -1) );
      if ( millis() - evac_time_start > evac_turn_time ) {
        curry_state = 'A';
//        Serial.println(curry_state);
        differentialSteer(0, 0);
      }

      if (see_ball && !see_out) {
        see_out = false;
        curry_state = 'A';
        differentialSteer(0, 0);
      }
     }
     break;

     case 'P':
     {
//        if (front_b_dist < 40) {differentialSteer(-0.5, 0);}
//        else {differentialSteer(0.4, 0);}
        differentialSteer(0, 0);
  
        claw_down();
        claw_grab();
//        Serial.println((millis() - pickup_timer));
        if ((millis() - pickup_timer) > 1000) {
          if (front_b_dist < 80) {
//            Serial.println("hello");
            if (front_b_dist > 50) {
              curry_state = 'S';
              claw_up();
              pickup_timer = millis();
            } else {
              curry_state = 'Q';
              claw_open();
              pickup_timer = millis();
            }
          } else {
            claw_open();
            curry_state = 'A';
          }
        }
     }
     break;
     
     case 'Q':
     {
      differentialSteer(-0.5, 0);
      if (millis() - pickup_timer > 500) {
        differentialSteer(0, 0);
        pickup_timer = millis();
        curry_state = 'P';
      }
     }
     break;

     case 'S':
     {
       switch(pickup_state) {
        case 0:
        {
          if ((millis() - pickup_timer) > 1000) {
            pickup_state++;
            pickup_timer = millis();
            claw_open(); // Insert sorting logic here
          }
        }
        break;

        case 1:
        {
          if ((millis() - pickup_timer) > 1000) {
            pickup_state=0;
            claw_down();
            pickup_timer = millis();
            curry_state = 'A';
            ball_counter++;
          }
        }
        break;
       }
     }
     break;

     

//     case 'C':
//     {
//      evac_speed = 0.8;
//      if (l_side_dist > r_side_dist) {
//        evac_rotation = 1;
//      } else {
//        evac_rotation = -1;
//      }
//      if (l_side_dist + r_side_dist) {
//        evac_rotation = 0;
//        evac_speed = 0;
//      }
//      differentialSteer(evac_speed, evac_rotation);
//      if (evac_time_start - millis() > 5000) {
//        curry_state = 'D';
//      }
//     }
//     break;

//     case 'F' :
//     {
//      differentialSteer(1, 0);
//      if (front_dist.curr < 60) {
//        pickup = true;
//        curry_state = 'A';
//      }
//      
//     }
//     break;

     case 'G': // Centering algorithm - Deposit start
     {
//      Serial.print("r: ");
//      Serial.print(rotation);
//      Serial.print(" s: ");
//      Serial.println(speed);
      int s_l, s_r, f;
      s_l = l_side_dist;
      s_r = r_side_dist;
//      f = front_b_dist;
      f = front_dist.out_of_range ? 0 : front_dist.curr;
      int diff = s_r - s_l;
      double cen_r = constrain(diff * r_k_p, -1, 1);
      double cen_s = constrain(f * s_k_p, 0.001, 1);
//      Serial.println(cen_r);
      differentialSteer(0.92, pow(cen_r, cen_s));
      if (fabs(s_r - 450) <= 170 && fabs(s_l - 450) <= 170 && fabs(diff) <= 65 && fabs(f - 30) <= 10) {
        curry_state = 'H';
      }
     }
     break;

     case 'H': // Spinning to see black
     { 
       differentialSteer(0.9, 1);
       if (see_deposit) {
        curry_state = 'I';
        evac_forward_timer = millis();
       }
     }
     break;

     case 'I':
     {
       differentialSteer(0.9, deposit_dev);
       if ((millis() - evac_forward_timer) > 1000 && !see_deposit) {
        curry_state = 'H';
       }
       if (front_b_dist < 30) {
        curry_state = 'J';
        deposit_timer = millis();
        claw_arm_deposit();
        claw_pinch();
       }
     }
     break;

     case 'J':
     {
       differentialSteer(0.6, 0);
       switch(deposit_state) {
        case 0:
        {
          claw_arm_deposit();
          claw_pinch();
          // look at state I
          if ((millis() - deposit_timer) > 700) {
            deposit_state++;
            deposit_timer = millis();
            servos_angle[Servos::COMPART] = 180 - 40;
            servo_change = true;
          }
        }
        break;

        case 1:
        {
          if ((millis() - deposit_timer) > 2000) {
            deposit_state++;
            deposit_timer = millis();
            servos_angle[Servos::COMPART] = 180 - 70;
            servo_change = true;
          }
        }
        break;
        
//        case 2:
//        {
//          if ((millis() - deposit_timer) > 1500) {
//            deposit_state++;
//            deposit_timer = millis();
//            servos_angle[Servos::COMPART] = 180 - 80;
//            servo_change = true;
//          }
//        }
//        break;

        case 2:
        {
          if ((millis() - deposit_timer) > 800) {
            servos_angle[Servos::COMPART] = 180;
            claw_open();
            claw_up();
//            deposit_state = 0;
            
          }
        }
        break;
       }
     }
     break;

//    case 'E':
//    {
//      limit_s_r(speed, rotation);
//      differentialSteer(speed, rotation);
//      if (see_green_line) { 
//        curry_state = 'l';
//        see_green_line = false;
//      }
//      if (deposit_app) {
//        curry_state = 'D';
//      }
//    }
//    break;

   // Wall tracking
   case 'W':
   {
    differentialSteer(0.7, (70 - l_side_dist) *  0.04);
   }
   break;
  
    
    #endif
    }
  }
}

/**functions**/
void initMotor() {
  int i;
  for ( i = 0; i < MotorNum; i++ ) {
    digitalWrite(MotorPin[i].enPin, LOW);

    pinMode(MotorPin[i].enPin, OUTPUT);
    pinMode(MotorPin[i].directionPin, OUTPUT);
  }
}

/**  motorNumber: M1, M2
direction:          Forward, Backward **/
void setMotorDirection( int motorNumber, int direction ) {
  digitalWrite( MotorPin[motorNumber].directionPin, direction);
}

/** speed:  0-100   * */
inline void setMotorSpeed( int motorNumber, int speed ) {
  analogWrite(MotorPin[motorNumber].enPin, 255.0 * (speed / 100.0) ); //PWM
}

void setMotorVelocity(int motorNumber, int speed) {
  if (speed < 0) {
    setMotorDirection(motorNumber, Backward);
  } else {
    setMotorDirection(motorNumber, Forward);
  }
  setMotorSpeed(motorNumber, abs(speed));
}

void differentialSteer(double speed, double rotation) 
{
  if(rotation > 1) rotation = 1;
  if(rotation < -1) rotation = -1;
  if (rotation<=0)
  {
    setMotorVelocity(M2, 100*speed); 
    setMotorVelocity(M1, -100*(speed-2*speed*fabs(rotation)));
  }
  if (rotation>=0)
  {
    setMotorVelocity(M2, 100*(speed-2*speed*fabs(rotation)));
    setMotorVelocity(M1, -100*speed); 
  }
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void tofinit(VL53L1X & sensor, uint8_t multiport) {
  tcaselect(multiport); 
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    while (1) {
      Serial.print(multiport);
      Serial.println(" Failed to detect and initialize sensor!");
    }
  }
  sensor.setDistanceMode(VL53L1X::Medium);
  sensor.startContinuous(20);
}

int pwmangle(double angle, int max_angle) {
  return (int)(angle/max_angle * 2000 + 500);
}

void limit_s_r(double & speed, double & rotation) {
  rotation = rotation >= 1 ? 1 : rotation;
  rotation = rotation <= -1 ? -1 : rotation;

  speed = speed >= 1 ? 1 : speed;
  speed = speed <= -1 ? -1 : speed;
}

void echoInterrupt() {
//  Serial.print(newPulseDurationAvailable);
//  Serial.println("hello");
  if (digitalRead(echoPin) == HIGH) {
    pulseInTimeBegin = micros();
  } else {
    pulseInTimeEnd = micros();
    newPulseDurationAvailable = true;
    if (pulseInTimeEnd - pulseInTimeBegin > 7000) {
      front_dist.out_of_range = true;
    } else {
      front_dist.out_of_range = false;
      
    }
  }
}

void claw_grab() {
  servos_angle[Servos::LEFT] = 100;
  servos_angle[Servos::RIGHT] = 300 - 100;
  servo_change = true;
}

void claw_pinch() {
  servos_angle[Servos::LEFT] = 120;
  servos_angle[Servos::RIGHT] = 300 - 120;
  servo_change = true;
}

void claw_arm_deposit() {
  servos_angle[Servos::ARM] = 50;
  servo_change = true;
}

void compart_A() {
  servos_angle[Servos::LEFT] = 32;
  servos_angle[Servos::RIGHT] = 300 - 32;
  servo_change = true;
}

void compart_B() {
  
}

void claw_open() {
  servos_angle[Servos::LEFT] = open_angle;
  servos_angle[Servos::RIGHT] = 300 - open_angle;
  servo_change = true;
}

void claw_up() {
  servos_angle[Servos::ARM] = 165;
  servo_change = true;
}

void claw_down() {
  servos_angle[Servos::ARM] = 0;
  servo_change = true;
}

void triggerPulse() {
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);
}
