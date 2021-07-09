//Default stepping = 200 steps/revolution. 360/200=1.8(degree)
//| micro step | pulse/rev | degree/pulse |
//------------------------------------------
//|     4      |    800    |     0.45     |

#include <AccelStepper.h>
#include <Arduino.h>
#include "uwb.h"
#include "kalman.hpp"

const double wheel_R = 12.5;
const long ppr = 800; // pulse/rev
const long rpm = 120;
const int std_distance = 50;
const int safe_distance = 20;
long distanceToFollow = 0, stepToFollow = 0;

int dirMultiplier = -1; // = 1: positive direction, = -1: negative direction
bool newData, runAllowed = false, turnAllowed = false; // booleans for new data from serial, and runallowed flag

float max_speed ; // <max_speed> steps/second
long accel = 400; // <accel> steps/second^2
int pul;
int dir;

int gta_value = 0;
int gta_count = 0;
long gta_last = 0;

AccelStepper stpLeft(1, 8, 9); //(1,left_PUL,left_DIR)
AccelStepper stpRight(1, 12, 13); //(1,right_PUL,right_DIR)

Kalman_Filter kdistance;
Kalman_Filter kangual;
int straight_angle = 8;

long stepper_perf_counter = 0;
unsigned long last_counter = 0;

struct SpeedFactor {
  double right;
  double left;

};

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  //    Serial1.println();
  //    Serial1.println("user_cmd 1");
  //    Serial1.println();
  max_speed = (rpm * ppr) / 60;

  stpRight.setMaxSpeed(max_speed);
  stpRight.setAcceleration(accel);
  stpRight.disableOutputs();

  stpLeft.setMaxSpeed(max_speed);
  stpLeft.setAcceleration(accel);
  stpLeft.disableOutputs();
}

void loop() {
  struct uwb_data uwb;

  if (checkSerial(&uwb) == 0) {

    float angual = kangual.measure(uwb.angual);
    //float angual = uwb.angual;
    float distance = kdistance.measure(uwb.distance);
    //float distance = uwb.distance;

    bool needTurn = GottaTurnAround((int)distance);
    if (needTurn && !turnAllowed) {
      stepToFollow = 1150; //800
      RotateTo(stpRight, -stepToFollow, max_speed);
      RotateTo(stpLeft, stepToFollow, max_speed);
      Serial.println("Turning Around!");
      turnAllowed = true;
    }
    if (turnAllowed) {
      if (stpLeft.distanceToGo() == 0 && stpRight.distanceToGo() == 0) {
        Serial.println("Stop Turning Around!");
        turnAllowed = false;
      }
    } else if (distance > std_distance) {
      distanceToFollow = distance - std_distance;
      stepToFollow = (distanceToFollow * ppr) / (wheel_R * PI);
      //Serial.print(stepToFollow);
      //Serial.println(" steps to follow!");
      SpeedFactor factor = CtrlDirection(angual);

      RotateRelative(stpRight, stepToFollow, factor.right * max_speed);
      RotateRelative(stpLeft, stepToFollow, factor.left * max_speed);
      //RotateRelative(stpRight, stepToFollow, 1 * max_speed);
      //RotateRelative(stpLeft, stepToFollow, 1 * max_speed);
      //Serial.print("Speed:");
      //Serial.println(max_speed);
    }
    else {
      //Serial.println("STOP!!!");
      runAllowed = false;
      stpRight.stop();
      stpLeft.stop();
    }
  }

  motorRun();
}

int checkSerial(struct uwb_data *uwb) {
  byte buf[300];

  if (Serial1.available() > 0) {
    while (true) {
      buf[0] = Serial1.read();
      //Serial.println(buf[0], HEX);
      if (buf[0] == 0x2a) {
        // Read head byte
        //Serial.println("-");
        break;
      } else if (buf[0] == 0xFF) {
        // Nothing to read, return
        return -1;
      }
      //Serial.print(".");
    }

    do {
      buf[1] = Serial1.read();
    } while (buf[1] == 0xFF); // Loop if length == -1
    // Length is count from 'sn' to 'acc_Z'
    // Not included last 2 bytes(check & foot)
    Serial1.readBytes(buf + 2, buf[1] + 2); 

    int ret = deserialize_uwb_data(buf, uwb);
    if (ret == -1) {
      Serial.println("Error: decoding UWB data!");
      for (int i = 0; i < buf[1] + 4; i++) {
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      return -1;
    } else if (ret == -2) {
      Serial.println("Warning: checksum failed!");
      return -1;
    }

    //dump_uwb_data(uwb);
    return 0;
  } else {
    return -1;
  }
}

void motorRun() {
  if (runAllowed == true) {
    stpRight.enableOutputs();
    stpLeft.enableOutputs();
    stpRight.run();
    stpLeft.run();
  } else {
    stpRight.disableOutputs();
    stpLeft.disableOutputs();
  }
  perf_count();
}

void motorRunSpeed() { //no accel cost speed to run
  if (runAllowed == true) {
    stpRight.enableOutputs();
    stpLeft.enableOutputs();
    stpRight.runSpeed();
    stpLeft.runSpeed();
  } else {
    stpRight.disableOutputs();
    stpLeft.disableOutputs();
  }
  perf_count();
}

void RotateTo(AccelStepper& stepper, long receivedSteps, float receivedSpeed) {
  runAllowed = true; //allow running - this allows entering the RunTheMotor() function.
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(receivedSpeed);
  //stepper.setSpeed(receivedSpeed);
  stepper.moveTo(receivedSteps);
}

void RotateRelative(AccelStepper& stepper, long receivedSteps, float receivedSpeed) {
  runAllowed = true; //allow running - this allows entering the RunTheMotor() function.
  stepper.setMaxSpeed(receivedSpeed);
  stepper.move(dirMultiplier * receivedSteps);
}

bool GottaTurnAround(int d) {
  gta_count++;

  if (gta_count >= 5) {
    long dis_diff = d - gta_last;
    if (dis_diff > 4) {
      // 計算5次平均距離差，有增長的趨勢
      gta_value++;
    } else if (dis_diff < 0) {
      gta_value = 0;
    }
    gta_last = d;

    Serial.print("gta_value: ");
    Serial.println(gta_value);
    gta_count = 0;
    // 判斷是否令車子迴轉
    return (gta_value >= 5);
  } else {
    return false;
  }

  
}

SpeedFactor CtrlDirection(float ang) {
  float speedF;
  SpeedFactor factor;
  if (ang <= -80 || ang >= 80) {
    speedF = 0.04;
  }
  else if (ang <= -70 || ang >= 70) {
    speedF = 0.08; //0.1
  }
  else if (ang <= -60 || ang >= 60) {
    speedF = 0.15; //0.2
  }
  else if (ang <= -50 || ang >= 50) {
    speedF = 0.25; //0.3
  }
  else if (ang <= -40 || ang >= 40) {
    speedF = 0.4; //0.4
  }
  else if (ang <= -30 || ang >= 30) {
    speedF = 0.6; //0.6
  }
  else if (ang <= -20 || ang >= 20) {
    speedF = 0.7; //0.7
  }
  else if (ang <= -8 || ang >= 8) {
    speedF = 0.8; //0.8
  } else {
    speedF = 1;
  }

  if (ang > 0) { //target at left side => turn LEFT
    factor.left = speedF;
    factor.right = 1.0;
  } else { //target at right side => turn RIGHT
    factor.right = speedF;
    factor.left = 1.0;
  }
  return factor;
}

void dump_uwb_data(const uwb_data *uwb)
{
  Serial.println("UWB data:");
  Serial.print("\tSN: ");
  Serial.println(uwb->sn);
  Serial.print("\tAddress: ");
  Serial.println(uwb->addr);
  Serial.print("\tAngual: ");
  Serial.println(uwb->angual);
  Serial.print("\tDistance: ");
  Serial.println(uwb->distance);
  Serial.print("\tAccelerate: <");
  Serial.print(uwb->acc_x);
  Serial.print(", ");
  Serial.print(uwb->acc_y);
  Serial.print(", ");
  Serial.print(uwb->acc_z);
  Serial.println(">");
}

inline void perf_count()
{
  stepper_perf_counter++;
  long t = millis() - last_counter;
  if (t > 5000) {
    last_counter = millis();
    Serial.print("motor.run() in last 5 sec: ");
    Serial.println(stepper_perf_counter);
    stepper_perf_counter = 0;
  }
}
