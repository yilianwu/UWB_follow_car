//Default stepping = 200 steps/revolution. 360/200=1.8(degree)
//| micro step | pulse/rev | degree/pulse |
//------------------------------------------
//|     4      |    800    |     0.45     |

#include <AccelStepper.h>
#include <Arduino.h>
#include "uwb.h"
#include "kalman.hpp"

int wheel_R = 12.5;
float ppr = 800; // pulse/rev
float rpm = 240;
int std_distance = 50;
int safe_distance = 20;
long distanceToFollow, stepToFollow = 0;

int dirMultiplier = -1; // = 1: positive direction, = -1: negative direction
bool newData, runAllowed, slowAllowed = false; // booleans for new data from serial, and runallowed flag

float max_speed ; // <max_speed> steps/second
long accel = 400; // <accel> steps/second^2
int pul;
int dir;

int state = -1;

int avg_sum = 0;
int avg_count = 0;
int avg_last = 0;

AccelStepper stpLeft(1, 8, 9); //(1,left_PUL,left_DIR)
AccelStepper stpRight(1, 10, 11); //(1,right_PUL,right_DIR)

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
  //    Serial.println();
  //    Serial.println("user_cmd 1");
  //    Serial.println();
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

    //    float angual = kangual.measure(uwb.angual);
    float angual = uwb.angual;
    //    float distance = kdistance.measure(uwb.distance);
    float distance = uwb.distance;

    int avg_distance = AvgDistance((int)distance);
    if (avg_distance > std_distance) {
      distanceToFollow = avg_distance - std_distance;
      stepToFollow = (distanceToFollow * ppr) / (wheel_R * PI);
      //      Serial.print(stepToFollow);
      //      Serial.println(" steps to follow!");
      SpeedFactor factor = CtrlDirection(angual);

      RotateRelative(stpRight, stepToFollow, factor.right * max_speed);
      RotateRelative(stpLeft, stepToFollow, factor.left * max_speed);
      //      Serial.print("Speed:");
      //      Serial.println(max_speed);
    }
    else {
      //Serial.println("STOP!!!");
      runAllowed = false;
      stpRight.stop();
      stpLeft.stop();
    }
  }
  motorRun();
  //  rightMotorRun();
  //  leftMotorRun();
}

int checkSerial(struct uwb_data *uwb) {
  byte buf[300];

  if (Serial.available() > 0) {
    while (true) {
      buf[0] = Serial.read();
      //Serial.println(buf[0], HEX);
      if (buf[0] == 0x2a) {
        // Read head byte
        //Serial.println("-");
        break;
      } else if (buf[0] == 0xFF) {
        // Nothing to read
        return -1;
      }
      //Serial.print(".");
    }

    do {
      buf[1] = Serial.read();
    } while (buf[1] == 0xFF); // Loop if length == -1
    Serial.readBytes(buf + 2, buf[1] + 2);

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

void rightMotorRun() {
  if (runAllowed == true) {
    stpRight.enableOutputs();
    //    stpLeft.enableOutputs();
    stpRight.run();
    //    stpLeft.run();
  }
  else {
    stpRight.disableOutputs();
    //    stpLeft.disableOutputs();
  }
}

void leftMotorRun() {
  if (runAllowed == true) {
    //    stpRight.enableOutputs();
    stpLeft.enableOutputs();
    //    stpRight.run();
    stpLeft.run();
  }
  else {
    //    stpRight.disableOutputs();
    stpLeft.disableOutputs();
  }
}

void RotateRelative(AccelStepper & stepper, long receivedSteps, float receivedSpeed) {
  runAllowed = true; //allow running - this allows entering the RunTheMotor() function.
  stepper.setMaxSpeed(receivedSpeed);
  stepper.move(dirMultiplier * receivedSteps);

}

int AvgDistance(int d) {
  if (avg_count >= 5) {
    avg_last = avg_sum / 5;
    avg_sum = 0;
    avg_count = 0;
  }

  avg_sum += d;
  avg_count++;

  int avg_cur = avg_sum / avg_count;
  return (avg_last + avg_cur) / 2;
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
