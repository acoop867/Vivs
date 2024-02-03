/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Student                                          */
/*    Created:      Fri Feb 02 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Opticall             optical       7               
// backleft             motor         17              
// middleleft           motor         16              
// frontleft            motor         19              
// backright            motor         14              
// middleright          motor         13              
// frontright           motor         12              
// gyroo                inertial      6               
// intake               motor         15              
// catapult             motor         20              
// Controller1          controller                    
// leftwing             digital_out   A               
// intakepistonb        digital_out   B               
// rightwing            digital_out   H               
// RotationRight        rotation      21              
// RotationLeft         rotation      18              
// ---- END VEXCODE CONFIGURED DEVICES ----


#include "vex.h"
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

// defining our functions 

void forwardPIDD(float setpoint) {
  float error = setpoint - backleft.position(degrees);
  float power;
  float i = 1;
  double kP = 0.11;
  while (abs(error) > i) {
    error = setpoint - backleft.position(degrees);
    power = error * kP;
    Brain.Screen.printAt(60, 15, "%f", backleft.position(degrees));
    setBase(power);
  }
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(60, 15, "%f", backleft.position(degrees));
  stopBase();
}

void autofire() {
  while (true) {
  if (Opticall.hue() < 105 && Opticall.hue() > 65) {
    catapult.spin(forward, 50, pct);
    Brain.Screen.print("Hue: %.2f", Opticall.hue());
    Brain.Screen.setCursor(2,2);
  }
  else {
    catapult.stop();
    Brain.Screen.print("Hue: %.2f", Opticall.hue());
    Brain.Screen.setCursor(2,2);
  }

}
  wait(10, msec);
  Brain.Screen.clearScreen();
}

void sensordisplays () {
  while (true) {
    Brain.Screen.printAt(15, 15, "%f", RotationRight.position(degrees));
    Brain.Screen.printAt(30, 30, "%f", RotationLeft.position(degrees));
    Brain.Screen.printAt(45, 45, "%f", gyroo.heading(degrees));
    Brain.Screen.printAt(60, 60, "Hue: %.2f", Opticall.hue());
    wait(10,msec);
    Brain.Screen.clearScreen();
  }
}

void clearmotors () {
  frontleft.resetPosition();
  frontright.resetPosition();
  backleft.resetPosition();
  backright.resetPosition();
  middleleft.resetPosition();
  middleright.resetPosition();
}

void clearall() {
  frontleft.resetPosition();
  frontright.resetPosition();
  backleft.resetPosition();
  backright.resetPosition();
  middleleft.resetPosition();
  middleright.resetPosition();
  gyroo.resetHeading();
  RotationLeft.resetPosition();
  RotationRight.resetPosition();
}

void setwings(int setpoint) {
  leftwing.set(setpoint);
  rightwing.set(setpoint);
}

void setall(int setpoint) {
  leftwing.set(setpoint);
  rightwing.set(setpoint);
}

void pre() {
  clearall();
  // thread d(autofire);
  Controller1.Screen.setCursor(2,2);
  Controller1.Screen.print("Calibrating sensors...");
  gyroo.calibrate();
  wait(2.5, seconds);
  Controller1.Screen.setCursor(2,2);
  Controller1.Screen.print("      We're live!       ");
  wait(3, seconds);
  Controller1.Screen.setCursor(2,2);
  Controller1.Screen.print("      Go Luca !!!       ");
}

void cPID(int setpoint) {
  float kP = 0.89;
  float kI = 0.02;
  float kD = 0.1;
  float error = setpoint - catapult.position(degrees);
  float integral = 0;
  float derivative = 0;
  float prevError = 0;
  float i = 0;
  float power;

  while (abs(error) > i) {
  error = setpoint - catapult.position(degrees);
 
  float P = kP * error;
  integral += error;
  float I = kI * integral;
  derivative = error - prevError;
  float D = kD * derivative;

  power = P + I + D; 

  catapult.spin(forward, power, pct);
  prevError = error;

  }
  catapult.stop(); // Stop the drivetrain when the loop exits
}

void stockD() {
  clearall();
  turnPID(29, 100, -1);
  forwardPIDD(750);
  intake.spin(reverse, 100, pct);
  wait(.3,sec);
  intake.stop();
  turnPID(27, 90, -1);
  wait(.1, sec);
  forwardPIDD(-100);
  clearall();
  setwings(true);
  clearmotors();
  turnPID(180, 75, -1);
  turnPID(360, 80, -1);
  wait(.01,sec);
  setwings(false);
  turnPID(440, 90, -1);
  clearmotors();
  forwardPIDD(-+815);
  turnPID(445, 90, -1);
  forwardPIDD(-1630);
  Controller1.Screen.print("simple defensive COMPLETE!");
}

void stockO() {
  clearall();
  turnPID(-29, 100, -1);
  forwardPIDD(1320);
  intake.spin(reverse, 100, pct);
  wait(.3,sec);
  intake.stop();
  turnPID(-27, 90, -1);
  wait(.1, sec);
  forwardPIDD(-100);
  clearall();
  setwings(true);
  clearmotors();
  turnPID(-180, 75, -1);
  turnPID(-360, 80, -1);
  wait(.01,sec);
  setwings(false);
  turnPID(-440, 90, -1);
  clearmotors();
  forwardPIDD(-815);
  turnPID(-440, 90, -1);
  forwardPIDD(-1630);
  Controller1.Screen.print("simple defensive COMPLETE!");
}

void offensive() {
  clearall();
  // start in the centre of the middle start tile w/ preload
  forwardPIDD(2050);
  turnPID(-90, 80, -1);
    clearmotors();
  // score
  intake.spin(reverse, 100, pct);
  forwardPIDD(115);
    wait(.1, sec);

  // collect triball a
  forwardPIDD(-300);
  clearmotors();
  turnPID(-11, 70, -1);
    clearmotors();
  intake.spin(forward, 100, pct);
  forwardPIDD(365);
    wait(.1, sec);

  // score
  forwardPIDD(0);
  turnPID(-90, 80, -1);
    clearmotors();
  intake.spin(reverse, 100, pct);
  forwardPIDD(600);
    wait(.1, sec);
    intake.stop();

  // collect triball b
  forwardPIDD(-750);
  turnPID(0, 80, -1);
    clearmotors();
  intake.spin(forward, 100, pct);
  forwardPIDD(200);
    wait(.1, sec);
  
  // score
  forwardPIDD(0);
  turnPID(-90, 80, -1);
    clearmotors();
  intake.spin(reverse, 100, pct);
  forwardPIDD(-1520);
    wait(.1, sec);
    intake.stop();
    clearmotors();
 
  // collect triball c
  // forwardPIDD(-700); 
  // turnPID(100, 85, -1);
  //   clearmotors();
  // intake.spin(forward, 100, pct);
  //   wait(.1, sec);
  //   intake.stop();
  // forwardPIDD(-500);
  // score

  // go hit bar
  forwardPIDD(-100); 
  turnPID(100, 85, -1);
    clearmotors();
  forwardPIDD(-500);  
}

void skills() {
clearall();
  catapult.spin(reverse, 98, pct);
  wait(33, seconds);
  catapult.stop();
  forwardPIDD(-100);
  turnPID(-40, 80, -1);
  clearmotors();
  forwardPIDD(-1500);
  catapult.spin(reverse, 100, pct);
  wait(.2, sec);
  catapult.stop();
  forwardPIDD(3000);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/
int current_auton_selection = 0;
bool auto_started = false;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  pre();
  //thread a(sensordisplays);
  while(auto_started == false){
    Brain.Screen.clearScreen();
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(100,100, "OFFENSE");
        break;
    case 1:
        Brain.Screen.printAt(100,100, "DEFENSE");
        break;
    case 2:
        Brain.Screen.printAt(100,100, "SKILLS");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
        current_auton_selection ++;
      } else if (current_auton_selection == 3){
        current_auton_selection = 0;
      }
      task::sleep(10);
    }
  }



/*---------------------------------------------------------------------------*/
/*                              Auton Task                                  */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  auto_started = true;
  switch(current_auton_selection){
    case 0:
      offensive();
      break;
    case 1:
      stockD();
      break;
    case 2:
      skills();
      break;
  }
  // stockD();
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // intakepistonb.set(true);
  catapult.setStopping(hold);
  stopBase();
  while (true) {
      float movement = (Controller1.Axis3.position()) + (Controller1.Axis1.position());
      float movement1 = (Controller1.Axis3.position()) - (Controller1.Axis1.position());
      frontleft.spin(forward, movement, pct);
      frontright.spin(forward, movement1, pct);
      //back dt
      backleft.spin(forward, movement, pct);
      backright.spin(forward, movement1, pct);
      // ------------
      middleleft.spin(forward, movement, pct);
      middleright.spin(forward, movement1, pct);
      // ------------
    if (Controller1.ButtonR2.pressing()) {
      (intake.spin(reverse, 90, pct));
    } else if (Controller1.ButtonR1.pressing()) {
      (intake.spin(forward, 90, pct));
    } else {
      intake.stop();
    }
    if (Controller1.ButtonY.pressing()) {
      (catapult.spin(reverse, 90, pct));
    } else if (Controller1.ButtonB.pressing()) {
      (catapult.spin(forward, 90, pct));
    } else {
      catapult.stop();
    }
  
    //pnue wings
    // all at once
    if (Controller1.ButtonUp.pressing()) {
      leftwing.set(false);
      rightwing.set(false);
    } else if (Controller1.ButtonLeft.pressing()) {
      leftwing.set(true);
      rightwing.set(true);
    }
  }

  while (1) {

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
