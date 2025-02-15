/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftback             motor         11              
// leftcenter           motor         12              
// leftfront            motor         13              
// rightback            motor         18              
// rightcenter          motor         19              
// rightfront           motor         20              
// catapult             motor         15              
// intake               motor         16              
// leftnatsu            digital_out   B               
// rightnatsu           digital_out   C               
// kaisei               digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

motor_group All(leftback,leftcenter, leftfront,rightback, rightcenter, rightfront); 

motor_group Right(rightback, rightcenter, rightfront);
motor_group Left(leftback, leftcenter, leftfront);
motor_group Int(intake,hook);
// A global instance of competition
competition Competition;



vision::signature vision_RED = vision::signature(1, 10519, 12579, 11549,-3137, -2611, -2874,2.5, 0);
vision::signature vision_BLUE = vision::signature(2, -3861, -3193, -3527,3571, 4937, 4254,2.5, 0);
vision tyler = vision(PORT12,50,vision_RED,vision_BLUE);



// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
bool sigma = false;
bool ty = false;
void rightChange(void) {
  if (sigma) {
    rightback.setReversed(true);
    rightcenter.setReversed(false);
    rightfront.setReversed(false);
    sigma = !sigma;
  }
  else {
    rightback.setReversed(false);
    rightcenter.setReversed(true);
    rightfront.setReversed(true);
    sigma = !sigma;
  }
}
void leftChange(void) {
  if (ty) {
    leftback.setReversed(false);
    leftcenter.setReversed(true);
    leftfront.setReversed(true);
    ty = !ty;
  }
  else {
    leftback.setReversed(true);
    leftcenter.setReversed(false);
    leftfront.setReversed(false);
    ty = !ty;
  }
}
int spinHook() {
  Int.setVelocity(100, percent);
  Int.spin(forward);
  wait(5, seconds); // Run intake for 5 seconds or adjust as needed
  Int.stop();
  return 0; // Tasks need to return an integer
}
int spinIntake() {
  intake.setVelocity(100, percent);
  intake.spin(forward);
  wait(8, seconds); // Run intake for 5 seconds or adjust as needed
  intake.stop();
  return 0; // Tasks need to return an integer
}
int spinIntakeBack() {
  intake.setVelocity(100, percent);
  intake.spin(reverse);
  wait(0.8, seconds); // Run intake for 5 seconds or adjust as needed
  intake.stop();
  return 0; // Tasks need to return an integer
}
int spinHooker() {
  hook.setVelocity(100, percent);
  hook.spin(forward);
  wait(0.7,seconds); // Run intake for 5 seconds or adjust as needed
  hook.stop();
  return 0; // Tasks need to return an integer
}
int spinIntakeBack2() {
  intake.setVelocity(100, percent);
  intake.spin(reverse);
  wait(0.6, seconds); // Run intake for 5 seconds or adjust as needed
  intake.stop();
  return 0; // Tasks need to return an integer
}
// Calibrate the inertial sensor
void calibrateInertial() {
    inertials.calibrate();
    while (inertials.isCalibrating()) {
        wait(100, msec); // Wait until calibration completes
    }
}
bool clampState = false; // Track the current state of `kaisei
void toggleClamp() {
    while (true) {
        if (Controller1.ButtonL1.pressing()) {
            clampState = !clampState; // Toggle the state
            clamp.set(clampState);    // Update `kaisei` with the new state

            // Wait until the button is released to prevent repeated toggling
            while (Controller1.ButtonL1.pressing()) {
                wait(10, msec);
            }
        }
        wait(20, msec); // Small delay to avoid excessive CPU usage
    }
}
bool blockerState = false; // Track the current state of `sishuoyuan`

void toggleBlocker() {
    while (true) {
        if (Controller1.ButtonL2.pressing()) {
            blockerState = !blockerState; // Toggle the state
            blocker.set(blockerState);    // Update `sishuoyuan` with the new state

            // Wait until the button is released to prevent repeated toggling
            while (Controller1.ButtonL2.pressing()) {
                wait(10, msec);
            }
        }
        wait(20, msec); // Small delay to avoid excessive CPU usage
    }
}


void initializeInertial() {
    inertials.calibrate();
    while (inertials.isCalibrating()) {
        wait(100, msec); // Wait for calibration to complete
    }
    wait(200, msec); // Extra delay for stability after calibration
}
// PID Constants
void clamper() {
  clamp.set(true);
}
void blockera() {
  blocker.set(true);
}


// Turn right function
void turnRight(float targetAngle) {
    // Get the current heading of the robot
    float currentAngle = inertials.heading(degrees);

    // Normalize targetAngle to be within 0-360 degrees
    targetAngle = fmod(targetAngle, 360.0);
    if (targetAngle < 0) {
        targetAngle += 360.0;
    }

    // Calculate the difference between the target angle and the current angle
    float angleDifference = targetAngle - currentAngle;

    // Normalize angle difference to the range -180 to 180 for shortest turn
    if (angleDifference > 180) {
        angleDifference -= 360;
    } else if (angleDifference < -180) {
        angleDifference += 360;
    }

    // Begin turning only if the difference is large enough
    if (fabs(angleDifference) > 5) {
        // Set a starting speed for turning
        int turnSpeed = 60;

        // Spin motors to turn right, and gradually reduce speed as we get closer
        while (fabs(targetAngle - inertials.heading(degrees)) > 5) {
            // Get the current angle
            currentAngle = inertials.heading(degrees);

            // Recalculate angle difference
            angleDifference = targetAngle - currentAngle;

            // Normalize angle difference to the range -180 to 180
            if (angleDifference > 180) {
                angleDifference -= 360;
            } else if (angleDifference < -180) {
                angleDifference += 360;
            }

            // Calculate a speed factor based on how close we are to the target angle
            // As the difference gets smaller, reduce the speed.
            turnSpeed = 60 * (fabs(angleDifference) / 180); // Speed decreases as angle difference gets smaller

            // Ensure the speed doesn't drop below a minimum value (e.g., 10)
            if (turnSpeed < 15) {
                turnSpeed = 15;
            }

            // Spin motors at the calculated speed
            All.spin(reverse, turnSpeed, percent);
        }

        // Stop the motors once the desired angle is reached
        All.stop();
    }
}
void turnLeft(float targetAngle) {
    // Get the current heading of the robot
    float currentAngle = inertials.heading(degrees);

    // Normalize targetAngle to be within 0-360 degrees
    targetAngle = fmod(targetAngle, 360.0);
    if (targetAngle < 0) {
        targetAngle += 360.0;
    }

    // Calculate the difference between the target angle and the current angle
    float angleDifference = targetAngle - currentAngle;

    // Normalize angle difference to the range -180 to 180 for shortest turn
    if (angleDifference > 180) {
        angleDifference -= 360;
    } else if (angleDifference < -180) {
        angleDifference += 360;
    }

    // Begin turning only if the difference is large enough
    if (fabs(angleDifference) > 5) {
        // Set a starting speed for turning
        int turnSpeed = 60;

        // Spin motors to turn left, and gradually reduce speed as we get closer
        while (fabs(targetAngle - inertials.heading(degrees)) > 5) {
            // Get the current angle
            currentAngle = inertials.heading(degrees);

            // Recalculate angle difference
            angleDifference = targetAngle - currentAngle;

            // Normalize angle difference to the range -180 to 180
            if (angleDifference > 180) {
                angleDifference -= 360;
            } else if (angleDifference < -180) {
                angleDifference += 360;
            }

            // Calculate a speed factor based on how close we are to the target angle
            // As the difference gets smaller, reduce the speed.
            turnSpeed = 60 * (fabs(angleDifference) / 180); // Speed decreases as angle difference gets smaller

            // Ensure the speed doesn't drop below a minimum value (e.g., 10)
            if (turnSpeed < 15) {
                turnSpeed = 15;
            }

            // Spin motors at the calculated speed
            All.spin(forward, turnSpeed, percent);
        }

        // Stop the motors once the desired angle is reached
        All.stop();
    }
}
void turning() {
  while (true) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(inertials.heading(degrees));
  }
}



// settings:
double kP = 0.2;
double kI = 0.0;
double kD = 0.0;

double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

int error;          // sensorValue - Desired Value : Positional Value
int prevError = 0;  // Position 20 miliseconds ago 
int totalError;     // totalError = totalError + error
int derivative = 0; // error - previous error : Speed

int turnError;          // sensorValue - Desired Value : Positional Value
int turnPrevError = 0;  // Position 20 miliseconds ago 
int turnTotalError;     // totalError = totalError + error
int turnDerivative = 0; // error - previous error : Speed

// Auton settings
int desiredValue = 0;
int desiredTurnValue = 0; 

bool resetDriveSensors = false;
// variables modifed for use
bool enableDrivePID = false;

motor_group LeftMotors = motor_group(leftback,leftcenter, leftfront);
motor_group RightMotors = motor_group(rightback, rightcenter, rightfront);


int drivePID(){

  while (enableDrivePID){

    if (resetDriveSensors){
      resetDriveSensors = false;
      LeftMotors.setPosition(0, degrees);
      RightMotors.setPosition(0, degrees);
      // inertialSense.setRotation(0, degrees);
    }

    ////////////////
    // Lateral Movement PID
    /////////////////

    // average of two positions
    int leftMotorPosition = LeftMotors.position(degrees);
    int rightMotorPosition = RightMotors.position(degrees);
    int averagePosition = -(leftMotorPosition + rightMotorPosition) / 2;

    error = averagePosition - desiredValue;

    // Deadzone check
    if (abs(error) < 5) { // Stop adjusting when within 5 units of the target
      LeftMotors.stop();
      RightMotors.stop();
      enableDrivePID = false;
    }

    derivative = error - prevError;
    totalError += error;

    // Reduce speed more significantly as it gets closer to the target
    double slowdownFactor = abs(error) / 80.0; // Increased divisor to slow down more
    if (slowdownFactor > 1.0) slowdownFactor = 1.0;
    if (slowdownFactor < 0.05) slowdownFactor = 0.05; // Even lower minimum speed for better control
    
    double lateralMotorPower = (error * kP + derivative * kD + totalError * kI) * slowdownFactor;

    // Cap motor power to prevent excessive speed
    if (lateralMotorPower > 8.0) lateralMotorPower = 8.0;
    if (lateralMotorPower < -8.0) lateralMotorPower = -8.0;

    LeftMotors.spin(forward, lateralMotorPower, voltageUnits::volt);
    RightMotors.spin(forward, lateralMotorPower*1.05, voltageUnits::volt);


    prevError = error;
    vex::task::sleep(20);
  }
  return 1;
}




void RedLeft(void) {
  enableDrivePID = true;
  vex::task PIDmodifier(drivePID);

  resetDriveSensors = true;
  desiredValue = -720;
  desiredTurnValue = 000;
  thread aaa(clamper);
  
  // All.setVelocity(65,percent);
  // All.spinFor(forward,0.875,seconds);
  // blocker.set(true);
  // thread threada(spinIntake);
  // leftChange();
  // turnLeft(330);
  // leftChange();
  // wait(2,seconds);
  // All.setVelocity(27,percent);
  // All.spinFor(reverse,0.6,seconds);
  // clamp.set(false);

  
  // All.setVelocity(30,percent);
  // All.spinFor(reverse,1.45,seconds);
  // wait(0.5,seconds);
  // clamp.set(false);
  // wait(0.8,seconds);
  // thread threada(spinHook);


  // turnLeft(15);

  // enableDrivePID = false;
  // vex::task PIDmodifier(drivePID);
  // resetDriveSensors = true;
  // desiredValue = 250;


}



void RedRight(void) {
  thread spinna(spinIntake);
  clamp.set(true);
  All.spinFor(0.85,seconds);

  Right.setVelocity(95,percent);
  All.spinFor(0.25,seconds);
  Right.setVelocity(100,percent);
  All.setVelocity(100,percent);

  enableDrivePID = true;
  vex::task PIDmodifier(drivePID); 
  resetDriveSensors = true;
  desiredValue = -160;
  desiredTurnValue = 000;

  wait(0.5,seconds);
  thread fh(spinIntakeBack);
  leftChange();
  turnRight(-3);
  leftChange();
  thread tii(blockera);
  wait(0.7,seconds);

  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = 360;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);

  wait(0.8,seconds);
  blocker.set(false);
  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = 80;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);
  wait(0.2,seconds);
  leftChange();
  turnLeft(180);
  leftChange();
  wait(0.3,seconds);
  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = 125;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);
  wait(0.2,seconds);
  clamp.set(false);
  thread kkaf(spinHook);
  wait(2,seconds);
  clamp.set(true);
  leftChange();
  turnLeft(75);
  leftChange();
  wait(0.2,seconds);
  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = 180;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);
  wait(0.6,seconds);
  clamp.set(false);
}

void BlueLeft(void) {


  thread spinna(spinIntake);
  clamp.set(true);
  All.spinFor(1.2,seconds);

  Left.setVelocity(95,percent);
  All.spinFor(0.25,seconds);
  Left.setVelocity(100,percent);
  wait(0.3,seconds);
  vex::task PIDmodifier(drivePID); 
  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = -95;
  desiredTurnValue = 000;

  wait(0.5,seconds);
  rightChange();
  turnLeft(53);
  rightChange();
  
  wait(0.5,seconds);
  thread fh(spinIntakeBack2);
  thread tii(blockera);
  wait(0.9,seconds);

  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = 350;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);
  
  wait(0.9,seconds);
  blocker.set(false);
  wait(0.4,seconds);

  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = 70;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);

  wait(0.7,seconds);
  leftChange();
  turnLeft(215);
  leftChange();

  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = 110;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);

  wait(0.7,seconds);
  clamp.set(false);
  wait(0.7,seconds);
  thread gnewr(spinHook);

  // enableDrivePID = true;
  // resetDriveSensors = true;
  // desiredValue = 100;
  // desiredTurnValue = 000;
  // PIDmodifier.stop();
  // PIDmodifier = vex::task(drivePID);

  // leftChange();
  // turnRight(285);
  // leftChange();

  // enableDrivePID = true;
  // resetDriveSensors = true;
  // desiredValue = 280;
  // desiredTurnValue = 000;
  // PIDmodifier.stop();
  // PIDmodifier = vex::task(drivePID);
  // wait(1.1,seconds);
  // clamp.set(false);
}

void BlueRight(void) {
  clamp.set(true);
  enableDrivePID = true;
  vex::task PIDmodifier(drivePID); 
  resetDriveSensors = true;
  desiredValue = -180;
  desiredTurnValue = 000;
  wait(0.7,seconds);

  Right.setVelocity(99,percent);
  All.spinFor(0.5,seconds);
  Right.setVelocity(100,percent);
  All.setVelocity(100,percent);
  
  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = -150;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);
  thread wefgk(spinIntake);
  thread jbner(spinHooker);
  wait(0.5,seconds);
  blocker.set(true);
  wait(0.9,seconds);
  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);
  wait(0.8,seconds);
  Right.setVelocity(99,percent);
  All.spinFor(reverse,0.5,seconds);
  Right.setVelocity(100,percent);
  All.setVelocity(100,percent);
  rightChange();
  turnLeft(270);
  rightChange();
  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = -400;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);
}
void AutonomousSkills(void) {
  thread spinna(spinHook);
  clamp.set(true);
  wait(0.7,seconds);
  enableDrivePID = true;
  vex::task PIDmodifier(drivePID); 
  resetDriveSensors = true;
  desiredValue = -130;
  desiredTurnValue = 000;
  wait(0.7,seconds);
  rightChange();
  turnRight(270);
  rightChange();
  wait(0.7,seconds);
  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = 240;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);
  wait(0.7,seconds);
  clamp.set(false);
  wait(0.7,seconds);
  rightChange();
  turnRight(100);
  rightChange();
  wait(0.7,seconds);
  thread spinna3(spinHook);
  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = -870;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);
  wait(0.7,seconds);
  rightChange();
  turnRight(215);
  rightChange();
  wait(0.7,seconds);
  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = -90;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);
  wait(1,seconds);
  rightChange();
  turnRight(270);
  rightChange();
  wait(1,seconds);
  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = -170;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);
  wait(1,seconds);
  clamp.set(true);
  enableDrivePID = true;
  resetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 000;
  PIDmodifier.stop();
  PIDmodifier = vex::task(drivePID);
}
void autonomous(void) {
  AutonomousSkills();
}





/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/





void spinLadybrown1() {
    while (spin.position(degrees) > -32)  {
      wait(10,msec);
      ladybrown.spin(forward, 30, velocityUnits::pct);
    }
    ladybrown.stop(hold); 
    
}

void spinLadybrown2() {
    timer t; // Start a timer
    t.clear();
    while (spin.position(degrees) > -160) {
        if (t.time(sec) > 3) { // Check if 3 seconds have passed
            ladybrown.stop(hold);
            return; // Exit the function
        }
        ladybrown.spin(forward, 100, velocityUnits::pct);
        wait(10, msec);
    }

    ladybrown.stop();
    wait(1.2, seconds);

    t.clear(); // Reset the timer for the reverse movement
    while (spin.position(degrees) < 0) {
        if (t.time(sec) > 3) { // Check if 3 seconds have passed
            ladybrown.stop(hold);
            return; // Exit the function
        }
        ladybrown.spin(reverse, 100, velocityUnits::pct);
        wait(10, msec);
    }
    ladybrown.stop(brakeType::brake);
}

bool ladybrownState = false; 
bool ground = false;

void controlHook() {
    bool objectDetected = false; // Track if an object has been detected
    hook.stop();
    while (true) {
        // Continuously read the distance
        double distance_cm = distancer.objectDistance(distanceUnits::cm);

        if (!objectDetected && distance_cm > 0 && distance_cm <= 4) {
            // Object detected: stop the motor and mark the state
            wait(1000, msec);
            objectDetected = true;
            hook.stop(hold);
        }

        // If an object has been detected, wait until R1 and R2 are released
        if (objectDetected) {
            if (!Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing()) {
                objectDetected = false; // Reset state when buttons are released
                hook.spinFor(reverse,0.2,seconds);
                hook.stop(hold);
                break;
            }
            wait(50, msec); // Avoid unnecessary CPU usage
            continue; // Skip the rest of the loop while objectDetected is true
        }

        // If no object is detected, keep spinning the motor
        hook.spin(forward, 100, percent);

        wait(50, msec); // Small delay for smoother operation
    }
}

void toggleLadyBrown() {
    while (true) {
        if (Controller1.ButtonR1.pressing() == 1 && Controller1.ButtonR2.pressing() == 1) {
          intake.spin(forward);
            if (!ladybrownState) {
                ladybrownState = !ladybrownState;
                ground = true;
                thread toggleThread(controlHook);
                spinLadybrown1();
            }
        }
        if (Controller1.ButtonR1.pressing() == 0 && Controller1.ButtonR2.pressing() == 0 && ground) {
            hook.stop();
            spinLadybrown2();
            ladybrownState = false;
            ground = false;
        }

        wait(20, msec); // Small delay to avoid excessive CPU usage
    }
}

bool emss = false; // Track the current state of `sishuoyuan`
bool emsp = false;
void toggleEm() {
    while (true) {

        if (Controller1.ButtonUp.pressing()) {
          if (!emss) {
            emss = !emss; // Toggle the state
            emsp = true;
            spinLadybrown2();
          }
        }
        if (Controller1.ButtonUp.pressing() == 0 && emsp) {
            emss = false;
            emsp = false;
        }
    }
}
bool colorCheck = false;

motor_group Ultra(intake,hook);
void stopHookAfterDelay() {
    wait(0.2, seconds);  // Keep the hook moving for 0.7 seconds
    hook.stop();         // Stop the hook
    wait(0.18, seconds);    // Wait 1 second before allowing manual control again
    colorCheck = false;  // Allow manual control again
}
void toggleIntake() {
  while (true) {
    tyler.takeSnapshot(vision_RED);
    if (tyler.objectCount >= 1 && !colorCheck) { 
      colorCheck = true;  // Prevent multiple detections
      stopHookAfterDelay();  // Call the function to handle stopping
    }
    if (Controller1.ButtonR1.pressing() == 0 && Controller1.ButtonR2.pressing() == 0 && !colorCheck){
      Ultra.stop();
    }
    if (Controller1.ButtonR1.pressing() == 1 && Controller1.ButtonR2.pressing() == 0 && !colorCheck)
    {
      if (!ladybrownState){
        Ultra.spin(forward);
      }
    }
    if (Controller1.ButtonR2.pressing() == 1 && Controller1.ButtonR1.pressing() == 0 && !colorCheck)
    {
      if (!ladybrownState){
        Ultra.spin(reverse);
      }
    }
  }
}



void usercontrol(void)
{
  // User control code here, inside the loop
  spin.resetPosition();
  thread toggleThread(toggleClamp);
  thread toggleSishuoyuanThread(toggleBlocker);
  thread toggleAA(toggleLadyBrown);
  thread toggleIII(toggleEm);
  thread toggleInta(toggleIntake);
  rightback.setReversed(false);
  rightcenter.setReversed(true);
  rightfront.setReversed(true);
  leftback.setReversed(true);
  leftcenter.setReversed(false);
  leftfront.setReversed(false);
  Ultra.setVelocity(100, percent);
  while (1)
  {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    double axis3 = Controller1.Axis3.position();
    double axis2 = Controller1.Axis2.position();
    double SPEED1, SPEED2;
    SPEED1 = fabs(axis3);
    SPEED2 = fabs(axis2);
    Left.setVelocity(SPEED1, percent);
    Right.setVelocity(SPEED2, percent);

    
    if (axis3 < 0)
    {
      Left.spin(forward);
    }
    else if (axis3 > 0)
    {
      Left.spin(reverse);
    }
    if (axis2 < 0)
    {
      Right.spin(forward);
    }
    else if (axis2 > 0)
    {
      Right.spin(reverse);
    }
    if (axis2 == 0 && axis3 == 0)
    {
      Right.stop();
      Left.stop();
    }
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}
//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control period
  calibrateInertial();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
