#include "vex.h"
#include "cmath" 
using namespace vex;
#include "thread"
#include "iostream"

// A global instance of competition
competition Competition;

//define your global instances of motors and other devices here
controller Controller1 = controller(controllerType::primary);
motor left_front_motor = motor(PORT1, gearSetting::ratio18_1, false);
motor left_back_motor = motor(PORT2, gearSetting::ratio18_1, false);
motor right_front_motor = motor(PORT3, gearSetting::ratio18_1, true);
motor right_back_motor = motor(PORT4, gearSetting::ratio18_1, true);
motor intake_motor = motor(PORT5, gearSetting::ratio6_1, false);
motor hook_motor = motor(PORT6, gearSetting::ratio6_1, false);
motor clamp_motor = motor(PORT7, gearSetting::ratio18_1, false);
motor wallstake_motor = motor(PORT8, gearSetting::ratio18_1, false);
optical optical_sensor = optical(PORT12);
brain Brain;

double storedPosition = 0.0; // The variable that will store the position of the hook motor when the optical sensor is triggered
bool redDetected = false; // This is a true or false value that will be used to detect if the optical sensor has detected red
double positionThreshold = 0 ; // The degrees needed spinned from when the optical sensor is triggered by red or blue
double hue = 0.0; // The hue value of the color detected by the optical sensor
double targetPosition = 0.0; 

void colorSort() {
  while (true) {
      double hue = optical_sensor.hue(); // Get the hue value for more accurate optical sensor readings
      if (hue < 20 || hue > 340) { // If the hue value reads red 
          redDetected = true; // set the bool to true 
      } else {
          redDetected = false;
      }
      if (redDetected) {
          storedPosition = hook_motor.position(degrees);  // Store the current position of the hook motor
          double targetPosition = storedPosition + positionThreshold; // Set the target position to the stored position plus the threshold
        
          while (hook_motor.position(degrees) >= targetPosition) { // Wait until the degrees spinned is greater than or equal to the target position
            wait(20, msec); // Short delay to prevent CPU overload
          }
          
          // Stop the motor with a hold to change its momentum
          hook_motor.stop(hold);
      }

            wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
        }
      }



enum hookState {STOPPED, FORWARD, SORTING }; // States of are hook and chain mech 
hookState hookCurrentState = STOPPED; // Set the initial state to Idle

void hookStopped() { // This function is used to set the state of the hook to stopped
  hook_motor.stop(hold); // Stop the hook motor
  hookCurrentState = STOPPED; // Set the current state to stopped
}

void hookMoving() { // This function is used to set the state of the hook to moving
  hook_motor.spin(forward, 100, percent); // Move the hook motor
  hookCurrentState = FORWARD; // Set the current state to moving
}

void hookSorting() { // This function is used to set the state of the hook to sorting
  colorSort(); // This function is used to sort the color of the ball
  hook_motor.spin(forward, 100, percent); // Move the hook motor
  hookCurrentState = SORTING; // Set the current state to sorting
}


void handleHookStateChange () { // This function is used to handle the state changes of the hook
  switch (hookCurrentState) {
      case STOPPED:
          hookStopped();
          break;
      case FORWARD:
          hookMoving();
          break;
      case SORTING:
          hookSorting();
          break;
  }
}

bool hookChangeStateButton = false; // This is a true or false value that if true will lead to the change of state
bool hookPreviousButtonState = false; // This is a true or false value that if true will lead to the change of state

enum wallStakeState { IDLE, LOADING_1, LOADING_2, SCORING }; // Name the states of the wall stake arm and will alow the driver to cycle through these
        wallStakeState currentState = IDLE; // set the initial state to Idle
    
  double kP = 0.5; // This is used to control the portional gain of the PD controller we found this number through tuning
  double kD = 0.1; // This is used to control the derivative gain of the PD controller we found this number through tuning
  // If these values are to high the motor will move up and down (oscillate)
  // If these values are to low the motor will not move at all 
  double targetPosition = 0.0;
  double currentPosition = 0.0;
  double error = 0.0;
  double previousError = 0.0; 
  double deltaTime = 0.01; // 10ms loop time 
  double maxMotorPower = 100.0; // % 
  
  // These constants will be used to calculate the torque needed to compensate for gravity
  // and using the angle of the arm at that moment
  double angle = 30.0; // Example angle in degrees
  double armMass = 10.0; // Example mass
  double gravity = 9.81; // Example gravity
  double armLength = 2.0; // Example length
  

  double externalGearRatio = 3.0;
  
  double calculateAdjustedTarget(double targetDegrees, double gearRatio) {
      return targetDegrees * gearRatio; // adjust the degrees based on the internal gear ratio of the motor
    }
  
  double lowPassFilter(double input, double prevOutput, double alpha) {
      return alpha * input + (1.0 - alpha) * prevOutput;
  }
  double calculateGravityCompensation(double angle) {
    return armMass * gravity * armLength ;
  }
  
  void moveArmToPosition(double target) { // This function uses a Pd control system to accurately move the wall stack scorer while accounting for gravity
    targetPosition = calculateAdjustedTarget(target, externalGearRatio); // Adjust the target position based on the external gear ratio
    wallstake_motor.setBrake(brakeType::coast); // Set the brake type to coast so that the motor can move freely
    previousError = 0.0;
      double filteredPosition = 0.0;
      double alpha = 0.1; // A value between 0 and 1 the larger the value the more reactive the pd contoller is to current readings
  
      while (true) {
          double rawPosition = wallstake_motor.position(degrees);
          filteredPosition = lowPassFilter(rawPosition, filteredPosition, alpha);
          currentPosition = filteredPosition;
  
          error = targetPosition - currentPosition; // Calculate the error to use P control to smoothly fix the error
  
          double derivative = (error - previousError) / deltaTime;
          double motorPower = (kP * error) + (kD * derivative);
  
          // Limit the motor power to the maximum allowed value
          motorPower = motorPower > maxMotorPower ? maxMotorPower : (motorPower < -maxMotorPower ? -maxMotorPower : motorPower);
  
          wallstake_motor.spin(forward, motorPower, percent); // Uses all the math we did above to move the arm
  
          previousError = error; // sets the error to previous error so that it can be used to caculate the derivatice in the next loop 
          
          if (error < 1 && error > -1) {
              wallstake_motor.stop(brakeType::hold); // Once error is small enough we stop the motor 
              break; // Exit the loop when the target position is reached
          }
          
          
          // Sleep for 10 ms to allow the loop to run without overloading the CPU
          sleep(deltaTime); // Sleep for 10 ms to allow the loop to run without overloading the CPU
      }
  }
  void stateIDLE() { // This function is used to set the state of the wall stake arm to idle
    
    moveArmToPosition(0); // Move the arm to the idle position 
    currentState = IDLE; // Set the current state to idle
  }
  
  void loading_1() { // This function is used to set the state of the wall stake arm to idle
    
    moveArmToPosition(50); // Move the arm to the loading_1 position 
    currentState = LOADING_1; // Set the current state to idle
  }
  
  void loading_2() { // This function is used to set the state of the wall stake arm to idle
    
    moveArmToPosition(100); // Move the arm to the loading_2 position 
    currentState = LOADING_2; // Set the current state to idle
  }
  
  void scoring() { // This function is used to set the state of the wall stake arm to idle
    
    moveArmToPosition(150); // Move the arm to the scoring position 
    currentState = SCORING; // Set the current state to idle
  }
  
  void handleArmStateChange () { // This function is used to handle the state changes of the wall stake arm
    switch (hookCurrentState) {
        case IDLE:
            stateIDLE();
            break;
        case LOADING_1:
            loading_1();
            break;
        case LOADING_2:
            loading_2();
            break;
        case SCORING:
            scoring();
            break;
    }
  }
  bool changeStateButton = false; // This is a true or false value that if true will lead to the change of state 
  // for the wall stake arm
  bool previousButtonState = false; 
              

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

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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

void usercontrol(void) {
  bool changeStateButton = false; // This is a true or false value that if true will lead to the change of state 
  // for the wall stake arm
  bool previousButtonState = false;
  bool hookChangeStateButton = false; // This is a true or false value that if true will lead to the change of state
  bool hookPreviousButtonState = false; // This is a true or false value that if true will lead to the change of state
  // User control code here, inside the loop
  while (1) {
    handleArmStateChange(); // This function is used to handle the state changes of the wall stake arm
    changeStateButton = Controller1.ButtonR1.pressing();
    
    
    if (changeStateButton && !previousButtonState) {
      // Cycle through the states of the wall stake arm
        
      switch (currentState) {
          case IDLE:
              currentState = LOADING_1;
              break;
          case LOADING_1:
              currentState = LOADING_2;
              break;
          case LOADING_2:
              currentState = SCORING;
              break;
          case SCORING:
              currentState = IDLE;
              break;
        }
        previousButtonState = changeStateButton;
        
        wait(20, msec); // This is used to prevent the button from being pressed multiple times
      }

      handleHookStateChange(); // This function is used to handle the state changes of the hook
      
      hookChangeStateButton = Controller1.ButtonL1.pressing();
      
      if (hookChangeStateButton && !hookPreviousButtonState) {
        // Cycle through the states of the wall stake arm
          
        switch (hookCurrentState) {
          case STOPPED:
          hookCurrentState = FORWARD;
          break;
        case FORWARD:
          hookCurrentState = SORTING;
          break;
        case SORTING:
          hookCurrentState = STOPPED;
          break;
        }
        hookPreviousButtonState = hookChangeStateButton; 
        
        wait(20, msec); // This is used to prevent the button from being pressed multiple times
      }

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20,msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  
  
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  usercontrol(); // Run the user control function
  // Run the pre-autonomous function.
  pre_auton();
 
  // Prevent main from exiting with an infinite loop.
  while (true) {
    
    wait(100, msec);
  }
 
  
} 


