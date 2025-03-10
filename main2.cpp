#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
controller Controller1 = controller(primary);
motor clampMotor = motor(PORT2, ratio18_1, false);

motor hookMotor = motor(PORT11, ratio6_1, false);

motor intakeMotor = motor(PORT3, ratio6_1, false);

pot potentiometer1 = pot(Brain.ThreeWirePort.A);
motor wallStakeMotor = motor(PORT1, ratio18_1, false);



// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       {author}                                                  */
/*    Created:      {date}                                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// Include the V5 Library
#include "vex.h"
#include "cmath"
  
// Allows for easier use of the VEX Library
using namespace vex;





   void wallStakeMotorControl () 
   {
    wallStakeMotor.setStopping(hold);
    wallStakeMotor.setMaxTorque(100,percent);
    wallStakeMotor.setVelocity(20,percent);
    while (true) {
        if (Controller1.ButtonUp.pressing()) {
            wallStakeMotor.spin(forward);
        } else if (Controller1.ButtonDown.pressing()) {
             wallStakeMotor.spin(reverse);
        } else {
             wallStakeMotor.stop();
        }
        
    }
}

enum clampState {UNCLAMPED, CLAMPED};
clampState currentClampState = UNCLAMPED;

// Function to unclamp
void clampUNCLAMPED() {
    clampMotor.setStopping(hold);
    clampMotor.spinToPosition(0, degrees);
    currentClampState = UNCLAMPED;
}

// Function to clamp
void clampCLAMPED() {
    clampMotor.spinToPosition(-190, degrees);
    currentClampState = CLAMPED;
}

// Function to handle state changes
void handleClampStateChange() {
    switch (currentClampState) {
        case UNCLAMPED:
            clampUNCLAMPED();
            break;
        case CLAMPED:
            clampCLAMPED();
            break;
        default:
            clampUNCLAMPED(); // Default to UNCLAMPED state
            break;
    }
}

// Control button and previous state
bool CchangeStateButton = false;
bool CpreviousStateButton = false;

// Function to control clamp motor
void clampMotorControl() {
    while (true) { // Continuously check button state and handle state change
        CchangeStateButton = Controller1.ButtonR2.pressing();

        if (CchangeStateButton && !CpreviousStateButton) {
            // Toggle the state of the clamp
            if (currentClampState == UNCLAMPED) {
                currentClampState = CLAMPED;
            } else {
                currentClampState = UNCLAMPED;
            }

            handleClampStateChange();
        }

        CpreviousStateButton = CchangeStateButton;

        wait(.2,seconds);
    } }
  





enum intakeMotorStates {OFF, ON};
intakeMotorStates currentIntakeState = OFF;

// Function to turn intake motor off
void intakeOFF() {
    intakeMotor.setStopping(hold);
    intakeMotor.stop();
    currentIntakeState = OFF;
}

// Function to turn intake motor on
void intakeON() {
    intakeMotor.setMaxTorque(100, percent);
    intakeMotor.setVelocity(100, percent);
    intakeMotor.spin(forward);
    currentIntakeState = ON;
}

// Function to handle state changes
void handleIntakeStateChange() {
    switch (currentIntakeState) {
        case OFF:
            intakeOFF();
            break;
        case ON:
            intakeON();
            break;
        default:
            intakeOFF(); // Default to OFF state
            break;
    }
}

// Control button and previous state
bool IchangeStateButton = false;
bool IpreviousButtonState = false;

// Function to control intake motor
void intakeMotorControl() {
    while (true) { // Continuously check button state and handle state change
        IchangeStateButton = Controller1.ButtonR2.pressing();

        if (IchangeStateButton && !IpreviousButtonState) {
            // Toggle the state of the intake motor
            if (currentIntakeState == OFF) {
                currentIntakeState = ON;
            } else {
                currentIntakeState = OFF;
            }

            handleIntakeStateChange();
        }

        IpreviousButtonState = IchangeStateButton;

        wait(20, msec); // Prevent button from being pressed multiple times
    }
}







int main() { 
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
thread clampThread = thread(clampMotorControl);

thread intakeThread = thread(intakeMotorControl);
thread wallThread = thread(wallStakeMotorControl);
  // Begin project code


} 
