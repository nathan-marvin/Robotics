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
          double target = storedPosition + positionThreshold; // Set the target position to the stored position plus the threshold
        
          while (hook_motor.position(degrees) >= target) { // Wait until the degrees spinned is greater than or equal to the target position
            wait(20, msec); // Short delay to prevent CPU overload
          }
          
          // Stop the motor with a hold to change its momentum
          hook_motor.stop(hold);
      }

            wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
        }
      }
