enum wallStakeState { IDLE, LOADING_1, LOADING_2, SCORING };
     wallStakeState currentState = IDLE;

double kP = 0.5; // Proportional gain for PD controller
double kD = 0.1; // Derivative gain for PD controller
double targetPosition = 0.0;
double currentPosition = 0.0;
double error = 0.0;
double previousError = 0.0; 
double deltaTime = 0.01; // 10ms loop time 
double maxMotorPower = 100.0; // %

  double lowPassFilter(double input, double prevOutput, double alpha) {
      return alpha * input + (1.0 - alpha) * prevOutput;}

void moveArmMotorTo (double target) {
      targetPosition = target; // Set the target position directly
    wallStakeMotor.setBrake(brakeType::coast); // Set the brake type to coast so that the motor can move freely
    previousError = 0.0;
    double filteredPosition = 0.0;
    double alpha = 0.1; // A value between 0 and 1; the larger the value, the more reactive the PD controller is to current readings
}
    
            intakeMotor.stop();
        while (1) {
        double rawPosition = potentiometer1.angle(degrees); // Read the position from the potentiometer
        filteredPosition = lowPassFilter(rawPosition, filteredPosition, alpha);
        currentPosition = filteredPosition;

        error = targetPosition - currentPosition; // Calculate the error to use P control to smoothly fix the error

        double derivative = (error - previousError) / deltaTime;
        double motorPower = (kP * error) + (kD * derivative);

        // Limit the motor power to the maximum allowed value
        motorPower = motorPower > maxMotorPower ? maxMotorPower : (motorPower < -maxMotorPower ? -maxMotorPower : motorPower);

        wallStakeMotor.spin(forward, motorPower, percent); // Uses all the math we did above to move the arm

        previousError = error; // Sets the error to previous error so that it can be used to calculate the derivative in the next loop

        if (error > -1 && error < 1)
            wallstake_motor.stop(brakeType::hold); // Once the error is small enough, stop the motor
            break; // Exit the loop when the target position is reached
        }
