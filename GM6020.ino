#include <Servo.h>

Servo motor;  // Create a Servo object to control the motor

const int PWM_PIN = 9;  // PWM output pin for motor control

void setup() {
  Serial.begin(115200);  // Initialize serial monitor
  motor.attach(PWM_PIN); // Attach the Servo object to the PWM pin

  Serial.println("Enter a command:");
  Serial.println("'A' followed by an angle (0 to 180 degrees) to set motor angle");
  Serial.println("'D' to stop the motor");
}

void loop() {
  if (Serial.available() > 0) {
    // Read input as a string
    String input = Serial.readStringUntil('\n');

    // Check for 'A' command to set angle
    if (input.startsWith("A")) {
      // Extract angle value from the input string
      int pw = input.substring(1).toInt();
      pw = map(pw, 0, 360, 1080, 1920);

      // Ensure pw is within valid range
      if (pw < 1080 || pw > 1920) {
        Serial.println("Invalid angle. Please enter an angle between 0 and 180 degrees.");
        return;
      }

      // Write the angle value to the motor
      motor.write(pw);

      // Print angle value for debugging
      Serial.print("Motor angle set to: ");
      Serial.println(pw);

    } else if (input.equals("D")) {  // Check for 'D' command to stop the motor
      stopMotor();
      Serial.println("Motor stopped.");
    } else {
      Serial.println("Invalid command. Use 'A' followed by angle or 'D' to stop the motor.");
    }
  }

  delay(100); // Adjust delay as needed
}

// Function to stop the motor
void stopMotor() {
  motor.write(90); // Set angle to neutral value (90 degrees) to stop the motor
}
