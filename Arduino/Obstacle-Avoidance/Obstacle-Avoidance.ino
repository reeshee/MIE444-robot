#include <Arduino.h>

// Motor pin definitions
const int in1 = 7;
const int in2 = 8;
const int enA = 9;
const int in3 = 13;
const int in4 = 12;
const int enB = 11;

// Serial communication command
String command;

void setup() {
  // Initialize motor pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Check for command from serial
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    command.trim();  // Remove any whitespace
    Serial.println("Received Command: " + command);  // Debugging line

    if (command.startsWith("rotate_right")) { // For the pathfinding
      Serial.println("Action: rotateRight()");
      rotateRight();
    } else if (command.startsWith("rotate_left")) { // For the pathfinding
      Serial.println("Action: rotateLeft()");
      rotateLeft();
    } else if (command.equals("move_forward")) { // For the pathfinding
      Serial.println("Action: moveForward()");
      moveForward();
    } else if (command.equals("move_backward")) { //For pathfinding
      Serial.println("Action: moveBackward()");
      moveBackward();


    } else if (command.startsWith("obs_rotateLeft")) { // for obstacle avoidance
      Serial.println("Action: obs_rotateLeft()");
      obs_rotateLeft();
    } else if (command.equals("obs_rotateRight")) { // for obstacle avoidance
      Serial.println("Action: obs_rotateRight()");
      obs_rotateRight();
    } else if (command.equals("obs_moveBackward")) { // for obsactle avoidance move back
      Serial.println("Action: obs_moveBackward()");
      obs_moveBackward();
    } else if (command.startsWith("obs_smallrotateLeft")) { // for obstacle avoidance
      Serial.println("Action: obs_smallrotateLeft()");
      obs_smallrotateLeft();
    } else if (command.equals("obs_smallrotateRight")) { // for obstacle avoidance
      Serial.println("Action: obs_smallrotateRight()");
      obs_smallrotateRight();
    } else if (command.equals("obs_moveForward")) { // for obsactle avoidance move back
      Serial.println("Action: obs_moveForward()");
      obs_moveForward();
    } else if (command.startsWith("adjust")) {
      int angle = extractAngleFromCommand(command, "adjust");
      Serial.println("Rotating left by " + String(angle) + " degrees.");
      adjust(angle);
    } else {
      Serial.println("Unknown Command: " + command);
    }
  }
}
void obs_moveForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 100);  // Set speed (adjust as needed)
  analogWrite(enB, 106);  // Set speed (adjust as needed)
  delay(150);  // Placeholder: Replace with appropriate timing for desired distance
  stopMotors();
}
void obs_moveBackward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 100);  // Set speed (adjust as needed)
  analogWrite(enB, 106);  // Set speed (adjust as needed)
  delay(50);  // Placeholder: Replace with appropriate timing for desired distance
  stopMotors();
}
void obs_rotateRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 100);  // Set speed (adjust as needed)
  analogWrite(enB, 106);  // Set speed (adjust as needed)
  delay(60);  // Placeholder: Replace with appropriate timing for desired angle
  stopMotors();
}

void obs_rotateLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 100);  // Set speed (adjust as needed)
  analogWrite(enB, 106);  // Set speed (adjust as needed)
  delay(60);  // Placeholder: Replace with appropriate timing for desired angle
  stopMotors();
}
void obs_smallrotateRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 100);  // Set speed (adjust as needed)
  analogWrite(enB, 106);  // Set speed (adjust as needed)
  delay(33);  // Placeholder: Replace with appropriate timing for desired angle
  stopMotors();
}

void obs_smallrotateLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 100);  // Set speed (adjust as needed)
  analogWrite(enB, 106);  // Set speed (adjust as needed)
  delay(33);  // Placeholder: Replace with appropriate timing for desired angle
  stopMotors();
}

void adjust(int angle) {
  long rotationTime = calculateRotationTime(angle);
  Serial.println("Rotating left by " + String(angle) + " degrees.");
  Serial.println("Rotation time (ms): " + String(rotationTime));  // Debugging line

  // Set motors to rotate left
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 100);  // Set speed (adjust as needed)
  analogWrite(enB, 106);

  delay(rotationTime);
  stopMotors();
}

int extractAngleFromCommand(String cmd, String prefix) {
  // Remove the prefix from the command
  String angleStr = cmd.substring(prefix.length());
  angleStr.trim();  // Remove any leading/trailing whitespace

  if (angleStr.length() == 0) {
    Serial.println("Error: No angle provided. Using default angle of 0.");
    return 0;
  }

  // Debug: Print the angleStr to check its content
  Serial.println("angleStr: '" + angleStr + "'");

  // Validate that angleStr contains only digits
  for (int i = 0; i < angleStr.length(); i++) {
    if (!isDigit(angleStr.charAt(i))) {
      Serial.println("Error: Invalid angle provided. Using default angle of 0.");
      return 0;
    }
  }

  int angle = angleStr.toInt();

  // Normalize angle to be within [0, 360)
  angle = angle % 360;

  return angle;
}

long calculateRotationTime(int angle) {
  long timeForFullRotation = 1000;  // Time in milliseconds for 360 degrees
  long rotationTime = (timeForFullRotation * angle) / 360;
  return rotationTime;
}

void moveForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 100);  // Set speed (adjust as needed)
  analogWrite(enB, 106);  // Set speed (adjust as needed)
  delay(100);  // Placeholder: Replace with appropriate timing for desired distance
  stopMotors();
}

void moveBackward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 100);  // Set speed (adjust as needed)
  analogWrite(enB, 106);  // Set speed (adjust as needed)
  delay(100);  // Placeholder: Replace with appropriate timing for desired distance
  stopMotors();
}

void rotateRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 100);  // Set speed (adjust as needed)
  analogWrite(enB, 106);  // Set speed (adjust as needed)
  delay(300);  // Placeholder: Replace with appropriate timing for desired angle
  stopMotors();
}

void rotateLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 100);  // Set speed (adjust as needed)
  analogWrite(enB, 106);  // Set speed (adjust as needed)
  delay(300);  // Placeholder: Replace with appropriate timing for desired angle
  stopMotors();
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}