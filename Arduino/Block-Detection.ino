#define TRIGGER_PIN 12
#define ECHO_PIN 9
#define MAX_DISTANCE_CM 400  // Maximum valid distance for HC-SR04 (4 meters)
#define MIN_DISTANCE_CM 2    // Minimum valid distance for HC-SR04 (2 cm)

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600); // Initialize Serial Monitor
  Serial.println("Type 'scan' and press Enter to take a measurement.");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input until newline
    input.trim(); // Remove any leading/trailing whitespace

    if (input.equalsIgnoreCase("scan")) {
      // Trigger the ultrasonic sensor
      digitalWrite(TRIGGER_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIGGER_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGGER_PIN, LOW);

      // Measure the echo time
      long duration = pulseIn(ECHO_PIN, HIGH);

      // Check if no echo was received
      if (duration == 0) {
        Serial.println("Object not in front."); // No echo received
        return;
      }

      // Calculate distance in centimeters
      float distance = (duration * 0.0343) / 2;

      // Validate the distance
      if (distance < MIN_DISTANCE_CM || distance > MAX_DISTANCE_CM) {
        Serial.println("Object not in front."); // Out of range
        return;
      }

      // Print the valid distance
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
    } else {
      Serial.println("Invalid command. Type 'scan' to take a measurement.");
    }
  }
}
