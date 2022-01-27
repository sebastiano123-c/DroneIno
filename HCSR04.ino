void dangerAlert() {
  // Clears the trigPin condition
  digitalWrite(PIN_PROXIMITY_SENSOR_TRIG, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(PIN_PROXIMITY_SENSOR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_PROXIMITY_SENSOR_TRIG, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(PIN_PROXIMITY_SENSOR_ECHO, HIGH);
  // Calculating the distance
  int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  if (distance < DANGER_DISTANCE){
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
}
