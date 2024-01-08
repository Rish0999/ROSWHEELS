void setup() {
  Serial.begin(9600);
}

void loop() {
  // Serial.println(1);
  if (Serial.available() > 0) {
    // String linear_x = Serial.readString();
    // Serial.println(linear_x);
    int linear_x = Serial.read();
    int linear_y = Serial.read();
    int linear_z = Serial.read();
    int ang_x = Serial.read();
    int ang_y = Serial.read();
    int ang_z = Serial.read();

    // Serial.println(str(linear_x) + " " + str(linear_y) + " " + str(linear_z) + " " + str(ang_x) +  " " + str(ang_y) + " " + str(ang_z));
    Serial.println(linear_x);
    Serial.println(ang_z);
  }
}
