void setup() {
  Serial.begin(9600);
}
void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    //Serial.print("You sent me: ");
    Serial.println(data);
  }
  //Serial.println(analogRead(A0));
}
