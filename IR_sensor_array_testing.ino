void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue1 = analogRead(A2); // each sensors is assigned to a respctive analog pin in arduino
  int sensorValue2 = analogRead(A3);
  int sensorValue3 = analogRead(A4);
  int sensorValue4 = analogRead(A5);
  int sensorValue5 = analogRead(A6);
  int sensorValue6 = analogRead(A7);

  Serial.print(sensorValue1); //serial output of sensor readings 
  Serial.print(","); 
  Serial.print(sensorValue2);
  Serial.print(","); 
  Serial.print(sensorValue3);
  Serial.print(","); 
  Serial.print(sensorValue4);
  Serial.print(","); 
  Serial.print(sensorValue5);
  Serial.print(","); 
  Serial.println(sensorValue6);
  delay(100);
}
