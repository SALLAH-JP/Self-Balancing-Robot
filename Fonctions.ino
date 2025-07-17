void dmpDataReady() {
  mpuInterrupt = true;
}

void buttonInt() {
  mode += 1;
  mode = mode % 4;

  LedMode();
}


void LedMode() {  
  switch (mode) {
    case 0:
    // Mode Balancing : JAUNE
    digitalWrite(RED_PIN, LOW);
    digitalWrite(GREEN_PIN, LOW);
    //digitalWrite(BLUE_PIN, HIGH);
    break;

    case 1:
    // Mode RemoteControl : VERT
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(GREEN_PIN, LOW);
    //digitalWrite(BLUE_PIN, HIGH);
    break;

    case 2:
    // Mode LineTracking : ROUGE
    digitalWrite(RED_PIN, LOW);
    digitalWrite(GREEN_PIN, HIGH);
    //digitalWrite(BLUE_PIN, HIGH);
    break;

    case 3:
    // Mode Off : ETEINT
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(GREEN_PIN, HIGH);
    //digitalWrite(BLUE_PIN, LOW);
    break;
  }
}

void Backward() { //Code to rotate the wheel Backward
  vitesse = output;

  analogWrite(LEFT_MOTOR_PIN1, vitesse);
  analogWrite(LEFT_MOTOR_PIN2, 0);
  analogWrite(RIGHT_MOTOR_PIN1, vitesse);
  analogWrite(RIGHT_MOTOR_PIN2, 0);
  Serial.print("F"); //Debugging information 
}

void Forward() { //Code to rotate the wheel Forward
  vitesse = output*-1;

  analogWrite(LEFT_MOTOR_PIN1, 0);
  analogWrite(LEFT_MOTOR_PIN2, vitesse);
  analogWrite(RIGHT_MOTOR_PIN1, 0);
  analogWrite(RIGHT_MOTOR_PIN2, vitesse);
  Serial.print("R");
}

void Stop() { //Code to stop both the wheels
  analogWrite(LEFT_MOTOR_PIN1, 0);
  analogWrite(LEFT_MOTOR_PIN2, 0);
  analogWrite(RIGHT_MOTOR_PIN1, 0);
  analogWrite(RIGHT_MOTOR_PIN2, 0);
}

void ControlForward() {
  setpoint = 181;
  start = millis();
  Serial.print("CF");
}

void ControlBackward() {
  setpoint = 171;
  start = millis();
  Serial.print("CB");
}

void Right() {
  vitesse = output;

  analogWrite(LEFT_MOTOR_PIN1, 0);
  analogWrite(LEFT_MOTOR_PIN2, vitesse);
  analogWrite(RIGHT_MOTOR_PIN1, 0);
  analogWrite(RIGHT_MOTOR_PIN2, 0);
  Serial.print("L");
}

void Left() {
  vitesse = output;

  analogWrite(LEFT_MOTOR_PIN1, 0);
  analogWrite(LEFT_MOTOR_PIN2, 0);
  analogWrite(RIGHT_MOTOR_PIN1, 0);
  analogWrite(RIGHT_MOTOR_PIN2, vitesse);
  Serial.print("R");
}
