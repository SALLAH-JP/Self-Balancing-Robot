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
      Kmix = 150;
      // Mode RemoteControl : VERT
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(GREEN_PIN, LOW);
      //digitalWrite(BLUE_PIN, HIGH);
      break;

    case 2:
      Kmix = 25;
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

void driveMotors() {

  pwmL = output - Kmix * delta;
  pwmR = output + Kmix * delta;

  // Gauche
  if (pwmL >= 0) {
    analogWrite(LEFT_MOTOR_PIN1, (int)pwmL);
    analogWrite(LEFT_MOTOR_PIN2, 0);
  } else {
    analogWrite(LEFT_MOTOR_PIN1, 0);
    analogWrite(LEFT_MOTOR_PIN2, (int)(-pwmL));
  }
  // Droite
  if (pwmR >= 0) {
    analogWrite(RIGHT_MOTOR_PIN1, (int)pwmR);
    analogWrite(RIGHT_MOTOR_PIN2, 0);
  } else {
    analogWrite(RIGHT_MOTOR_PIN1, 0);
    analogWrite(RIGHT_MOTOR_PIN2, (int)(-pwmR));
  }
}

void Stop() { //Code to stop both the wheels
  analogWrite(LEFT_MOTOR_PIN1, 0);
  analogWrite(LEFT_MOTOR_PIN2, 0);
  analogWrite(RIGHT_MOTOR_PIN1, 0);
  analogWrite(RIGHT_MOTOR_PIN2, 0);
}
