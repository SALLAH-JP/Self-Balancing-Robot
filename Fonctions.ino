void buttonInt() {
  mode += 1;
  mode = mode % 3;

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

void driveMotors(float torque, float turn) {

  pwmL = torque - turn;
  pwmR = torque + turn;

  pwmL = constrain(pwmL, -255, 255);
  pwmR = constrain(pwmR, -255, 255);

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

float estimateVelocity(float angle, float dt) {

  if(dt < 0.001) dt = 0.01;
    
  // Calcul de la dérivée
  float angular_velocity = (angle - last_angle) / dt;

  // Filtre passe-bas pour lisser
  const float alpha = 0.7;
  estimated_velocity = alpha * estimated_velocity + (1 - alpha) * angular_velocity;

  last_angle = angle;

  return estimated_velocity;
}
