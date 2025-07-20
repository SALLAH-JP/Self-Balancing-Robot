#include "header.h"
#include "imu_helpers.h"

void setup() {
    Serial.begin(115200);


    // IR Receiver Setup and interrupt
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    delay(1000);
    if ( !initIMU() ) {
        Serial.println(F("IMU connection problem... Disabling!"));
        return;
    }
    delay(1000);

    // PID Angle
    pid_angle.SetMode(AUTOMATIC);
    pid_angle.SetSampleTime(5); // 5ms = 200Hz
    pid_angle.SetOutputLimits(-200, 200);
    setpoint_angle = EQUILIBRE; // Votre angle d'équilibre

    // PID Vitesse
    pid_vitesse.SetMode(AUTOMATIC);
    pid_vitesse.SetSampleTime(20); // 20ms = 50Hz
    pid_vitesse.SetOutputLimits(-5, 5); // Limite l'angle cible
    setpoint_vitesse = 0; // Vitesse initiale nulle
    

    // Button Seput and interrupt with the module PinChangeInterrupt
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInt, 2);


    // Led RGB Setup
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    //pinMode(BLUE_PIN, OUTPUT);
    LedMode(); // POUR SYNCHRRONISER LE MODE ET LA LED


    // Tracking Sensor Setup
    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);

    //Initialise the Motor output pins
    pinMode(LEFT_MOTOR_PIN1, OUTPUT);
    pinMode(LEFT_MOTOR_PIN2, OUTPUT);

    pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN2, OUTPUT);

    //By default turn off both the motors
    analogWrite(LEFT_MOTOR_PIN1, LOW);
    analogWrite(LEFT_MOTOR_PIN2, LOW);

    analogWrite(RIGHT_MOTOR_PIN1, LOW);
    analogWrite(RIGHT_MOTOR_PIN2, LOW);
}

void loop() {

    
    if ( mode == 3 ) {
        Stop();
    } else {
        balancing();
    }

    remoteControl();
}


void balancing() {
    dt = (millis() - last_time) / 1000.0f;
    last_time = millis();

    if ( hasDataIMU() ) { // when IMU has received the package
        // read pitch from the IMU

        input_angle = getPitchIMU();
        input_vitesse = estimateVelocity(input_angle, dt);
        pid_vitesse.Compute();

        setpoint_angle = EQUILIBRE + output_vitesse;
        
        pid_angle.Compute();
        
        Serial.print(input_angle); Serial.print(" =>"); Serial.println(output_angle);
        if (abs(input_angle - EQUILIBRE) > 40) {
            Stop();
            return;
        }
        
        if (mode == 1) {
            // Remote control - géré dans remoteControl()
        } else if (mode == 2) {
            lineTracking();
        } else {
            steering = 0; // Mode 0: pas de direction
            setpoint_vitesse = 0;
        }

        driveMotors(output_angle, steering);

    }
}


void remoteControl() {
    // **on décode réellement ici**  
    if (IrReceiver.decode()) {

        cmd = IrReceiver.decodedIRData.command;
        Serial.print(F("Nouvelle cmd = 0x")); Serial.println(cmd, HEX);

        steering = 0;
        setpoint_vitesse = 0;

        if ( ( (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) && (millis() - lastRemote > 200) ) || !(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) ) {

            if ( mode == 1) {
                if ( cmd == 0x18 ) setpoint_vitesse = 1.0;
                if ( cmd == 0x08 ) steering = -150;
                if ( cmd == 0x5A ) steering = +150;
                if ( cmd == 0x52 ) setpoint_vitesse = -1.0;
            }

            if ( cmd == 0x45 ) mode = 3;
            else if ( cmd == 0x44 ) mode = 1;
            else if ( cmd == 0x19 ) mode = 2;
            else if ( cmd == 0x07 ) mode = 0;
            else if ( cmd == 0x46 ) buttonInt();

            LedMode();
            lastRemote = millis();
        }

        IrReceiver.resume();
    }
}


void lineTracking() {

    leftValue = digitalRead(LEFT_SENSOR_PIN);
    rightValue = digitalRead(RIGHT_SENSOR_PIN);

    steering = 0;
    setpoint_vitesse = 0.5;


    if ( leftValue == HIGH) steering = -30;
    else if (rightValue == HIGH ) steering = +30;

}
