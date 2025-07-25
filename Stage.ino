#include "header.h"

void setup() {
    Serial.begin(115200);


    // IR Receiver Setup and interrupt
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    initIMU();

    // PID Angle
    setpoint_angle = EQUILIBRE;
    pid_angle.SetMode(AUTOMATIC);
    pid_angle.SetSampleTime(10);
    pid_angle.SetOutputLimits(-200, 200);
    

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
        driveMotors(0, 0, 0);
    } else {
        balancing();
    }

    remoteControl();
}


void balancing() {

    if (!dmpReady) return;

    if ( hasDataIMU() ) {

        pid_angle.Compute();

        steering = 0;
        vitesse = 1;
                
        Serial.print(input_angle); Serial.print(" =>"); Serial.println(output_angle);
        if (abs(input_angle - EQUILIBRE) > 40) {
            driveMotors(0, 0, 0);
            return;
        }
        
        if (mode == 2) lineTracking();
        else {
            setpoint_angle = EQUILIBRE;
            if (mode == 1) directionRemoteControl();
        }



        driveMotors(output_angle, vitesse, steering);

    } else {
        input_angle = getPitchIMU();
    }
}


void remoteControl() {
    // **on décode réellement ici**  
    if (IrReceiver.decode()) {

        cmd = IrReceiver.decodedIRData.command;
        Serial.print(F("Nouvelle cmd = 0x")); Serial.println(cmd, HEX);

        if ( ( (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) && (millis() - lastRemote > 200) ) || !(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) ) {

            if ( mode == 1 || mode == 2) {
                if ( mode == 1) {
                    if ( cmd == 0x18 ) startForward = millis();
                    if ( cmd == 0x08 ) startLeft = millis();
                    if ( cmd == 0x5A ) startRight = millis();
                    if ( cmd == 0x52 ) startBackward = millis();
                }

                //if ( cmd == 0x09 ) controlVitesse = constrain(controlVitesse + 10, 50, 255);
                //if ( cmd == 0x15 ) controlVitesse = constrain(controlVitesse - 10, 50, 255);
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

    if ( leftValue == LOW && rightValue == LOW ) setpoint_angle = constrain(setpoint_angle + .0001, -2.5, -1.9);
    else {  
        setpoint_angle = -2.5;
        if ( leftValue == HIGH) steering = -25;
        else if (rightValue == HIGH ) steering = +25;
    }

}

void directionRemoteControl() {

    if (millis() - startForward < 100) vitesse = +100;
    else if (millis() - startBackward < 100) vitesse = -100;
    else if (millis() - startLeft < 100) steering = -50;
    else if (millis() - startRight < 100) steering = +50;

}

