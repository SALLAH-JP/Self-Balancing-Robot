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

    if ( hasDataIMU() ) { // when IMU has received the package
        // read pitch from the IMU
        input = getPitchIMU();
        pid.Compute();
        setpoint = EQUILIBRE;
        delta = 0.0;
        
        Serial.print(input); Serial.print(" =>"); Serial.println(output);
        if (input > EQUILIBRE - 40 && input < EQUILIBRE + 40) {

            if ( mode == 1 ) {
                directionRemoteControl();
            } else if ( mode == 2 ) {
                lineTracking();
            }

            driveMotors();

        } else {
            Stop();
        }
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
                    if ( cmd == 0x18 ) setpoint = EQUILIBRE + 5;
                    if ( cmd == 0x08 ) delta = -1.0;
                    if ( cmd == 0x5A ) delta = +1.0;
                    if ( cmd == 0x52 ) setpoint = EQUILIBRE - 5;
                }
            }

            if ( cmd == 0x45 ) mode = 3;
            if ( cmd == 0x44 ) mode = 1;
            if ( cmd == 0x19 ) mode = 2;
            if ( cmd == 0x07 ) mode = 0;
            if ( cmd == 0x46 ) buttonInt();

            if ( cmd == 0x0 )  Serial.println(F("Touche inconnue"));

            LedMode();
            lastRemote = millis();
        }

        IrReceiver.resume();
    }
}


void directionRemoteControl() {

}


void lineTracking() {

    leftValue = digitalRead(LEFT_SENSOR_PIN);
    rightValue = digitalRead(RIGHT_SENSOR_PIN);

    if ( leftValue == LOW && rightValue == LOW ) {
        setpoint = EQUILIBRE + 0.50;
    }


    if ( leftValue == HIGH) { 
        Serial.println("Left Line detected");
        delta = -1.0;
        setpoint = EQUILIBRE;
    } else if (rightValue == HIGH ) {
        Serial.println("Right Line detected");
        delta = +1.0;
        setpoint = EQUILIBRE;
    }

}
