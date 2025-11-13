#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

// Initialize LCD display
LiquidCrystal_I2C lcd(0x27,16,4);

// Initialize servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Initialize Bluetooth module
SoftwareSerial bt1(2,3); /* (Rx,Tx) */

// Define servo motor parameters
const int MIN_PULSE_WIDTH = 500;
const int MAX_PULSE_WIDTH = 2500;
const int DEFAULT_PULSE_WIDTH = 1500;
const int FREQUENCY = 50;
const float FREQUENCY_SCALE = (float)FREQUENCY * 4096 / 1000000;

// // Function to calculate pulse width for a given angle
// int pulseWidth(int angle) {
//     int pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
//     int analog_value = int(pulse_wide * FREQUENCY_SCALE);
//     Serial.println(analog_value);
//     return analog_value;
// }


int servoAngles[6]; // Global array to hold the angles for all six servos

// Function to calculate pulse width for a given angle
int pulseWidth(int angle) {
    int pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    return int(pulse_wide * FREQUENCY_SCALE);
}

// Define buttons and potentiometers
int leftButton = 7;
int rightButton = 8;
int potentiometer1 = A0;
int potentiometer2 = A1;
int potentiometer3 = A2;
int potentiometer4 = A3;
int potentiometer5 = A4;

// Define record section
int buttonStartRecord = 9;
int buttonStopRecord = 10;
int buttonPlayRecord = 11;
int ledStopRecord = 12;
int ledStartRecord = 13;

bool isRecord = false;
bool isPlay = false;
int indexRecord = 0;

// Define maximum moves and servos
const int moveCount = 10;
const int servoNumber = 6;


int movesServos[moveCount][servoNumber]; //max of 10 moves

// Setup function
void setup() {
    Serial.begin(9600);
    bt1.begin(9600);

    // Initialize LCD display
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.begin(16, 4);      // set up the LCD's number of columns and rows
    lcd.setCursor(0, 0);
    lcd.print("2023 GDIP F1 :");
    lcd.setCursor(0, 1);
    lcd.print("6 DOF ROBOTIC ARM ");
    lcd.setCursor(0, 2);
    lcd.print("PROTOTYPE 1");

    // Initialize servo driver
    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);

    // Move servo motors to initial positions
    pwm.setPWM(1,0,pulseWidth(125));
    pwm.setPWM(2,0,pulseWidth(180));
    pwm.setPWM(3,0,pulseWidth(190));
    pwm.setPWM(4,0,pulseWidth(180));
    pwm.setPWM(5,0,pulseWidth(190));
    pwm.setPWM(6,0,pulseWidth(80));

    pinMode(leftButton, INPUT);
    pinMode(rightButton, INPUT);

  //Record section
    pinMode(buttonStartRecord, INPUT);
    pinMode(buttonStopRecord, INPUT);
    pinMode(buttonPlayRecord, INPUT);

    pinMode(ledStartRecord, OUTPUT);
    pinMode(ledStopRecord, OUTPUT);


}

// Main loop
void loop() {
     String command = "";

    // Read commands from serial port or Bluetooth module
    if (Serial.available()) {
        command = Serial.readStringUntil('\n');
        command.trim(); // Trim any newline or carriage return characters
        Serial.println("Received: " + command); // Echo the command back to the serial monitor
    } else if (bt1.available()) {
        command = bt1.readStringUntil('\n');
        command.trim(); // Trim any newline or carriage return characters
        Serial.println("Received via BT: " + command); // Echo the command from BT
    }

    // Execute command if available
    if (command != "") {
        executeCommand(command);
    }
}

// Function to print the pulse widths for the current angles
void printServoPulseWidths() {
    for (int i = 0; i < 6; i++) {
        int currentPulseWidth = pulseWidth(servoAngles[i]);
        Serial.print("Servo ");
        Serial.print(i);
        Serial.print(" Angle: ");
        Serial.print(servoAngles[i]);
        Serial.print(" Pulse Width: ");
        Serial.println(currentPulseWidth);
    }
}


void MoveToPick() {
    // Define the sequence of angles for each servo in the pick-up motion
    int angles[10][servoNumber] = {
        // HIP, WAIST, SHOULDER, ELBOW, WRIST, CLAW
        {125, 180, 190, 180, 190, 80},    // Initial position
        {150, 110, 100, 190, 90, 80}, // Move to above the object
        {150, 50, 180, 150, 180, 80}, // Lower towards the object
        {150, 50, 180, 150, 180, 140},// Close claw to grab the object
        {150, 90, 100, 150, 90, 140},// Lift the object
        {125, 180, 190, 180, 190, 120},    // Return to initial position with object
        {150, 110, 100, 190, 90, 80}, // Move the object
        {150, 40, 180, 150, 180, 80}, // Lower  the object
        {150, 40, 180, 150, 180, 80}, // OPEN claw to RELEASE the object
        {125, 180, 190, 180, 190, 120}    // Return to initial position with object
    };
    
    

    // Execute the sequence
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < servoNumber; j++) {
            pwm.setPWM(j + 1, 0, pulseWidth(angles[i][j]));
        }
        delay(2000); // Wait for 1 second between each step for smooth movement
    }

    // Optionally, open the claw to release the object at the end
    // pwm.setPWM(6, 0, pulseWidth(0)); // Open the claw
}

void MoveToStart() {
    pwm.setPWM(1,0,pulseWidth(125));
    pwm.setPWM(2,0,pulseWidth(180));
    pwm.setPWM(3,0,pulseWidth(190));
    pwm.setPWM(4,0,pulseWidth(180));
    pwm.setPWM(5,0,pulseWidth(190));
    pwm.setPWM(6,0,pulseWidth(80));
    
    
    
}

// Correct the moveServo to use 0-based index for servoAngles array
void moveServo(int servoChannel, int angle) {
    int servoIndex = servoChannel - 1; // Convert 1-based index to 0-based index
    servoAngles[servoIndex] = angle;

    // Then set the servo to the new angle using the 1-based channel number
    pwm.setPWM(servoChannel, 0, pulseWidth(angle));
}



void executeCommand(String command) {

   command.trim(); // Trim whitespace
    command.toUpperCase(); // Convert to upper case for case-insensitive comparison

    // Define servo commands
    struct ServoCommand {
        const char* name;
        int channel;
    };

    ServoCommand commands[] = {
        {"HIP ", 1},
        {"WAIST ", 2},
        {"SHOULDER ", 3},
        {"ELBOW ", 4},
        {"WRIST ", 5},
        {"CLAW ", 6}
    };

    // Check for the MOVE_TO_PICK command
    if (command == "MOVE_TO_PICK") {
        MoveToPick();
        Serial.println("Executing MoveToPick sequence");
        return;
    }
    if (command == "GETVALUE") {
         printServoPulseWidths();
         return;
    }
    if (command == "MOVE_TO_START") {
        MoveToStart();
        Serial.println("Executing MoveToPick sequence");
        return;
    }

    

    // // Parse and execute other servo commands
    // for (const auto& cmd : commands) {
    //     if (command.startsWith(cmd.name)) {
    //         int angle = command.substring(strlen(cmd.name)).toInt();
    //         pwm.setPWM(cmd.channel, 0, pulseWidth(angle));
    //         Serial.println(String(cmd.name) + "moved to " + String(angle) + " degrees");
    //         return;
    //     }
    // }
    // Corrected usage in executeCommand

    // Parse and execute servo commands
    for (const auto& cmd : commands) {
        if (command.startsWith(cmd.name)) {
            int angle = command.substring(strlen(cmd.name)).toInt();
            // Use the channel directly since moveServo now handles the index conversion
            moveServo(cmd.channel, angle);
            Serial.println(String(cmd.name) + "moved to " + String(angle) + " degrees");
            return;
        }
    }


    // Unknown command
    Serial.println("Unknown command: " + command);
}
