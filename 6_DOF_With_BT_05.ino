//recording functions and testing
//LCD updates
//add are you sure feature and Initialize movesServos at the Start of Recording:



#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// Initialize LCD display
LiquidCrystal_I2C lcd(0x27,16,4);
// Initialize servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Initialize Bluetooth module
SoftwareSerial bt1(6,7); /* (Rx,Tx) */ for Jakes PCB

// Define servo motor parameters
const int MIN_PULSE_WIDTH = 500;
const int MAX_PULSE_WIDTH = 2500;
const int DEFAULT_PULSE_WIDTH = 1500;
const int FREQUENCY = 50;
const float FREQUENCY_SCALE = (float)FREQUENCY * 4096 / 1000000;

// Arm geometry for inverse kinematics (millimeters)
const float BASE_HGT  = 90.2f;  // L1  base height     (J1 -> J2)
const float HUMERUS  = 105.0f;  // L2  shoulder link   (J2 -> J3)
const float ULNA     = 127.0f;  // L3  elbow link      (J3 -> J4)
const float GRIPPER  = 75.0f;   // L4  wrist -> tool   (J4 -> TCP)

const float RAD_TO_DEG = 180.0f / PI;
const float DEG_TO_RAD = PI / 180.0f;

// Servo calibration helpers (adjust to match the physical build)
const float SERVO_NEUTRAL[6]   = {100.0f, 90.0f, 110.0f, 95.0f, 180.0f,   0.0f};
const float SERVO_DIRECTION[6] = {  1.0f,  1.0f,  -1.0f,  1.0f,   1.0f,   1.0f};
const float SERVO_SCALE[6]     = {  1.0f,  1.0f,   1.0f,  1.0f,   1.0f,   1.0f};

// Define maximum moves and servos
const int moveCount = 10;
const int servoNumber = 6;

int servoAngles[6] = {100, 90, 110, 95, 180, 0}; // Initial angles for each servo
int movesServos[moveCount][servoNumber]; //max of 10 moves
// Global variable to track state
bool waitingForConfirmation = false; // safety check, stop user from acidently runing the squence when not ready
bool isRecord = false;
bool isPlay = false;
int indexRecord = 0;

// Function Prototypes function must be declared before it is called unless it is defined above the point where it is called.
int pulseWidth(int angle);
void executeCommand(String command);
void printServoPulseWidths();
void MoveToStart();
void moveServo(int servoChannel, int angle);
void StartRecordingMovements();
void StopRecordingMovements();
void PlayRecordedMovements();
bool inverseKinematics(float x, float y, float z, float wristTiltDeg, float& hipDeg, float& shoulderDeg, float& elbowDeg, float& wristDeg);
bool moveToPose(float x, float y, float z, float wristTiltDeg, float clawAngle);
bool parseFloats(String args, float* values, int expectedCount);


//////////////////// Setup function///////////////////////////////////////////////////////
void setup() {
    Serial.begin(9600);
    bt1.begin(9600);

    // Initialize LCD display
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("2023 GDIP F1 :");
    lcd.setCursor(0, 1);
    lcd.print("6 DOF ROBOTIC ARM ");
    lcd.setCursor(0, 2);
    lcd.print("PROTOTYPE 4");

    // Initialize servo driver
    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);

   
    // Initialize servoAngles array with predefined angles
    servoAngles[0] = 100;
    servoAngles[1] = 90;
    servoAngles[2] = 110;
    servoAngles[3] = 95;
    servoAngles[4] = 180;
    servoAngles[5] = 0;

    // Move servo motors to initial positions based on servoAngles
    for (int i = 0; i < 6; i++) {
        pwm.setPWM(i + 1, 0, pulseWidth(servoAngles[i]));
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop
void loop() {
    String command = "";

    if (Serial.available() || bt1.available()) {
        // Read command from Serial or Bluetooth
        command = (Serial.available() ? Serial.readStringUntil('\n') : bt1.readStringUntil('\n'));
        command.trim();

        if (waitingForConfirmation) {
            if (command.equalsIgnoreCase("YES")) {
                PlayRecordedMovements();
            } else {
                Serial.println("Playback cancelled.");
                bt1.println("Playback cancelled."); // Send cancellation message over Bluetooth
            }
            waitingForConfirmation = false; // Reset the confirmation flag
        } else {
            // Process the command normally
            executeCommand(command);
        }
    }
}

/////////////////// Function to start recording servo movements///////////////////////////
void StartRecordingMovements() {
    isRecord = true;
    indexRecord = 0;

    // Initialize movesServos with current servo angles 
    //the movesServos array is filled with the current angles of each servo. 
    //This way, if you don't move a particular servo during a recording, it retains its initial angle 
    for (int i = 0; i < servoNumber; i++) {
        for (int j = 0; j < moveCount; j++) {
            movesServos[j][i] = servoAngles[i];
        }
    }

    Serial.println("Recording started.");
    lcd.clear();
    lcd.print("Recording...");
    bt1.println("Recording started.");
}


/////////////////// Function to stop recording servo movements////////////////////////////////
void StopRecordingMovements() {
    isRecord = false;
    Serial.println("Recording stopped. Final recorded moves:");
    bt1.println("Recording stopped. Final recorded moves:");  // Added
    for (int i = 0; i < indexRecord; i++) {
        Serial.print("Move "); Serial.print(i); Serial.println(":");
        bt1.print("Move "); bt1.print(i); bt1.println(":");  // Added
        for (int j = 0; j < servoNumber; j++) {
            Serial.print("Servo "); Serial.print(j + 1);
            Serial.print(": Angle "); Serial.println(movesServos[i][j]);
            bt1.print("Servo "); bt1.print(j + 1);  // Added
            bt1.print(": Angle "); bt1.println(movesServos[i][j]);  // Added
        }
    }
    lcd.clear();
    lcd.print("Recorded Moves:"); // Show how many moves were recorded
    lcd.setCursor(0, 1);
    lcd.print(indexRecord);
}

/////////////////////Function to play back recorded movements///////////////////////////
void PlayRecordedMovements() {
    Serial.println("Starting playback...");
    bt1.println("Starting playback...");  // Added
    lcd.clear();
    lcd.print("Playing Moves..."); // Indicate playback on the LCD
    isPlay = true;
    for (int i = 0; i < indexRecord; i++) {
        Serial.print("Move: "); Serial.println(i);
        bt1.print("Move: "); bt1.println(i);  // Added
        for (int j = 0; j < servoNumber; j++) {
            int pulse = pulseWidth(movesServos[i][j]);
            Serial.print("Servo "); Serial.print(j + 1); 
            Serial.print(" Angle: "); Serial.print(movesServos[i][j]);
            Serial.print(" Pulse Width: "); Serial.println(pulse);
            bt1.print("Servo "); bt1.print(j + 1);  // Added
            bt1.print(" Angle: "); bt1.print(movesServos[i][j]);  // Added
            bt1.print(" Pulse Width: "); bt1.println(pulse);  // Added
            pwm.setPWM(j + 1, 0, pulse);
        }
        delay(1000);  // Delay between moves, adjust as needed
    }
    for (int i = 0; i < indexRecord; i++) {
        lcd.setCursor(0, 1);
        lcd.print("Move: ");
        lcd.print(i + 1); // Update the move number on the LCD
        // ... rest of the playback code
    }

    isPlay = false;
    Serial.println("Playback completed.");
    bt1.println("Playback completed.");  // Added
    lcd.clear();
    lcd.print("Playback done."); // Indicate playback is done
}
//////////////////////////////////////////////////////////////////////////////////////////////////
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
        bt1.print("Servo ");  // Added
        bt1.print(i);  // Added
        bt1.print(" Angle: ");  // Added
        bt1.print(servoAngles[i]);  // Added
        bt1.print(" Pulse Width: ");  // Added
        bt1.println(currentPulseWidth);  // Added
    }
}

void MoveToStart() {
    
    servoAngles[0] = 100; //HIP servo
    servoAngles[1] = 90;  //WAIST servo
    servoAngles[2] = 10;  //SHOUDLER servo
    servoAngles[3] = 95;  //ELBOW servo
    servoAngles[4] = 180; //WRIST servo
    servoAngles[5] = 0;   //CLAW servo

    // Use a loop to set each servo to its start position
    for (int i = 0; i < 6; i++) {
        pwm.setPWM(i + 1, 0, pulseWidth(servoAngles[i]));
    }
    Serial.println("Moved to start positions.");
    bt1.println("Moved to start positions.");  // Added
    lcd.clear();
    lcd.print("Moved to Start"); // Indicate the arm has moved to the start position
    lcd.setCursor(0, 2);
    lcd.print("Warning ARM is ready");
}


    

//////////////////////////////////////////////////////////////////////////////////////////////////
// Correct the moveServo to use 0-based index for servoAngles array
void moveServo(int servoChannel, int angle) {
    int servoIndex = servoChannel - 1; // Convert 1-based index to 0-based index
    servoAngles[servoIndex] = angle;
    int pulse = pulseWidth(angle);

    Serial.print("Moving Servo "); Serial.print(servoChannel);
    Serial.print(" to Angle: "); Serial.print(angle);
    Serial.print(" (Pulse Width: "); Serial.print(pulse); Serial.println(")");
    bt1.print("Moving Servo "); bt1.print(servoChannel);
    bt1.print(" to Angle: "); bt1.print(angle);
    bt1.print(" (Pulse Width: "); bt1.print(pulse); bt1.println(")"); // Adding Bluetooth print

    pwm.setPWM(servoChannel, 0, pulse);
}


// Function to calculate pulse width for a given angle
int pulseWidth(int angle) {
    int pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    return int(pulse_wide * FREQUENCY_SCALE);
}
//////////////////////////////////////////////////////////////////////////////////////////////

bool parseFloats(String args, float* values, int expectedCount) {
    args.trim();
    int collected = 0;
    while (collected < expectedCount && args.length() > 0) {
        int separator = args.indexOf(' ');
        String token;
        if (separator == -1) {
            token = args;
            args = "";
        } else {
            token = args.substring(0, separator);
            args = args.substring(separator + 1);
        }
        token.trim();
        if (token.length() == 0) {
            args.trim();
            continue;
        }
        values[collected++] = token.toFloat();
        args.trim();
    }
    return collected == expectedCount;
}

bool inverseKinematics(float x, float y, float z, float wristTiltDeg, float& hipDeg, float& shoulderDeg, float& elbowDeg, float& wristDeg) {
    float planarRadius = sqrt(x * x + y * y);
    float wristPitchRad = wristTiltDeg * DEG_TO_RAD;
    float hipRad = atan2(y, x);
    hipDeg = hipRad * RAD_TO_DEG;

    float wristZ = z - BASE_HGT;
    float wristX = planarRadius - (GRIPPER * cos(wristPitchRad));
    float wristY = wristZ - (GRIPPER * sin(wristPitchRad));

    float reach = sqrt((wristX * wristX) + (wristY * wristY));
    float reachMax = HUMERUS + ULNA;
    float reachMin = fabs(HUMERUS - ULNA);

    if (reach > reachMax || reach < reachMin) {
        return false;
    }

    if (reach < 1e-3f) {
        reach = 1e-3f;
    }

    float shoulderOffset = atan2(wristY, wristX);
    float shoulderInner = acos(constrain((HUMERUS * HUMERUS + reach * reach - ULNA * ULNA) / (2.0f * HUMERUS * reach), -1.0f, 1.0f));
    float shoulderRad = shoulderOffset + shoulderInner;

    float elbowInner = acos(constrain((HUMERUS * HUMERUS + ULNA * ULNA - reach * reach) / (2.0f * HUMERUS * ULNA), -1.0f, 1.0f));
    float elbowRad = PI - elbowInner;

    float wristRad = wristPitchRad - shoulderRad - elbowRad;

    shoulderDeg = shoulderRad * RAD_TO_DEG;
    elbowDeg = elbowRad * RAD_TO_DEG;
    wristDeg = wristRad * RAD_TO_DEG;
    return true;
}

bool moveToPose(float x, float y, float z, float wristTiltDeg, float clawAngle) {
    float hipDeg = 0.0f;
    float shoulderDeg = 0.0f;
    float elbowDeg = 0.0f;
    float wristDeg = 0.0f;

    if (!inverseKinematics(x, y, z, wristTiltDeg, hipDeg, shoulderDeg, elbowDeg, wristDeg)) {
        Serial.println("IK target is out of reach.");
        bt1.println("IK target is out of reach.");
        return false;
    }

    int servoTargets[6];
    servoTargets[0] = constrain(int(SERVO_NEUTRAL[0] + hipDeg * SERVO_DIRECTION[0] * SERVO_SCALE[0]), 0, 180);
    servoTargets[1] = constrain(int(SERVO_NEUTRAL[1] + shoulderDeg * SERVO_DIRECTION[1] * SERVO_SCALE[1]), 0, 180);
    servoTargets[2] = constrain(int(SERVO_NEUTRAL[2] + elbowDeg * SERVO_DIRECTION[2] * SERVO_SCALE[2]), 0, 180);
    servoTargets[3] = constrain(int(SERVO_NEUTRAL[3] + wristDeg * SERVO_DIRECTION[3] * SERVO_SCALE[3]), 0, 180);
    servoTargets[4] = servoAngles[4]; // Preserve current wrist rotate
    servoTargets[5] = constrain(int(clawAngle), 0, 180);

    for (int i = 0; i < 6; i++) {
        moveServo(i + 1, servoTargets[i]);
    }

    Serial.print("MoveToPose -> x: "); Serial.print(x);
    Serial.print(" y: "); Serial.print(y);
    Serial.print(" z: "); Serial.print(z);
    Serial.print(" wrist: "); Serial.println(wristTiltDeg);
    bt1.print("MoveToPose -> x: "); bt1.print(x);
    bt1.print(" y: "); bt1.print(y);
    bt1.print(" z: "); bt1.print(z);
    bt1.print(" wrist: "); bt1.println(wristTiltDeg);
    return true;
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

    // Check for special commands first
    if (command == "PLAY_MOVEMENTS") {
        waitingForConfirmation = true;
        Serial.println("Are you sure you want to play movements? (YES/NO)");
        bt1.println("Are you sure you want to play movements? (YES/NO)"); // Send confirmation request over Bluetooth
    }
    
    else if (command.startsWith("MOVE_IK ")) {
        float params[5];
        if (!parseFloats(command.substring(8), params, 5)) {
            Serial.println("MOVE_IK expects 5 values: X Y Z WristPitchDeg ClawAngle");
            bt1.println("MOVE_IK expects 5 values: X Y Z WristPitchDeg ClawAngle");
        } else {
            bool ok = moveToPose(params[0], params[1], params[2], params[3], params[4]);
            if (!ok) {
                Serial.println("MOVE_IK failed: pose unreachable");
                bt1.println("MOVE_IK failed: pose unreachable");
            }
        }
    }
    
    else if (command == "GETVALUE") {
        printServoPulseWidths();
    } else if (command == "MOVE_TO_START") {
        MoveToStart();
        Serial.println("Executing MoveToStart");
    } else if (command == "START_RECORD") {
        StartRecordingMovements();
    } else if (command == "STOP_RECORD") {
        StopRecordingMovements();
    } else if (command == "SAVE_MOVE") {  // New command to increment indexRecord
        if (isRecord && indexRecord < moveCount - 1) {
            indexRecord++;  // Increment the record index
            Serial.print("Move saved at index ");
            Serial.println(indexRecord);
            bt1.print("Move saved at index "); // Adding Bluetooth print
            bt1.println(indexRecord); // Adding Bluetooth print
        } else {
            Serial.println("Cannot save move, either not recording or out of space.");
            bt1.println("Cannot save move, either not recording or out of space.");
        }
    } else {
        // Parse and execute servo commands
        bool isCommandExecuted = false;
        for (const auto& cmd : commands) {
            if (command.startsWith(cmd.name)) {
                isCommandExecuted = true;
                int angle = command.substring(strlen(cmd.name)).toInt();
                moveServo(cmd.channel, angle);
                Serial.println(String(cmd.name) + " moved to " + String(angle) + " degrees");

                // If we are in recording mode, save this movement
                if (isRecord && indexRecord < moveCount) {
                    movesServos[indexRecord][cmd.channel - 1] = angle;  // Store the angle in the recording array

                    Serial.print("Recording Move "); Serial.print(indexRecord);
                    Serial.print(" for Servo "); Serial.print(cmd.channel);
                    Serial.print(": Angle "); Serial.println(angle);
                }
                break; // Exit the loop after handling the command
            }
        }

        if (!isCommandExecuted) {
            // If the command was not recognized
            Serial.println("Unknown command: " + command);
            bt1.println("Unknown command: " + command); // Adding Bluetooth print
        }
    }
}
