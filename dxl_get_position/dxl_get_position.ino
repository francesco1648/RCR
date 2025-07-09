#include "Dynamixel_ll.h"
#include "include/mod_config.h"

#define ProfileAcceleration 10
#define ProfileVelocity 20





// Create individual motor objects for setup (if needed for individual writes).
int32_t valueToSend = 0;
int32_t pos0_1a1b[2] = {0, 0};
int32_t pos0_2 = 0;
int32_t pos0_3 = 0;
int32_t pos0_4 = 0;
int32_t pos0_5 = 0;

int32_t pos_1a1b[2] = {0, 0};
int32_t pos_2 = 0;
int32_t pos_3 = 0;
int32_t pos_4 = 0;
int32_t pos_5 = 0;
int32_t pos_6 = 0; // Assuming motor 6 is defined elsewhere in your code.

const uint8_t motorIDs[] = {SERVO_ARM_1a_PITCH_ID, SERVO_ARM_1b_PITCH_ID};
const uint8_t numMotors = sizeof(motorIDs) / sizeof(motorIDs[0]);

DynamixelLL dxlARM(Serial1, 0); // an instance for syncWrite (ID not used).
DynamixelLL motorARM1aPitch(Serial1, SERVO_ARM_1a_PITCH_ID);
DynamixelLL motorARM1bPitch(Serial1, SERVO_ARM_1b_PITCH_ID);
DynamixelLL motorARM2Pitch(Serial1, SERVO_ARM_2_PITCH_ID);
DynamixelLL motorARM3Roll(Serial1, SERVO_ARM_3_ROLL_ID);
DynamixelLL motorARM4Pitch(Serial1, SERVO_ARM_4_PITCH_ID);
DynamixelLL motorARM5Roll(Serial1, SERVO_ARM_5_ROLL_ID);


void setup() {

   Serial.begin(115200);
   while(!Serial) {
    delay(10); // Wait for Serial to be ready
    // Wait for Serial to be ready
    }
   Serial1.setTX(0);
  Serial1.setRX(1);
  dxlARM.begin(1000000);
  dxlARM.enableSync(motorIDs, numMotors);
  Serial.println("inizzializing motors");
  dxlARM.setTorqueEnable(false);
  motorARM2Pitch.setTorqueEnable(false);
  motorARM3Roll.setTorqueEnable(false);
  motorARM4Pitch.setTorqueEnable(false);
  motorARM5Roll.setTorqueEnable(false);
  // Set operating mode for all motors to position mode

   motorARM1aPitch.setDebug(true);
  motorARM1bPitch.setDebug(true);
  motorARM2Pitch.setDebug(true);
  motorARM3Roll.setDebug(true);
  motorARM4Pitch.setDebug(true);
  motorARM5Roll.setDebug(true);
  dxlARM.setDebug(true);

  dxlARM.setOperatingMode(3);
  motorARM2Pitch.setOperatingMode(3);
  motorARM3Roll.setOperatingMode(3);
  motorARM4Pitch.setOperatingMode(3);
  motorARM5Roll.setOperatingMode(3);

  motorARM1aPitch.setProfileVelocity(ProfileVelocity);
  motorARM1aPitch.setProfileAcceleration(ProfileAcceleration);
  motorARM1bPitch.setProfileVelocity(ProfileVelocity);
  motorARM1bPitch.setProfileAcceleration(ProfileAcceleration);
  motorARM2Pitch.setProfileVelocity(ProfileVelocity);
  motorARM2Pitch.setProfileAcceleration(ProfileAcceleration);
  motorARM3Roll.setProfileVelocity(ProfileVelocity);
  motorARM3Roll.setProfileAcceleration(ProfileAcceleration);
  motorARM4Pitch.setProfileVelocity(ProfileVelocity);
  motorARM4Pitch.setProfileAcceleration(ProfileAcceleration);
  motorARM5Roll.setProfileVelocity(ProfileVelocity);
  motorARM5Roll.setProfileAcceleration(ProfileAcceleration);

  // Configure Drive Mode for each motor:
  motorARM1aPitch.setDriveMode(false, false, false);
  motorARM1bPitch.setDriveMode(false, false, false);
  motorARM2Pitch.setDriveMode(false, false, false);
  motorARM3Roll.setDriveMode(false, false, false);
  motorARM4Pitch.setDriveMode(false, false, false);
  motorARM5Roll.setDriveMode(false, false, false);




  // Declare and initialize the arrays

  // Get present position for all motors
  dxlARM.getPresentPosition(pos0_1a1b);
  motorARM2Pitch.getPresentPosition(pos0_2);
  motorARM3Roll.getPresentPosition(pos0_3);
  motorARM4Pitch.getPresentPosition(pos0_4);
  motorARM5Roll.getPresentPosition(pos0_5);
  Serial.print("\nThe motors are initialised.");

  delay(2000);
}

void loop() {
  // Read the current positions and loads.
  dxlARM.getPresentPosition(pos_1a1b);
  motorARM2Pitch.getPresentPosition(pos_2);
  motorARM3Roll.getPresentPosition(pos_3);
  motorARM4Pitch.getPresentPosition(pos_4);
  motorARM5Roll.getPresentPosition(pos_5);

  Serial.print("Current positions: ");
  Serial.print("1a: ");
  Serial.print(pos_1a1b[0]);
  Serial.print(", 1b: ");
  Serial.print(pos_1a1b[1]);
  Serial.print(", 2: ");
  Serial.print(pos_2);
  Serial.print(", 3: ");
  Serial.print(pos_3);
  Serial.print(", 4: ");
  Serial.print(pos_4);
  Serial.print(", 5: ");
  Serial.print(pos_5);



  delay(1000); // Wait for a bit before the next loop.

  // Repeat loop...
}
