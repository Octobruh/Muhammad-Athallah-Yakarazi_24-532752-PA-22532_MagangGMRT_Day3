// Muhammad Athallah Yakarazi
// 24/532752/PA/22532

#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Define which device is connected to which pin
#define SERVO1_PIN 17
#define SERVO2_PIN 16 
#define SERVO3_PIN 18
#define SERVO4_PIN 5
#define SERVO5_PIN 4 

#define SCL_PIN 22
#define SDA_PIN 21
#define PIR_PIN 19

// Global variables
Adafruit_MPU6050 mpu;
Servo servo1, servo2, servo3, servo4, servo5;

// Set the initial position of the servos to be 90 degree from range 0 to 180 degree
const int INITIAL_POS = 90;

// When did yaw motion stop?
// Is the system currently yawing?
// More explanation in the part of the code where these variables are used.
long yawStopTime = 0;
bool isYawing = false;

// Threshold to decide if the gyro is "moving" or "stopped", also because
// it is quite difficult to set the wokwi slider close to center so this also doubles
// to make it easier to tell that the yaw stopped without exactly setting the wokwi
// slider dead at at 0
const float YAW_MOTION_THRESHOLD = 1.0; 

void setup() {
	Serial.begin(115200);
	Serial.println("Hello, ESP32!");

	pinMode(PIR_PIN, INPUT);

	servo1.attach(SERVO1_PIN);
	servo2.attach(SERVO2_PIN);
	servo3.attach(SERVO3_PIN);
	servo4.attach(SERVO4_PIN);
	servo5.attach(SERVO5_PIN);

	// Try to initialise the MPU
	if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
			delay(10);
		}
	}
	Serial.println("MPU6050 Found!");

	// Set MPU measurement ranges
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
	mpu.setGyroRange(MPU6050_RANGE_250_DEG);
	mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
	Serial.println("MPU6050 Configured.");

	// Set all servos to their initial 90 degree position.
	servo1.write(INITIAL_POS);
	servo2.write(INITIAL_POS);
	servo3.write(INITIAL_POS);
	servo4.write(INITIAL_POS);
	servo5.write(INITIAL_POS);
	  
	Serial.println("Servos centered. Setup complete.");
	delay(1000);
}

void loop() {
	int pirValue = digitalRead(PIR_PIN);

	// Get new sensor events with the readings
	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);

	// If PIR detect something it will run the servo commands otherwise the MPU will run the servo commands
	if (pirValue != 0) {
	  
		Serial.println("PIR Motion Detected!");
			
		// Move the servos to set direction
		// int free_pos = 150;
		servo1.write(0);
		servo2.write(67);
		servo3.write(69);
		servo4.write(90);
		servo5.write(120);

		// Wait so the servos don't go back to 90 degree too soon
		delay(1000);

		Serial.println("Returning to initial position.");
			
		servo1.write(INITIAL_POS);
		servo2.write(INITIAL_POS);
		servo3.write(INITIAL_POS);
		servo4.write(INITIAL_POS);
		servo5.write(INITIAL_POS);
			
		// Wait a bit before checking sensors again
		delay(1000);

	} else {
		
		//Accelerometer to measure the tilt (pitch and roll)
		float roll_rad = atan2(a.acceleration.y, a.acceleration.z);
		float pitch_rad = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));

		// Convert radians to degrees (range will be approx -90 to 90)
		float roll_angle = roll_rad * 180.0 / 3.14159;
		float pitch_angle = pitch_rad * 180.0 / 3.14159;
			
		// roll positive then servo 1 & 2 negative, roll negative then servo 1 & 2 positive
		int servo1_pos = INITIAL_POS - roll_angle;
		int servo2_pos = INITIAL_POS - roll_angle;

		// pitch positive then servo 3 & 4 positive, pitch negative then servo 3 & 4 negative
		int servo3_pos = INITIAL_POS + pitch_angle;
		int servo4_pos = INITIAL_POS + pitch_angle;

		// display current roll and pitch angle
		Serial.print("Roll angle : ");
		Serial.println(roll_angle);
		Serial.print("Pitch angle : ");
		Serial.println(pitch_angle);

		// Use constrain() to make sure we send valid values 0 to 180 to the servos.
		servo1.write(constrain(servo1_pos, 0, 180));
		servo2.write(constrain(servo2_pos, 0, 180));
		servo3.write(constrain(servo3_pos, 0, 180));
		servo4.write(constrain(servo4_pos, 0, 180));

		// get yaw rate from sensor
		float yaw_rate = g.gyro.z; 

		// Check if the yaw rate is above the "moving" threshold
		if (abs(yaw_rate) > YAW_MOTION_THRESHOLD) {
			// System is yawing
			isYawing = true;
			yawStopTime = 0; // Reset stop timer
				  


			// the yaw_rate is scaled to be a small change from the center.
			// the '15.0' multiplier is to get the right sensitivity for the yaw servo
			// higher values makes the servo move more
			int servo5_pos = INITIAL_POS + (yaw_rate * 15.0);
			servo5.write(constrain(servo5_pos, 0, 180));
			
		} else if (isYawing) {
			// The system was yawing, but just stopped.
			isYawing = false;
			yawStopTime = millis(); // Record the time it stopped
			Serial.println("Yaw motion stopped. Starting 1s timer.");
		}

		// Check if the system has been stopped for 1 second
		// then return to initial position and reset the timer so this if conditional not run again
		if (yawStopTime > 0 && (millis() - yawStopTime > 1000)) {
			Serial.println("1s has passed. Returning servo 5 to center.");
			servo5.write(INITIAL_POS); 
			yawStopTime = 0; 
		}
	}

	  delay(20); // this speeds up the simulation
}
