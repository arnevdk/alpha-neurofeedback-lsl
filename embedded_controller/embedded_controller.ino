#include <PID_v1.h>

const int trig_pin = 26;
const int echo_pin = 16;
const int motorPin = 14;
int height = 0;            // Desired height (target distance in cm)
float distance_cm;         // Measured distance in cm
double Setpoint, Input, Output; // PID variables

// PID Constants - start with these and tune as necessary
double Kp = 1.0;  // Proportional gain
double Ki = 0.05; // Integral gain
double Kd = 0.1;  // Derivative gain

// Vitesse du son dans l'air
#define SOUND_SPEED 340       // Speed of sound in air in m/s
#define TRIG_PULSE_DURATION_US 10

long ultrason_duration;

// Create the PID instance
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
    Serial.begin(115200);
    pinMode(motorPin, OUTPUT);
    pinMode(trig_pin, OUTPUT);  // Configure trig as output
    pinMode(echo_pin, INPUT);   // Configure echo as input

    // Set initial PID parameters
    Setpoint = height;  // Desired height
    myPID.SetMode(AUTOMATIC); // Set PID to automatic mode
    myPID.SetOutputLimits(120, 255); // Constrain PID output to match PWM range
}

void loop() {
    // Prepare the signal for ultrasonic sensor
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(TRIG_PULSE_DURATION_US);
    digitalWrite(trig_pin, LOW);

    // Measure the time of the pulse returned
    ultrason_duration = pulseIn(echo_pin, HIGH);

    // Calculate the distance in cm
    distance_cm = (ultrason_duration * SOUND_SPEED) / (2 * 10000.0); // Convert to cm

    // Print distance to Serial Monitor
    //Serial.print("Distance (cm): ");
    Serial.println(distance_cm);

    // Set PID Input to the measured distance
    Input = distance_cm;

    // Check for input height value from Serial (user input)
    if (Serial.available() > 0) {
        height = Serial.readStringUntil('\n').toInt();  // Read the desired height
        Setpoint = height;                              // Update Setpoint to target height
    }

    // Compute the PID output
    myPID.Compute();

    // Use the PID Output to set the motor speed
    analogWrite(motorPin, Output);

    // Print debugging information
   // Serial.print("Height Setpoint: ");
   // Serial.println(Setpoint);
  //  Serial.print("Measured Distance: ");
 //   Serial.println(Input);
  //  Serial.print("Motor Speed (PID Output): ");
   // Serial.println(Output);

    // Small delay for stability, keep it low to maintain fast response
    delay(5);
}
