/////////////////////////////////////////////////////////////////////////////////////
// Program to control FOV Stepper Motors using Accel Library
/////////////////////////////////////////////////////////////////////////////////////
// Y Axis Stepper
#include <AccelStepper.h>
#include <MultiStepper.h>

#define ENABLE_Y 19
#define DIR_Y 13
#define STEP_Y 14
#define motorInterfaceType 1
AccelStepper stepper_Y = AccelStepper(motorInterfaceType, STEP_Y, DIR_Y);

#define ENABLE_X 23
#define DIR_X 32
#define STEP_X 33
AccelStepper stepper_X = AccelStepper(motorInterfaceType, STEP_X, DIR_X);

MultiStepper steppers;
long positionMove[2];

#define xConvert (26000 / 102)
#define yConvert (18500 / 64)

float xSpd = 8000;
float ySpd = 8000;

int vibeMode = 6;

void stepperSetup()
{
    pinMode(ENABLE_X, OUTPUT);
    pinMode(ENABLE_Y, OUTPUT);
    digitalWrite(ENABLE_X, HIGH);
    digitalWrite(ENABLE_Y, HIGH);
    // Configure each stepper
    stepper_X.setMaxSpeed(25000);
    stepper_Y.setMaxSpeed(25000);

    // Then give them to MultiStepper to manage
    steppers.addStepper(stepper_X);
    steppers.addStepper(stepper_Y);
}

// End Stop Hall Sensor
#define hall_X 36
#define hall_Y 38

// Homing Sequence Variables
// X AXIS
long initial_homing_X = -1; // Used to Home Stepper at startup
// Y AXIS
long initial_homing_Y = 1; // Used to Home Stepper at startup

void hallSensorsSetup()
{
    pinMode(hall_X, INPUT);
    pinMode(hall_Y, INPUT);
}

// Variables for main loop
char inBuff[] = "----------------------------------------------------------------";
int bytesread;

void setup()
{
    Serial.begin(9600);

    hallSensorsSetup();
    homeSteppers();
    pwmPinsSetup();
    stepperSetup();
    pwmPinsSetup();
    stepper_X.setCurrentPosition(0);
    stepper_Y.setCurrentPosition(0);
}

void loop()
{
    loopMovement();
}

long r = 20;            // Constant radius
float theta;            // Angle
float delta = PI / 100; // Increment

float a[9] = {0, PI / 4, PI / 2, (3 * PI) / 4, PI, (5 * PI) / 4, (3 * PI) / 2, (7 * PI) / 4, 2 * PI};
void loopMovement()
{

    moveStepsToPos(0, 0);
    speedCalc(0, 0, 48, 26);
    vibeMode = 0;
    coreSetup();

    moveStepsToPos(48, 26);
    speedCalc(48, 26, 90, 0);
    vibeMode = 0;
    coreSetup();

    moveStepsToPos(90, 0);
    speedCalc(90, 0, 48, 26);
    vibeMode = 0;
    coreSetup();

    moveStepsToPos(48, 26);

    float prevX = 48;
    float prevY = 26;

    Serial.println("First Circle");
    for (int i = 0; i < 9; i++)
    {
        theta = a[i];
        float newX = 48 + r + cos(theta) * r;
        float newY = 26 + sin(theta) * r;
        speedCalc(prevX, prevY, newX, newY);
        moveStepsToPos(newX, newY);
        prevX = newX;
        prevY = newY;
    }

    speedCalc(prevX, prevY, 90, 50);
    moveStepsToPos(90, 50);
    speedCalc(90, 50, 48, 26);
    moveStepsToPos(48, 26);

    prevX = 48;
    prevY = 26;

    Serial.println("Second Circle");
    for (int i = 0; i < 9; i++)
    {
        theta = a[i];
        float newX = 48 + r + cos(theta) * r;
        float newY = 26 + sin(theta) * r;
        speedCalc(prevX, prevY, newX, newY);
        moveStepsToPos(newX, newY);
        prevX = newX;
        prevY = newY;
    }

    speedCalc(prevX, prevY, 0, 50);
    moveStepsToPos(0, 50);
    speedCalc(0, 50, 48, 26);
    moveStepsToPos(48, 26);
}

void speedCalc(float x1, float y1, float x2, float y2)
{                // calculate required stepper speed based on distance the ball has the travel within an alotted time
    float t = 2; // timeframe to complete movement

    // TODO: how are the constants here calculated
    // adjust coordinates back to mm
    // x1 = x1*1.875;
    // x2 = x2*1.875;
    // y1 = y1*2.5;
    // y2 = y2*2.5;
    float dx = abs(x2 - x1); // distance to next coordinate
    float dy = abs(y2 - y1);
    float sx = dx / t; // in mm/s //speed needed to get to next point within allowed timeframe
    float sy = dy / t;
    xSpd = (sx * (26000 / 103)); // convert speed  mm/s to stepper speed
    ySpd = (sy * (18500 / 65));

    // Top speed is 20000 before motors start jamming
    if (xSpd > 18000)
    {
        xSpd = 18000;
    }
    if (ySpd > 18000)
    {
        ySpd = 18000;
    }
    Serial.print("X Speed Calulated: ");
    Serial.println(xSpd);
    Serial.print("Y Speed Calulated: ");
    Serial.println(ySpd);
}

void moveStepsToPos(long x, long y)
{
    // Serial.print("X: "); Serial.print(x);
    // Serial.print("Y: "); Serial.println(y);
    stepper_X.setMaxSpeed(xSpd);
    stepper_Y.setMaxSpeed(ySpd);
    stepper_X.setAcceleration(xSpd * 50);
    stepper_Y.setAcceleration(ySpd * 50);
    // Serial.println("Moving Steppers!");
    digitalWrite(ENABLE_X, LOW);
    digitalWrite(ENABLE_Y, LOW);
    positionMove[0] = x * xConvert;
    positionMove[1] = y * yConvert;
    steppers.moveTo(positionMove);
    while (stepper_X.distanceToGo() != 0 || stepper_Y.distanceToGo() != 0)
    {
        stepper_X.run();
        stepper_Y.run();
    }

    // steppers.runSpeedToPosition(); // Blocks until all are in position
    // delay(1000);
    digitalWrite(ENABLE_X, HIGH);
    digitalWrite(ENABLE_Y, HIGH);
}

void homeSteppers()
{
    /////////////////////////////////////////////////////////////////////////////////////////
    // X AXIS HOMING
    ////////////////////////////////////////////////////////////////////////////////////////
    float homingSpd = 15000.0;
    Serial.println("Homing X Axis");
    digitalWrite(ENABLE_X, LOW);
    digitalWrite(ENABLE_Y, HIGH);
    delay(5); // Wait for EasyDriver wake up

    //  Set Max Speed and Acceleration of each Steppers at startup for homing
    stepper_X.setMaxSpeed(homingSpd);     // Set Max Speed of Stepper (Slower to get better accuracy)
    stepper_X.setAcceleration(homingSpd); // Set Acceleration of Stepper

    // Start Homing procedure of Stepper Motor at startup

    Serial.print("Stepper X is Homing . . . . . . . . . . . ");

    while (digitalRead(hall_X))
    { // Make the Stepper move CCW until the switch is activated
        // Serial.println(digitalRead(home_switch));
        stepper_X.moveTo(initial_homing_X); // Set the position to move to
        initial_homing_X--;                 // Decrease by 1 for next move if needed
        stepper_X.run();                    // Start moving the stepper
        delay(1);
    }

    stepper_X.setCurrentPosition(0);      // Set the current position as zero for now
    stepper_X.setMaxSpeed(homingSpd);     // Set Max Speed of Stepper (Slower to get better accuracy)
    stepper_X.setAcceleration(homingSpd); // Set Acceleration of Stepper
    initial_homing_X = -1;

    while (!digitalRead(hall_X))
    { // Make the Stepper move CW until the switch is deactivated
        stepper_X.moveTo(initial_homing_X);
        stepper_X.run();
        initial_homing_X++;
        delay(1);
    }

    stepper_X.setCurrentPosition(0);
    Serial.println("Homing X Axis Completed");

    /////////////////////////////////////////////////////////////////////////////////////////
    // Y AXIS HOMING
    ////////////////////////////////////////////////////////////////////////////////////////
    Serial.println("Homing Y Axis");
    digitalWrite(ENABLE_Y, LOW);
    digitalWrite(ENABLE_X, HIGH);
    delay(3); // Wait for EasyDriver wake up

    //  Set Max Speed and Acceleration of each Steppers at startup for homing
    stepper_Y.setMaxSpeed(homingSpd);     // Set Max Speed of Stepper (Slower to get better accuracy)
    stepper_Y.setAcceleration(homingSpd); // Set Acceleration of Stepper

    // Start Homing procedure of Stepper Motor at startup

    Serial.print("Stepper Y is Homing . . . . . . . . . . . ");

    while (digitalRead(hall_Y))
    { // Make the Stepper move CCW until the switch is activated
        // Serial.println(digitalRead(home_switch));
        stepper_Y.moveTo(initial_homing_Y); // Set the position to move to
        initial_homing_Y--;                 // Decrease by 1 for next move if needed
        stepper_Y.run();                    // Start moving the stepper
        delay(1);
    }

    stepper_Y.setCurrentPosition(0);      // Set the current position as zero for now
    stepper_Y.setMaxSpeed(homingSpd);     // Set Max Speed of Stepper (Slower to get better accuracy)
    stepper_Y.setAcceleration(homingSpd); // Set Acceleration of Stepper
    initial_homing_Y = 1;

    while (!digitalRead(hall_Y))
    { // Make the Stepper move CW until the switch is deactivated
        stepper_Y.moveTo(initial_homing_Y);
        stepper_Y.run();
        initial_homing_Y++;
        delay(1);
    }

    stepper_Y.setCurrentPosition(0);
    Serial.println("Homing Y Axis Completed");

    // moveStepsToPos(1, 1);
    stepper_X.setCurrentPosition(1);
    stepper_Y.setCurrentPosition(1);

    digitalWrite(ENABLE_X, HIGH);
    digitalWrite(ENABLE_Y, HIGH);
}

//////////////////////////
// Vibration Motor Setup//
/////////////////////////

// Vibration Motor in pitch
#define VIB_GPIO1 9
#define PWM1_Ch 0
#define PWM1_Res 8
#define PWM1_Freq 1000

// Vibration motor on body
#define VIB_GPIO2 21
#define PWM2_Ch 1
#define PWM2_Res 8
#define PWM2_Freq 1000

void pwmPinsSetup()
{

    // Setup Motor 1
    ledcAttachPin(VIB_GPIO1, PWM1_Ch);
    ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
    // Set up Motor 2
    ledcAttachPin(VIB_GPIO2, PWM2_Ch);
    ledcSetup(PWM2_Ch, PWM2_Freq, PWM2_Res);

    ledcWrite(PWM1_Ch, 0);
    ledcWrite(PWM2_Ch, 0);
}

///////////////////////////////////////////////////////////////////////////
// Vibration Motor On Second Core
///////////////////////////////////////////////////////////////////////////
// Vibration functionality is handled by ESP32's second core to provide multitasking

// Multitasking RTOS Cores
TaskHandle_t PWMVibe;

void coreSetup()
{
    xTaskCreatePinnedToCore(
        pwmMotor,    // Function that should be called
        "Motor PWM", // Name of the task (for debugging)
        10000,       // Stack size (bytes)
        NULL,        // Parameter to pass
        1,           // Task priority
        &PWMVibe,    // Task handle
        0            // Select Which Core
    );
}

// 1 = body, 2 = pitch
// vibration response depending on events
void pwmMotor(void *pvParametersm)
{

    // VibeMode = 0 HOME PASS#########################
    if (vibeMode == 0)
    {
        ledcWrite(PWM1_Ch, 200);
        delay(300);
        ledcWrite(PWM1_Ch, 0);
    }

    // VibeMode = AWAY PASS#############################
    if (vibeMode == 1)
    {
        ledcWrite(PWM2_Ch, 200);
        delay(300);
        ledcWrite(PWM2_Ch, 0);
    }

    if (vibeMode == 2)
    {
        ledcWrite(PWM1_Ch, 200);
        delay(100);
        ledcWrite(PWM1_Ch, 0);
    }

    if (vibeMode == 3)
    {
        ledcWrite(PWM2_Ch, 200);
        delay(100);
        ledcWrite(PWM2_Ch, 0);
    }

    // VibeMode = 4 Away Goal Scored
    if (vibeMode == 4)
    {
        ledcWrite(PWM1_Ch, 200);
        ledcWrite(PWM2_Ch, 200);
        delay(25);
        ledcWrite(PWM1_Ch, 0);
        ledcWrite(PWM2_Ch, 0);
        delay(25);
        ledcWrite(PWM1_Ch, 200);
        ledcWrite(PWM2_Ch, 200);
        delay(25);
        ledcWrite(PWM1_Ch, 0);
        ledcWrite(PWM2_Ch, 0);
    }

    // VibeMode = HOME GOAL
    if (vibeMode == 5)
    {
        ledcWrite(PWM1_Ch, 200);
        ledcWrite(PWM2_Ch, 200);
        delay(50);
        ledcWrite(PWM1_Ch, 0);
        ledcWrite(PWM2_Ch, 0);
    }

    vTaskDelete(NULL);
}
