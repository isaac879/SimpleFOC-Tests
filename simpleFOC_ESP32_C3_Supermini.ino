#include <SimpleFOC.h>

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

#define PIN_IN1 5
#define PIN_IN2 6
#define PIN_IN3 7
#define PIN_EN 10
// #define PIN_DRV_GND 2
#define PIN_DIR 4

#define ARRAY_LENGTH 60

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C); // magnetic sensor instance - MagneticSensorI2C

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11); //11 pole pairs
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_EN); //PWM pins and En pin

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

float target_value = 8; //Set point variable

float target_valueP = 12.5;
float target_valueI = 0.02;
float target_valueD = 0.0;
float target_valueL = 10.0;

float target_valueV = 12;
float target_valueC = 0.5;
float target_valueR = 100;

uint32_t msTime = 0;
uint32_t prevTime = 0;
float tartetPos = 0;
uint32_t loopCount = 0;

float position_array[ARRAY_LENGTH];

float targetpos = 0;
float value = 0.0;
float  inc = 0.02;

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

Commander command = Commander(Serial); // instantiate the commander
void doTarget(char* cmd) { command.scalar(&target_value, cmd); }
void doTargetP(char* cmd) { command.scalar(&target_valueP, cmd); }
void doTargetI(char* cmd) { command.scalar(&target_valueI, cmd); 
    inc = target_valueI;
}
void doTargetD(char* cmd) { command.scalar(&target_valueD, cmd); }
void doTargetL(char* cmd) { command.scalar(&target_valueL, cmd); }

void doTargetV(char* cmd) { command.scalar(&target_valueV, cmd); }
void doTargetC(char* cmd) { command.scalar(&target_valueC, cmd); }
void doTargetR(char* cmd) { command.scalar(&target_valueR, cmd); }

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void randomiseArray(float * arr, uint16_t len){

    for(uint16_t i = 0; i < len; i++){ //Fill the array
        arr[i] = i;
    }

    randomSeed(5); //Seed should actually be random

    float temp = 0;

    for(uint16_t i = len - 1; i > 0 ; i--){
        
        temp = arr[i];
        uint16_t swapindex = random(0, i); //Upper bound exclusive
        arr[i] = arr[swapindex];
        arr[swapindex] = temp;
    }

    for(int16_t i = 0; i < len; i++){
        Serial.printf("%d: %.2f\n ", i, arr[i]);
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup() {
    pinMode(PIN_DIR, OUTPUT);
    digitalWrite(PIN_DIR, LOW);
    Wire.setClock(1000000); //The maximum SCL frequency is 1 MHz.(From AS5600 datasheet)
    sensor.init(); //Initialise magnetic sensor hardware
    motor.linkSensor(&sensor); //Link the motor to the sensor

    //Driver config
    driver.voltage_power_supply = 12; //Power supply voltage (V)
    driver.init();
    motor.linkDriver(&driver); //Link the motor and the driver

    motor.phase_resistance = 15.0;
    motor.current_limit = 0.5;
    motor.voltage_limit = 12; //Maximal voltage to be sent to the motor
    
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // choose FOC modulation (optional)

    motor.controller = MotionControlType::angle; //set motion control loop to be used
    // motor.controller = MotionControlType::velocity;
    // motor.controller = MotionControlType::velocity_openloop;
    // motor.controller = MotionControlType::torque;

    //Velocity PI controller parameters
    motor.PID_velocity.P = 0.03; //0.06
    motor.PID_velocity.I = 5.0; //3.0
    motor.PID_velocity.D = 0.00001;
    motor.PID_velocity.limit = 1;
    // motor.PID_velocity.limitIntegral = 1;
    motor.PID_velocity.output_ramp = 100; //1000
    
    motor.P_angle.P = 15; //Angle P controller
    motor.P_angle.I = 0;
    motor.P_angle.D = 0;
    motor.P_angle.limit = 20;
    
    motor.LPF_velocity.Tf = 0.005f; // velocity low pass filtering time constant. The lower the less filtered

    motor.velocity_limit = 24; // maximal velocity of the position control
    // motor.voltage_sensor_align = 1;
    
    Serial.begin(115200); // use monitoring with serial
    // motor.useMonitoring(Serial); // comment out if not needed

    motor.init(); // initialize motor
    motor.initFOC(); // align sensor and start FOC

    command.add('T', doTarget, "target"); // add target command T
    command.add('P', doTargetP, "target"); 
    command.add('I', doTargetI, "target"); 
    command.add('D', doTargetD, "target"); 
    command.add('L', doTargetL, "target"); 

    command.add('R', doTargetR, "target"); 
    command.add('V', doTargetV, "target"); 
    command.add('C', doTargetC, "target"); 

    Serial.print(F("zero_electric_angle: "));
    Serial.println(motor.zero_electric_angle);//0.75
    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target value using serial terminal:"));

    randomiseArray(position_array, ARRAY_LENGTH);

    _delay(1000);
    // motor.target(0);
    motor.move(0);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

float setDetentPos(float detents, float pos){ //Fix for negatives
    static float prevSnapPoint = 0;

    float targetPos = 0;
    float spacing = (2 * PI) / detents; //Space between detents
    float detentNum = pos / spacing;
    
    int16_t dir = ((detentNum - (int16_t)detentNum) > 0) ? 1 : -1;

    if(abs(detentNum - (int16_t)detentNum) > 0.5){
        targetPos = ((int16_t)detentNum + dir) * spacing;

    }
    else{
        targetPos = ((int16_t)detentNum) * spacing;
    }

    if(prevSnapPoint != targetPos){
        motor.PID_velocity.reset();
    }
    
    prevSnapPoint = targetPos;
    return targetPos;
}



void loop() {
    while(1){ //Needed to prevent the stutter caused by serialEventRun(). See: https://www.reddit.com/r/esp32/comments/t9aqr6/what_on_earth_is_my_esp32_doing_between_loops/
        // motor.P_angle.P = target_value;
        // motor.PID_velocity.P = target_valueP;
        // motor.PID_velocity.I = target_valueI;
        // motor.PID_velocity.D = target_valueD;
        // motor.PID_velocity.limit = target_valueL;
        motor.loopFOC(); //Main FOC algorithm function. The faster you run this function the better
        // loopCount++;

        // motor.move(target_value);
        // motor.monitor(); // function intended to be used with serial plotter to monitor motor variables. significantly slowing the execution down!!!!
        // target_value = setDetentPos(10, sensor.getAngle());
        // targetpos = target_value;//setDetentPos(target_value, sensor.getAngle());
        targetpos = setDetentPos(target_value, sensor.getAngle());
        motor.move(targetpos);
        command.run(); // user communication

        // motor.PID_velocity.output_ramp = target_valueR;
        // motor.current_limit = target_valueC;
        motor.voltage_limit = target_valueV;

        // motor.P_angle.P = target_valueP;
        // motor.P_angle.I = target_valueI;
        // motor.P_angle.D = target_valueD;
        // motor.P_angle.limit = target_valueL;
        // motor.P_angle.limitIntegral = target_valueL;

        // msTime = millis();
        // if(msTime - prevTime > 1000){
        //     // targetpos = setDetentPos(10, sensor.getAngle());
        //     prevTime = msTime;
        //     // Serial.printf("Position: %.3fRads\n", sensor.getAngle());
        //     // Serial.printf("targetpos: %.3fRads\n", targetpos);
        //     // Serial.printf("Loop Frequency: %d\n\n", loopCount); //1534
        //     // loopCount = 0;
        // }

        // if(msTime - prevTime > 1000){ //Full rotations
        //     targetpos = targetpos + 2 * PI;
        //     prevTime = msTime;

        //     loopCount = 0;
        // }

        // inc = target_valueI;
        // targetpos = pow(value, 3) * motor.velocity_limit;

        // if(msTime - prevTime > 100){ //Speed ramps
        //     value += inc;

        //     if(abs(value) >= 1.0){
        //         inc = -inc;
        //         value = constrain(value, -1.0, 1.0);
        //     }
            
        //     prevTime = msTime;
        //     // Serial.printf("inc: %.3fR/S\n", inc);
        //     // Serial.printf("value: %.3f\n", value);
        //     // Serial.printf("targetpos: %.3fR/S\n\n", targetpos);
        //     // loopCount = 0;
        // }
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
