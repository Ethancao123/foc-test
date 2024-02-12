#include <Arduino.h>
#include <SimpleFOC.h>

//#define DEBUG
#define MAX_RPS 300
#define TIMEOUT_MILLIS 50
#define DEADZONE 3
#define REVERSED 1 //Used in macros 1 for conventional, -1 for reverse
#define TORQUE_RANGE 12 //from negative to positive
#define ANGLE_RANGE 10 //from negative to positive but centered around 2

// BLDC motor instance BLDCMotor(polepairs, (R), (KV 1100))
BLDCMotor motor = BLDCMotor(7, 0.1, 1750, 0.01/1000);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5048_I2C);
LowsideCurrentSense currentsense = LowsideCurrentSense(0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// commander instance
#ifdef DEBUG
Commander command = Commander(Serial);
void doTarget(char* cmd){command.motor(&motor, cmd);}
#endif

int maxPowerMillis;
float hammerTorque;
volatile uint16_t servoPulse = 1500;
volatile uint32_t last_isr = 0;
bool motoren = false;
int Hcommand = 0;

#ifndef DEBUG
void ServoPulseUpdate() {
    static uint32_t startTime = 0;
    uint32_t curTime = micros();
    
    if (digitalRead(A_PWM)) // Pin transitioned from low to high
        startTime = curTime; // Start counting pulse time
    else // Pin transitioned from high to low
    servoPulse = (uint16_t)(curTime - startTime);
    if(servoPulse <= 2200 && servoPulse >= 800) {
        last_isr = millis();
    }
}
#endif

void initArm(){
    int startMillis = millis();
    int prevLimit = motor.current_limit;
    motor.current_limit = 8;
    MotionControlType previous = motor.controller;
    motor.controller = MotionControlType::velocity;
    while(millis() - startMillis < 1500){
        motor.loopFOC();
        motor.move(-10 * REVERSED);
    }
    motor.sensor_offset = motor.shaft_angle;
    motor.current_limit = prevLimit;
    motor.move(1);
    motor.loopFOC();
    motor.move(1);
    motor.controller = previous;
}

void doHammer(char *a) {
    Hcommand = atoi(a);
}
void setup() {
    #ifdef DEBUG
    Serial.begin(115200);
    motor.useMonitoring(Serial);
    #endif
    // driver.pwm_frequency = 15000;
    // set I2C clock speed
    Wire.setClock(400000);
    // initialize sensor
    sensor.init();
    // link sensor to motor
    motor.linkSensor(&sensor);
    // set power supply voltage
    driver.voltage_power_supply = 12;
    // initialize driver
    driver.init();
    // link driver to motor
    motor.linkDriver(&driver);
    delay(5);
    currentsense.linkDriver(&driver); 
    delay(5);
    currentsense.init();
    currentsense.skip_align = true;
    motor.linkCurrentSense(&currentsense);
    
    motor.voltage_sensor_align = 1.25;
    motor.velocity_index_search = 6;

    motor.controller = MotionControlType::angle;
    motor.torque_controller = TorqueControlType::voltage;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;


    // velocity PID controller
    motor.PID_velocity.P = 3;
    motor.PID_velocity.I = 0.5;
    motor.PID_velocity.D = 0.01;
    motor.PID_velocity.output_ramp = 1000;
    motor.PID_velocity.limit = 20;
    motor.LPF_velocity.Tf = 0.07;

    motor.P_angle.P = 6;
    motor.P_angle.I = 0.2;
    motor.P_angle.D = 0.05;
    motor.P_angle.output_ramp = 500;

    motor.LPF_angle = 0.1;
    motor.voltage_limit = 18;
    motor.velocity_limit = 70000;
    motor.current_limit = 8;

    maxPowerMillis = 250;
    hammerTorque = 50;

    motor.init();
    motor.initFOC();

    pinMode(A_PWM, INPUT_PULLDOWN);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    #ifndef DEBUG
    attachInterrupt(digitalPinToInterrupt(A_PWM), ServoPulseUpdate, CHANGE);
    #endif

    motor.enable();
    motoren = true;
    initArm();
    // use monitoring
    #ifdef DEBUG
    // start serial
    char temp = 'm';
    command.add('M', doTarget, &temp);
    command.add('H', doHammer, &temp);
    #endif
}

int hammerStart;
float preHammerTarget;
bool prevHammerState = false;

void loop() {
    // main FOC algorithm function
    // the faster you run this function the better
    float target = NOT_SET;
    motor.loopFOC();

    #ifndef DEBUG
    int pulse = servoPulse; //prevent interrupt from overriding during loop
    //check for timeout
    if(millis() - last_isr > TIMEOUT_MILLIS || (pulse > 1373 && pulse < 1629)) {
        target = 0;
        motor.disable();
        motoren = false;
    } else if(!motoren){
        motor.enable();
        motoren = true;
    } 
    if(pulse < 1343 && pulse > 1085) { //lower range
        motor.controller = MotionControlType::torque;
        target = map(pulse,1086,1342,-1*TORQUE_RANGE, TORQUE_RANGE);
    }
    else if(pulse < 1916 && pulse > 1658) { //higher range
        motor.controller = MotionControlType::angle;
        target = map(pulse,1658,1916,-1*ANGLE_RANGE, ANGLE_RANGE);
    }
    else if(pulse > 1975){ //peak range
        if(!prevHammerState) { //if low to high
            motor.controller = MotionControlType::torque;
            hammerStart = millis();
            prevHammerState = Hcommand;
            preHammerTarget = motor.target;
        }
        if(millis() - hammerStart < maxPowerMillis) {
            target = hammerTorque;
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            digitalWrite(LED_BUILTIN, LOW);
        }
    } else { //switch off
        if(prevHammerState) {
            motor.controller = MotionControlType::angle;
            prevHammerState = Hcommand;
            target = preHammerTarget; 
        }
    }
    #endif
    motor.move(target);

    #ifdef DEBUG
    // motor.move();
    
    // significantly slowing the execution down
    motor.monitor();
    // user communication
    command.run();
    #endif

}


