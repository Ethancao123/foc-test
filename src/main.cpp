#include <Arduino.h>
#include <SimpleFOC.h>

#define MAX_RPS 300
#define TIMEOUT_MILLIS 50
#define DEADZONE 3
#define DEBUG

// BLDC motor instance BLDCMotor(polepairs, (R), (KV 1100))
BLDCMotor motor = BLDCMotor(7, 0.1, 1400);

// BLDC driver instance BLDCDriver6PWM(phA_h, phA_l, phB_h, phB_l, phC_h, phC_l, (en))
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// position / angle sensor instance
MagneticSensorI2C sensor = MagneticSensorI2C(AS5048_I2C);

// inline current sense instance InlineCurrentSense(R, gain, phA, phB, phC)
LowsideCurrentSense currentsense = LowsideCurrentSense(0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// commander instance
#ifdef DEBUG
Commander command = Commander(Serial);
void doTarget(char* cmd){command.motor(&motor, cmd);}
#endif

volatile uint16_t servoPulse = 1500;
volatile uint32_t last_isr = 0;
bool motoren = false;

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

int hammer = 0;

void rapidHammer(char* a){
    hammer = atoi(a);
}

void setup() {
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
    
    motor.voltage_sensor_align = 1;
    motor.velocity_index_search = 3;



    // set motion control type to torque (default)
    motor.controller = MotionControlType::angle;

    // set torque control type to voltage (default)
    motor.torque_controller = TorqueControlType::voltage;

    // set FOC modulation type to sinusoidal
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

    // set motor voltage limit, this limits Vq
    motor.voltage_limit = 13;
    // set motor velocity limit
    motor.velocity_limit = 70000;
    // set motor current limit, this limits Iq
    motor.current_limit = 10;


    
    // initialize motor
    motor.init();

    // align sensor and start FOC
    motor.initFOC();

    pinMode(A_PWM, INPUT_PULLDOWN);
    //consider using timer instead
    attachInterrupt(digitalPinToInterrupt(A_PWM), ServoPulseUpdate, CHANGE);
    motor.enable();
    motoren = true;

    // use monitoring
    #ifdef DEBUG
    // start serial
    Serial.begin(115200);
    motor.useMonitoring(Serial);
    char temp = 'm';
    command.add('M', doTarget, &temp);
    command.add('L', rapidHammer,"rapid hammer");
    #endif
}

void loop() {
    // main FOC algorithm function
    // the faster you run this function the better
    motor.loopFOC();
    #ifndef DEBUG
    int pulse = servoPulse;
    if(pulse < 1500 + DEADZONE && pulse > 1500 - DEADZONE)
        pulse = 1500;
    int target = map(pulse, 1000, 2000, -MAX_RPS, MAX_RPS);
    
    //failsafe
    if(millis() - last_isr > TIMEOUT_MILLIS) {
        target = 0;
        motor.disable();
        motoren = false;
    } else if(!motoren){
        motor.enable();
        motoren = true;
    }
    motor.move(target);
    #endif
    

    #ifdef DEBUG
    static int lastmillis = millis();
    static int point = 0;
    if(hammer != 0){
        if(millis() - lastmillis > 250) {
            lastmillis = millis();
        }
        if(millis() - lastmillis < 125) {
            point = hammer;
        } else {
            point = 0;
        }
        motor.move(point);
    } else {
        motor.move();
    }
    
    // significantly slowing the execution down
    motor.monitor();
    // user communication
    command.run();
    #endif

}


