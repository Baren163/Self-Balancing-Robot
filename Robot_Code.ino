
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motorL = AFMS.getMotor(1);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);

// Include libraries, define variables

#include <avr/io.h>
#include <avr/interrupt.h>

#define CPU_clock 16000000
#define GYRO_CONFIG 27
#define FS_SEL0 3
#define FS_SEL1 4
#define GYRO_X_H 67
#define GYRO_X_L 68
#define ACCEL_Z_H 63
#define ACCEL_Y_H 61
#define ACCEL_X_H 59
#define SMPRT_DIV 25
#define CONFIG 26
#define SEND_START_CONDITION 100
#define SEND_START_CONDITION_AND_SET_TWINT 228
#define TWCR_INITIALISE 68
#define SET_TWINT 196
#define CLEAR_TWEA_FOR_NACK_AND_SET_TWINT 132
#define SEND_STOP_CONDITION 212
#define SLAVE_ADDRESS 104
#define DRC 7    // Data read complete
#define STN 6    // Stop Now
#define GVN 5    // Gyro value negative
#define CWMPU 4  //  Communicating with gyro
#define OFFSET -530
#define RAD_TO_DEG 57.29578
#define kp 14
#define ki 0.15
#define kd 0.9
#define alpha 0.98

uint8_t IsrExitFlow;
uint8_t isrFunction;
int16_t gyroValue;  // data type 'short', signed 16 bit variable
uint8_t myRegister;
float gyroAngle = 0;
float gyroAccelZ;
float gyroAccelY;
float accAngle;
float error;
float prevError;
int16_t motorPower;
float motorPowerIntegral;
float angle;
float prevAngle;
float target;

unsigned long tempTime;
unsigned long time;

//#include <Wire.h> // The Wire library is what Arduino uses to communicate with I2C devices however I will be creating my own I2C driver


// TWI Functions

//  Initialise the TWI peripheral
void twiInitialise(uint8_t bitRateGenerator) {

  // Activate internal pullups for twi
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);

  TWCR = TWCR_INITIALISE;  // Setting control register bits

  TWBR = bitRateGenerator;  // Setting TWBR to 18 for a SCL frequency of 100kHz

  TWSR &= !(1 << TWPS1) & !(1 << TWPS0);  // Setting pre scaler bits to zero (Pre scaler = 1)

  //Serial.println("Initialised");
}


void writeMPU(uint8_t registerToWrite, uint8_t valueToWrite) {

  while (1) {

    // While TWINT is 0 wait in this loop
    while (!(TWCR & (1 << TWINT))) {
      ;
    }

    TWCR = TWCR_INITIALISE;  // Set TWINT to clear interrupt

    switch (TWSR) {

      case 8:
        //  Start condition has been transmitted
        //Serial.println("TWSR reads 8");
        TWCR = TWCR_INITIALISE;

        TWDR = (SLAVE_ADDRESS << 1);  // Load SLA + W

        break;

      case 16:
        // A repeated start condition has been transmitted

        break;

      case 24:
        // SLA+W has been transmitted; ACK has been received

        //Serial.println("SLA+W has been transmitted; ACK has been received, sending RA");

        TWDR = registerToWrite;  // Load register address (107)

        break;

      case 32:
        // SLA+W has been transmitted; NOT ACK has been received

        break;

      case 40:
        // Data byte has been transmitted; ACK has been received

        if (myRegister & (1 << STN)) {
          IsrExitFlow = 3;
          isrFunction = 1;
          break;
        }

        //Serial.println("Data byte has been transmitted; ACK has been received, sending '9'");

        TWDR = valueToWrite;  // Load decimal 9 into register 107 to clear sleep bit, disable temperature sensor and select Gyro X clock

        myRegister |= (1 << STN);  // stop_now++;

        break;

      case 48:
        // Data byte has been transmitted; NOT ACK has been received

        break;

      case 56:
        // Arbitration lost in SLA+W or data bytes

        break;

      case 64:
        // SLA+R has been transmitted; ACK has been received, data byte will be received and ACK will be returned

        break;

      case 80:
        // Data byte has been received; ACK has been returned, data byte will be stored and NACK will be returned

        break;

      case 88:
        // Data byte has been received; NOT ACK has been returned, data byte will be store and STOP condition will be sent to end transmission

        break;

      default:
        break;
    }

    switch (IsrExitFlow) {

      case 0:
        TWCR = SET_TWINT;  // 0b11000101
        break;

      case 1:
        //Serial.println("Repeated start");
        TWCR = SEND_START_CONDITION_AND_SET_TWINT;
        break;

      case 2:
        //Serial.println("Return NACK");
        TWCR = CLEAR_TWEA_FOR_NACK_AND_SET_TWINT;

        gyroValue = (TWDR);

        //Serial.print("High byte stored in gyroValue: ");
        //Serial.println(gyroValue);
        break;

      case 3:
        //Serial.println("STOP condition will be sent");
        TWCR = SEND_STOP_CONDITION;
        return;
        break;

      default:
        break;
    }
  }
}


int16_t readMPU(uint8_t registerToRead) {

  int16_t readValue;

  // While communication with gyro device bit is set
  while (1) {

    // While TWINT is 0 wait in this loop
    while (!(TWCR & (1 << TWINT))) {
      ;
    }

    IsrExitFlow = 0;

    TWCR = TWCR_INITIALISE;  // Set TWINT to clear interrupt


    // Read from GYRO_X
    switch (TWSR) {

      case 8:
        //  Start condition has been transmitted
        //Serial.println("TWSR reads 8");
        TWCR = TWCR_INITIALISE;

        TWDR = (SLAVE_ADDRESS << 1);  // Load SLA + W
        break;

      case 16:
        // A repeated start condition has been transmitted

        TWDR = ((SLAVE_ADDRESS << 1) + 1);  // Load SLA + R
        //Serial.println(TWDR);
        break;

      case 24:
        // SLA+W has been transmitted; ACK has been received
        TWDR = registerToRead;  // Write the gyro data register address to the slave
        break;

      case 32:
        // SLA+W has been transmitted; NOT ACK has been received
        break;

      case 40:
        // Data byte has been transmitted; ACK has been received
        IsrExitFlow = 1;  // Exit ISR with start condition (Repeated START)
        break;

      case 48:
        // Data byte has been transmitted; NOT ACK has been received
        break;

      case 56:
        // Arbitration lost in SLA+W or data bytes
        break;

      case 64:
        // SLA+R has been transmitted; ACK has been received, data byte will be received and ACK will be returned

        // IsrExitFlow = 0;

        break;

      case 80:
        // Data byte has been received; ACK has been returned, data byte will be stored and NACK will be returned

        //Serial.print("TWDR value at supposed data receival: ");
        //Serial.println(TWDR);

        IsrExitFlow = 2;  // Return NACK

        break;

      case 88:
        // Data byte has been received; NOT ACK has been returned, data byte will be store and STOP condition will be sent to end transmission

        //gyroValue += ((uint16_t) (TWDR << 8));

        readValue = readValue << 8;

        readValue += TWDR;

        IsrExitFlow = 3;

        break;

      default:
        break;
    }


    switch (IsrExitFlow) {

      case 0:
        TWCR = SET_TWINT;  // 0b11000101
        break;

      case 1:
        //Serial.println("Repeated start");
        TWCR = SEND_START_CONDITION_AND_SET_TWINT;
        break;

      case 2:
        //Serial.println("Return NACK");
        TWCR = CLEAR_TWEA_FOR_NACK_AND_SET_TWINT;

        readValue = (TWDR);

        //Serial.print("High byte stored in gyroValue: ");
        //Serial.println(gyroValue);
        break;

      case 3:
        //Serial.println("STOP condition will be sent");
        TWCR = SEND_STOP_CONDITION;

        // readValue /= 10;
        readValue -= OFFSET;

        readValue = ((float)readValue / 32767) * 250;

        return readValue;

        break;

      default:
        break;
    }
  }

}





void setup() {
  // Setup I2C registers for recieving data, initialise variables, set digital I/O pins
  sei();  // Enable global interrupts

  //Serial.begin(9600);

  //while(!Serial);

  //Serial.println("Serial begun");

  if (!AFMS.begin()) {  // create with the default frequency 1.6KHz
                        // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    //Serial.println("Could not find Motor Shield. Check wiring.");
    while (1)
      ;
  }
  // Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  motorL->setSpeed(150);
  motorL->run(FORWARD);
  // turn on motor
  motorL->run(RELEASE);

  motorR->setSpeed(150);
  motorR->run(FORWARD);
  // turn on motor
  motorR->run(RELEASE);

  gyroValue = 0;

  //stop_now = 0;
  //dataReadComplete = 1;
  //GVN = 0;
  myRegister = 128;

  twiInitialise(18);

  TWCR = SEND_START_CONDITION;
  writeMPU(107, 9); // Initialise the MPU


  // Set start angle as balance point

  TWCR = SEND_START_CONDITION;
  gyroAccelZ = readMPU(ACCEL_Z_H);

  TWCR = SEND_START_CONDITION;
  gyroAccelY = readMPU(ACCEL_Y_H);

  // Calculate accAngle
  accAngle = atan2(gyroAccelY, gyroAccelZ);
  accAngle *= RAD_TO_DEG;

}



void loop() {
  // Set I2C recieved data to variables, calculate angle of rotational displacement for the X axis, Implement PID control for motor speed


  delay(10);


  // Take Readings

  TWCR = SEND_START_CONDITION;
  gyroValue = readMPU(GYRO_X_H);

  TWCR = SEND_START_CONDITION;
  gyroAccelZ = readMPU(ACCEL_Z_H);

  TWCR = SEND_START_CONDITION;
  gyroAccelY = readMPU(ACCEL_Y_H);


  // Calculate accAngle
  accAngle = atan2(gyroAccelY, gyroAccelZ);
  accAngle *= RAD_TO_DEG;


  // Calculate gyroAngle
  tempTime = millis();
  time = (tempTime - time);
  gyroAngle = (0.01 * gyroValue);


  // Complementary Filter
  angle = (alpha * (angle + gyroAngle)) + ((1 - alpha) * accAngle);


  // PID Control
  // Proportional
  error = angle - target;
  motorPower = error * kp;

  // Integral
  motorPowerIntegral += (error * time) * ki;
  if (motorPowerIntegral > 254) {
    motorPowerIntegral = 255;
  } else if (motorPowerIntegral < -254) {
    motorPowerIntegral = -255;
  }
  motorPower += motorPowerIntegral;

  // Derivative (already given from gyroscope value)
  motorPower += gyroValue * kd;


  // Limit motor speed (so it doesn't 'overflow')
  if (motorPower > 254) {
    motorPower = 255;
  } else if (motorPower < -254) {
    motorPower = -255;
  }

  // Determine direction to drive motor in
  if (motorPower < 0) {
    motorL->run(FORWARD);
    motorR->run(FORWARD);
  } else {
    motorL->run(BACKWARD);
    motorR->run(BACKWARD);
  }

  motorL->setSpeed(abs(motorPower));
  motorR->setSpeed(abs(motorPower));


  time = millis();
  prevError = error;
  prevAngle = angle;

}