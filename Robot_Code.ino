
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

// Include libraries, define variables

#include <avr/io.h>
#include <avr/interrupt.h>

#define CPU_clock 16000000
#define GYRO_CONFIG 27
#define FS_SEL0 3
#define FS_SEL1 4
#define GYRO_XOUT_H 67
#define GYRO_XOUT_L 68
#define SMPRT_DIV 25
#define CONFIG 26
#define SEND_START_CONDITION 100
#define SEND_START_CONDITION_AND_SET_TWINT 228
#define TWCR_INITIALISE 68
#define SET_TWINT 196
#define CLEAR_TWEA_FOR_NACK_AND_SET_TWINT 132
#define SEND_STOP_CONDITION 212
#define SLAVE_ADDRESS 104
#define DRC 7
#define STN 6
#define GVN 5
#define CWGD 4

uint8_t IsrExitFlow;
uint8_t isrFunction;
int16_t gyroValue;  // data type 'short', signed 16 bit variable
uint8_t myRegister;

//#include <Wire.h> // The Wire library is what Arduino uses to communicate with I2C devices however I will be creating my own I2C driver


// TWI Functions

//  Initialise the TWI peripheral
void twiInitialise(uint8_t bitRateGenerator) {

  // Activate internal pullups for twi
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);

  TWCR = TWCR_INITIALISE;   // Setting control register bits

  TWBR = bitRateGenerator;  // Setting TWBR to 18 for a SCL frequency of 100kHz

  TWSR &= !(1 << TWPS1) & !(1 << TWPS0);  // Setting pre scaler bits to zero (Pre scaler = 1)

  //Serial.println("Initialised");

}





void setup() {
  // Setup I2C registers for recieving data, initialise variables, set digital I/O pins
  sei();  // Enable global interrupts

  //Serial.begin(9600);

  //while(!Serial);

  //Serial.println("Serial begun");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    //Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
 // Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);

  gyroValue = 0;

  isrFunction = 0;  // Initialise MPU-6050

  //stop_now = 0;
  //dataReadComplete = 1;
  //GVN = 0;
  myRegister = 128;

  twiInitialise(18);

}


int16_t getGyroRateX() {

}


void loop() {
  // Set I2C recieved data to variables, calculate angle of rotational displacement for the X axis, Implement PID control for motor speed

  //Serial.println("Begin");

  // if (gyroValue > 50) {
  //   myMotor->run(FORWARD);
  //   myMotor->setSpeed(250);
  //   delay(10);
  // } else if (gyroValue < -50) {
  //   myMotor->run(BACKWARD);
  //   myMotor->setSpeed(250);
  //   delay(10);
  // } else {
  //   myMotor->run(RELEASE);
  //   myMotor->setSpeed(0);
  //   delay(10);
  // }

  delay(100);

  myRegister |= (1 << CWGD);

  if (myRegister & (1 << DRC)) {
  // Transmit start condition and in interrupt clear the TWSTA bit
  TWCR = SEND_START_CONDITION;
  myRegister &= ~(1 << DRC); // dataReadComplete = 0;
  }

  //Serial.println("Start condition sent");

  timeA

    // While communication with gyro device bit is set
  while (myRegister & (1 << CWGD)) {

    // While TWINT is 0 wait in this loop
    while (!(TWCR & (1 << TWINT))) {
      ;
    }

    IsrExitFlow = 0;

    TWCR = TWCR_INITIALISE; // Set TWINT to clear interrupt

    switch (isrFunction) {

      case 0:
      // Initialise MPU-6050

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

            TWDR = 107; // Load register address

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

            TWDR = 9; // Load decimal 9 into register 107 to clear sleep bit, disable temperature sensor and select Gyro X clock

            myRegister |= (1 << STN); // stop_now++;

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


      break;
      

      case 1:
      // Read from GYRO_XOUT
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
            TWDR = GYRO_XOUT_H; // Write the gyro data register address to the slave
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

            gyroValue = gyroValue << 8;

            gyroValue += TWDR;

            IsrExitFlow = 3;

            break;

          default:
            break;
        
        }

      break;


      default:
      //Serial.println("Done");
      break;

    }

    switch(IsrExitFlow) {

      case 0:
      TWCR = SET_TWINT; // 0b11000101
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
      myRegister |= (1 << DRC); // dataReadComplete = 1
      myRegister &= ~(1 << CWGD); // Communication with Gyro device = 0

      Serial.print("Gyro value = ");

      gyroValue /= 10;
      gyroValue += 53;
      
      Serial.println(gyroValue);

      break;

      default:
      break;
    }

  }



  
  //Serial.println(TWSR);

  

}