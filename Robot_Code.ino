
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
#define PWR_MGMT_1 107
#define SEND_START_CONDITION 101
#define SEND_START_CONDITION_AND_SET_TWINT 229
#define TWCR_INITIALISE 69
#define SET_TWINT 197
#define CLEAR_TWEA_FOR_NACK_AND_SET_TWINT 133
#define SEND_STOP_CONDITION 213
#define SLAVE_ADDRESS 104
#define DRC 7
#define STN 6
#define GVN 5

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

  Serial.println("Initialised");

}

//  TWI_INT interrupt service routine
ISR(TWI_vect) {

  IsrExitFlow = 0;

  TWCR = TWCR_INITIALISE; // Set TWINT to clear interrupt

  switch(isrFunction) {

    case 0:
    // Initialise MPU-6050

      switch (TWSR) {
        
        case 8:
          //  Start condition has been transmitted

          TWCR = TWCR_INITIALISE;

          TWDR = (SLAVE_ADDRESS << 1);  // Load SLA + W
          break;

        case 16:
          // A repeated start condition has been transmitted

          break;

        case 24:
          // SLA+W has been transmitted; ACK has been received

          TWDR = PWR_MGMT_1; // Load power management 1 register
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
    // We are reading the gyroscopes measurements

      switch(TWSR) {

        case 8:
          //  Start condition has been transmitted, load SLA+W

          TWDR = (SLAVE_ADDRESS << 1); // Load SLA+W, W = 0

          TWCR = TWCR_INITIALISE;  // Clearing the start bit so we don't transmit another one

          break;

        case 16:
          // A repeated start condition has been transmitted, now put slave registers address you want to read with read bit

          TWDR = ((SLAVE_ADDRESS << 1) + 1);  // Load SLA + R

          break;

        case 24:
          // SLA+W has been transmitted; ACK has been received, load data byte

          TWDR = GYRO_XOUT_H;

          break;


        case 40:
          // Data byte has been transmitted; ACK has been received, send a repeated start to intiate reading of slaves register

          IsrExitFlow = 1;  // Exit ISR with start condition (Repeated START)

          break;

        case 56:
          // Arbitration lost in SLA+R or NOT ACK bit
          break;

        case 64:
          // SLA+R has been transmitted; ACK has been received, return ACK to continue reading (increment register to GYRO_XOUT_L)

          // IsrExitFlow = 0;
          
          break;

        case 72:

          break;

        case 80:
          //  Data byte has been received; ACK has been returned, give gyroValue the measurement data and return NACK for next data

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
          Serial.println("Reached default case");
          break;
      }

      break;


    default:
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
    myRegister |= (1 << DRC); // dataReadComplete = 1;

    Serial.print("Gyro value = ");
    
    gyroValue /= 100;
    gyroValue += 5;

    Serial.println(gyroValue);

    break;

    default:
    break;
  }

}


void setup() {
  // Setup I2C registers for recieving data, initialise variables, set digital I/O pins
  sei();  // Enable global interrupts

  Serial.begin(9600);

  while(!Serial);

  Serial.println("Serial begun");

  gyroValue = 0;

  isrFunction = 0;  // Initialise MPU-6050

  //stop_now = 0;
  //dataReadComplete = 1;
  //GVN = 0;
  myRegister = 128;

  twiInitialise(18);

}

void loop() {
 // Set I2C recieved data to variables, calculate angle of rotational displacement for the X axis, Implement PID control for motor speed

  if (myRegister & (1 << DRC)) {
    // Transmit start condition and in interrupt clear the TWSTA bit
    TWCR = SEND_START_CONDITION;
    myRegister &= ~(1 << DRC); // dataReadComplete = 0;
  }


  delay(100);
  
}
