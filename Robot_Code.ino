
// Include libraries, define variables

#include <avr/io.h>
#include <avr/interrupt.h>

#define CPU_clock 16000000
#define SLAVE_ADDRESS 104
#define GYRO_XOUT_H 67
#define GYRO_XOUT_L 68

#define SEND_START_CONDITION 101
#define TWCR_INITIALISE 69
#define SET_TWINT 197
#define CLEAR_TWEA_FOR_NACK 5
#define SEND_STOP_CONDITION 85

uint8_t isrFunction;
uint16_t gyroValue;

//#include <Wire.h> // The Wire library is what Arduino uses to communicate with I2C devices however I will be creating my own I2C driver





  
// TWI Functions

//  Initialise the TWI peripheral
void twiInitialise(uint8_t bitRateGenerator) {


  // Activate internal pullups for twi
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);

  TWCR = TWCR_INITIALISE;   // Setting control register bits

  TWBR |= bitRateGenerator;  // Setting TWBR to 18 for a SCL frequency of 100kHz

  TWSR &= !(1 << TWPS1) & !(1 << TWPS0);  // Setting pre scaler bits to zero (Pre scaler = 1)

  Serial.println("Initialised");

}

//  TWI_INT interrupt service routine
ISR(TWI_vect) {

  Serial.println("Interrupt executed");


  Serial.print("Entering ISR with TWSR: ");
  Serial.println(TWSR);

  TWCR = TWCR_INITIALISE; // Set TWINT to clear interrupt

  switch(isrFunction) {

    case 0:
    // Initialise MPU-6050

      switch (TWSR) {
        
        case 8:
          //  Start condition has been transmitted

          break;

        case 16:
          // A repeated start condition has been transmitted

          break;

        case 24:
          // SLA+W has been transmitted; ACK has been received


          break;

        case 32:
          // SLA+W has been transmitted; NOT ACK has been received

          break;

        case 40:
          // Data byte has been transmitted; ACK has been received

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

    case 1:
    // We are reading the gyroscopes measurements

      switch(TWSR) {

        case 8:
          //  Start condition has been transmitted, load SLA+W
          TWDR &= 0;  // Clear data register

          TWDR = (SLAVE_ADDRESS << 1); // Load SLA+W, W = 0

          TWCR = TWCR_INITIALISE;  // Clearing the start bit so we don't transmit another one

          Serial.println("Start condition has been transmitted, load SLA+W");

          break;

        case 16:
          // A repeated start condition has been transmitted, now put slave registers address you want to read with read bit

          TWDR = (TWAR & 254) | 1;  // Load SLA+R, R = 1

          Serial.println("A repeated start condition has been transmitted, now put slave registers address you want to read with read bit");

          break;

        case 24:
          // SLA+W has been transmitted; ACK has been received, load data byte

          TWDR = GYRO_XOUT_H;

          Serial.println("SLA+W has been transmitted; ACK has been received, load data byte");

          break;


        case 40:
          // Data byte has been transmitted; ACK has been received, send a repeated start to intiate reading of slaves register

          TWCR = SEND_START_CONDITION;  // Setting the start bit so we transmit a repeated start

          Serial.println("Data byte has been transmitted; ACK has been received, send a repeated start to intiate reading of slaves register");

          break;

        case 56:
          // Arbitration lost in SLA+R or NOT ACK bit
          Serial.println("Arbitration lost in SLA+R or NOT ACK bit");
          break;

        case 64:
          // SLA+R has been transmitted; ACK has been received, return ACK to continue reading (increment register to GYRO_XOUT_L)

          TWCR = TWCR_INITIALISE;  //  Make Sure ACK is sent after data is received

          Serial.println("SLA+R has been transmitted; ACK has been received, return ACK to continue reading (increment register to GYRO_XOUT_L)");
          break;

        case 72:

          break;

        case 80:
          //  Data byte has been received; ACK has been returned, give gyroValue the measurement data and return NACK for next data

          gyroValue = ((uint16_t)TWDR << 8);  //  Set the high byte of gyroValue

          TWCR = CLEAR_TWEA_FOR_NACK; //  Make sure NACK is returned

          Serial.println("Data byte has been received; ACK has been returned, give gyroValue the measurement data and return NACK for next data");
          break;

        case 88:
          //  Data byte has been received; NOT ACK has been returned, give gyroValue low byte of data and send stop condition

          gyroValue |= TWDR;  //  Assign low byte of gyroValue

          TWCR = SEND_STOP_CONDITION; //  Send stop condition

          Serial.println("Data byte has been received; NOT ACK has been returned, give gyroValue low byte of data and send stop condition");
          break;

        default:
          Serial.println("Reached default case");
          break;
      }


    default:
    break;

  }

  
  Serial.print("Value of TWCR: ");
  Serial.println(TWCR);

  Serial.print("Value of TWSR: ");
  Serial.println(TWSR);

  TWCR = SET_TWINT; // Setting TWINT so we stop setting TWINT flag
 

}


void setup() {
  // Setup I2C registers for recieving data, initialise variables, set digital I/O pins
  
  sei();

  Serial.begin(9600);

  while(!Serial);

  Serial.println("Serial begun");

  twiInitialise(18);

  pinMode(4, OUTPUT);

}

void loop() {
 // Set I2C recieved data to variables, calculate angle of rotational displacement for the X axis, Implement PID control for motor speed

  delay(200);

  TWCR = SEND_START_CONDITION;   // Setting control register bits

  digitalWrite(4, HIGH);
  delay(500);
  digitalWrite(4, LOW);
  delay(500);

  Serial.print("Gyro value = ");
  Serial.println(gyroValue);
  Serial.println(TWCR);
  Serial.println(TWSTA);
  Serial.println(TWSR)
  
}
