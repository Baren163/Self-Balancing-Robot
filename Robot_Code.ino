
// Include libraries, define variables

#include <avr/io.h>
#include <avr/interrupt.h>

#define CPU_clock 16000000
#define SLAVE_ADDRESS 104

#define SEND_START_CONDITION 101
#define TWCR_INITIALISE 69
#define SET_TWINT 197

//#include <Wire.h> // The Wire library is what Arduino uses to communicate with I2C devices however I will be creating my own I2C driver

// TWI Functions

//  Initialise the TWI peripheral
void twiInitialise(uint8_t bitRateGenerator) {


  // Activate internal pullups for twi
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);

  TWBR |= bitRateGenerator;  // Setting TWBR to 18 for a SCL frequency of 100kHz

  TWSR &= !(1 << TWPS1) & !(1 << TWPS0);  // Setting pre scaler bits to zero (Pre scaler = 1)

}

//  TWI_INT interrupt service routine
ISR(TWI_vect) {

  Serial.println("Interrupt executed");

  // We are reading the gyroscopes measurements
  switch(TWSR) {

    case 8:
      //  Start condition has been transmitted
      Serial.println("Start condition transmitted, load SLA+W");

      TWCR = TWCR_INITIALISE;  // Clearing the start bit so we don't transmit another one

      // load SLA + W
      TWDR = (SLAVE_ADDRESS << 1);

      break;

    case 16:
      // A repeated start condition has been transmitted, now put slave registers address you want to read with read bit

      TWDR = (TWAR & 254) | 1;  // Load SLA+R, R = 1

      Serial.println("A repeated start condition has been transmitted, now put slave registers address you want to read with read bit");

      break;

    case 24:
      // SLA+W has been transmitted; ACK has been received, load data byte


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

      break;

    case 72:

      break;

    case 80:

      break;

    case 88:

      break;

    default:
      Serial.println("Reached default case");
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
