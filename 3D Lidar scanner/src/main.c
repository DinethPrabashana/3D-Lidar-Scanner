/*
 * Original Code for Stepper Motor and LiDAR Control
 * ------------------------------------------------------------------------------------
 * Contributors: Dineth Perera and Seleka Deemantha
 * The implementation is done without using any external libraries.
 * ------------------------------------------------------------------------------------
 * Description: This program is designed to control a stepper motor using an A4988 driver 
 * and a TFmini-S LiDAR sensor interfaced with an ATmega328P microcontroller. The program 
 * includes functionality for motor acceleration, deceleration, and constant speed rotation 
 * in both clockwise and counterclockwise directions. Additionally, it processes LiDAR sensor 
 * data to measure distances.
 * ------------------------------------------------------------------------------------
 * Stepper Motor Parameters:
 * - STEP_PIN (PD6) and DIR_PIN (PD5) control the motor's step and direction.
 * - MCU_RX (PD2) and MCU_TX (PD3) handle hardware serial communication.
 * - RX (PD0) and TX (PD1) handle FT232RL module communication.
 * - STEPS_PER_REVOLUTION defines the number of steps for a full revolution.
 * - FINAL_VELOCITY, DISPLACEMENT, and acceleration are used for calculating motor dynamics.
 * - totalSteps calculates the total number of steps based on the displacement.
 * -------------------------------------------------------------------------------------
 * Microstepping Configuration:
 * - MS_1 (PB1), MS_2 (PB0), and MS_3 (PD7) configure the microstepping mode.
 * -------------------------------------------------------------------------------------
 * LiDAR Sensor Parameters:
 * - dist, strength, check, and uart array store the LiDAR measurement data.
 * - HEADER defines the frame header for the LiDAR data package.
 * -------------------------------------------------------------------------------------
 * References : ATmega328P Datasheet
 *            : https://www.youtube.com/watch?v=dT0xxaG1DhM&list=PLD7F7ED1F3505D8D5
 * 
 */


#define F_CPU 16000000UL  // Define CPU frequency for delay functions

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <stdlib.h> // for itoa
#include <math.h>

// Stepper motor parameters
#define STEP_PIN PD6  // STEP pin (Pin 10)
#define DIR_PIN PD5   // DIR pin (Pin 9)
#define MCU_RX PD2    // hardware serial port (Pin 32)
#define MCU_TX PD3    // hardware receiver port (Pin 1)
#define RX PD0        // from the FT232RL to MCU which is MCU's Rx and FT232Rl's Transmitter (Pin 30)
#define TX PD1        // from the MCU to the FT232RL where FT232RL's Receiver and MCU Tx (Pin 31)
#define MS_1 PB1
#define MS_2 PB0
#define MS_3 PD7

const int STEPS_PER_REVOLUTION = 200; // Number of steps per revolution
const float FINAL_VELOCITY = 6 * M_PI; // Final angular velocity in rad/s
const float DISPLACEMENT = M_PI;       // Displacement in radians

float acceleration = 0.0;  // Angular acceleration (to be calculated)
int totalSteps = 0;        // Total steps for the motor movement

// LiDAR sensor parameters
int dist = 0;       // Actual distance measurements of LiDAR
int strength = 0;   // Signal strength of LiDAR
int check = 0;      // Save check value
int uart[9];        // Save data measured by LiDAR
const int HEADER = 0x59;  // Frame header of data package

void acceleration_cw();
void rotation_cw();
void deceleration_cw();
void acceleration_ccw();
void rotation_ccw();
void deceleration_ccw();
void delay_T(int delay_time);
void Lidar_data();

void setup() {
	// Set pins as outputs/inputs
	DDRD |= (1 << STEP_PIN) | (1 << DIR_PIN); // STEP and DIR pins as output
	DDRD &= ~(1 << MCU_RX); // MCU_RX as input
	DDRD |= (1 << MCU_TX);  // MCU_TX as output
	DDRD &= ~(1 << RX);     // RX as input
	DDRD |= (1 << TX);      // TX as output
	
	// micro stepping pins 
	DDRB |= (1 << MS_1);
	DDRB |= (1 << MS_2);
	DDRD |= (1 << MS_3);
	
	// initial the micro stepping pins are set to LOW logic
	PORTB &= ~(1 << MS_1);
	PORTB &= ~(1 << MS_2);
	PORTD &= ~(1 << MS_3);

	// Calculate angular acceleration
	acceleration = FINAL_VELOCITY * FINAL_VELOCITY / (2 * DISPLACEMENT);

	// Calculate total steps
	totalSteps = (DISPLACEMENT / (2 * M_PI)) * STEPS_PER_REVOLUTION;
}

void USART_Init(int ubrr){ // argument is the baudrate 
//Referring to the Data sheet Atmega328P page[149] 

	// Initialize serial communication for UART0
	UBRR0H = 0;
	UBRR0L = ubrr;  // Baud rate 9600 for 16MHz clock
	UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Enable RX and TX
	UCSR0C = (1 << USBS0) | (1 << UCSZ00); // 8-bit data, 1 stop bit
}


int main() {
	setup();
	USART_Init(9600) ;
	while (1) {
		Lidar_data();

		acceleration_cw();
		rotation_cw();
		deceleration_cw();

		acceleration_ccw();
		rotation_ccw();
		deceleration_ccw();
	}
	return 0;
}

void acceleration_cw() {
	float currentVelocity = 0.1; // Start with a small initial velocity to avoid division by zero
	float stepInterval;

	PORTB |= (1 << DIR_PIN); // Set DIR_PIN HIGH for clockwise direction
	for (int step = 0; step < totalSteps; step++) {
		stepInterval = 1.0 / (STEPS_PER_REVOLUTION * currentVelocity / (2 * M_PI));

		PORTB |= (1 << STEP_PIN); // Set STEP_PIN HIGH
		delay_T(ceil(stepInterval * 1e6)); // Adjust the pulse width if necessary
		PORTB &= ~(1 << STEP_PIN); // Set STEP_PIN LOW
		delay_T(ceil(stepInterval * 1e6));

		currentVelocity = sqrt(currentVelocity * currentVelocity + 2 * acceleration * (1.0 / STEPS_PER_REVOLUTION));
	}
}

void rotation_cw() {
	int step_time = 1.0 / (STEPS_PER_REVOLUTION * FINAL_VELOCITY / (2 * M_PI));

	PORTB |= (1 << DIR_PIN); // Set DIR_PIN HIGH for clockwise direction
	for (int i = 0; i < 200 * 11; i++) {
		PORTB |= (1 << STEP_PIN); // Set STEP_PIN HIGH
		_delay_us(ceil((step_time / 2)*1e6)); // Adjust delay for step speed
		PORTB &= ~(1 << STEP_PIN); // Set STEP_PIN LOW
		_delay_us(ceil((step_time / 2)*1e6));
	}
}

void deceleration_cw() {
	float currentVelocity = FINAL_VELOCITY;
	float stepInterval;

	PORTB |= (1 << DIR_PIN); // Set DIR_PIN HIGH for clockwise direction
	for (int step = 0; step < totalSteps; step++) {
		stepInterval = 1.0 / (STEPS_PER_REVOLUTION * currentVelocity / (2 * M_PI));

		PORTB |= (1 << STEP_PIN); // Set STEP_PIN HIGH
		delay_T(ceil(stepInterval * 1e6)); // Adjust the pulse width if necessary
		PORTB &= ~(1 << STEP_PIN); // Set STEP_PIN LOW

		delay_T(ceil(stepInterval * 1e6));

		currentVelocity = sqrt(currentVelocity * currentVelocity - 2 * acceleration * (1.0 / STEPS_PER_REVOLUTION));
	}
}

void acceleration_ccw() {
	float currentVelocity = 0.1; // Start with a small initial velocity to avoid division by zero
	float stepInterval;

	PORTB &= ~(1 << DIR_PIN); // Set DIR_PIN LOW for counterclockwise direction
	for (int step = 0; step < totalSteps; step++) {
		stepInterval = 1.0 / (STEPS_PER_REVOLUTION * currentVelocity / (2 * M_PI));

		PORTB |= (1 << STEP_PIN); // Set STEP_PIN HIGH
		delay_T(ceil(stepInterval * 1e6)); // Adjust the pulse width if necessary
		PORTB &= ~(1 << STEP_PIN); // Set STEP_PIN LOW
		delay_T(ceil(stepInterval * 1e6));

		currentVelocity = sqrt(currentVelocity * currentVelocity + 2 * acceleration * (1.0 / STEPS_PER_REVOLUTION));
	}
}

void rotation_ccw() {
	int step_time = 1.0 / (STEPS_PER_REVOLUTION * FINAL_VELOCITY / (2 * M_PI));

	PORTB &= ~(1 << DIR_PIN); // Set DIR_PIN LOW for counterclockwise direction
	for (int i = 0; i < 200 * 11; i++) {
		PORTB |= (1 << STEP_PIN); // Set STEP_PIN HIGH
		_delay_us(ceil((step_time / 2)*1e6)); // Adjust delay for step speed
		PORTB &= ~(1 << STEP_PIN); // Set STEP_PIN LOW
		_delay_us(ceil((step_time / 2)*1e6));
	}
}

void deceleration_ccw() {
	float currentVelocity = FINAL_VELOCITY;
	float stepInterval;

	PORTB &= ~(1 << DIR_PIN); // Set DIR_PIN LOW for counterclockwise direction
	for (int step = 0; step < totalSteps; step++) {
		stepInterval = 1.0 / (STEPS_PER_REVOLUTION * currentVelocity / (2 * M_PI));

		PORTB |= (1 << STEP_PIN); // Set STEP_PIN HIGH
		delay_T(ceil(stepInterval * 1e6)); // Adjust the pulse width if necessary
		PORTB &= ~(1 << STEP_PIN); // Set STEP_PIN LOW
		delay_T(ceil(stepInterval * 1e6));

		currentVelocity = sqrt(currentVelocity * currentVelocity - 2 * acceleration * (1.0 / STEPS_PER_REVOLUTION));
	}
}

// delay function 
void delay_T(int  delay_time){
	int Timer_Temp = 0 ;
	while (Timer_Temp < delay_time){
		Timer_Temp = Timer_Temp + 1 ;
	}
}

//uart communication 
//Referring to the Data sheet Atmega328P page[152]
//Referring to the Data sheet Atmega328P page[151]

void Lidar_data() {
	if (UCSR0A & (1 << RXC0)) { // Check if serial port has data input
		if (UDR0 == HEADER) { // Check data package frame header 0x59
			uart[0] = HEADER;
			if (UDR0 == HEADER) { // Check data package frame header 0x59
				uart[1] = HEADER;
				for (int i = 2; i < 9; i++) { // Save data in array
					uart[i] = UDR0;
				}
				check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
				if (uart[8] == (check & 0xff)) { // Verify received data as per protocol
					dist = uart[2] + uart[3] * 256; // Calculate distance value

					// Convert dist to a string
					char distStr[10];
					itoa(dist, distStr, 10);

					// Send dist string over UART
					for (int i = 0; distStr[i] != '\0'; i++) {
						while (!(UCSR0A & (1 << UDRE0)));
						UDR0 = distStr[i];
					}
				}
			}
		}
	}
}
