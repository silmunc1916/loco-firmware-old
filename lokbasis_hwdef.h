/*
 * lokbasis_hwdef.h
 *
 *  Created on: 22.10.2011
 *      Author: Michael Brunnbauer
 *
 *  Hardwaredefinitionen für Lokbasis Hardware
 *
 *
 *
 */

#ifndef LOKBASIS_HWDEF_H_
#define LOKBASIS_HWDEF_H_

#ifndef __AVR_ATmega2561__
#define __AVR_ATmega2561__
#endif
// mögliche Board Konfiguration
#if defined(__AVR_ATmega2560__)
	#define HW_TESTBOARD1		// Testboard1 (WLAN:uart2, log:uart3)
#elif defined(__AVR_ATmega2561__)
	#define HW_TESTBOARD2		// Testboard2 (WLAN:uart0, log:-)
#endif


// Tesboard1
#if defined( HW_TESTBOARD1 )

// UART Definitionen
#define WLAN 2	// uart2 für WLAN
#define RFID 1	// uart1 für RFID
#define LOG  3	// uart3 für log/debug-ausgabe über USB

#define UART_BAUDRATE_1	9600	// RFID-Verbindung
#define UART_BAUDRATE_2	38400	// WLAN-Verbindung
#define UART_BAUDRATE_3	250000	// USB-Verbindung zum PC

#define UART_SETTING_1 ((F_CPU/16L/UART_BAUDRATE_1)-1)	// Berechnung der UART Konfigurationsparameter
#define UART_SETTING_2 ((F_CPU/16L/UART_BAUDRATE_2)-1)
#define UART_SETTING_3 ((F_CPU/16L/UART_BAUDRATE_3)-1)

#define TESTLED		1		// Funktion TestLED ist vorhanden

#define MOTORC_L6205 1	// verwendeter Motorcontroller

#if defined( MOTORC_L6205 )
#define MC_INV_PWM_4_REV 1	// PWM für Rückwärts invertieren	// bei L6205 nötig, für MOTORC_POLOLU24V20 nicht
#endif
#endif

//---------------------------------------------------------------------------------------------



// Tesboard2
#if defined( HW_TESTBOARD2 )

// UART Definitionen
#define WLAN 0	// uart0 für WLAN
//#define RFID 1	// uart1 für RFID

#define UART_BAUDRATE_RFID	9600	// RFID-Verbindung
#define UART_BAUDRATE_WLAN	38400	// WLAN-Verbindung
#define UART_BAUDRATE_USB	250000	// USB-Verbindung zum PC

#define UART_SETTING_RFID ((F_CPU/16L/UART_BAUDRATE_RFID)-1)	// Berechnung der UART Konfigurationsparameter
#define UART_SETTING_WLAN ((F_CPU/16L/UART_BAUDRATE_WLAN)-1)
#define UART_SETTING_USB ((F_CPU/16L/UART_BAUDRATE_USB)-1)


// Motorcontroller
#define MOTOR1_ADC 	PF7 	// Input, ADC6 Motor 1
#define MOTOR2_ADC 	PF6 	// Input,ADC7 Motor 2
#define MOTOR_DIR 	PB4 	// Output, Motoren Richtung
#define MOTOR1_PWM 	PB6 	// Output, OC1B Reserve 1. Motor PWM, 	Umjumperbar (PB5/PB6 ), ob mit 1. oder 2. Motor-PWM versorgt wird
#define MOTOR2_PWM 	PB5 	// Output, OC1A 2. Motor PWM
#define MOTOR_RESET PB7 	// Output, Motoren Reset
#define MOTOR1_FF1 	PG2 	// Input, Motor1 FF1
#define MOTOR1_FF2 	PG1 	// Input, Motor1 FF2
#define MOTOR2_FF1 	PG0 	// Input, Motor2 FF1
#define MOTOR2_FF2 	PG5 	// Input, Motor2 FF2
#define DDR_MOTOR_ADC	DDRF
#define DDR_MOTOR_OUT	DDRB
#define DDR_MOTOR_IN	DDRG
#define PORT_MOTOR	PORTB
#define PIN_MOTOR_ADC PINF
#define PIN_MOTOR PING
#define MOTORC_POLOLU24V20 1	// verwendeter Motorcontroller

// WLAN-Modul
#define WLAN_RESET 	PE2 	// Output, F1: Reset bei Wiznet: (active HIGH) 1 = reset, 1 für 3s = Factory Default
#define WLAN_RX 	PE0 	// Input, Atmega RX, Wiznet Data out
#define WLAN_TX 	PE1 	// Output, Atmega TX, Wiznet Data in
#define WLAN_DATAMODE 	PE3 	// Output, F2: HW-Trigger bei Wiznet, 0: CMD, 1:Data
#define DDR_WLAN	DDRE
#define PORT_WLAN	PORTE
#define PIN_WLAN	PINE
#define WLAN_ONOFF	PE4		// Output 0: Wiznet ausgeschalten, 1: Wiznet ein

// Versorgungs-Modul
#define AKKU_OFF 	PA2 		// Output, AKKU/Schiene Umschaltung (0: Akku (stromloser Zustand), 1: Schiene (muß geschalten bleiben!) [auf ULN2803]
#define SYSTEM_OFF 	PA0 		// Output, Versorgung aus (bei Abschaltung) [auf ULN2803]
#define SYSTEM_ON 	PA1 		// Output, Versorgung ein [auf ULN2803]
#define AKKU_ADC 	PF5 		// Input, ADC Akku-Spannung
#define SCHIENE_ADC PF4 		// Input, ADC Schienenspannung
#define DDR_SYS_ADC DDRF
#define DDR_SYS		DDRA
#define PORT_SYS	PORTA
#define PIN_SYS_ADC PINF

// Test-LEDs am MC-Modul
#define TESTLED		1			// Funktion TestLED ist vorhanden
#define TESTLED0 	PF0			// (JP 1-5 gesetzt), alternative Funktion: ADC0
#define TESTLED1 	PF1 		// (JP 1-6 gesetzt), alternative Funktion: ADC1
#define TESTLED2 	PF2 		// (JP 1-7 gesetzt), alternative Funktion: ADC2
#define TESTLED3 	PF3 		// (JP 1-8 gesetzt), alternative Funktion: ADC3
#define DDR_TESTLED DDRF
#define PORT_TESTLED PORTF

// I2C LED-Controller
#define LEDC_TLC59116	1			//LED-Controller TLC59116 ist vorhanden
#define I2C_RESET   0xD6			// 0xD6 = Software Reset I2C Bus Adresse
#define LEDC_MODE2_BLINK	32		// Mode 2 = Blinken
#define LEDC_MODE2_DIMM		0		// Mode 2 = Dimmen
#define LEDC1  0xC0      			// I2C-Adresse des LED-Controller

#endif	// HW_TESTBOARD2


// allgemeine Definitionen

#define POWER_FROM_RAILS		0	// Versorgung über Schienenstrom
#define POWER_FROM_BATTERY		1	// Versorgung aus AKKU


#endif	// LOKBASIS_HWDEF_H_
