/*
 * main.h
 *
 *  Created on: 16.11.2010
 *      Author: Michael Brunnbauer
 */

#ifndef MAIN_H_
#define MAIN_H_


//#define LOKBASIS_NO_TEST	// für Testmodus remmen - diverse Sicherheitsabfragen werden für's Testen deaktiviert


// Macros für Bit-Operationen
#define setbit(P,BIT) 	((P) |= (1<<(BIT)))
#define clearbit(P,BIT)	((P) &= ~(1<<(BIT)))

#define UART_MAXSTRLEN 32		// maximal erlaubte Länge eines Befehls "<*>" incl. <> - ursprünglicher Wert: 64, derzeit wg. Problemen 32!!

#define DEVICE_STRLEN 25		// maximal erlaubte Länge eines Strings für Namen in den DeviceProperties
#define DEVICE_OPTPROP_COUNT 8		// Anzahl der möglichen optionalen DeviceProperties

#define ALIVE_INTERVAL 5		//Prüfintervall = 5 Sekunden! -> Lok stoppt nach 5sek herrenloser Fahrt

// Statusdefinitionen für Interrupts und Main-Schleife
#define STATE_1X_PRO_SEK 1		// Timer meldet sich 1x pro Sekunde
#define STATE_5X_PRO_SEK 2		// Timer meldet sich 5x pro Sekunde
#define STATE_WLAN_DATA 4		// WLAN Daten vorhanden
#define STATE_RFID_DATA 8		// RFID Daten vorhanden
#define STATE_USB_DATA 16		// USB Daten vorhanden (Bootloader? oder Terminal
#define STATE_SPI_DATA 32		// SPI-ADC-Daten vorhanden
#define STATE_7 64				//
#define STATE_8 128				//

// Config-Definitionen
#define CONFIG_DO_SPIADC 1		//	Signal "Motor messen" für ATtiny26 erzeugen, wenn bit gesetzt ist
#define CONFIG_2 2
#define CONFIG_3 3
#define CONFIG_4 4
#define CONFIG_5 5
#define CONFIG_6 6
#define CONFIG_7 7
#define CONFIG_8 8

// WLAN-Definitionen
#define WLAN_WIZNET	1		// WLAN-Modul Wiznet WIZ610wi
#define WLAN_RN131	2		// WLAN-Modul  RN-131

// definitionen für Steuerung
#define RICHTUNG_VW	1		// Richtung: vorwärts = 1
#define RICHTUNG_RW	0		// Richtung: rückwärts = 0

// extern verwendete Variablen
extern unsigned int speed;
extern unsigned int speed_soll;
extern unsigned char richtung;
extern unsigned char richtung_soll;
extern unsigned char speedstep_korrektur;
extern volatile unsigned char motor_reg;	// Variable für Motor-Regelung ein/aus (auch für isr verwendet)
extern uint8_t motorerror;				// Errorcode von Motorcontroller: 0 = kein Error
extern unsigned char cfg_wlan;		// welches WLAN-Modul wird verwendet? Wert siehe: WLAN-Definitionen
extern unsigned char alivecount;
extern volatile char alone;
extern volatile unsigned char state;		// Status-Variable für Interrupts und Main-Schleife
extern volatile unsigned char config;		// Config-Variable für Interrupts und Main-Schleife (für diverse globale Einstellungen)
extern char wlan_string[UART_MAXSTRLEN+1];  // globaler String zum Abspeichern des vom WLAN empfangen Strings in read_wlan
extern char rfid_string[13];  				// globaler String zum Abspeichern des vom RFID-Board empfangen Strings in read_rfid
extern char usb_string[UART_MAXSTRLEN+1];  	// globaler String zum Abspeichern des von USB empfangen Strings in read_usb
extern volatile uint8_t uart_wait_no_more;	// kennung für read_wlan(), das lange genug gewartet wurde (muss dort wieder auf 0 gesetzt werden)
extern volatile uint8_t uart_ov_limit;		// in read_wlan() zu setzendes limit (wg. "langen" befehlen bis 45s!!) // muss nicht volatile sein?

// bei folgenden 4 variablen war kein extern vorhanden!!!?!
//extern unsigned int U_Motor1;				//Variablen für Spannungswerte vom ATtiny26-Board
//extern unsigned int U_Motor2;				// Wert 65535 = ungültig/unbekannt bzw. nicht gemessen
//extern unsigned int U_Schiene;
//extern unsigned int U_Akku;


// für Power-Management
extern volatile uint8_t relaycouter;
extern uint8_t prepare_switch_off;

// LokData.
struct devdata_t {
    char kennung[4];					// Kennung, ob gültige daten vorhanden sind ("WBC")
    char lokname[DEVICE_STRLEN+1];		// Devicename
    char owner[DEVICE_STRLEN+1];		// Besitzerkennung
    char ssid[11];
    char modelname[DEVICE_STRLEN+1];	// Lok-Modellbezeichnung
    uint8_t notstoptimeout;
// später    int16_t pwmf;	// verwendete pwm-freuqenz // jetzt wäre einfacher int8_t Nummer der pwf-frequenz
} ;

extern struct devdata_t lokdata;

extern char iamlok[DEVICE_STRLEN+10];

extern const char dev_swname[] PROGMEM;
extern const char dev_swversion[] PROGMEM;

struct opData{
    char name[DEVICE_STRLEN+1];		// Name des opt. Properties
    uint8_t exists;					// 0: optProp. ist nicht vorhanden. 1: vorhanden
};

extern struct opData optprops[DEVICE_OPTPROP_COUNT] PROGMEM;
extern const char wlanModules[3][DEVICE_STRLEN+1] PROGMEM;

// LED-Controller Variablen (für 1 LED-Controller TCL 59116)
struct LEDdata {
	unsigned int exist;			// welche LEDs existieren (LED 0 - 15 -> bit 0 - 15)
	//unsigned int group;			// welche LEDs gehören zur Gruppe  ( - " - ) -> outputmode pro LED
    //uint8_t max_value[16];		// Maximalwert für Helligkeit (bei 18mA -> wenn I_F einer LED geringer -> Wert muss im Verhältnis reduziert werden)
    uint8_t value[16];			// Helligkeit der LEDs (darf nie > max_value werden!!)
    uint8_t group_value;		// Gruppenhelligkeit
    uint8_t blinkfreq;			// 2Hz // PWM Wert -> Blinkfrequenz = 24 / (wert + 1) [Hz]
    uint8_t groupmode_blink;	// welcher Gruppenmodus: LEDC_MODE2_BLINK oder LEDC_MODE2_DIMM
    uint8_t outputmode[16];		// Mode pro LED für die LEDOUT register (0: immer aus, 1: immer voll ein, 2: ein + helligkeit per pwm, 3: ein + helligkeit per pwm + helligkeit/blink per gruppen/blink register
};

extern struct LEDdata LED1;


#endif /* MAIN_H_ */
