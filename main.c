/*
 * main.c
 *
 *	Version: 0.15	- neu: EEPROM LokData, DeviceProperties, HW Definitionen
 *  Created on: 04.07.2010 - 01.03.2012
 *  Author: Michael Brunnbauer
 */


#ifndef F_CPU
#define F_CPU 16000000UL	// für 16MHz CPU-Takt
#endif


#ifndef EEMEM	//#include <avr/eeprom.h> ist zwar angegeben, EEMEM Definition funktioniert aber trotzdem nicht!!
#define EEMEM __attribute__((section(".eeprom")))	// deshalb hier nochmal
#endif

#include <avr/io.h>
#include <string.h>		// für "strcmp"
#include <stdlib.h>		// für "itoa"
#include <util/delay.h>	// für delay_ms()
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/twi.h>	// für I2C

#include "lokbasis_hwdef.h"		// Hardware-Definitionen für die verschiedenen Boards
#include "main.h"
#include "uart.h"
#include "speed.h"				// Geschwindigkeitsregelung
#include "commands.h"			// Befehle auswerten
#include "wlan.h"
#include "funktionen.h"			// allgemeine Funktionen (Hardware und andere)
#include "i2cmaster.h"			// I2C Funktionen
#include "i2clcd.h"				// I2C LCD Funktionen - für's Debugging mit Display


//----------------------------------------------------------------------------------------------
volatile unsigned char state = 0;		// Status-Variable für Interrupts und Main-Schleife
volatile unsigned char config = 0;		// Config-Variable (für diverse globale Einstellungen) [1:SPI-ADC ein]

// Variablen für lesen von uart2 - interrupt-verarbeitung (müssen deshalb volatile sein!!)
volatile uint8_t uart_str_complete=0;
volatile uint8_t uart_str_count=0;

volatile uint8_t uart_wait_no_more=0;	// kennung für read_wlan(), dass lange genug gewartet wurde (muss dort wieder auf 0 gesetzt werden)
volatile unsigned int uart_ov_count=0;	// counter-overflow für isr (warten auf wlan)
volatile uint8_t uart_ov_limit=10;		// in read_wlan() zu setzendes limit (wg. "langen" befehlen bis 45s!!) // muss nicht volatile sein?

unsigned int timer5_count = 0;			// zähler für timer5 (für isr)

char wlan_string[UART_MAXSTRLEN+1]="";  // globaler String zum Abspeichern des vom WLAN empfangen Strings in read_wlan
//char usb_string[UART_MAXSTRLEN+1]="";  	// globaler String zum Abspeichern des von USB empfangen Strings in read_usb
char rfid_string[13]="";  				// globaler String zum Abspeichern des vom RFID-Board empfangen Strings in read_rfid

unsigned int speed = 0;					// globale variable für den Motor-Speed
unsigned int speed_soll = 0;			// globale variable für den Motor-Speed - Vorgabe vom Controller
unsigned char richtung = RICHTUNG_VW;			// globlae variable für die Richtung (vorwärts = 1, rückwärts = 0)
unsigned char richtung_soll = RICHTUNG_VW;		// globlae variable für die Richtung - Vorgabe vom Controller (vorwärts = 1, rückwärts = 0)
unsigned char speedstep_korrektur = 0;			// Korrekturvariable, falls 8bit oder 9bit pwm statt 10bit verwendet wird
										// gibt an, um wieviele bit der speedwert geshiftet werden muss, damit er für die 10bit Verarbeitung passt (im timer3 interrupt)

unsigned char alivecount = 0;	// globlae Variable zum Zählen der Meldungen von der Gegenstelle
volatile char alivesecs = 0;	// globlae Variable die zählt, wie lange die Zählung geht -> man kann variable bestimmen, in welchem intervall geprüft wird

volatile unsigned int uein = 0;
volatile unsigned char motor_reg = 1;	// Variable für Motor-Regelung ein/aus (auch für isr verwendet)
volatile unsigned char blink = 1;		// zeigt an, dass geblinkt werden soll
uint8_t motorerror = 0;					// Errorcode von Motorcontroller: 0 = kein Error

//unsigned int U_Motor1 = 65535;				//Variablen für Spannungswerte vom ATtiny26-Board
//unsigned int U_Motor2 = 65535;				// Wert 65535 = ungültig/unbekannt bzw. nicht gemessen
//unsigned int U_Schiene = 65535;
//unsigned int U_Akku = 65535;


// für Power-Management
volatile uint8_t powersource = POWER_FROM_RAILS;	// Versorgung über Schienen (später: POWER_FROM_BATTERY) bei Start
volatile uint8_t relaycouter = 0;					// counter für Abschaltung relayimpuls: 0:inaktiv / 1: Impuls beenden / >1: countdown per timer5 (4ms Intervall)
													// für Versorgung ein/aus
volatile uint8_t relay_stopimp = 0;					// 1: zeigt der Schleife an, dass der Relais-Impuls beendet werden soll
uint8_t prepare_switch_off = 0;				// 1: zeigt der Schleife an, dass sich die Lok ausschalten soll, sobald sie steht (bzw. langsam genug ist)

//für SPI interupt
//volatile unsigned char adcdaten[16]="";  // empfangene Daten vom SPI-Master
//volatile unsigned char spicounter = 0;  // Counter für empfangene Daten vom SPI-Master


struct devdata_t lokdata;			// Lokdata zur Laufzeit
struct devdata_t EElokdata EEMEM;	// Lokdata EEPROM variable

uint8_t eetest[4] EEMEM = { 2,4,6,8 };
char iamlok[DEVICE_STRLEN + 10] = "";			// für Lok-Meldung

// LokData im Flash
const char dev_swname[] PROGMEM = "WBlokbasis";     // -> keine Änderung durch User -> flash
const char dev_swversion[] PROGMEM = "0.15";   		// -> keine Änderung durch User -> flash

unsigned char cfg_wlan = WLAN_WIZNET;	// welches WLAN-Modul wird verwendet? Wert siehe: WLAN-Definitionen // const machen?
										// ist auch index für wlanModules[index][26] -> der Klartextname für das Modul

// vorhandene optionale DeviceProperties definieren. struct opData ist in main.h deklariert
struct opData optprops[DEVICE_OPTPROP_COUNT] PROGMEM = {
		{ "snd", 	0 },
		{ "horn", 	0 },
		{ "lt", 	1 },
		{ "steam", 	0 },
		{ "mc", 	1 },
		{ "rfid", 	1 },
		{ "adc", 	1 },
		{ "sl", 	0 }
};

// Strings für Name der WLAN-Module (wird in den Deviceproperties verlangt)
const char wlanModules[3][DEVICE_STRLEN+1] PROGMEM = {
		{ "unbekannt" },
		{ "Wiznet WIZ610wi" },
		{ "RN-131" }
};


struct LEDdata LED1 = {
		.exist = 0xFFFF,
		//.max_value = { 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255 },
		//.value = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		.value = { 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127 },
		.group_value = 127,		// (globale Helligkeit wird nicht verwendet) im Blink-Modus: On:Off Verhältnis (PWM-Duty Cycle)
		.blinkfreq = 11,
		.groupmode_blink = LEDC_MODE2_BLINK,	// (globale Helligkeit wird nicht verwendet)
		.outputmode = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

// Strings im Flash
const char txtp_iamlok[] PROGMEM = "<iamlok:";
const char txtp_sd[] PROGMEM = "<sd:";
const char txtp_cmdend[] PROGMEM = ">";


int main(void)
{
	char test[UART_MAXSTRLEN+1] = "";			// für string-bearbeitungen
	uint32_t loop_count = 0;					// Zähler für Hauptschleife
	uint32_t last_loop_count = 0;					// letztes gültiges Ergebnis des Zählers für Hauptschleife
	//char waitcount = 0;							// Zähler für WLAN-Abfragen



	// ========================  Hardware Initialisierung  ========================================================


#if defined( HW_TESTBOARD2 )	// Ausgänge für TestLEDs setzen
		DDR_TESTLED |= (1<<TESTLED0) | (1<<TESTLED1) | (1<<TESTLED2) | (1<<TESTLED3);	// als Ausgang setzen
		PORT_TESTLED |= (1<<TESTLED0) | (1<<TESTLED1) | (1<<TESTLED2) | (1<<TESTLED3);	// LED1-3 auf 1 setzen -> "finster"
#endif


#if defined( HW_TESTBOARD2 )
		// Initialisierung Powermanagement: Achtung Inerrupts sind noch nicht aktiv (noch kein countdown für Impulsdauer - Impuls dadurch länger, kein Problem)
		DDR_SYS |= (1<<AKKU_OFF) | (1<<SYSTEM_OFF) | (1<<SYSTEM_ON);	// als Ausgang setzen (Relays für Powermanagement)
		system_power_on();	// schnellstens das Relais für die Versorgung einschalten!!
		//set_powersource(POWER_FROM_RAILS);		// für Test ohne Akku - sonst kann man es weglassen, da das monostabile Relais im stromlosen Zustand auf Akku (POWER_FROM_BATTERY) steht
#endif


#if defined( HW_TESTBOARD1 )
	unsigned char spi_error = 255;				// 0 - kein Error - zeigt an, ob die aktuellen SPI-Daten gültig sind
	unsigned char adcdaten_sic[16]="";			// zum Kopieren der empfangenen Daten vom SPI-Master
#endif

	// Initialisierung  -----------------------------------------------------------------
	#if defined( HW_TESTBOARD1 )	// MC-Board Hardware für Testboard1
		setbit(DDRD,5);		// LED: Port D bit 5 auf Ausgang (1) schalten
		clearbit(DDRE,5);	// USB: Port E bit 5 auf Eingang (0) schalten. Ist 1 wenn USB angeschlossen ist
	#endif

	wlan_init(cfg_wlan);	// WLAN (ua. UART-RX-Interrupt wird hier aktiviert)


	// I2C Initialisierung
	DDRC |= (1<<PD0)| (1<<PD1);	// I2C: SDA (PD1) und SCL (PD0) Pins auf Ausgang setzten
	PORTC |= (1<<PD0)|(1<<PD1);	// I2C: und auf 1 setzen (Ruhezustand)
	//TODO: PortC/PortD mischmasch!!!

	i2c_init();             	// initialize I2C library
	//ledcontrol_reset();			// LED-Controller reseten
	//ledcontrol_init();			// und auf Anfangswerte stellen

	lcd_init();					// LCD initialisieren
	lcd_command(LCD_CLEAR);		// LCD-Anzeige loeschen
	lcd_backlight(ON);			// Hintergrundbeleuchtung an





	#if defined( RFID )
		init_uart(RFID, UART_SETTING_RFID);	// uart für rfid
	#endif

	#if defined( LOG )
		init_uart(LOG, UART_SETTING_3);	// uart für rfid (nur bei Testboard1)
	#endif


		// LED-Test: 3s alle LEDs leuchten lassen
		PORT_TESTLED &= ~(1<<TESTLED1);	// Testled0 einschalten
		PORT_TESTLED &= ~(1<<TESTLED1);	// Testled1 einschalten
		PORT_TESTLED &= ~(1<<TESTLED2);	// Testled2 einschalten
		PORT_TESTLED &= ~(1<<TESTLED3);	// Testled3 einschalten
		warte_ms(3000);
		PORT_TESTLED |= (1<<TESTLED0);	// wieder ausschalten
		PORT_TESTLED |= (1<<TESTLED1);	// wieder ausschalten
		PORT_TESTLED |= (1<<TESTLED2);	// wieder ausschalten
		PORT_TESTLED |= (1<<TESTLED3);	// wieder ausschalten

		warte_ms(10000);	// Test
		wlan_on(1);	// Wiznet WLAN-Modul jetzt einschalten

		//wlan_command_query2("<WR>", test);		// Wiznet restart // ändern: nur senden, nicht auf Antwort warten!!


	// EEPROM Daten auslesen ---------------------------------------------------------------------------
	eeprom_read_block((void*)&lokdata, (const void*)&EElokdata, sizeof(lokdata));
	// checken, ob die Kennung für gültige Lok-Daten vorhanden ist
	if (strncmp(lokdata.kennung, "WBC", 3))	// Ergebnis = 0 bei Match !!
	{
		// lokdata Initialisierung (wenn noch keine sinnvollen Daten im EEPROM vorhanden sind
		// noch ändern auf _P, damit die Strings im FLASH liegen
		strlcpy(lokdata.kennung, "WBC", 4);
		strlcpy(lokdata.lokname, "neue Lok", DEVICE_STRLEN+1);
		strlcpy(lokdata.owner, "unbekannt", DEVICE_STRLEN+1);
		strlcpy(lokdata.modelname, "unbekannt", DEVICE_STRLEN+1);
		strlcpy(lokdata.ssid, "WBROUT1", 11);
		//lokdata.pwmf = 15000;
		lokdata.notstoptimeout = ALIVE_INTERVAL;	// Anfangswert aus Programm setzen, wenn noch kein Userwert eigenstellt wurde
		eeprom_update_block((const void*)&lokdata, (void*)&EElokdata, sizeof(lokdata));	// speichern
	}

	// iamlok string erzeugen
	memset(iamlok, 0, DEVICE_STRLEN+10);	// string leeren
	strlcpy_P(iamlok, txtp_iamlok, DEVICE_STRLEN+10);
	strlcat(iamlok, lokdata.lokname, DEVICE_STRLEN+10);
	strlcat_P(iamlok, txtp_cmdend, DEVICE_STRLEN+10);


	// SPI-Ports für Kommunikation mit ADC-Board ATTiny26 (nur Testboard1)--------------------------------------------------
#if defined( HW_TESTBOARD1 )	// für Testboard1
	setbit(DDRH,7);		// PH7: auf Ausgang - Signal Motor-Messen für ADC-Board ATTiny26
	clearbit(PORTH,7);	// PH7: 0 ausgeben -> steigende Flanke aktiviert Interrupt am ADC-Board
	clearbit(DDRB,PB2);	// MOSI (PB2)Input
	setbit(DDRB,PB3);	// MISO (PB3)Output
	clearbit(DDRB,PB1);	// SCK (PB1)Input
	clearbit(DDRB,PB0);	// SS (PB0) Input

	SPCR = (1<<SPIE) | (1<<SPE);	// Interrupt für SPI-"End of Transmission" aktivieren + SPI enable
	// bit5=0 -> MSB of the data word is transmitted first
	// bit4=0 -> Slave (1=Master)
	// bit3=0 -> CPOL: SCK is LOW when idle (Takt zuerst "rising", dann "falling")
	// bit2=0 -> CPHA: (zuerst) lesen, dann schreiben
	// bit1-0: SCK Freuqenz wenn Master (keine Bedeutung am Slave)
	SPDR = 255; 			// SPI: erstes (sinnloses) Datenbyte setzen
#endif
	//---------------------------------------------------------------------------------------------------------

#if defined( HW_TESTBOARD1 )	// für Testboard1 - ev. später auch für Testboard 2 bei Motor-Messung?
	// timer 4 initialisieren - für BACK-EMF-Messung ------------------------------------
	TCNT4 = 0;			// set timer counter initial value
	OCR4A = 1023;			// TOP value setzen

	setbit(TCCR4B,CS42);// start timer with prescaler 1024 -> 15,26Hz / 65ms
	clearbit(TCCR4B,CS41);
	setbit(TCCR4B,CS40);
	clearbit(TCCR4B,WGM43);// mode 4 - TOP= OCR5A = 1023  F_timer = f_clk / (prescaler * (1+ TOP))
	setbit(TCCR4B,WGM42);
	clearbit(TCCR4A,WGM41);
	clearbit(TCCR4A,WGM40);

	// setbit(TIMSK4,TOIE4);// enable timer overflow interrupt -> nicht bei mode 4  !!!!
	setbit(TIMSK4,OCIE4A);	// enable output compare match interrupt
#endif

	// timer 5 als neuer interner timer (statt 0 + 3) mit 244Hz (statt 61 und 30) (passt für atmege2560 und 2561) ------------------------
	TCNT5 = 0;				// set timer counter initial value
	OCR5A = 255;			// TOP value setzen

	setbit(TCCR5B,CS52);	// start timer with prescaler 256 -> 244Hz = 4ms
	clearbit(TCCR5B,CS51);
	clearbit(TCCR5B,CS50);

	clearbit(TCCR5B,WGM53);	// Mode 4 (0100) OCR5A = 255, damit nur bis 255 gezählt wird
	setbit(TCCR5B,WGM52);
	clearbit(TCCR5A,WGM51);
	clearbit(TCCR5A,WGM50);

	setbit(TIMSK5,OCIE5A);	// enable output compare match interrupt
	// setbit(TIFR5,OCF5A);

	// Ausgänge für Motorsteuerung, PWM ---------------
	init_motorctrl();


	sei();	// Interrupts aufdrehen

	// ========================  Hardware Initialisierung abgeschlossen  ================================================


	lcd_printlc(1,1,(unsigned char *)"lokbasis2 v0.15");
	lcd_printlc(2,1,(unsigned char *)"LCD funktioniert");
	warte_ms(5000);
	lcd_command(LCD_CLEAR);




	/*
	log_puts_P("Lok-Basis wird gestartet...\r\n");

	if (cfg_wlan == WLAN_WIZNET) { log_puts_P("WLAN-Modul = Wiznet\r\n"); }
	else if (cfg_wlan == WLAN_RN131) { log_puts_P("WLAN-Modul = RN131\r\n"); }


	log_puts_P("Abfrage der SSID... ");
	warte_ms(2000);

	if (cfg_wlan == WLAN_WIZNET)	// bei Wiznet SSID prüfen
	{
		wlan_command_query2("<DS>", test);		// Abfrage der SSID

	// Test: vergleich mit lokdata.ssid funkt. noch nicht
		if (strncmp(test, "WBROUT1", 7))		// wenn SSID nicht = WBROUT1 -> WLAN-Modul konfigurieren // oder andere Aktion (weil fremde Anlage?)
		//if (strncmp(test, lokdata.ssid, 7))		// wenn SSID nicht = WBROUT1 -> WLAN-Modul konfigurieren // oder andere Aktion (weil fremde Anlage?)
		{
			log_puts_P(" passt nicht!\r\n");
			PORT_TESTLED &= ~(1<<TESTLED2);	// Testled2 einschalten
			//setup_wlan();
			//wlanreset();
			//PORT_TESTLED |= (1<<TESTLED2);	// wieder ausschalten
		}
		else
		{
			log_puts_P(" OK.\r\n");
			PORT_TESTLED &= ~(1<<TESTLED1);	// Testled1 einschalten
		}
	}
	else if (cfg_wlan == WLAN_RN131)
	{
		rn131_command_query("get wlan\r\n", "Chan=", test, UART_MAXSTRLEN);	// in test kommt alles vor "Chan=" zurück!!!!!!!

		if (strcasestr(test, "SSID=WBROUT1"))		// wenn SSID nicht = WBROUT1 -> WLAN-Modul konfigurieren
		{
			log_puts_P(" OK.\r\n");
		}
		else
		{
			log_puts_P(" passt nicht!\r\n");
			rn131_init(test, UART_MAXSTRLEN);
		}
	} */



	// Testweise RN-131 neu initialisieren
	//log_puts_P("RN-131 wird neu initialisiert\r\n");
	//rn131_init(test, UART_MAXSTRLEN);

	//warte_ms(5000);
	//rn131console();

/*
	// auf WLAN-Verbindung warten und dann eigene Identität senden
	if (cfg_wlan == WLAN_WIZNET)
	{
		serial_cmd_ein();
		uartx_puts_P(WLAN, "<QP>");	// WLAN-Verbindung (zum AP bzw. Router) abfragen <S0> wäre: "nicht verbunden", <S1_...> "verbunden"
		serial_cmd_aus();
		warte_ms(1000);
		waitcount = 0;
		uint8_t connected = 0;

		do
		{
			read_wlan(30);
			waitcount++;
			//if (waitcount==4)	// nach 30*4=120s
			//{
			//	wlanreset();	// Reset für WLAN-Modul ausführen
			//}

			if ((!strncmp(wlan_string, "<S1", 3))) { connected = 1; }
			if (waitcount > 10) { connected = 1; }	// test bei (waitcount > 10) aus Schleife aussteigen!
			if (!connected) { warte_ms(10000); }	// warten, wenn noch nicht verbunden

		} while (!connected);	// nur die ersten 3 Zeichen checken
		PORT_TESTLED &= ~(1<<TESTLED3);	// Testled3 einschalten

		log_puts_P("WLAN verbunden!\r\n");
		blink = 0;	// WLAN = verbunden -> blinken beenden

	#if defined( TESTLED )	// Verbindung ok signalisieren
		led_aus();
	#endif
	}
	else if (cfg_wlan == WLAN_RN131)
	{
		unsigned int constat = 0;
		char *zahl;
		do
		{
			rn131_command_query("show net\r\n", "<", test, UART_MAXSTRLEN);
			warte_ms(2000);
			rn131_command_query("show connection\r\n", "<", test, UART_MAXSTRLEN);
			zahl = strstr(test, "8");	// "8" finden, ab dort 4 Zeichen
			zahl[4] = 0;	// nach den 4 Ziffern terminieren
			constat = strtol(zahl, NULL, 16);

		} while ((constat & 16) != 16);	// erst weitergehen, wenn WLAN-Verbindung zum AP besteht
	}
	*/

	//---------------------------------
	// <RC> Get Connection status (TCP) <S1>: Connected, <S0>: Not Connected


	// Konfiguration: uart: WLAN: Senden an MAIN und/oder AUX
	// beide: <WE0>, nur MAIN: <WE1>, nur AUX: <WE2>
	// Meldung über UDP aussenden (über TCP macht's erst bei bestehender TCP-Verbindung Sinn ;-)

/*
	log_puts_P("Warte auf TCP-Verbindung...!\r\n");
	if (cfg_wlan == WLAN_WIZNET)
	{
		//wlan_command_query("<WE2>", test);	// senden an AUX (UDP) einstellen
		//warte_ms(200);
		uint8_t wait = 1;

		do	// Warteschleife für tcp-Verbindung
		{
			uartx_puts(WLAN, iamlok);		// Meldung über AUX / (UDP) an Server und controller // im Servermode zwecklos
													// ausser er hat vom "Control" schon eine Meldung bekommen -> daher in Schleife
			wlan_command_query2("<RC>", test);		// Abfragen ob tcp-Verbindung existiert
			if (strncmp(test, "1", 1)) { wait = 0; } // warten bis Abfrage 0 ergibt (0 for match !!)
			else { warte_ms(10000); }
		} while (wait);


		// TCP-Verbindung steht jetzt!
		//wlan_command_query("<WE1>", test);	// senden an Main (TCP) einstellen
		PORT_TESTLED |= (1<<TESTLED1) | (1<<TESTLED2) | (1<<TESTLED3);	// alles Testleds ausschalten

	}
	else if (cfg_wlan == WLAN_RN131)
	{
		unsigned int constat = 0;
		char *zahl;
		do	// Warteschleife für tcp-Verbindung
		{
			warte_ms(500);
			rn131_command_query("show connection\r\n", "<", test, UART_MAXSTRLEN);
			zahl = strstr(test, "8");	// "8" finden, ab dort 4 Zeichen
			zahl[4] = 0;	// nach den 4 Ziffern terminieren
			constat = strtol(zahl, NULL, 16);

		} while ((constat & (unsigned int)15) != 1);	// erst weitergehen, wenn TCP = connected
	}

*/


	//warte_ms(500);
	// Diverse Informationen an Gegenstelle melden
	uartx_puts(WLAN, iamlok);	//Meldung 1x an Controller über tcp, wenn bereits verbunden (muss eindeutig sein!!)
	if (cfg_wlan == WLAN_RN131) { uartx_puts_P(WLAN, "#"); }	// zeichen für sofort senden

	//uartx_puts_P(WLAN, "<owneri:");
	//uartx_puts(WLAN, lokdata.owner);
	//uartx_puts_P(WLAN, ">");

	log_puts_P("TCP verbunden. Bin bereit!\r\n");

	// RN-131 UDP-Broadcast abdrehen (wird im normalen betrieb nicht benötigt)
	if (cfg_wlan == WLAN_RN131)
	{
		rn131_command_query("set broadcast interval 0\r\n", "<", test, UART_MAXSTRLEN);	// in test kommt alles vor "Chan=" zurück!!!!!!!
		// broadcast wird sofort beendet, die config aber nicht gespeichert, damit es beim nächsten Reboot wieder funktioniert!
	}

	//PORT_TESTLED &= ~(1<<TESTLED3);	// Testled3 einschalten
	//ledcontrol_setall();	// LED-Controller einstellen
	//PORT_TESTLED &= ~(1<<TESTLED2);	// Testled2 einschalten
	blink = 0;	// WLAN = verbunden -> blinken beenden
	//alone = 0; // für Notstop im Echtbetrieb wieder aktivieren!

	/*
	// i2c LED nochmal testen
	lcd_command(LCD_CLEAR);
	lcd_printlc(1,1,(unsigned char *)"Test2 i2c-LED");
	ledcontrol_led_setoutputmode(0, 1);	// LED fix ein
	lcd_printlc(2,1,(unsigned char *)"fertig");
	warte_ms(5000);
	ledcontrol_led_setoutputmode(0, 0);	// LED aus
	lcd_printlc(2,1,(unsigned char *)"LED aus");
	*/

	PORT_TESTLED |= (1<<TESTLED1);	// wieder ausschalten

//-------------------------------------------------------------------------------------------------------------
//------                       H A U P T S C H L E I F E                                               --------
//-------------------------------------------------------------------------------------------------------------


	while (1)	// Hauptschleife (endlos)
	{
		loop_count++;	// Zähler für Hauptschleife
		unsigned char error = 0;


		if (alivesecs >= lokdata.notstoptimeout)	// Prüfintervall = ALIVE_INTERVAL Sekunden! -> Lok stoppt nach ALIVE_INTERVAL sek herrenloser Fahrt
		{
			//lcd_printlc(2,9,(unsigned char *)"alivechk");
			if (alivecount == 0)	// prüfen, ob in diesem Prüfintervall Meldungen der Gegenstelle einglangt sind
			{
				//alone = 1;

				log_puts_P("Alone: stop!\r\n");

#if defined( LOKBASIS_NO_TEST )

				speed_soll = 0;
#endif

				// hier noch einbauen, dass wenn speed auf 0 ist, die tcp connection gecheckt wird und bei fehlen derselben
				// auf eine warten, udp-kontaktmeldungen abgeben und erst weiterlaufen, wenn wieder eine tcp-verbindung vorhanden ist
				// -> solange keine anderen lebeswichtigen systeme erhalten werden müssen (akku check..)
			}
				alivecount = 0;	// Zähler rücksetzen
				alivesecs = 0;
		}


		// Powermanagement: checken ob sich die Lok abschalten soll
		// prepare_switch_off


		if (prepare_switch_off)
		{
			if (speed < 100)
			{
				// Daten speichern, wenn nötig
				prepare_switch_off = 0;	// Kennung zurücksetzen, da die Schleife noch ein paarmal durchlaufen werden kann
				system_power_off();
			}
		}

		// Powermanagment: void system_power_on() / void system_power_off() Impuls beenden (countdown im timer 5 isr)
		if (relay_stopimp)
		{
			PORT_SYS &= ~((1<<SYSTEM_OFF) | (1<<SYSTEM_ON));	// beide Ausgänge auf 0 setzen -> Impuls aus (egal, welcher es war)
			TIMSK5 &= ~(1<<OCIE5A);	// disable timer5 output compare match interrupt
			relay_stopimp = 0;	// Kennung zurücksetzen
			TIMSK5 |= (1<<OCIE5A);	// enable timer5 output compare match interrupt
		}



#if defined( MOTORC_POLOLU24V20 )	//Error von MotorControllern "Pololu 24v20" checken
		//lcd_printlc(2,9,(unsigned char *)"motorpol");
		uint8_t error_alt = motorerror;
		motorerror = 0;
		if (PIN_MOTOR & (1<<MOTOR1_FF1)) { motorerror |= 1; } 		else { motorerror &= ~1; }	// bit 0
		if (PIN_MOTOR & (1<<MOTOR1_FF2)) { motorerror |= (1<<1); } 	else { motorerror &= ~2; }	// bit 1
		if (PIN_MOTOR & (1<<MOTOR2_FF1)) { motorerror |= (1<<2); } 	else { motorerror &= ~4; }	// bit 2
		if (PIN_MOTOR & (1<<MOTOR2_FF2)) { motorerror |= (1<<3); } 	else { motorerror &= ~8; }	// bit 3

#if defined( LOKBASIS_NO_TEST )
		/* für Tests deaktivieren
		if (motorerror > 0)	// bei Error: Speed auf 0 setzen -> stop (geschieht nicht sofort!!)
		{
			// beim ersten Auftreten an Controller melden
			if (error_alt == 0) { uartx_puts_P(WLAN, "<error:motor>"); }

			speed_soll = 0;

			// ACHTUNG: speed wird in funktion set_speed() langsam verringert (Aufruf nur alle 250ms)
			// daher hier gleich mal "extra" aufrufen!! (aber nur 1x)
			if (error_alt == 0) { set_speed(); }
		}
		*/
#endif
#endif


#if defined( HW_TESTBOARD1 )	// für Testboard1
		if (state & STATE_SPI_DATA)		// neue ADC-Daten vom ATTiny-Board sind vorhanden
		{
			unsigned char a = 0;
			spi_error = 0;
			//clearbit(PORTH,7);	// Signale Motor messen für ATtiny26 (Interrupt auf steigender Flanke) wieder in Ruhestellung bringen

			clearbit(SPCR,SPIE);		// SPI-Interrupt sperren, damit die Daten nicht durch neue überschreiben werden können
			for (a=0;a<16;a++)			// SPI-Daten (16 Byte) kopieren, damit die ISR nicht solange blokiert werdne muss
			{
				adcdaten_sic[a] = adcdaten[a];
			}
			setbit(SPCR,SPIE);		// SPI-Interrupt wieder aktivieren


			if (!(adcdaten_sic[0]==255)) { spi_error++; }
			for (a=1;a<8;a++)			// SPI Byte 0 bis 7 Fehlerprüfung auf korrekten Wert
			{
				if (!(adcdaten_sic[a]==a)) { spi_error++; }
			}

			log_puts_P("SPI-Daten: ");
			// nur für test

			for (a=0;a<16;a++)			// 16 Byte werden übertragen
			{
				itoa(adcdaten_sic[a], test, 10);
				log_puts(test);
				log_puts_P(" ");

			}
			if (spi_error) { log_puts_P("(fehlerhaft!)");}
			log_puts_P("\r\n");


			for (a=0;a<4;a++)			// 16 Byte werden übertragen
			{
				unsigned int data = 0;
				data = adcdaten_sic[(a*2)+8] + (adcdaten_sic[(a*2)+9] << 8);

				switch (a)
				{
					case 0:
					U_Motor1 = data;
					break;

					case 1:
					U_Motor2 = data;
					break;

					case 2:
					U_Schiene = data;
					break;

					case 3:
					U_Akku = data;
					break;
				}
			}

			cli();
			state &= ~STATE_SPI_DATA;	// state-flag zurücksetzen
			sei();
		}
#endif


		if (state & STATE_5X_PRO_SEK)
		{
			//lcd_printlc(2,9,(unsigned char *)"setspeed");
			set_speed();		// setzt die aktuelle Geschwindigkeit und checkt, ob speed-Änderungen "gesoftet" werden müssen

			cli();
			state &= ~STATE_5X_PRO_SEK;	// state-flag zurücksetzen
			sei();
		}

		if (state & STATE_1X_PRO_SEK)		// Meldung an Server bei == 1 / Steuergerät (ca. 1x pro Sekunde)
		{
			//lcd_printlc(2,9,(unsigned char *)"1/sek   ");
			PORT_TESTLED ^= (1<<TESTLED3);	// Testled 3 änder bei jedem durchlauf

			memset(test, 0, UART_MAXSTRLEN+1);	// string leeren
			strlcpy_P(test, txtp_sd, DEVICE_STRLEN+10);	// Rückmeldung der Geschwindigkeit
			itoa(speed, test+4, 10);
			//uartx_puts_P(WLAN, "<sd:");
			strlcat_P(test, txtp_cmdend, DEVICE_STRLEN+10);
			uartx_puts(WLAN, test);
			//uartx_puts_P(WLAN, ">");
			if (cfg_wlan == WLAN_RN131) { uartx_puts_P(WLAN, "#"); }	// zeichen für sofort senden

			/*
			ltoa(loop_count, test, 10);	// testweise Rückmeldung des loopcount -> später an handy
			log_puts_P("loopcount = ");
			log_puts(test);
			log_puts_P("\r\n");
			*/

// Spannungsmeldung Schiene, Akku und ev. Motoren (?) für Testboard 2 fehlt noch. auch einarbeiten (CONFIG_DO_ADC)
#if defined( HW_TESTBOARD1 )	// für Testboard1
			if(config & CONFIG_DO_SPIADC)	// Spannungen nur ausgeben, wenn entsprechende Config gesetzt ist
			{
				// Spannung an Controller melden
				if (U_Schiene < 65535)
				{
					float U_mess;
					U_mess = (U_Schiene * 0.0271);			// Umrechnung in die echte Spannung
					dtostrf(U_mess,1,3,test);				// Umwandlung in string, 3 Nachkommastellen
				}
				else
				{
					strcpy(test, "unbekannt");				// wenn U_Schiene = 65535, dann gibt es keinen gültigen Wert
				}


				uartx_puts_P(WLAN, "<uein:");	// ändern auf <us:
				uartx_puts(WLAN, test);
				uartx_puts_P(WLAN, ">");
				if (cfg_wlan == WLAN_RN131) { uartx_puts_P(WLAN, "#"); }	// zeichen für sofort senden

				if (spi_error)
				{
					log_puts_P("U_Schiene: ERROR\r\n");
				}
				else
				{
					log_puts_P("U_Schiene: ");					// testweise Ausgabe
					log_puts(test);
					log_puts_P("\r\n");
				}

				// testweise Ausgabe MotorADC-Werte
				itoa(U_Motor1, test, 10);
				log_puts_P("U_Motor1=");
				log_puts(test);
				log_puts_P("  ");

				itoa(U_Motor2, test, 10);
				log_puts_P("U_Motor2=");
				log_puts(test);
				log_puts_P("\r\n");
			}

			spi_error = 255;	// wenn nicht jede Sekunde ein neuer Wert kommt, Daten ungültig erklären!!
#endif

			// speed am Display ausgeben
			lcd_printlc(1,1,(unsigned char *)"              ");	// Anzeige löschen
			memset(test, 0, UART_MAXSTRLEN+1);	// string leeren	// "nnn nnn nnn n "
			itoa(speed, test, 10);
			lcd_printlc(1,1,(unsigned char *)test);

			memset(test, 0, UART_MAXSTRLEN+1);	// string leeren
			itoa(speed_soll, test, 10);
			lcd_printlc(1,7,(unsigned char *)test);


			last_loop_count = loop_count;	// ermittelten loopcount sichern
			loop_count = 0;	// loopcount sekündlich zurücksetzen -> ergibt loops/s
			cli();
			state &= ~STATE_1X_PRO_SEK;	// state-flag zurücksetzen
			sei();

		}

		// WLAN Commands checken un abarbeiten

		//lcd_command(LCD_CLEAR);
		//lcd_printlc(1,1,(unsigned char *)"hole wlandaten");
		//lcd_printlc(2,9,(unsigned char *)"rd wlan ");
		read_wlan(1);	// Daten von WLAN holen (1: wenn nicht gleich was da ist, wieder beenden)

		/*
		// test: check wlan_string
		lcd_command(LCD_CLEAR);
		lcd_printlc(1,1,(unsigned char *)"wlan: ");
		//lcd_printlc(1,7,(unsigned char *)tz);
		lcd_printlc(2,1,(unsigned char *)wlan_string);
		warte_ms(5000);
		*/

		if(strlen(wlan_string) > 0)	// lange Prüfung nur, wenn Befehl nicht leer
		{
			//lcd_printlc(2,9,(unsigned char *)"cmd ausw");
			befehl_auswerten();
		}

		#if defined( RFID )
		if (state & STATE_RFID_DATA)	// RFID Daten sind vorhanden
		{
			//lcd_printlc(2,9,(unsigned char *)"rfid chk");
			error = read_rfid();		// Daten vom RFID holen

			if(error < 255)			// error 255 = keine daten vorhanden, alle anderen errors ausgeben
			{
				itoa(error, test, 10);
				log_puts_P("RFID error = ");
				log_puts(test);
				log_puts_P("\r\n");
			}

			if(error == 0)			// daten sind gültig
			{
				uartx_puts_P(WLAN, "<rfid:");
				uartx_puts(WLAN, rfid_string);
				uartx_puts_P(WLAN, ">");
				if (cfg_wlan == WLAN_RN131) { uartx_puts_P(WLAN, "#"); }	// zeichen für sofort senden
			}

		}
		#endif // RFID

	}


} // main ende

//-----------------------------------------------------------------------------------------
//----------------   Interrupts   ---------------------------------------------------------
//-----------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------------
// isr für timer 5 output compare match A interrupt: 244 Hz = 4ms
ISR(TIMER5_COMPA_vect) {

	timer5_count++;

	if (uart_ov_limit > 0)	//	wlan_read Wartezeit checken
	{
		uart_ov_count++;
		if(uart_ov_count > (uart_ov_limit * 244))
		{
			uart_wait_no_more = 1;
			uart_ov_count = 0;
		}
	}

	if (timer5_count % 48 == 0)	// 5x pro Sekunde. state: STATE_5X_PRO_SEK
	{
		state |= STATE_5X_PRO_SEK;	// state setzen
	}

	if (timer5_count == 244)	// 1x pro Sekunde
	{
		state |= STATE_1X_PRO_SEK;	// state setzen
		alivesecs++;

		#if defined( HW_TESTBOARD1 )	// LEDblinken
			if (blink == 1) { PORTD ^= (1<<PD5);  }  //* XOR, Kurzschreibweise für LED-Blinken
		#elif defined( HW_TESTBOARD2 )
			if (blink == 1) { PORT_TESTLED ^= (1<<TESTLED0);  }
		#endif

		timer5_count = 0;	// es wird nur bis 30 gezählt -> 1 Sekunde, dann wieder bei 0 anfangen
	}

	// Powermanagement-Relais Impuls counter checken
	if (relaycouter > 0)
	{
		relaycouter--;
		if (relaycouter == 1)	{ relay_stopimp = 1; }	// Kennung für Haupschleife setzen, dass der Impuls jetzt beendet werden soll
	}
}

//----------------------------------------------------------------------------------------------------------
/* interrupt handler for SPI Transfer (1 byte) complete interrupt */

ISR(SPI_STC_vect)
{
#if defined( HW_TESTBOARD1 )
	clearbit(PORTH,7);	// Signale Motor messen für ATtiny26 (Interrupt auf steigender Flanke) wieder in Ruhestellung bringen
	// wird jetzt in der Haupt-Schleife bei der SPI-auswertung gemacht

	unsigned char data = 0;

	SPDR = 0; 	// (sinnloses) neues Datenbyte setzen -> geht laut Dateblatt schon vor Lesen!!
	data = SPDR;
	if (data == 255)		// spicounter korrigieren: 255 kommt nur als Start-Byte!!!!
	{
		spicounter = 0;
	}
	adcdaten[spicounter] = data;
	spicounter++;

	if (spicounter == 16)
	{
		state |= STATE_SPI_DATA;	// wenn Übertragung komplett (derzeit 16 byte) -> state setzen
		spicounter = 0;
	}
	// vom Anfang der ISR testweise hierher gestellt
	setbit(PORTL,1);	// Enable Motor A - nach Messung wieder auf HIGH
	setbit(PORTL,7);	// Enable Motor B - nach Messung wieder auf HIGH
#endif
}

//----------------------------------------------------------------------------------------------------------
// Interrupt handler für OCR4A Compare Match Interrupt (Aufgabe: alle 65ms soll Motor-Messung gestartet werden)

ISR(TIMER4_COMPA_vect)
{
#if defined( HW_TESTBOARD1 )
	if((motor_reg == 1) && (config & CONFIG_DO_SPIADC))	// wenn Motor-Regelung laufen soll und SPIADC gemacht wird (sonst wird nie wieder aufgedreht..)
	{
		clearbit(PORTL,1);	// Enable Motor A - für Messung auf LOW stellen -> Motorcontroller hochohmig schalten
		clearbit(PORTL,7);	// Enable Motor B - für Messung auf LOW stellen -> Motorcontroller hochohmig schalten
							// wird im SPI-Interrupt wieder auf HIGH gestellt, sobald SPI-Daten empfangen wurden
	}
	else
	{
		// zur Sicherheit dann aufdrehen
		setbit(PORTL,1);	// Enable Motor A - nach Messung wieder auf HIGH
		setbit(PORTL,7);	// Enable Motor B - nach Messung wieder auf HIGH
	}

	//ADC-Daten vom ATTiny26 anfordern
	//clearbit(PORTH,7);
	if (config & CONFIG_DO_SPIADC)	// nur ausführen, wenn entsprechende config gesetzt ist
	{
		setbit(PORTH,7);	// Signale Motor messen für ATtiny26 (->Interrupt auf steigender Flanke)
		// clearbit(PORTH,7);  // -> nicht hier -> kann doch auch bei Empfang der SPI-Daten gemacht werden
	}
#endif
}

