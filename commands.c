/*
 * commands.c
 *
 *  Created on: 08.12.2010
 *      Author: Michael Brunnbauer
 */

#include <avr/io.h>
#include <string.h>		// für "strcmp"
#include <stdlib.h>		// für "itoa"
#include <util/delay.h>	// für delay_ms()
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "lokbasis_hwdef.h"		// Hardware-Definitionen für die verschiedenen Boards
#include "i2clcd.h"				// I2C LCD Funktionen
#include "main.h"
#include "commands.h"
#include "uart.h"
#include "speed.h"
#include "wlan.h"
#include "funktionen.h"	// Funktionen für Hauptschleife und commands.c


//-----------------------------------------------------------------------------------------
// befehl_auswerten: wlan_string wird ausgewertet und entsprechende Befehle umgesetzt
//-----------------------------------------------------------------------------------------
void befehl_auswerten(void)
{
	char test[UART_MAXSTRLEN+1];
	memset(test, 0, UART_MAXSTRLEN+1);	// text leeren


	if(!strcmp(wlan_string, "<stop>"))
	{
		cli();
		speed_soll = 0;
		sei();
	}

	else if(!strcmp(wlan_string, "<off>"))
	{
			cli();
			speed_soll = 0;
			sei();
			system_power_off();
	}

	else if(!strcmp(wlan_string, "<stopall>"))
		{
				cli();
				speed_soll = 0;
				sei();
				system_power_off();
		}

	else if(!strncmp(wlan_string, "<richtung:", 10))
	{
		log_puts_P("Richtungseinstellung wird durchgeführt!\r\n");
		strncpy(test, wlan_string+10, 2);
		test[2] = (char) 0;

		if(!strcmp(test, "vw"))		// vorwärts
		{
			cli();
			richtung_soll = RICHTUNG_VW;
			sei();
		}
		else if(!strcmp(test, "rw"))
		{
			cli();
			richtung_soll = RICHTUNG_RW;
			sei();
		}
	}

	else if(!strncmp(wlan_string, "<sd:", 4))	// Command für speed setzen
	{

		char * ende =  strchr(wlan_string, (int)'>');
		int size = ende - (wlan_string+4);
		strncpy(test, wlan_string+4, size);		// die beliebig lange Zahl rauskopiern

		//strncpy(test, wlan_string+4, 3);		// die dreistellige Zahl rauskopiern
		//test[3] = (char) 0;					// nullen ist nicht notwendig

		cli();
		speed_soll = atoi(test);
		sei();

#if defined( LOKBASIS_NO_TEST )
		if (motorerror != 0)	// Geschwindigkeit nur annehmen, wenn kein Problem beim Motorcontroller vorliegt
		{
			cli();
			speed_soll = 0;
			sei();
		}
#endif

		log_puts_P("Neuer speed = ");
		log_puts(test);
		log_puts_P("\r\n");

		/*
		memset(test, 0, UART_MAXSTRLEN+1);	// string leeren
		itoa(speed_soll, test, 10);
		lcd_printlc(2,1,(unsigned char *)"SD: ");			// Test
		lcd_printlc(2,4,(unsigned char *)test);
		*/

	}
	/*
	else if(!strcmp(wlan_string, "<speedtest>"))	// Command speedtest
	{
		speedtest();
	}
	 */

	else if(!strncmp(wlan_string, "<l1:", 4))	// Licht ein "Lichtname" (vorerst LED-Nummer vom Controller)
	{
		lcd_printlc(1,1,(unsigned char *)"ein: Licht Nr:");			// Test
		uint8_t lednummer = 0;
		strncpy(test, wlan_string+4, strlen(wlan_string+4)-1);		// die Zahl rauskopiern test sollte leer sein
		lcd_printlc(2,1,(unsigned char *)test);						// Test
		lednummer = atoi(test);
		ledcontrol_led_setoutputmode(lednummer, 2);	// LED ein, Helligkeit lt. PWM-Helligkitsregister (127 -> halb hell)
		lcd_printlc(1,1,(unsigned char *)"ein: fertig!");			// Test
	}

	else if(!strncmp(wlan_string, "<l0:", 4))	// Licht aus "Lichtname" (vorerst LED-Nummer vom Controller)
	{
		lcd_printlc(1,1,(unsigned char *)"aus: Licht Nr:");			// Test
		uint8_t lednummer = 0;
		strncpy(test, wlan_string+4, strlen(wlan_string+4)-1);		// die Zahl rauskopiern test sollte leer sein
		lcd_printlc(2,1,(unsigned char *)test);						// Test
		lednummer = atoi(test);
		ledcontrol_led_setoutputmode(lednummer, 0);
	}

	else if(!strncmp(wlan_string, "<lb1:", 4))	// Licht Blinken "Lichtname" (vorerst LED-Nummer vom Controller)
	{
		lcd_printlc(1,1,(unsigned char *)"Blink: Licht Nr:");			// Test
		uint8_t lednummer = 0;
		strncpy(test, wlan_string+5, strlen(wlan_string+5)-1);		// die Zahl rauskopiern test sollte leer sein
		lcd_printlc(2,1,(unsigned char *)test);						// Test
		lednummer = atoi(test);
		ledcontrol_led_setoutputmode(lednummer, 3);	// Led ein, sollte blinken
		lcd_printlc(1,1,(unsigned char *)"Blink: fertig!");			// Test
	}

	else if(!strncmp(wlan_string, "<lb0:", 4))	// Licht Blinken aus "Lichtname" (vorerst LED-Nummer vom Controller)
	{
		lcd_printlc(1,1,(unsigned char *)"Baus: Licht Nr:");			// Test
		uint8_t lednummer = 0;
		strncpy(test, wlan_string+5, strlen(wlan_string+5)-1);		// die Zahl rauskopiern test sollte leer sein
		lcd_printlc(2,1,(unsigned char *)test);						// Test
		lednummer = atoi(test);
		ledcontrol_led_setoutputmode(lednummer, 0);	// Led aus
	}

	else if(!strcmp(wlan_string, "<lr>"))	// Lichtchip reset und init (alle LEDs ausgeschalten)
	{
		ledcontrol_reset();
		//ledcontrol_init();	// testweise deaktiviert

	}

	else if(!strcmp(wlan_string, "<la1>"))	// Lichtchip reset und init (alle LEDs ausgeschalten)
	{
		ledcontrol_allonoff(1);

	}

	else if(!strcmp(wlan_string, "<la0>"))	// Lichtchip reset und init (alle LEDs ausgeschalten)
	{
		ledcontrol_allonoff(0);

	}

	else if(!strncmp(wlan_string, "<ia:", 4))
	{
		alivecount++;
	}
	else if(!strncmp(wlan_string, "<iamcontrol:", 12))	// UDP-Meldung -> auf UDP antworten!!
	{

		alivecount++;
		if (cfg_wlan == WLAN_WIZNET)
		{
			//serial_cmd_ein();
			//uartx_puts_P(WLAN, "<WE2>");	// sende an AUX
			//serial_cmd_aus();
			//log_puts_P("Rueckmeldung kommt!\r\n");
			uartx_puts(WLAN, iamlok);	// rückmelden!!
			//serial_cmd_ein();
			//uartx_puts_P(WLAN, "<WE1>");	// zurückstellen auf: sende an Main
			//serial_cmd_aus();
		}
	}

	else if(!strcmp(wlan_string, "<ping>"))	// mit "<pong>" antworten
	{
		uartx_puts_P(WLAN, "<pong>");
		if (cfg_wlan == WLAN_RN131) { uartx_puts_P(WLAN, "#"); }	// zeichen für sofort senden
	}

	else if(!strncmp(wlan_string, "<fpwm:", 6))	// PWM-Frequenz setzen
	{
		char pwmf = 0;
		strncpy(test, wlan_string+6, 1);		// die einstellige Zahl rauskopiern
		test[1] = (char) 0;	// string mit 0-Byte abschließen
		pwmf = atoi(test);
		log_puts_P("Neue PWM-Frequenz # = ");
		log_puts(test);
		log_puts_P("\r\n");
		init_pwm(pwmf);		// neuen PWM-Modus setzen
	}

	else if(!strncmp(wlan_string, "<mr:", 4))	// Motorregelung an / aus -> derzeit
	{
		if(!strncmp(wlan_string+4, "0", 1))		// Motorregelung deaktivieren
		{
			motor_reg = 0;
			log_puts_P("Motorregelung aus!\r\n");
		}
		else if(!strncmp(wlan_string+4, "1", 1))		// Motorregelung aktivieren
		{
			motor_reg = 1;
			log_puts_P("Motorregelung ein!\r\n");
		}
	}

#if defined( HW_TESTBOARD1 )
	else if(!strncmp(wlan_string, "<tspi:", 6))	// SPI (Interrupt) an / aus
	{
		if(!strncmp(wlan_string+6, "0", 1))		// SPI deaktivieren
		{
			spi_aus();
			log_puts_P("SPI aus!\r\n");
		}
		else if(!strncmp(wlan_string+6, "1", 1))		// SPI aktivieren
		{
			spi_ein();
			log_puts_P("SPI ein!\r\n");
		}
	}
#endif

	else if(!strncmp(wlan_string, "<iwlan:", 7))	// Status & Konfigurationsdaten des WLAN-Moduls werden angefordert
	{												// gleich hier zerlegen und einzeln senden, wg. RAM
		// Achtung: Kat2-Befehl!! Speed muss 0 sein (alt+neu)!!! -> abprüfen -> noch einbauen
		unsigned char statustype;
		char statusstring[UART_MAXSTRLEN+1] = "";
		char statusnrstring[4] = "";		// zum Aufbereiten der Statusnummer als 2stelliger String (zur sicherheit + 3. Stelle reserviert
		unsigned char i;

		strncpy(test, wlan_string+7, 2);		// die 2stellige Zahl rauskopiern, bei strlcpy muss man aber +1 angeben, da nur size-1 Zeichen kopiert werden
		test[2]= 0;
		statustype = atoi(test);
		log_puts_P("iwlan: statustyp = ");	// Test
		log_puts(test);
		log_puts_P("\r\n");

		//log_puts_P("Befehl: iwlan\r\n");	// Test
		uartx_puts_P(WLAN, "<swlan:start>");	// Start
		if (cfg_wlan == WLAN_RN131) { uartx_puts_P(WLAN, "#"); }	// zeichen für sofort senden

		if (statustype != 0)
		{
			log_puts_P("iwlan: für 1 Typ\r\n");	// Test
			wlan_prepare_status(statustype, test);	// Text für den statustype in test ablegen
			strlcpy(statusstring, "<swlan:", UART_MAXSTRLEN);
			itoa(statustype, statusnrstring, 10);
			if (statustype < 10)	// bei 1 bis 9 eine führende '0' einfügen!!
			{
				statusnrstring[2] = 0;
				statusnrstring[1] = statusnrstring[0];
				statusnrstring[0] = '0';
			}
			strlcat(statusstring, statusnrstring, UART_MAXSTRLEN);
			strlcat(statusstring, ":", UART_MAXSTRLEN);
			strlcat(statusstring, test, UART_MAXSTRLEN);
			strlcat(statusstring, ">", UART_MAXSTRLEN);
			if (cfg_wlan == WLAN_RN131) { strlcat(statusstring, "#", UART_MAXSTRLEN); }	// zeichen für sofort senden
			uartx_puts(WLAN, statusstring);

		}
		else
		{
			log_puts_P("iwlan: für 00\r\n");	// Test

			for (i = 1; i < 10; i++)		// alle Daten ausgeben, derzeit nur 01-09
			{
				wlan_prepare_status(i, test);	// Text für den statustype in test ablegen
				strlcpy(statusstring, "<swlan:", UART_MAXSTRLEN);
				itoa(i, statusnrstring, 10);
				if (i < 10)	// bei 1 bis 9 eine führende '0' einfügen!!
				{
					statusnrstring[2] = 0;
					statusnrstring[1] = statusnrstring[0];
					statusnrstring[0] = '0';
				}
				strlcat(statusstring, statusnrstring, UART_MAXSTRLEN);
				strlcat(statusstring, ":", UART_MAXSTRLEN);
				strlcat(statusstring, test, UART_MAXSTRLEN);
				strlcat(statusstring, ">", UART_MAXSTRLEN);
				if (cfg_wlan == WLAN_RN131) { strlcat(statusstring, "#", UART_MAXSTRLEN); }	// zeichen für sofort senden
				uartx_puts(WLAN, statusstring);

				log_puts_P("Befehl wurde gesendet: ");
				log_puts(statusstring);
				log_puts_P("\r\n");
			}
		}
		uartx_puts_P(WLAN, "<swlan:end>");	// Ende
		if (cfg_wlan == WLAN_RN131) { uartx_puts_P(WLAN, "#"); }	// zeichen für sofort senden
	}

	else if(!strcmp(wlan_string, "<ogp>"))	// vorhandene optionale Deviceproperties melden
	{
		int i;
		for (i = 0; i < DEVICE_OPTPROP_COUNT; i++)
		{
			struct opData op;
			memcpy_P((void*)&op, (PGM_VOID_P)&optprops[i], sizeof(op));
			uartx_puts_P(WLAN, "<have:");
			uartx_puts(WLAN, op.name);	// funktioniert das so?
			uartx_puts_P(WLAN, ">");
		}
	}

	else if(!strcmp(wlan_string, "<ownerg>"))	// Besitzer des Devices rückmelden
	{
		uartx_puts_P(WLAN, "<owneri:");
		uartx_puts(WLAN, lokdata.owner);
		uartx_puts_P(WLAN, ">");
	}

	else if(!strcmp(wlan_string, "<swg>"))	// Loksoftware Name rückmelden
	{
		uartx_puts_P(WLAN, "<swi:");
		//uartx_puts_P(WLAN, dev_swname);
		uartx_puts_P(WLAN, ">");
	}

	else if(!strcmp(wlan_string, "<swvg>"))	// Loksoftware Version rückmelden
	{
		uartx_puts_P(WLAN, "<swvi:");
		//uartx_puts_P(WLAN, dev_swversion);
		uartx_puts_P(WLAN, ">");
	}

	else if(!strcmp(wlan_string, "<typg>"))	// Lok-Typ rückmelden
	{
		uartx_puts_P(WLAN, "<typi:");
		uartx_puts(WLAN, lokdata.modelname);
		uartx_puts_P(WLAN, ">");
	}

	else if(!strcmp(wlan_string, "<typg>"))	// Name des verwendeten WLAN-Moduls rückmelden
	{
		uartx_puts_P(WLAN, "<wnameg:");
		//uartx_puts_P(WLAN, wlanModules[cfg_wlan]);
		uartx_puts_P(WLAN, ">");
	}

	else if(!strcmp(wlan_string, "<M2>"))	// WLAN Setup
	{
		serial_cmd_ein();
		log_puts_P("Starte WLAN-Konfiguration\r\n");
		setup_wlan();
		log_puts_P("WLAN-Konfiguration beendet!\r\n");
		serial_cmd_aus();
	}
	else if(!strncmp(wlan_string, "<M0:", 4))		//Command für WLAN senden
	{
		log_puts_P("Befehl an WLAN-Modul senden\r\n");


		if (cfg_wlan == WLAN_WIZNET)
		{
			strlcpy(test, "<", UART_MAXSTRLEN);
			strncpy(test, wlan_string+4, strlen(wlan_string+4)-1);
			strlcat(test, ">", UART_MAXSTRLEN);
			serial_cmd_ein();
			uartx_puts(WLAN, test);
			serial_cmd_aus();


		}
		else if (cfg_wlan == WLAN_RN131)
		{
			strncpy(test, wlan_string+4, strlen(wlan_string+4)-1);
			uartx_puts(WLAN, "$$$");	//Umschalten in command-Mode

			do
			{
				read_wlan(30);	// *** ACHTUNG: funkt mit read_wlan() nicht!!!
				log_puts(wlan_string);	// alles ausgeben, was empfangen wird
				log_puts_P("\r\n");
			} while (strncmp(wlan_string, "CMD", 3));	// warten auf die richtige antwort

			uartx_puts(WLAN, test);
			read_wlan(60);
			log_puts_P("Antwort: ");
			log_puts(wlan_string);	// alles ausgeben, was empfangen wird
			log_puts_P("\r\n");
		}

	}

	else if(!strcmp(wlan_string, "<M1>"))	// Get Data flow (aux/main)
	{
		serial_cmd_ein();
		log_puts_P("WLAN: checking Data flow mode\r\n");
		uartx_puts_P(WLAN, "<RE>");
		read_wlan(40);
		log_puts_P("Reply: ");
		log_puts(wlan_string);
		log_puts_P("\r\n");
		serial_cmd_aus();
	}
	else if(!strcmp(wlan_string, "<MF>"))	// WLAN reset to factory defaults
	{
		serial_cmd_ein();
		log_puts_P("WLAN: setting WLAN to Factory defaults\r\n");
		uartx_puts_P(WLAN, "<WF>");
		read_wlan(40);
		log_puts_P("Reply: ");
		log_puts(wlan_string);
		log_puts_P("\r\n");
		serial_cmd_aus();
	}

	else if(!strcmp(wlan_string, "<M3>"))	// Led ein
	{
		log_puts_P("Led ein\r\n");
		led_ein();
	}
	else if(!strcmp(wlan_string, "<M4>"))	// Led aus
	{
		log_puts_P("Led aus\r\n");
		led_aus();
	}
	else if(!strcmp(wlan_string, "<M5>"))	// WLAN Firmware Version ausgeben
	{

		serial_cmd_ein();
		log_puts_P("WLAN Firmware Version: \r\n");
		uartx_puts_P(WLAN, "<RF>");
		read_wlan(30);
		log_puts_P("Reply: ");
		log_puts(wlan_string);
		log_puts_P("\r\n");
		serial_cmd_aus();

	}

	else if(!strcmp(wlan_string, "<M6>"))	// WLAN Status ausgeben
	{
		serial_cmd_ein();
		wlan_status();
		serial_cmd_aus();
	}

	else if(!strcmp(wlan_string, "<M7>"))	// WLAN: RN131 + Wiznet: Command-Test-Modus (Terminal-Funktion)
	{
		if (cfg_wlan == WLAN_RN131)
		{
			unsigned int c;
			unsigned char cindex = 0;
			char exit = 0;
			char test2[2] = "";
			memset(test, 0, UART_MAXSTRLEN+1);	// string leeren

			warte_ms(500);
			log_puts_P("RN131 wird auf Command-Mode geschaltet!\r\n");
			uartx_puts_P(WLAN, "$$$");

			while (!exit)
			{
				do
				{
					c = uartx_getc(WLAN);	//wlan lesen
					if (!( c & UART_NO_DATA))
					{
						if (cindex >= UART_MAXSTRLEN) { cindex = 0; }	// textbuffer nicht überlaufen lassen, altes wieder überschreiben
						// Achtung: "EXIT" könnte getrennt werden!!
						test2[0] = (char) c;
						test[cindex] = test2[0];
						log_puts(test2);	// letztes Zeichen an usb schicken
						cindex++;
					}
				} while (!( c & UART_NO_DATA));	// solange wiederholen, bis keine daten mehr da sind

				do
				{
					c = log_getc();	// usb lesen
					if (!( c & UART_NO_DATA))
					{
						test2[0] = (char) c;
						uartx_puts(WLAN, test2);	// letztes Zeichen an wlan schicken
					}
				} while (!( c & UART_NO_DATA));	// solange wiederholgen, bis keine daten mehr da sind

				if (strstr(test, "EXIT")) { exit = 1; }		// wenn "EXIT" im text vom wlan vorhanden ist -> modus beenden
			}
			log_puts_P("\r\nRN131 Command-Mode wird verlassen!\r\n");
		}

		else if (cfg_wlan == WLAN_WIZNET)	// Version für Wiznet für Test
		{
			char exit = 0;
			uartx_puts_P(WLAN, "<log:Konsolen-Modus aktiv>");

			while (!exit)
			{
				wlan_read_cmd(test);

				if (strlen(test) > 2) // cmd im commandmode an wiznet senden
				{
					if (strncmp(test, "<exit>", 6)) { exit = 1; }
					else
					{
						serial_cmd_ein();
						uartx_puts(WLAN, test);	// cmd ans Wiznet senden
					}
				}

				if (!exit)	// antwort nur abfragen, wenn kein exit gekommen ist
				{
					wlan_read_cmd(test);	// Als Antwort kommt ebenfalls ein "<xyz>"
					serial_cmd_aus();
					uartx_puts(WLAN, test);	// Antwort an Gegenstelle senden
				}

			}
			uartx_puts_P(WLAN, "\r\nKonsolen-Modus wird verlassen!\r\n");
		}
	}

	else
	{
		// Bearbeite ungültiges Kommando
		log_puts_P("Error: ungültiger Befehl: ");
		log_puts(wlan_string);
		log_puts_P("\r\n");
	}

}
