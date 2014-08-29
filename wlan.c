/*
 * wlan.c
 *
 *  Created on: 29.01.2011
 *      Author: Michael Brunnbauer
 *      Funktionen für die verschiedenen WLAN-Module
 */

#include <avr/io.h>
#include <string.h>		// für "strcmp"
#include <stdlib.h>		// für "itoa"
#include <util/delay.h>	// für delay_ms()
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "lokbasis_hwdef.h"
#include "main.h"
#include "uart.h"
#include "wlan.h"
#include "funktionen.h"


// WLAN Wiznet-Modul einschalten (über Pin WLAN_ONOFF)
// 0: ausschalten, 1: einschalten
void wlan_on(unsigned char wlan_on)
{
	if (wlan_on == 1)
	{
		PORT_WLAN |= (1<<WLAN_ONOFF);	// 1: WLAN-Modul Wiznet einschalten
	}
	else
	{
		PORT_WLAN &= ~(1<<WLAN_ONOFF);	// 0: WLAN-Modul Wiznet ausschalten
	}
}


//--------------------------------------------------------------------------------------------
// WLAN initialisieren
// wlan_typ: siehe WLAN-Definitionen in main.h

void wlan_init(unsigned char wlan_typ)
{
#if defined( HW_TESTBOARD1 )	// Testboard 1
	setbit(DDRH,3);		// WLAN F1: Reset: Port H bit 3 auf Ausgang (1) schalten
	setbit(DDRH,2);		// WLAN F2: (unterschiedlich): Port H bit 2 auf Ausgang (1) schalten

		if (wlan_typ == WLAN_WIZNET)
		{
			clearbit(PORTH,3);	// WLAN F1: Resetleitung auf 0 setzen (active HIGH). 1= reset, 1 für 3s = Factory Default
			setbit(PORTH,2);	// WLAN F2: HWTRIGGER auf 1 setzen (1: Datenmodus, 0: Command-Modus)
		}
		else if (wlan_typ == WLAN_RN131)
		{
			setbit(PORTH,3);	// WLAN F1: Resetleitung auf 1 setzen (active LOW).
			clearbit(PORTH,2);	// WLAN F2: Aufwecken auf 0 setzen (Force_Awake: min. 31us Puls)
		}

		led_aus();
		init_uart(WLAN, UART_SETTING_2);

#elif defined( HW_TESTBOARD2 )	// Testboard2 (immer WLAN=Wiznet)

		DDR_WLAN |= (1<<WLAN_RESET) | (1<<WLAN_DATAMODE)  | (1<<WLAN_ONOFF);	// als Ausgänge für Wiznet setzen (nach Reset ist alles 0 (=Input))
		PORT_WLAN |= (1<<WLAN_DATAMODE);	// HWTRIGGER (Datamode) auf 1 (Data) setzen
		PORT_WLAN &= ~(1<<WLAN_ONOFF);	// WLAN-Modul Wiznet ausgeschalten
		init_uart(WLAN, UART_SETTING_WLAN);

#endif
}


//--------------------------------------------------------------------------------------------
// WLAN Command Mode einschalten
// für: Wiznet

void serial_cmd_ein(void)
{
	#if defined( HW_TESTBOARD1 )
		clearbit(PORTH,2);
	#else
		clearbit(PORT_WLAN, WLAN_DATAMODE);
	#endif
	warte_ms(250);
}

//--------------------------------------------------------------------------------------------
// WLAN Command Mode ausschalten
// für: Wiznet

void serial_cmd_aus(void)
{
	#if defined( HW_TESTBOARD1 )
		setbit(PORTH,2);
	#else
		setbit(PORT_WLAN, WLAN_DATAMODE);
	#endif
	warte_ms(250);
}

//--------------------------------------------------------------------------------------------
// WLAN Status und Konfiguration ermitteln und aufbereiten
// für: Wiznet

void wlan_prepare_status(unsigned char type, char *text)
{
	char comtxt[UART_MAXSTRLEN+1] = "";
	char *tmptxt;
	memset(text, 0, UART_MAXSTRLEN+1);	// text leeren

	switch (type)
	{
						case 1:	// SSID
							if (cfg_wlan == WLAN_WIZNET)
							{
								wlan_command_query2("<DS>", text);
							}
							else if (cfg_wlan == WLAN_RN131)
							{
								rn131_command_query("get wlan\r\n", "Chan=", comtxt, UART_MAXSTRLEN);	// in test kommt alles vor "Chan=" zurück!!!!!!!
								tmptxt  = strstr(comtxt, "SSID=");
								if (tmptxt)	{ strlcpy(text, tmptxt+5, UART_MAXSTRLEN); }
								remcrlf(text);
							}
						break;

						case 2:	// IP
							if (cfg_wlan == WLAN_WIZNET)
							{
								wlan_command_query2("<RI>", text);
							}
							else if (cfg_wlan == WLAN_RN131)
							{
								rn131_command_query("get ip\r\n", ":", comtxt, UART_MAXSTRLEN);
								tmptxt  = strstr(comtxt, "IP=");
								if (tmptxt)	{ strlcpy(text, tmptxt+3, UART_MAXSTRLEN); }
								remcrlf(text);
							}
						break;

						case 3:	// SUBnetmask
							if (cfg_wlan == WLAN_WIZNET)
							{
								wlan_command_query2("<RS>", text);
							}
							else if (cfg_wlan == WLAN_RN131)
							{
								rn131_command_query("get ip\r\n", "GW=", comtxt, UART_MAXSTRLEN);
								tmptxt  = strstr(comtxt, "NM=");
								if (tmptxt)	{ strlcpy(text, tmptxt+3, UART_MAXSTRLEN); }
								remcrlf(text);
							}
						break;

						case 4:	// DHCP-Server enabled?
							if (cfg_wlan == WLAN_WIZNET)
							{
								wlan_command_query2("<RD>", text);
							}
							else if (cfg_wlan == WLAN_RN131)
							{
								rn131_command_query("get ip\r\n", "IP=", comtxt, UART_MAXSTRLEN);
								tmptxt  = strstr(comtxt, "DHCP=");	// hier OFF statt 0
								if (tmptxt)	{ strlcpy(text, tmptxt+5, UART_MAXSTRLEN); }
								remcrlf(text);
							}
						break;

						case 5:	// Channel
							if (cfg_wlan == WLAN_WIZNET)
							{
								wlan_command_query2("<DC>", text);
							}
							else if (cfg_wlan == WLAN_RN131)
							{
								rn131_command_query("get wlan\r\n", "ExtAnt=", comtxt, UART_MAXSTRLEN);
								tmptxt  = strstr(comtxt, "Chan=");
								if (tmptxt)	{ strlcpy(text, tmptxt+5, UART_MAXSTRLEN); }
								remcrlf(text);
							}
						break;

						case 6:	// Security Status
							if (cfg_wlan == WLAN_WIZNET)
							{
								wlan_command_query2("<DU>", text);
							}
							else if (cfg_wlan == WLAN_RN131)
							{
								rn131_command_query("get wlan\r\n", "Mask=", comtxt, UART_MAXSTRLEN);
								tmptxt  = strstr(comtxt, "Auth=");
								if (tmptxt)	{ strlcpy(text, tmptxt+5, UART_MAXSTRLEN); }
								remcrlf(text);
							}
						break;

						case 7:	// Client mode (0:client / 1:mixed / 2:server)
							if (cfg_wlan == WLAN_WIZNET)
							{
								wlan_command_query2("<RM>", text);
							}
							else if (cfg_wlan == WLAN_RN131)
							{
								rn131_command_query("get ip\r\n", "MTU=", comtxt, UART_MAXSTRLEN);
								tmptxt  = strstr(comtxt, "PROTO=");	// "TCP," kommt bei client mode mixed (tcp client+server) zurück
								if (tmptxt)	{ strlcpy(text, tmptxt+6, UART_MAXSTRLEN); }
								remcrlf(text);
							}
						break;

						case 8:	// Wireless Mode (0:AP/1:Gateway/2:AP+WDS/3:Client)
							if (cfg_wlan == WLAN_WIZNET)
							{
								wlan_command_query2("<DO>", text);
							}
							else if (cfg_wlan == WLAN_RN131)
							{
								strlcpy(text, "not implemented", UART_MAXSTRLEN);
							}
						break;

						case 9:	// active Clients
							if (cfg_wlan == WLAN_WIZNET)
							{
								wlan_command_query2("<DL>", text);
							}
							else if (cfg_wlan == WLAN_RN131)
							{
								strlcpy(text, "not implemented", UART_MAXSTRLEN);
							}
						break;

						case 10:	// Site Survey
							if (cfg_wlan == WLAN_WIZNET)
							{
								wlan_command_query2("<DI>", text);
							}
							else if (cfg_wlan == WLAN_RN131)
							{
								strlcpy(text, "not implemented", UART_MAXSTRLEN);
							}
						break;
	}
	memset(wlan_string, 0, UART_MAXSTRLEN+1);	// string leeren
}

//--------------------------------------------------------------------------------------------
// WLAN Status -Daten abfragen für wlan_prepare_status()
// für text nicht wlan_string verwenden!!!
// wartet (max 10 reads mit max 10 Sekunden Wartezeit) auf erfolgreiche Rückmeldung "<S*> und liefert "*" im text zurück
//
// für: Wiznet


// vom wlan einen vollständigen cmd "<*>" inklusive <> einlesen (alles andere ignorieren, solange warten, bis vollständig)
// und in char *text ablegen (muss mindestens UART_MAXSTRLEN+1 groß sein!!)

void wlan_read_cmd(char *text)
{
	uint8_t cmd = 0;
	uint8_t exit = 0;
	uint8_t cindex = 0;
	unsigned int c;

	memset(text, 0, UART_MAXSTRLEN+1);	// string leeren
	do
	{
		c = uartx_getc(WLAN);	//wlan lesen
		if (!( c & UART_NO_DATA))
		{
			if (cindex >= UART_MAXSTRLEN) { cindex = 0; }	// textbuffer nicht überlaufen lassen, notfalls altes wieder überschreiben

			if ((char)c == 60) { cmd = 1; }	// > Befehl beginnt, "<" mitnehmen

			if (cmd)	// wenn text innerhalb von <>, dann speichern (diesmal mit <> !!!)
			{
				text[cindex] = (char) c;
				cindex++;
				text[cindex] = (char) 0;	// zur sicherheit String immer "nullen"
			}
			if ((char)c == (char)62) { cmd = 0; exit = 1; }	// abschließendes Zeichen > wurde empfangen
		}
	} while (!exit);	// solange wiederholen, bis cmd mit > vollständig ist
}


void wlan_command_query(const char *command, char *text)
{

	char waitcount = 0;
	char wait = 1;
	memset(text, 0, UART_MAXSTRLEN+1);	// text leeren
	serial_cmd_ein();
	uartx_puts(WLAN, command);
	serial_cmd_aus();

	do
	{
		read_wlan(10);
		waitcount++;

		if (!strncmp(wlan_string, "<S", 2)) { wait = 0; }	// checken, ob erfolgreiche Rückmeldung erhalten
		if (waitcount==10)	{	wait = 0;	}	// nicht unendlich warten
		//Test
		log_puts_P("Reply: ");
		log_puts(wlan_string);
		log_puts_P("\r\n");

	} while (wait);	// auf eine erfolgreiche Rückmeldung warten



	char* startpos = strstr(wlan_string, "<S" );
	char* endpos = strchr(wlan_string, '>' );
	if ((startpos != NULL) && (endpos != NULL))		// wenn keines der beiden NULL ist
	{
		strncpy(text, startpos+2, endpos - (startpos+2));	// endpos-1 - startpos+2
	}

	// Test
	/*
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n"); */
	/*
	log_puts_P("Extracted: ");
	log_puts(text);
	log_puts_P("\r\n"); */

}

//--------------------------------------------------------------------------------------------
// andere Testvariante
// für: Wiznet

void wlan_command_query2(const char *command, char *text)
{
	char waitcount = 0;
	char wait = 1;
	memset(text, 0, UART_MAXSTRLEN+1);	// text leeren

	log_puts_P("WIZNET-Befehl: ");
	log_puts(command);
	log_puts_P("\r\n");

	do
	{
		serial_cmd_ein();
		uartx_puts(WLAN, command);
		serial_cmd_aus();
		warte_ms(250);
		read_wlan(5);	// nicht warten
		waitcount++;

		if (!strncmp(wlan_string, "<S", 2)) { wait = 0; }	// checken, ob erfolgreiche Rückmeldung erhalten
		if (waitcount==10)	{	wait = 0;	}	// nicht unendlich warten
		if (wait) {warte_ms(5000); }

		//Test
		/*
		log_puts_P("Reply: ");
		log_puts(wlan_string);
		log_puts_P("\r\n"); */

	} while (wait);	// auf eine erfolgreiche Rückmeldung warten


	char* startpos = strstr(wlan_string, "<S" );
	char* endpos = strchr(wlan_string, '>' );
	if ((startpos != NULL) && (endpos != NULL))		// wenn keines der beiden NULL ist
	{
		strncpy(text, startpos+2, endpos - (startpos+2));	// endpos-1 - startpos+2
	}
	// Test
		/* log_puts_P("Reply: ");
		log_puts(wlan_string);
		log_puts_P("\r\n"); */
		log_puts_P("Extracted: ");
		log_puts(text);
		log_puts_P("\r\n");
}


//--------------------------------------------------------------------------------------------
// liest einen string "<blabla>" vom wlan ein - hört erst nach ">" auf!! alles andere wird ignoriert!!!
// Achtung: nach wait_sek Sekunden sollte die Funktion mit Fehler abgebrochen werden
// bei wait_sek == 0 wird ewig gewartet, bis ein vollständiger "<blabla>" empfangen wurde -> wird beim timer0 interrupt gemanaged
// bei wait_sek == 1 wird nicht gewartet, wenn gar kein Zeichen vorhanden ist
// für: Wiznet+RN131

void read_wlan(uint8_t wait_sek)
{
	unsigned int c;
	unsigned char index = 0;
	unsigned char complete = 0;
	unsigned char command = 0;

	uart_wait_no_more = 0;				// kennung für lange-genug-auf-uart-gewartet zurücksetzen

	clearbit(TIMSK5,OCIE5A);	// Timer5 interrupt abdrehen
	uart_ov_limit = wait_sek;	// erlaubte Dauer der Abfrage bis Abbruch in Sekunden setzen (Umrechnung in s noch nötig!!)
	setbit(TIMSK5,OCIE5A);		// Timer5 interrupt aufdrehen

	//wlan_string[index] = (char) 0;	// wlan_string 1. Stelle leeren -> string ist leer!
	memset(wlan_string, 0, UART_MAXSTRLEN+1);	// string leeren

	while((!(complete)) && (!(uart_wait_no_more)))
	{
		         // uart_getc() returns in the lower byte the received character and
		         // in the higher byte (bitmask) the last receive error
		         // UART_NO_DATA is returned when no data is available.

		        c = uartx_getc(WLAN);

		        if ( c & UART_NO_DATA )
		        {
		             //no data available from UART
		        	// bei wait_sek = 1: wenn noch gar kein Zeichen im Puffer ist -> nicht weiter warten
		        	if ((wait_sek == 1) && (index == 0)) {  complete = 1; }
		        }
		        else
		        {
		            /*
		             * new data available from UART - check for Frame or Overrun error
		             */
		            if ( c & UART_FRAME_ERROR )
		            {
		                /* Framing Error detected, i.e no stop bit detected */
		                log_puts_P("UART Frame Error\r\n");
		            }
		            if ( c & UART_OVERRUN_ERROR )
		            {
		                 // Overrun, a character already present in the UART UDR register was
		                 // not read by the interrupt handler before the next character arrived,
		                 // one or more received characters have been dropped
		                log_puts_P("UART Overrun Error\r\n");
		            }
		            if ( c & UART_BUFFER_OVERFLOW )
		            {
		                 // We are not reading the receive buffer fast enough,
		                 // one or more received character have been dropped
		                log_puts_P("Buffer overflow error vom UART\r\n");
		            }

		            // empfangenes Zeichen verarbeiten

		            if ((char)c == 60)	// > Befehl beginnt, "<" mitnehmen
		            {
		            	if (command == 0) { command = 1; }	// alles ist "normal"
		            	else if (command == 1)	// ein Befehl war unvollständig -> alten verwerfen und mit dem neuen weitermachen
		            	{
		            		index = 0;
		            		memset(wlan_string, 0, UART_MAXSTRLEN+1);	// string leeren
		            	}
		            }

		            if (command)
		            {
		            	if (index < UART_MAXSTRLEN)
		            	{
		            		wlan_string[index] = (char) c;
		            		index++;
		            		//wlan_string[index] = (char) 0;	// zur sicherheit String immer "nullen"
		            	}
		            	else
		            	{
		            		log_puts_P("ERROR: Empfangener String war laenger als vereinbart!\r\n");
		            		complete = 1;
		            		memset(wlan_string, 0, UART_MAXSTRLEN+1);	// string leeren
		            		//if (index == UART_MAXSTRLEN) { wlan_string[index + 1] = (char) 0; }
		            	}
		            }

		            if ((char)c == (char)62)	// > abschließendes Zeichen wurde empfangen
		            {
		            	complete = 1;
		            	command = 0;
		            }
		        }
		    }
	if (uart_wait_no_more)
	{
		if (wait_sek > 3)	// nur melden, wenn eine längere Wartezeit eingestellt ist, sonst zu nervig
		{
			log_puts_P("Error: Wartezeit für uart2 ist abgelaufen!\r\n");
		}
	}
}

//--------------------------------------------------------------------------------------------
// WLAN Status ermitteln
// für: Wiznet


void wlan_status(void)
{
		log_puts_P("WLAN Status\r\n");
		uartx_puts_P(WLAN, "<QP>");
		read_wlan(30);
		log_puts_P("Reply: ");
		log_puts(wlan_string);
		log_puts_P("\r\n");
}

//--------------------------------------------------------------------------------------------
// WLAN Senden ausschalten
// für: Wiznet

void wlan_tx_off(void)
{
		log_puts_P("WLAN TX Power off\r\n");
		uartx_puts_P(WLAN, "<GP0>");
		read_wlan(30);
		log_puts_P("Reply: ");
		log_puts(wlan_string);
		log_puts_P("\r\n");
}

//--------------------------------------------------------------------------------------------
// WLAN Senden einschalten
// für: Wiznet

void wlan_tx_on(void)
{
		log_puts_P("WLAN TX Power on\r\n");
		uartx_puts_P(WLAN, "<GP3>");
		read_wlan(30);
		log_puts_P("Reply: ");
		log_puts(wlan_string);
		log_puts_P("\r\n");
}

//--------------------------------------------------------------------------------------------
// WLAN IP-Adresse setzen
// für: Wiznet

void wlan_set_ip(void)
{
		log_puts_P("WLAN: Set IP\r\n");
		uartx_puts_P(WLAN, "<WI10.0.0.121>");
		read_wlan(30);
		log_puts_P("Reply: ");
		log_puts(wlan_string);
		log_puts_P("\r\n");
}

//--------------------------------------------------------------------------------------------
// WLAN IP-Adresse auslesen
// für: Wiznet

void wlan_get_ip(void)
{
		log_puts_P("WLAN: Get IP\r\n");
		uartx_puts_P(WLAN, "<RI>");
		read_wlan(30);
		log_puts_P("Reply: ");
		log_puts(wlan_string);
		log_puts_P("\r\n");
}

//--------------------------------------------------------------------------------------------
// WLAN Reset ausführen (Wiznet läuft bei ATMega-Reset weiter!!)
// für: Wiznet

void wlanreset(void)
{

	#if defined( HW_TESTBOARD1 )
		if (cfg_wlan == WLAN_WIZNET)
			{
				setbit(PORTH,3);	// Resetleitung auf 1 setzen. 1= reset, 1 für 3s = Factory Default
				warte_ms(500);		// 0,5s lang Reset
				clearbit(PORTH,3);	// Resetleitung auf 0 setzen. 1= reset, 1 für 3s = Factory Default
			}
			else if (cfg_wlan == WLAN_RN131)
			{
				// hier fehlt noch was
			}
	#elif defined( HW_TESTBOARD2 )	// wiznet
		setbit(PORT_WLAN, WLAN_RESET);		// Resetleitung auf 1 setzen. 1= reset, 1 für 3s = Factory Default
		warte_ms(500);						// 0,5s lang Reset
		clearbit(PORT_WLAN, WLAN_RESET);	// Resetleitung auf 0 setzen.
	#endif

}

//--------------------------------------------------------------------------------------------
// WLAN Grundeinstellungen durchführen
// für: Wiznet

unsigned char setup_wlan(void)
{
	unsigned char error = 255;

	log_puts_P("WLAN: Einstellungen werden durchgeführt...\r\n");
	serial_cmd_ein();

	// WI Set IP Address <xxx.xxx.xxx.xxx>
	log_puts_P("WLAN: Set IP Address\r\n");
	uartx_puts_P(WLAN, "<WI10.0.0.1>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	// WS Set Subnet Mask <xxx.xxx.xxx.xxx>
	log_puts_P("WLAN: Set Subnet Mask\r\n");
	uartx_puts_P(WLAN, "<WS255.255.255.000>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	// WD Set DHCP Server 1:Enable, 0:Disable <x>
	log_puts_P("WLAN: Disable DHCP Server\r\n");
	uartx_puts_P(WLAN, "<WD0>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	// WV Set DNS Server 1:Enable, 0:Disable <1:xxx.xxx.xxx.xxx[_xx.xx.xx.xx]> or<0>
	log_puts_P("WLAN: Disable DNS Server\r\n");
	uartx_puts_P(WLAN, "<WV0>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	// GB Set Wireless 0: 11b+g, 2: 11b, 3:11g, 6: n, 9:b+g+n
	log_puts_P("WLAN: Set Wireless mode\r\n");
	uartx_puts_P(WLAN, "<GB0>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	// GO Set Operation Mode 0:AP, 1:Gateway, 2: AP+WDS, 3:Client <x> (dauert lange!!)
	log_puts_P("WLAN: Set Operation Mode\r\n");
	uartx_puts_P(WLAN, "<GO3>");
	read_wlan(60);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");


	// GS Set SSID 1~32 chars <xxxx~>
	log_puts_P("WLAN: Set SSID\r\n");
	uartx_puts_P(WLAN, "<GSWBROUT1>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	// GC Set Channel Auto_0, 1~13 <x>
	log_puts_P("WLAN: Set Channel\r\n");
	uartx_puts_P(WLAN, "<GC6>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	// GP Set Tx Power 0: off, 1-16: power(dBm), <xx> -> testweise auf 3 -< später mehr!!
	log_puts_P("WLAN: Set Tx Power\r\n");
	uartx_puts_P(WLAN, "<GP3>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	// GH Set Broadcast SSID 0:Enable, 1:Disable <x>
	log_puts_P("WLAN: Disable Broadcast SSID\r\n");
	uartx_puts_P(WLAN, "<GH1>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	// GU Set Security Control
	/*AuthMode_Encrypt[_KeyLength_KeyFormat_KeyValue_radiusPasswd_radiusIP_radiusPort]
	AuthMode: 0(Open or Shared), 1(Open), 2(802.1x), 3(Shared),
	4(WPA), 5(WPA-PSK), 6(WPA2), 7(WPA2-PSK),
	Encrypt: 0(None),1 (WEP), 2(TKIP), 3(AES), 4(TKIP_AES)
	KeyLength: 0(None), 1(WEP64), 2(WEP128)
	KeyFormat(WEP): 0(Ascii), 1(Hex)
	KeyFormat(WPA-PSK): 0(Passphrase), 1(Hex)
	(WPA-PSK KeyValue: 8~63byte)
	<x_x_x_x_x_x_x_x>
	(dauert lange!!)
	*/
	// Radius params können weggelassen werden: <GU5_2_0_0_12345678>
	// aber es darf kein Unterstrich in der Passphrase sein!!!!
	// oder der Unterstrich muss verdoppelt werden!! _ -> __
	log_puts_P("WLAN: Set Security Control\r\n");
	uartx_puts_P(WLAN, "<GU7_4_2_0_0_Secret-Passwd!>");
	read_wlan(50);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	//------------------ serial -----------------------------
	// WK Set Protocol TCP_0, UDP_1 <x>
	log_puts_P("WLAN: Set Protocol\r\n");
	uartx_puts_P(WLAN, "<WK0>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	// WM Set Mode 0:Client, 1:Mixed, 2:Server <x>
	log_puts_P("WLAN: Set ser. Op. Mode\r\n");
	uartx_puts_P(WLAN, "<WM2>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	/*
	// WX Set Server IP Server IP address <xxx.xxx.xxx.xxx>
	log_puts_P("WLAN: Set Server IP\r\n");
	uartx_puts_P(WLAN, "<WX010.000.000.120>");
	read_wlan(40);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");


	// WP Set Port 0~65535 <xxxxx>	im Testprogramm: Port=8000
	log_puts_P("WLAN: Set Server Port\r\n");
	uartx_puts_P(WLAN, "<WP8000>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");
*/

	// WB Set Baudrate_DataBit_Parity_Flow_Stopbits
	/*eg. [Baudrate]1: 115200, 2: 57600,
	3: 38400, 4: 19200, 5: 9600,
	6: 4800, 7: 2400,8: 1200
	[data byte] 7: 7bit, 9: 8bit	<- ????????????????????????????????? jetzt 8: 8bit
	[parity] 0: no parity, 1: Odd, 2: Even
	[Flow] 0: no, 1: Xon/Xoff, 2: RTS/CTS
	[Stopbits]; 1: 1stop, 2:2stop
	<xxxxx>
	*/
	log_puts_P("WLAN: serial Settings\r\n");
	uartx_puts_P(WLAN, "<WB3_8_0_0_1>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	// OT Set Time 0~65535 <xxxxx>	<- deaktivieren
	log_puts_P("WLAN: sending delimiter time: off\r\n");
	uartx_puts_P(WLAN, "<OT0>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	// OS Set Size 0~255 <Sxxx>	<- deaktivieren
	log_puts_P("WLAN: sending delimiter size: off\r\n");
	uartx_puts_P(WLAN, "<OS0>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	// OC Set Char 00~ff <xx>	<- 00 als Sende-kenn-zeichen
	log_puts_P("WLAN: sending delimiter character: 00 on\r\n");
	uartx_puts_P(WLAN, "<OC00>");
	read_wlan(30);
	log_puts_P("Reply: ");
	log_puts(wlan_string);
	log_puts_P("\r\n");

	error = 0;	// für ersten Test
	serial_cmd_aus();

	// returnwert: Fehlercode: 0: alles erfolgreich
	return error;
}

//--------------------------------------------------------------------------------------------
// WLAN Kommando an WLAN absetzen und auf Antwort prüfen
// command: der abzusetzende Befehlstext, reply: die zu erwartende Antwort, text: hier sollen eventuelle daten hineingeschreiben werden
// textlen: max. länge des textes, die geschrieben werden darf! (also freier Platz -1 wg. abschließendem 0-Byte)
// ist reply leer (Länge=0), so wird nicht auf eine Antwort gewartet
// für: RN131
// Achtung: sollte noch umgebaut werden, "ERR" im Fehlerfall wird ignoriert, es wird nur auf die positive Antwort gewartet
// -> reply_ok, replay_fail sollte übergeben werden

void rn131_command_query(const char *command, const char *reply, char *text, int textlen)
{
	unsigned int c;
	unsigned char cindex = 0;
	unsigned char rindex = 0;
	char fertig = 0;								// status für suche nach antwort
	char interntxt[UART_MAXSTRLEN+1] = "";			// für string-bearbeitungen
	char *replyintext;									// zum speichern der reply-position im text (reply soll entfernt werden)

	memset(text, 0, textlen+1);	// text leeren
	uartx_puts_P(WLAN, "$$$");
	// warten auf "CMD"

	do
	{
		c = uartx_getc(WLAN);	//wlan lesen
		if (c < UART_NO_DATA)	// wenn kein Fehler aufgetreten ist
		{
			if (cindex <= UART_MAXSTRLEN)
			{
				interntxt[cindex] = (char) c;
				log_puts(interntxt+cindex);	// für Test an usb schicken
				cindex++;
			}
		}
	} while (!(strcasestr(interntxt, "CMD")));	// solange wiederholen, bis "CMD" im text enthalten ist

	uartx_puts(WLAN, command);	// den eigentlichen Befehl absetzen

	cindex = 0;

	do
	{
		c = uartx_getc(WLAN);	//wlan lesen
		if (c < UART_NO_DATA)
		{
			if (cindex <= textlen)
			{
				text[cindex] = (char) c;
				log_puts(text+cindex);	// für Test an usb schicken
				cindex++;
			}
			else { fertig = 1; log_puts_P("Abbruch!\r\n");}	// Abbruch, wenn text bereits voll ist
		}
		replyintext = strcasestr(text, reply);
		if (replyintext)
		{
			fertig = 1;
			log_puts_P("Antwort gefunden!\r\n");

			while(replyintext[rindex] != 0)	// text text ab reply löschen (bis 0-byte)
			{
				replyintext[rindex] = 0;	// replyintext++ = 0; müsste auch gehen -> rindex wird dann garnicht gebraucht
				rindex++;
			}

		}	// reply wurde gefunden

	} while (!fertig);		// Achtung: wenn reply leer ist, wird nicht gewartet




	uartx_puts_P(WLAN, "exit\r\n");					// wieder aus Befehlsmodus aussteigen
	memset(interntxt, 0, UART_MAXSTRLEN+1);		// interntxt leeren
	cindex = 0;
	log_puts_P("exit wurde gesendet\r\n");

	do
	{
		c = uartx_getc(WLAN);	//wlan lesen
		if (c < UART_NO_DATA)
		{
			if (cindex <= UART_MAXSTRLEN-10)	// reserve lassen, da sonst mst mitkommt -> checken, woher!!!!?!
			{
				interntxt[cindex] = (char) c;
				log_puts(interntxt+cindex);	// für Test an usb schicken
				cindex++;
			}

			else	// interntxt von vorne beginnen, , cindex korrekt setzen, wenn der anfang vom EXIT vohrer dabei war-> problem
			{
				memset(interntxt, 0, UART_MAXSTRLEN+1);		// interntxt leeren
				cindex = 0;
				interntxt[cindex] = (char) c;
				log_puts(interntxt+cindex);	// für Test an usb schicken
			}
		}
	} while (!(strcasestr(interntxt, "EXIT")));	// solange wiederholen, bis "EXIT" im text enthalten ist

	log_puts_P("EXIT gefunden!\r\n");		// Test

}

void rn131_init(char *text, int textlen)	// text ist für ausgaben
{
	unsigned int c;
	unsigned char cindex = 0;
	char interntxt[UART_MAXSTRLEN+1] = "";			// für string-bearbeitungen

	memset(text, 0, textlen+1);	// text leeren
	log_puts_P("\r\nRN-131 Konfig startet\r\n");
	uartx_puts_P(WLAN, "$$$");
	// warten auf "CMD"

	do
	{
		c = uartx_getc(WLAN);	//wlan lesen
		if (c < UART_NO_DATA)	// wenn kein Fehler aufgetreten ist
		{
			if (cindex <= UART_MAXSTRLEN)
			{
				interntxt[cindex] = (char) c;
				log_puts(interntxt+cindex);	// für Test an usb schicken
				cindex++;
			}
		}
	} while (!(strcasestr(interntxt, "CMD")));	// solange wiederholen, bis "CMD" im text enthalten ist

	// die eigentlichen Befehlw absetzen

	uartx_puts_P(WLAN, "set ip dchp 1\r\n");
	log_puts_P(".");
	warte_ms(2000);
	uartx_puts_P(WLAN, "set ip localport 8000\r\n");
	log_puts_P(".");
	warte_ms(2000);
	uartx_puts_P(WLAN, "set ip protocol 2\r\n");
	log_puts_P(".");
	warte_ms(2000);
	uartx_puts_P(WLAN, "set ip remote 8000\r\n");
	log_puts_P(".");
	warte_ms(2000);
	uartx_puts_P(WLAN, "set uart baud 38400\r\n");
	log_puts_P(".");
	warte_ms(2000);
	uartx_puts_P(WLAN, "set uart flow 0\r\n");
	log_puts_P(".");
	warte_ms(2000);
	uartx_puts_P(WLAN, "set wlan auth 4\r\n");
	log_puts_P(".");
	warte_ms(2000);
	uartx_puts_P(WLAN, "set wlan channel 6\r\n");
	log_puts_P(".");
	warte_ms(2000);
	uartx_puts_P(WLAN, "set wlan ext_antenna 0\r\n");
	log_puts_P(".");
	warte_ms(2000);
	uartx_puts_P(WLAN, "set wlan join 1\r\n");
	log_puts_P(".");
	warte_ms(2000);
	uartx_puts_P(WLAN, "set wlan phrase Secret-Passwd!\r\n");
	log_puts_P(".");
	warte_ms(2000);
	uartx_puts_P(WLAN, "set wlan rate 8\r\n");
	log_puts_P(".");
	warte_ms(2000);
	uartx_puts_P(WLAN, "set wlan ssid WBROUT1\r\n");
	log_puts_P(".");
	warte_ms(2000);
	uartx_puts_P(WLAN, "save\r\n");
	log_puts_P("\r\ngespeichert\r\n");
	warte_ms(2000);
	uartx_puts_P(WLAN, "reboot\r\n");
	warte_ms(10000);



}


void rn131console()
{
	unsigned int cin;
		unsigned int cout;
		unsigned char t;
		unsigned char x = 0;
		log_puts_P("RN-131 Test-Konsole: \r\n");
		do
		{
			cin = log_getc();
			cout = uartx_getc(WLAN);


			if (cin < UART_NO_DATA)	// wenn kein Fehler aufgetreten ist
			{
				x =  (unsigned char) cin;
				uartx_putc(WLAN, x);
			}

			if (cout < UART_NO_DATA)	// wenn kein Fehler aufgetreten ist
			{
				t =  (unsigned char) cout;
				log_putc(t);
			}
		} while (x != 46);
}


