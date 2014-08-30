/*
 * funktionen.c
 *
 *  Created on: 08.01.2011
 *      Author: Michael Brunnbauer
 *
 *      diverse Funktionen - besonders für Hardware
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
#include "funktionen.h"
#include "i2cmaster.h"			// I2C Funktionen


#if defined( HW_TESTBOARD1 )	// MC-Board Hardware für Testboard1
//--------------------------------------------------------------------------------------------
// SPI aktivieren
//

void spi_ein(void)
{
	//setbit(SPCR,SPIE);		// SPI-Interrupt aktivieren
	cli();
	config |= CONFIG_DO_SPIADC;	// config-Flag setzen
	sei();
}

//--------------------------------------------------------------------------------------------
// SPI dektivieren
//

void spi_aus(void)
{
	//clearbit(SPCR,SPIE);			// SPI-Interrupt sperren, damit die Daten nicht durch neue überschreiben werden können
	cli();
	config &= ~CONFIG_DO_SPIADC;	// config-flag zurücksetzen (deaktivieren)
	sei();
	U_Motor1 = 65535;				// Wert jeweils auf ungültig/nicht gemessen setzen!!
	U_Motor2 = 65535;
	U_Schiene = 65535;
	U_Akku = 65535;
}
#endif

/* -----------------------------------------------------------------------------------------------------------*/
/*! Eine schnelle MEM->SPI Blocksende Routine mit optimierungen auf Speed.
 * \param  buffer    Zeiger auf den Puffer der gesendet werden soll.
 * \param  Datalenght  Anzahl der Bytes die gesedet werden soll.
 */
/* -----------------------------------------------------------------------------------------------------------*/
void SPI_FastMem2Write(unsigned char * buffer, unsigned int Datalenght)
{
  unsigned int Counter = 0;
  unsigned char data;

  // erten Wert senden
  SPDR = buffer[ Counter++ ];
  while( Counter < Datalenght )
  {
    // Wert schon mal in Register holen, schneller da der Wert jetzt in einem Register steht und nicht mehr aus dem RAM geholt werden muss
    // nachdem das Senden des vorherigen Wertes fertig ist
    data = buffer[ Counter ];
    // warten auf fertig
    while(!(SPSR & (1<<SPIF)));
    // Wert aus Register senden
    SPDR = data;
    Counter++;
  }
  while(!(SPSR & (1<<SPIF)));
  return;
}

/* -----------------------------------------------------------------------------------------------------------*/
/*! Eine schnelle SPI->MEM Blockempfangroutine mit optimierungen auf Speed.
 * \warning Auf einigen Controller laufen die Optimierungen nicht richtig. Bitte teil des Sourcecode der dies verursacht ist auskommentiert.
 * \param  buffer    Zeiger auf den Puffer wohin die Daten geschrieben werden sollen.
 * \param  Datalenght  Anzahl der Bytes die empfangen werden sollen.
 */
/* -----------------------------------------------------------------------------------------------------------*/
void SPI_FastRead2Mem(unsigned char * buffer, unsigned int Datalenght)
{
  unsigned int Counter = 0;
  unsigned char data;

  // dummywrite
  SPDR = 0x00;

  while( Counter < Datalenght )
  {
    // warten auf fertig
    while(!(SPSR & (1<<SPIF)));

    // einfache Optimierung
    // Daten einlesen in Register
    data = SPDR;
    // dummy-write
    SPDR = 0x00;

/*    // bessere Optimierung, aber nicht auf jeden controller
    // dummy-write
    SPDR = 0x00;
    // Daten einlesen in Register
    data = SPDR;
*/
    // Register speichern
    buffer[ Counter++ ] = data;
  }
  while(!(SPSR & (1<<SPIF)));
  return;
}



//-----------------------------------------------------------------------------------
/*
void read_usb(void)
{
	unsigned int c;
	unsigned char index = 0;
	unsigned char complete = 0;

	//char test[20];
	//usb_string[index] = (char) 0;
	memset(usb_string, 0, UART_MAXSTRLEN+1);	// string leeren

	while(complete == 0)
	{
		         // uart_getc() returns in the lower byte the received character and
		         // in the higher byte (bitmask) the last receive error
		         // UART_NO_DATA is returned when no data is available.

		        c = log_getc();



		        log_putc(c);	// echo
		        if ( c & UART_NO_DATA )
		        {
		            //no data available from UART
		        	// log_puts_P("keine Daten vom WLAN\r\n");
		        }
		        else
		        {

		            // new data available from UART - check for Frame or Overrun error

		            if ( c & UART_FRAME_ERROR )
		            {
		                // Framing Error detected, i.e no stop bit detected
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
		                log_puts_P("Buffer overflow error\r\n");
		            }

		            // empfangenes Zeichen verarbeiten
		            usb_string[index] = (char)c;
		            index++;

		            if (index > UART_MAXSTRLEN) { log_puts_P("ERROR: Empfangener String war laenger als vereinbart!\r\n"); }
		            if ((char)c == (char)62)	// > abschließendes Zeichen wurde empfangen - war: (unsigned char)
		            {
		            	complete = 1;
		            	usb_string[index] = (char)0;
		            }
		        }
		    }
}
*/

//-----------------------------------------------------------------------------------
// von RFID-Board an UART1 lesen. Wenn kein Zeichen im Buffer, sofort wieder beenden

#if defined( RFID )
unsigned char read_rfid(void)
{
	unsigned int c=0;
	unsigned char b=0;
	unsigned char index = 0;
	unsigned char complete = 0;
	unsigned char checksum = 0;
	unsigned char check = 0;
	unsigned char counter = 0;
	unsigned char error = 0;		// unbekannter error bzw. keine Daten vorhanden

	rfid_string[index] = (char)0;
	char checksum_string[3] = "";


	while(complete == 0)
	{
		         // uart_getc() returns in the lower byte the received character and
		         // in the higher byte (bitmask) the last receive error
		         // UART_NO_DATA is returned when no data is available.

		        c = uartx_getc(RFID);
		        cli();
		        state &= ~STATE_RFID_DATA;	// state-flag zurücksetzen
		        sei();

		        //log_putc(c);	// echo
		        if ( c & UART_NO_DATA )
		        {
		            //no data available from UART
		        	// log_puts_P("keine Daten vom WLAN\r\n");
		        	// wenn noch gar kein Zeichen im Puffer ist -> nicht weiter warten
		        	if (counter == 0)
		        	{
		        		complete = 1;
		        		error = 255;
		        	}
		        }
		        else
		        {
		             // new data available from UART - check for Frame or Overrun error
		            if ( c & UART_FRAME_ERROR )
		            {
		                /* Framing Error detected, i.e no stop bit detected */
		                log_puts_P("UART Frame Error\r\n");
		                error = 1;
		            }
		            if ( c & UART_OVERRUN_ERROR )
		            {
		                 // Overrun, a character already present in the UART UDR register was
		                 // not read by the interrupt handler before the next character arrived,
		                 // one or more received characters have been dropped
		                log_puts_P("UART Overrun Error\r\n");
		                error = 2;
		            }
		            if ( c & UART_BUFFER_OVERFLOW )
		            {
		                 // We are not reading the receive buffer fast enough,
		                 // one or more received character have been dropped
		                log_puts_P("Buffer overflow error\r\n");
		                error = 3;
		            }

		            // empfangenes Zeichen verarbeiten

		            if (counter == 0)	// RFID-Startzeichen
		            {
		            	if ((char)c == (char)2)
		            	{
		            		counter++;
		            		//log_puts_P("Startzeichen für RFID-String empfangen\r\n");
		            	}
		            	else
		            	{
		            		// log_puts_P("ERROR: Falsches RFID-Startzeichen!\r\n");
		            	}
		            }

		            else if ((0 < counter) && (counter < 11))	// RFID-Code-Daten
		            {
		            	rfid_string[index] = (char) c;
		            	counter++;
		            	index++;
		            }

		            else if (counter == 11)		// RFID-Checksum (HI)byte 1
		            {
		            	checksum_string[0] = (char) c;
		            	rfid_string[index] = (char) 0;	// abschließende 0 in den rfid_string schreiben
		            	counter++;
		            }

		            else if (counter == 12)		// RFID-Checksum (LO)byte 2
		            {
		            	//checksum = (char)c;
		            	checksum_string[1] = (char) c;
		            	counter++;
		            	checksum = (unsigned char) strtol(checksum_string, NULL, 16);

		            	// Checksum zur Kontrolle aus rfid_string errechnen
		            	char testcheck[3] = "";
		            	unsigned char teil = 0;
		            	check = 0;

		            	for (b=0;b<5;b++)		// Checksumme errechnen
		            	{
		            		strlcpy(testcheck, rfid_string+(b*2), 2);
		            		testcheck[2] = (char)0;
		            		teil =(unsigned char) (unsigned char) strtol(testcheck, NULL, 16);
		            		check = check ^ teil;
		            	}

		            	if (!(check == checksum))
		            	{
		            		log_puts_P("ERROR: Vergleich der Checksumme fehlgeschlagen!\r\n");
		            		error = 4;
		            	}
		            	//else { log_puts_P("RFID-Checksumme ist OK!\r\n"); }

		            }

		            else if (counter == 13)		// RFID-Endzeichen
		            {
		            	if ((char)c == (char)3)
		            	{
		            		//log_puts_P("Endzeichen für RFID-String empfangen\r\n");
		            	}
		            	else
		            	{
		            		log_puts_P("ERROR: erwartetes RFID-Endzeichen nicht erhalten!\r\n");
		            		error = 5;
		            	}
		            	complete = 1;	//keine weiteren Zeichen holen!
		            }

		            /*
		            	//--- Test Ausgabe --------------
		            	char testchar[3] = "";

		            	log_puts_P("Zeichen bei counter ");
		            	itoa(counter, testchar, 10);
		            	log_puts(testchar);

		            	log_puts_P(" = ");
		            	testchar[0] = (char)0;
		            	testchar[1] = (char)0;
		            	testchar[2] = (char)0;
		            	itoa((char)c, testchar, 10);
		            	log_puts(testchar);
		            	log_puts_P(" = ");
		            	log_putc((char)c);

		            	log_puts_P("\r\n");
		            	//--------------------------------

						*/
		        }
		    }
	return error;
}
#endif // RFID

//-----------------------------------------------------------------------------------

void init_uart(uint8_t uartnr, const unsigned int uartsetting)
{
	switch (uartnr)
	{
	case 0:
		UART_TxHead = 0;
		UART_TxTail = 0;
		UART_RxHead = 0;
		UART_RxTail = 0;

		/* Baudrate einstellen */
		UBRR0H = (unsigned char) (uartsetting >> 8);	// H muss vor L geschrieben werden!!
		UBRR0L = (unsigned char) (uartsetting);

		//UCSRB = (1<<RXEN) | (1<<TXEN);
		setbit(UCSR0B,RXEN0);	// empfangen einschalten
		setbit(UCSR0B,TXEN0);	// senden  einschalten
		setbit(UCSR0B,RXCIE0);	// RX Interrupt einschalten
		clearbit(UCSR0B,UCSZ02);// UCSZ02 -> 0 für 8bit

		/* Asynchron, 8N1 */
		//UCSRC = (1<<URSEL) | (3<<UCSZ0);
		setbit(UCSR0C,UMSEL01);		// URSEL
		clearbit(UCSR0C,UMSEL00);	// asynchrone Übertragung
		setbit(UCSR0C,UCSZ00);		// 8bit
		setbit(UCSR0C,UCSZ01);
		clearbit(UCSR0C,UPM00);		// no parity
		clearbit(UCSR0C,UPM01);
		clearbit(UCSR0C,USBS0);		// 1 stopbit
		break;

	case 1:
		UART1_TxHead = 0;
		UART1_TxTail = 0;
		UART1_RxHead = 0;
		UART1_RxTail = 0;

		/* Baudrate einstellen */
		UBRR1H = (unsigned char) (uartsetting >> 8);	// H muss vor L geschrieben werden!!
		UBRR1L = (unsigned char) (uartsetting);

		//UCSRB = (1<<RXEN) | (1<<TXEN);
		setbit(UCSR1B,RXEN1);	// empfangen einschalten
		setbit(UCSR1B,TXEN1);	// senden  einschalten
		setbit(UCSR1B,RXCIE1);	// RX Interrupt einschalten
		clearbit(UCSR1B,UCSZ12);// UCSZ12 -> 0 für 8bit

		/* Asynchron, 8N1 */
		//UCSRC = (1<<URSEL) | (3<<UCSZ0);
		setbit(UCSR1C,UMSEL11);		// URSEL
		clearbit(UCSR1C,UMSEL10);	// asynchrone Übertragung
		setbit(UCSR1C,UCSZ10);		// 8bit
		setbit(UCSR1C,UCSZ11);
		clearbit(UCSR1C,UPM10);		// no parity
		clearbit(UCSR1C,UPM11);
		clearbit(UCSR1C,USBS1);		// 1 stopbit
		break;

	#if defined( ATMEGA_USART23 )
	case 2:
		UART2_TxHead = 0;
		UART2_TxTail = 0;
		UART2_RxHead = 0;
		UART2_RxTail = 0;

		/* Baudrate einstellen */
		UBRR2H = (unsigned char) (uartsetting >> 8);	// H muss vor L geschrieben werden!!
		UBRR2L = (unsigned char) (uartsetting);

		/* Empfänger und Sender einschalten */
		//UCSRB = (1<<RXEN) | (1<<TXEN);
		setbit(UCSR2B,RXEN2);	// empfangen einschalten
		setbit(UCSR2B,TXEN2);	// senden einschalten
		setbit(UCSR2B,RXCIE2);	// RX Interrupt einschalten
		clearbit(UCSR2B,UCSZ22);// UCSZ22 -> 0 für 8bit

		/* Asynchron, 8N1 */
		//UCSRC = (1<<URSEL) | (3<<UCSZ0);
		setbit(UCSR2C,UMSEL21);		// URSEL
		clearbit(UCSR2C,UMSEL20);	// asynchrone Übertragung
		setbit(UCSR2C,UCSZ20);		// 8bit
		setbit(UCSR2C,UCSZ21);
		clearbit(UCSR2C,UPM20);		// no parity
		clearbit(UCSR2C,UPM21);
		clearbit(UCSR2C,USBS2);		// 1 stopbit
		break;

	case 3:
		UART3_TxHead = 0;
		UART3_TxTail = 0;
		UART3_RxHead = 0;
		UART3_RxTail = 0;

		/* Baudrate einstellen */
		UBRR3H = (unsigned char) (uartsetting >> 8);	// H muss vor L geschrieben werden!!
		UBRR3L = (unsigned char) (uartsetting);

		/* Empfänger und Sender einschalten */
		//UCSRB = (1<<RXEN) | (1<<TXEN);
		setbit(UCSR3B,RXEN3);	// empfangen einschalten
		setbit(UCSR3B,TXEN3);	// senden einschalten
		setbit(UCSR3B,RXCIE3);	// RX Interrupt einschalten
		clearbit(UCSR3B,UCSZ32);// UCSZ32 -> 0 für 8bit

		/* Asynchron, 8N1 */
		//UCSRC = (1<<URSEL) | (3<<UCSZ0);
		setbit(UCSR3C,UMSEL31);		// URSEL
		clearbit(UCSR3C,UMSEL30);	// asynchrone Übertragung
		setbit(UCSR3C,UCSZ30);		// 8bit
		setbit(UCSR3C,UCSZ31);
		clearbit(UCSR3C,UPM30);		// no parity
		clearbit(UCSR3C,UPM31);
		clearbit(UCSR3C,USBS3);		// 1 stopbit
		break;
		#endif // ATMEGA_USART23

	default:
		break;
	}
}


/* setup_uartx wurde ersetzt durch init_uart(uint8_t uartnr)
//-----------------------------------------------------------------------------------
void setup_uart0(void)
{
    UART_TxHead = 0;
    UART_TxTail = 0;
    UART_RxHead = 0;
    UART_RxTail = 0;

	// Baudrate einstellen
	UBRR0H = (unsigned char) (UART_SETTING_1 >> 8);	// H muss vor L geschrieben werden!!
	UBRR0L = (unsigned char) (UART_SETTING_1);

	// nur Empfänger einschalten, Sender nicht!! von RFID-Board wird nur gelesen
	//UCSRB = (1<<RXEN) | (1<<TXEN);
	setbit(UCSR0B,RXEN0);	// empfangen einschalten
	clearbit(UCSR0B,TXEN0);	// senden ausschalten
	setbit(UCSR0B,RXCIE0);	// RX Interrupt einschalten
	clearbit(UCSR0B,UCSZ02);// UCSZ12 -> 0 für 8bit

	// Asynchron, 8N1
	//UCSRC = (1<<URSEL) | (3<<UCSZ0);
	setbit(UCSR0C,UMSEL01);		// URSEL
	clearbit(UCSR0C,UMSEL00);	// asynchrone Übertragung
	setbit(UCSR0C,UCSZ00);		// 8bit
	setbit(UCSR0C,UCSZ01);
	clearbit(UCSR0C,UPM00);		// no parity
	clearbit(UCSR0C,UPM01);
	clearbit(UCSR0C,USBS0);		// 1 stopbit
}

//-----------------------------------------------------------------------------------
void setup_uart1(void)
{
    UART1_TxHead = 0;
    UART1_TxTail = 0;
    UART1_RxHead = 0;
    UART1_RxTail = 0;

	// Baudrate einstellen
	UBRR1H = (unsigned char) (UART_SETTING_1 >> 8);	// H muss vor L geschrieben werden!!
	UBRR1L = (unsigned char) (UART_SETTING_1);

	// nur Empfänger einschalten, Sender nicht!! von RFID-Board wird nur gelesen
	//UCSRB = (1<<RXEN) | (1<<TXEN);
	setbit(UCSR1B,RXEN1);	// empfangen einschalten
	clearbit(UCSR1B,TXEN1);	// senden ausschalten
	setbit(UCSR1B,RXCIE1);	// RX Interrupt einschalten
	clearbit(UCSR1B,UCSZ12);// UCSZ12 -> 0 für 8bit

	// Asynchron, 8N1
	//UCSRC = (1<<URSEL) | (3<<UCSZ0);
	setbit(UCSR1C,UMSEL11);		// URSEL
	clearbit(UCSR1C,UMSEL10);	// asynchrone Übertragung
	setbit(UCSR1C,UCSZ10);		// 8bit
	setbit(UCSR1C,UCSZ11);
	clearbit(UCSR1C,UPM10);		// no parity
	clearbit(UCSR1C,UPM11);
	clearbit(UCSR1C,USBS1);		// 1 stopbit
}

void setup_uart2(void)
{
#if defined( ATMEGA_USART23 )

    UART2_TxHead = 0;
    UART2_TxTail = 0;
    UART2_RxHead = 0;
    UART2_RxTail = 0;

	// Baudrate einstellen
	UBRR2H = (unsigned char) (UART_SETTING_2 >> 8);	// H muss vor L geschrieben werden!!
	UBRR2L = (unsigned char) (UART_SETTING_2);

	// Empfänger und Sender einschalten
	//UCSRB = (1<<RXEN) | (1<<TXEN);
	setbit(UCSR2B,RXEN2);	// empfangen einschalten
	setbit(UCSR2B,TXEN2);	// senden einschalten
	setbit(UCSR2B,RXCIE2);	// RX Interrupt einschalten
	clearbit(UCSR2B,UCSZ22);// UCSZ22 -> 0 für 8bit

	// Asynchron, 8N1
	//UCSRC = (1<<URSEL) | (3<<UCSZ0);
	setbit(UCSR2C,UMSEL21);		// URSEL
	clearbit(UCSR2C,UMSEL20);	// asynchrone Übertragung
	setbit(UCSR2C,UCSZ20);		// 8bit
	setbit(UCSR2C,UCSZ21);
	clearbit(UCSR2C,UPM20);		// no parity
	clearbit(UCSR2C,UPM21);
	clearbit(UCSR2C,USBS2);		// 1 stopbit

#endif
}

void setup_uart3(void)
{
#if defined( ATMEGA_USART23 )
    UART3_TxHead = 0;
    UART3_TxTail = 0;
    UART3_RxHead = 0;
    UART3_RxTail = 0;

	// Baudrate einstellen
	UBRR3H = (unsigned char) (UART_SETTING_3 >> 8);	// H muss vor L geschrieben werden!!
	UBRR3L = (unsigned char) (UART_SETTING_3);

	// Empfänger und Sender einschalten
	//UCSRB = (1<<RXEN) | (1<<TXEN);
	setbit(UCSR3B,RXEN3);
	setbit(UCSR3B,TXEN3);
	setbit(UCSR3B,RXCIE3);	// RX Interrupt einschalten
	clearbit(UCSR3B,UCSZ32);// UCSZ32 -> 0 für 8bit

	// Asynchron, 8N1
	//UCSRC = (1<<URSEL) | (3<<UCSZ0);
	setbit(UCSR3C,UMSEL31);		// URSEL
	clearbit(UCSR3C,UMSEL30);	// asynchrone Übertragung
	setbit(UCSR3C,UCSZ30);		// 8bit
	setbit(UCSR3C,UCSZ31);
	clearbit(UCSR3C,UPM30);		// no parity
	clearbit(UCSR3C,UPM31);
	clearbit(UCSR3C,USBS3);		// 1 stopbit

#endif
}
*/

//-----------------------------------------------------------------------------------

void led_ein(void)
{
	#if defined( HW_TESTBOARD1 )
		clearbit(PORTD,5);
	#elif defined( HW_TESTBOARD2 )
		clearbit(PORT_TESTLED,TESTLED0);	// verwendet nur die 1. LED
	#endif
}

void led_aus(void)
{
	#if defined( HW_TESTBOARD1 )
		setbit(PORTD,5);
	#elif defined( HW_TESTBOARD2 )
		setbit(PORT_TESTLED,TESTLED0);		// verwendet nur die 1. LED
	#endif
}

//-----------------------------------------------------------------------------------
// Motorsteuerung initialisieren

void init_motorctrl(void)
{
#if defined( HW_TESTBOARD1 )	// für Testboard1
	// Ausgänge für Motorsteuerung -------------------------------------------------------------------------
		setbit(DDRL,0);		// Port L bit 0 auf Ausgang (1) schalten
		setbit(DDRL,1);		// Port L bit 1 auf Ausgang (1) schalten
		setbit(DDRL,6);		// Port L bit 6 auf Ausgang (1) schalten
		setbit(DDRL,7);		// Port L bit 7 auf Ausgang (1) schalten

		clearbit(PORTL,0);	// IN1A - Richtung für Motor A. Standard: vorwärts auf 0
		clearbit(PORTL,6);	// IN1B - Richtung für Motor B. Standard: vorwärts auf 0
		setbit(PORTL,1);	// Enable Motor A - immer auf HIGH
		setbit(PORTL,7);	// Enable Motor B - immer auf HIGH

		// Timer 1 für Motor-PWM initialisieren --------------------------------------
		// OCR1A (16bit) Output Compare Register - Duty Cycle bzw. speed setzen (MAX = 1023)
		// speed = 0;  wird bereits bei der Variablendefinition gesetzt!
		OCR1A = 0;
		init_pwm(8);	// Mode 8: 15656Hz 9bit prescaler 1  - Timer 1 für Motor-PWM initialisieren
		//-------------------------------------------------------------------------------------------------------
#elif defined( HW_TESTBOARD2 )	// Testboard2

		DDR_MOTOR_OUT |= (1<<MOTOR_DIR) | (1<<MOTOR1_PWM) | (1<<MOTOR2_PWM) | (1<<MOTOR_RESET ) ;	// als Ausgänge setzen (nach Reset ist alles 0 (=Input))
		//PORT_MOTOR |= (1<<MOTOR_DIR);	// Richtung auf 1 setzen // je nach Motorcontroller
		PORT_MOTOR &= ~(1<<MOTOR_DIR);   // Motor-Richtung auf 0 // bei diesem Controller: vorwärts
		PORT_MOTOR |= (1<<MOTOR_RESET);   // Motor-Reset (ctive low) auf 1, sonst schläft der Motorcontroller

		DDR_MOTOR_IN &= ~((1<<MOTOR1_FF1) | (1<<MOTOR1_FF2)  | (1<<MOTOR2_FF1)  | (1<<MOTOR2_FF2));	// Fehlerpins als Eingänge setzen

		// Timer 1 für Motor-PWM initialisieren --------------------------------------
		OCR1A = 0;
		OCR1B = 0;
		init_pwm(8);	// Mode 8: 15656Hz 9bit prescaler 1  - Timer 1 für Motor-PWM initialisieren

#endif
}



void init_pwm(char freq_pwm)	// Timer 1 für Motor-PWM initialisieren
{

	speedstep_korrektur = 0;	// gibt an, ob speed *2 oder *4 genommen werden muss (8/9bit), globale varaible für speed-stellung
	//clearbit(TIMSK1,OCIE1A);	// Compare Match Interrupt abdrehen (wird für Motor-Messung verwendet) - war für motor-mess-test

	if ((freq_pwm < 1) | (freq_pwm > 9)) { freq_pwm = 1; }

	/*
	Frequenztabelle (Phase correct PWM):
	1: 122Hz		8bit:256, 10bit:64
	2: 245Hz		9bit:64
	3: 490Hz		8bit:64
	4: 977Hz		10bit:8
	5: 1957Hz		9bit:8
	6: 3921Hz		8bit:8
	7: 7820Hz		10bit:1
	8: 15656Hz		9bit:1
	9: 31372Hz		8bit:1

    Phase correct PWM: f-pwm = 16MHz / (2 * prescaler * TOP )	TOP: 10bit=1023, 9bit=511, 8bit=255
	*/

	setbit(DDRB,5);				// DDR_OC1A (=DDR_PB5) Port pin für PWM-Ausgabe auf Output setzen
	setbit(DDRB,6);				// DDR_OC1B (=DDR_PB6) Port pin für PWM-Ausgabe 2. Motor auf Output setzen
	//clearbit(PRR0,PRTIM1);	// Power Reduction für Timer 1 deaktivieren
	setbit(TCCR1A,COM1A1);		// Compare Output mode for channel A = non-inverted
	clearbit(TCCR1A,COM1A0);	// Compare Output mode for channel A = non-inverted
	setbit(TCCR1A,COM1B1);		// Compare Output mode for channel B = non-inverted
	clearbit(TCCR1A,COM1B0);	// Compare Output mode for channel B = non-inverted
	clearbit(TCCR1B,WGM13);		// Phase correct PWM, waveform generation bits - die oberen 2 sind für alle gleich!
	clearbit(TCCR1B,WGM12);

	switch (freq_pwm)
	{
		case 1:	// 122Hz 8bit:256, 10bit:64*
			setbit(TCCR1A,WGM11);
			setbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 64
			setbit(TCCR1B,CS11);		// prescaler = 64
			setbit(TCCR1B,CS10);		// prescaler = 64
		break;

		case 2:	// 245Hz 9bit:64
			setbit(TCCR1A,WGM11);
			clearbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 64
			setbit(TCCR1B,CS11);		// prescaler = 64
			setbit(TCCR1B,CS10);		// prescaler = 64
			speedstep_korrektur = 1;
		break;

		case 3:	// 490Hz 8bit:64
			clearbit(TCCR1A,WGM11);
			setbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 64
			setbit(TCCR1B,CS11);		// prescaler = 64
			setbit(TCCR1B,CS10);		// prescaler = 64
			speedstep_korrektur = 2;
		break;

		case 4:	// 977Hz 10bit:8
			setbit(TCCR1A,WGM11);
			setbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 8
			setbit(TCCR1B,CS11);		// prescaler = 8
			clearbit(TCCR1B,CS10);		// prescaler = 8
		break;

		case 5:	// 1957Hz 9bit:8
			setbit(TCCR1A,WGM11);
			clearbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 8
			setbit(TCCR1B,CS11);		// prescaler = 8
			clearbit(TCCR1B,CS10);		// prescaler = 8
			speedstep_korrektur = 1;
		break;

		case 6:	// 3921Hz 8bit:8
			clearbit(TCCR1A,WGM11);
			setbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 8
			setbit(TCCR1B,CS11);		// prescaler = 8
			clearbit(TCCR1B,CS10);		// prescaler = 8
			speedstep_korrektur = 2;
		break;

		case 7:	// 7820Hz 10bit:1
			setbit(TCCR1A,WGM11);
			setbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 1
			clearbit(TCCR1B,CS11);		// prescaler = 1
			setbit(TCCR1B,CS10);		// prescaler = 1
		break;

		case 8:	// 15656Hz 9bit:1
			setbit(TCCR1A,WGM11);
			clearbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 1
			clearbit(TCCR1B,CS11);		// prescaler = 1
			setbit(TCCR1B,CS10);		// prescaler = 1
			speedstep_korrektur = 1;
		break;

		case 9:	// 31372Hz 8bit:1
			clearbit(TCCR1A,WGM11);
			setbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 1
			clearbit(TCCR1B,CS11);		// prescaler = 1
			setbit(TCCR1B,CS10);		// prescaler = 1
			speedstep_korrektur = 2;
		break;

		default:	// 122Hz 8bit:256, 10bit:64*
			setbit(TCCR1A,WGM11);
			setbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 64
			setbit(TCCR1B,CS11);		// prescaler = 64
			setbit(TCCR1B,CS10);		// prescaler = 64
		break;
	}
	// speed muss für 8bit/9bit pwm um 1 bzw. 2 bit geshiftet werden, damit der speedwert passt (also halbiert bzw. geviertelt) -> in set_speed()

	// Compare Match Interrupt aufdrehen (für Motor Messung)
	// setbit(TIMSK1,OCIE1A); war für motor-mess-test
}

//---------------------------------------------------------
// Warte-Funktion (Delay)
// Parameter: Wartezeit in 0...65535 ms
// Stimmt für alle Optimierungslevel + evt. Interruptausfuehrungszeiten
// mit for-Schleife da Fkt. delay_ms nicht für grosse Wartezeit geeignet
//---------------------------------------------------------
void warte_ms(unsigned int wartezeit)
{
	unsigned int i;
	for (i=0;i<wartezeit;i++)
		//wiederhole diese schleife so oft wie Wartezeit angibt
	{
		_delay_ms(1);  //warte 1ms; Einbinden von delay.h noetig
	}
}

//--------------------------------------------------------------
// ersetzt CR und LF in einem String durch 0-bytes -> stringende
// (nicht besonders intelligent, aber einfach)
// Verbesserungsmöglichkeit: CR, LF aus dem string entfernen, alles andere vorher und nacher im string lassen
//--------------------------------------------------------------

void remcrlf(char *crlftext)
{
	int txtindex = 0;

	while(crlftext[txtindex] != 0)	// text text ab reply löschen (bis 0-byte)
	{
		if (crlftext[txtindex] == 10) 	{ crlftext[txtindex] = 0; }	// LF entfernen
		if (crlftext[txtindex] == 13) 	{ crlftext[txtindex] = 0; }	// CR entfernen
		txtindex++;
	}
}


//--------------------------------------------------------------------------------------------
// Logging (uart3 am Testboard1): Mapping funktionen für uartX_puts_P, uartX_puts (X = 0-3)
//


// uart3_getc() -> log_getc
unsigned int log_getc(void)
{
	int a = UART_NO_DATA;	// als leer vorbelegen

	#if defined( LOG )
		a = uart3_getc();
	#else
		// derzeit keine Funktion
	#endif

		return a;
}

// uart3_putc() -> log_putc
void log_putc(unsigned char data)
{
	#if defined( LOG )
		uartx_puts(LOG, s);
	#else
		// derzeit keine Funktion
	#endif
}


//uart3_puts() ->  log_puts()

void log_puts(const char *s )
{

	#if defined( LOG )
		uartx_puts(LOG, s);
	#else // keine uart als log-Ausgabe definiert
		// noch keine Funktion -> später als Daten über Wlan senden: "<log:text>"
	#endif
}

//uart3_puts_P() -> log_puts_P()

// ACHTUNG: PROGMEM Macro hier ein Problem - würde derzeit erst in uart-funktion im progmem abgelegt
void log_puts_P(const char *progmem_s )
{
	#if defined( HW_TESTBOARD1 )
		uart3_puts_P(*progmem_s);
	#elif defined( HW_TESTBOARD2 )
		// derzeit keine Funktion
	#endif
}

// I2C, LED-Controller-Funktionen

//TLC59116 Software Reset
void ledcontrol_reset()
{
	i2c_start(I2C_RESET);			//TLC auf Reset-Adresse ansprechen??????? // (reservierte Adresse, die für keinen Slave am Bus verwendet werden darf)
	i2c_write(0xA5);				// jetzt 2 Datenbytes A5, 5A, daran erkennt der TLC, dass er gemeint ist und resettet sich nach dem i2c_stop()
	i2c_write(0x5A);
	i2c_stop();										//TWI anhalten
}

//TLC59116 Initialisieren
void ledcontrol_init()
{
	uint8_t i;
	uint8_t count = 0;
	unsigned char error;

	do	// erfolgreichen Start versuchen
	{
		if (count > 0) { warte_ms(50); }		// kleine Pause, wenn Salve nicht bereit
		count++;
		error = i2c_start(LEDC1+I2C_WRITE);		// return 0 = device accessible, 1= failed to access device
	} while (error && (count < 3));				// nochmal versuchen, wenn Slave nicht bereit, aber nur 3 Versuche


	if(!error)			//Start: TLC auf globaler Adresse im Write-Modus ansprechen
	{
		i2c_write(0x80);					//Controllregister ansprechen (0x80 = autoincrement for all registers, bei 00 beginnend)
		i2c_write(0x01);					//Mode1 auf Normal setzen (bei ELV als Mode0) (subaddresses deaktivieren, all-call-address aktivieren)
		i2c_write(LEDC_MODE2_BLINK);		//Mode2 auf Default setzen (0x00: group controll= dimming, 0x10: group controll= blinking)
		for(i=0;i<16;i++)
		{
			i2c_write(0);					//alle PWM Kanäle auf 0 (Helligkeit)
		}
		i2c_write(255);						//PWM Helligkeit fuer Gruppen auf maximum
		i2c_write(11);						//Gruppen-Blinkfrequenzregister auf 11
		for(i=0;i<4;i++)
		{
			i2c_write(0);					//Std: 0:Outputs ausschalten
		}
		i2c_stop();							//TWI Stoppen	// IREF Regidster muss nicht gesetzt werden (Defaults: alles auf 1 passt)
	}
}

void ledcontrol_setall()			//individuelle Helligkeit, Gruppenhelligkeit, Blinkfrequenz, Outputs, Modus "Blinken oder Gruppenhelligkeit"
{
	uint8_t i,j;
	uint8_t count = 0;
	unsigned char error;

	do	// erfolgreichen Start versuchen
	{
		if (count > 0) { warte_ms(50); }	// kleine Pause, wenn Salve nicht bereit
		count++;
		error = i2c_start(LEDC1+I2C_WRITE);	// LED-Controller 1 ansprechen: return 0 = device accessible, 1= failed to access device
	} while (error && (count < 3));		// nochmal versuchen, wenn Slave nicht bereit, aber nur 3 Versuche

	if(!error)
	{
		i2c_write(0x81);				//Controllregister setzen: beim 2. Register anfangen (Mode2), autoincrement for all
		i2c_write(LED1.groupmode_blink);				// LEDC_MODE2_BLINK oder LEDC_MODE2_DIMM
		for(i=0;i<16;i++)
		{
			//if (LED1.value[i] > LED1.max_value[i]) { LED1.value[i] = LED1.max_value[i]; }
			if (!(LED1.exist & (1<<i))) { LED1.value[i] = 0; }	// nicht existente LEDs immer auf 0 setzen
			i2c_write(LED1.value[i]);					//alle PWM Kanäle (Helligkeit) setzen
		}
		i2c_write(LED1.group_value);			// Gruppenhelligkeit
		i2c_write(LED1.blinkfreq);
		for(i=0;i<4;i++)				// 4 LEDS sind in einem LEDOUT-Register zusammengefaßt -> Wert muss erst zusammengerechnet werden
		{
			uint8_t output = 0;
			for(j=0;j<4;j++)
			{
				output |= (LED1.outputmode[j+(4*i)]<<(j*2));
			}
			i2c_write(output);				//Outputs setzen
		}
		i2c_stop();	// fertig
	}
}

// setzt den outputmode pro LED in der struct (berücksichtigt auch LED.exist) und gibt die Werte an den Chip aus
// Gruppen für on/off und Blinken/Leuchten (bit-Nummer = LED-Nummer)
void led_set_outputmode(unsigned int group_on, unsigned int group_blink)
{
	uint8_t i;

	for(i=0;i<16;i++)	// für jede LED den output port mode berechnen
	{
		if (!(LED1.exist & (1<<i))) { LED1.outputmode[i] = 0; }	// 0: LED aus, wenn sie nicht existiert
		else
		{
			if (group_on & (1<<i)) { LED1.outputmode[i] = 2; }	// 2: ein, Helligkeit über PWM
			else { LED1.outputmode[i] = 0; }

			if (group_blink & (1<<i)) { LED1.outputmode[i] = 3; }	// 3: ein, Helligkeit über PWM + blinken (globale Helligkeit wird nicht verwendet)
			else { LED1.outputmode[i] = 0; }
		}
	}
	ledcontrol_setoutput();	// Änderung ausgeben
}

// die LEDOUT-Register des Controllers anhand der LED struct Daten setzen
void ledcontrol_setoutput()	// Achtung: .exists wird hier noch nicht gecheckt
{
	uint8_t i,j;
	uint8_t count = 0;
	unsigned char error;
	//outputmode: 0: aus, 1: immer ein, 2: helligkeit per pwm, 3: helligkeit per pwm + helligkeit/blink per gruppen/blink register

	do	// erfolgreichen Start versuchen
	{
		if (count > 0) { warte_ms(50); }	// kleine Pause, wenn Slave nicht bereit
		count++;
		error = i2c_start(LEDC1+I2C_WRITE);	// return 0 = device accessible, 1= failed to access device
	} while (error && (count < 3));		// nochmal versuchen, wenn Salve nicht bereit, aber nur 3 Versuche


	if(!error)
	{
		i2c_write(0x94);				//Controllregister setzen: beim LEDOUT0 anfangen, autoincrement for all

		for(i=0;i<4;i++)
		{
			uint8_t output = 0;
			for(j=0;j<4;j++)
			{
				//output |= outputmode[0] | (outputmode[1]<<2) | (outputmode[2]<<4)  | (outputmode[3]<<6);
				output |= (LED1.outputmode[j+(4*i)]<<(j*2));
			}
			i2c_write(output);				//Outputs setzen
		}
		i2c_stop();	// fertig
	}
}

// nur Helligkeitswerte aus LED1.value[n] für alle LEDs im Controller setzen
void ledcontrol_setpwm()
{
	uint8_t i;
	uint8_t count = 0;
	unsigned char error;

	do	// erfolgreichen Start versuchen
	{
		if (count > 0) { warte_ms(50); }	// kleine Pause, wenn Salve nicht bereit
		count++;
		error = i2c_start(LEDC1+I2C_WRITE);	// return 0 = device accessible, 1= failed to access device
	} while (error && (count < 3));		// nochmal versuchen, wenn Slave nicht bereit, aber nur 3 Versuche

	if(!error)
	{
		i2c_write(0x82);				//Controllregister setzen: beim 1. PWM-Register (Helligkeit) anfangen, autoincrement for all, weil's egal ist
		for(i=0;i<16;i++)
		{
			//if (LED1.value[i] > LED1.max_value[i]) { LED1.value[i] = LED1.max_value[i]; }
			if (!(LED1.exist & (1<<i))) { LED1.value[i] = 0; }	// nicht existente LEDs immer auf 0 setzen
			i2c_write(LED1.value[i]);					//alle PWM Kanäle (Helligkeit) setzen
		}
		i2c_stop();	// fertig
	}
}

// eine LED ein/aus -schalten oder blinken (im LED struct speichern und über I2C an LED-Controller ausgeben)
void ledcontrol_led_setoutputmode(uint8_t lednumber, uint8_t value)
{
	uint8_t i;
	uint8_t output = 0;
	uint8_t add = 0;
	uint8_t count = 0;
	unsigned char error;

	LED1.outputmode[lednumber] = value;	// Wert muss in LED struct gespeichert werden

	// Adresse 20: LEDOUT0  (LED 0 - 3)
	// Adresse 21: LEDOUT1  (LED 4 - 7)
	// Adresse 22: LEDOUT2  (LED 8 - 11)
	// Adresse 23: LEDOUT3  (LED 12 - 15)
	if (lednumber >  3) { add++; }
	if (lednumber >  7) { add++; }
	if (lednumber > 11) { add++; }

	for(i=0;i<4;i++)
	{
		output |= LED1.outputmode[i+(4*add)]<<(i*2);	// 4 LEDS sind in ein Output-Register zusammengefasst
	}

	do	// erfolgreichen Start versuchen
	{
		if (count > 0) { warte_ms(50); }	// kleine Pause, wenn Slave nicht bereit
		count++;
		error = i2c_start(LEDC1+I2C_WRITE);	// return 0 = device accessible, 1= failed to access device
	} while (error && (count < 3));		// nochmal versuchen, wenn Salve nicht bereit, aber nur 3 Versuche

	if(!error)
	{
		i2c_write(20 + add);		//Controllregister setzen: zugehöriges LEDOUTx, no autoincrement (LEDOUT0 = Adresse 20 (=0x14)
		i2c_write(output);			// LEDOUTx schreiben
		i2c_stop();
	}
}

// nur Helligkeitswert für lednumber in LED1.value[n] setzen und an controller ausgeben
void ledcontrol_led_setpwm(uint8_t lednumber, uint8_t value)
{
	uint8_t adresse = 2;	// Adresse des ersten PWM-Registers im LED-controller
	uint8_t count = 0;
	unsigned char error;

	adresse += lednumber;
	LED1.value[lednumber] = value;	// in LED struct speichern
	//if (LED1.value[lednumber] > LED1.max_value[lednumber]) { LED1.value[lednumber] = LED1.max_value[lednumber]; }
	//if (!(LED1.exist & (1<<lednumber))) { LED1.value[lednumber] = 0; }	// nicht existente LEDs immer auf 0 setzen

	do	// erfolgreichen Start versuchen
	{
		if (count > 0) { warte_ms(50); }	// kleine Pause, wenn Salve nicht bereit
		count++;
		error = i2c_start(LEDC1+I2C_WRITE);	// return 0 = device accessible, 1= failed to access device
	} while (error && (count < 3));		// nochmal versuchen, wenn Slave nicht bereit, aber nur 3 Versuche

	if(!error)
	{
		i2c_write(adresse);				//Controllregister setzen: PWM-Register (Helligkeit), kein autoincrement
		i2c_write(value);					// PWM (Helligkeit) setzen
		i2c_stop();	// fertig
	}
}

void ledcontrol_allonoff(uint8_t on)	//alle LEDOUT ein oder ausschalten (ein: on=1, aus: on=0)
{
	uint8_t i;
	uint8_t output = 0;
	uint8_t count = 0;
	unsigned char error;

	if (on == 1) { output = 0x55; }	// alle ein (immer ein, PWM ignorieren) (0xAA wäre: ein mit PWM)
	else { output = 0; }	// auch nur testweise

	do	// erfolgreichen Start versuchen
	{
		if (count > 0) { warte_ms(50); }	// kleine Pause, wenn Slave nicht bereit
		count++;
		error = i2c_start(LEDC1+I2C_WRITE);	// return 0 = device accessible, 1= failed to access device
	} while ((error > 0) && (count < 3));		// nochmal versuchen, wenn Salve nicht bereit, aber nur 3 Versuche

	if(!error)
	{
		i2c_write(0x94);					//Controllregister setzen: beim LEDOUT0 anfangen, autoincrement for all

		for(i=0;i<4;i++)
		{
			i2c_write(output);				//Outputs setzen
		}
		i2c_stop();	// fertig
	}
}

// system_power_off und _on machen jeweils genaus das Gegenteil von dem, was eigentlich sein sollte -> checken, warum

void system_power_off()	// Relais für Stromversorgung ausschalten
{
	PORT_SYS &= ~(1<<SYSTEM_OFF);
	PORT_SYS |= (1<<SYSTEM_ON);
	TIMSK5 &= ~(1<<OCIE5A);	// disable timer5 output compare match interrupt
	relaycouter = 125;	// 500ms Impuls
	TIMSK5 |= (1<<OCIE5A);	// enable timer5 output compare match interrupt
}

void system_power_on()	// Relais für Stromversorgung einschalten
{
	PORT_SYS &= ~(1<<SYSTEM_ON);
	PORT_SYS |= (1<<SYSTEM_OFF);
	TIMSK5 &= ~(1<<OCIE5A);	// disable timer5 output compare match interrupt
	relaycouter = 125;	// 500ms Impuls
	TIMSK5 |= (1<<OCIE5A);	// enable timer5 output compare match interrupt
}

void set_powersource(uint8_t source)	// Relais (monostabil) für Umschaltung. Versorgung über Schiene / AKKU
//siehe Definitionen in lokbasis_hwdef:
//#define POWER_FROM_RAILS 	 	0	// Versorgung über Schienen
//#define POWER_FROM_BATTERY	1	// Versorgung aus AKKU
// Relais monostabil: stromloser Zustand = Versorgung aus AKKU
{
	if (source == POWER_FROM_RAILS)	// Umschalten auf Schiene
	{
		PORT_SYS |= (1<<AKKU_OFF);	// auf 1 setzen = Relais zieht an
	}
	else	// Umschalten auf AKKU
	{
		PORT_SYS &= ~(1<<AKKU_OFF);	// auf 0 setzen = Relais fällt ab
	}


}



