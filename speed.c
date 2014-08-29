/*
 * speed.c
 *
 *  Created on: 01.08.2010
 *      Author: Michael Brunnbauer
 */

#include <avr/io.h>
#include <string.h>		// für "strcmp"
#include <stdlib.h>		// für "itoa"
#include <util/delay.h>	// für delay_ms()
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "lokbasis_hwdef.h"
#include "i2clcd.h"				// I2C LCD Funktionen
#include "uart.h"
#include "speed.h"
#include "main.h"


//----------------------------------------------------------------------------------------------
// set_speed
// setzt als einzige Funktion den PWM Wert für die Geschwindigkeit!!!
// und glättet die Geschwindigkeitssprüge vom Steuerungsgerät bei Start / Stop / starken Beschleunigungen
//----------------------------------------------------------------------------------------------

void set_speed(void)
{
	int speeddelta = 0;
	int pwm = 0;							// für LCD-Ausgabe des speed-pwm-wertes
	//char pwmtest[UART_MAXSTRLEN+1] = "";	// für LCD-Ausgabe des speed-pwm-wertes

	//if (speed_soll >= 1000) {speed_soll = 999; }	// 1000 (und größer) vemeiden, da stringmäßig nur 3 stellige Zahlen verarbeitet werden!	// jetzt geht's auch bis 1023


	//---------- Anfangszustand: Speedänderung von 0 -> Richtung am Motor setzen --------------------
	// Richtungswechsel nur bei Speed == 0, dafür immer, wenn Speed-Änderung vorliegt
	if ((speed == 0) & (speed_soll != 0))
	{
		if(richtung_soll == RICHTUNG_VW)		// vorwärts
		{
#if defined( HW_TESTBOARD1 )
			clearbit(PORTL,0);	// Motor A vorwärts
			clearbit(PORTL,6);
#elif defined( HW_TESTBOARD2 )
			clearbit(PORT_MOTOR, MOTOR_DIR);	// Motoren vorwärts
#endif
			richtung = RICHTUNG_VW;
		}
		else if(richtung_soll == RICHTUNG_RW)		// rückwärts
		{
#if defined( HW_TESTBOARD1 )
			setbit(PORTL,0);	// Motor A rückwärts
			setbit(PORTL,6);
#elif defined( HW_TESTBOARD2 )
			setbit(PORT_MOTOR, MOTOR_DIR);	// Motoren rückwärts
#endif
			richtung = RICHTUNG_RW;
		}
	}


	//----- Geschwindigkeitsänderung ------------------------------------------------------------------------------

	if (richtung != richtung_soll)	// Richtungswechsel: speed auf 0 reduzieren
	{
		if(speed != 0)	// Richtungswechsel -> vorher stop!
		{
			speed = speed - speed / 5;
			if (speed > 50) { speed = speed - speed / 5;	}
			else { speed = 0; }
		}
	}

	if (speed != speed_soll)
	{
		if (richtung == richtung_soll)	// Richtung passt -> beschleunigen
		{
			speeddelta = speed_soll - speed;
			if (abs(speeddelta) > 25) { speed = speed + speeddelta / 5; }
			else { speed = speed_soll; }
		}
	}





	/* alte Version
	//----- Geschwindigkeitsänderung ------------------------------------------------------------------------------
	if (speed != speed_soll)
	{
		if (richtung != richtung_soll)	// Richtungswechsel: speed auf 0 reduzieren
		{
			if(speed != 0)	// Richtungswechsel -> vorher stop!
			{
				speed = speed - speed / 5;
				if (speed > 50) { speed = speed - speed / 5;	}
				else { speed = 0; }
			}
		}
		if (richtung == richtung_soll)	// Richtung passt -> beschleunigen
		{
			speeddelta = speed_soll - speed;
			if (abs(speeddelta) > 25) { speed = speed + speeddelta / 5; }
			else { speed = speed_soll; }
		}
	}
	 */


	//---- PWM-Wert setzen -------------------------------------------------------------------------
	// wird nur hier ausgeführt, sonst nirgends mehr!!!!!


#if defined( MC_INV_PWM_4_REV )	// PWM-Wert muß für Rückwärts invertiert werden

	if (richtung == RICHTUNG_VW) // vorwärts
	{
		OCR1A = (speed>>speedstep_korrektur);
		OCR1B = (speed>>speedstep_korrektur);
		pwm =   (speed>>speedstep_korrektur);
	}
	else if (richtung == RICHTUNG_RW)	// rückwärts
	{
		switch (speedstep_korrektur)
		{
		case 0:	// TOP = 1000 (nicht 1023 wg. dreistelligem speed-wert im command!)	// ab v0.15 auch bis 1023 möglich!
			OCR1A = 1023 - speed;
			OCR1B = 1023 - speed;
			pwm =   1023 - speed;
			break;

		case 1:	// TOP = 511
			OCR1A = 511 - (speed>>speedstep_korrektur);
			OCR1B = 511 - (speed>>speedstep_korrektur);
			pwm   = 511 - (speed>>speedstep_korrektur);
			break;

		case 2:	// TOP = 255
			OCR1A = 255 - (speed>>speedstep_korrektur);
			OCR1B = 255 - (speed>>speedstep_korrektur);
			pwm   = 255 - (speed>>speedstep_korrektur);
			break;
		}
	}
#else	// gleicher PWM-Wert für alle Richtungen
	OCR1A = (speed>>speedstep_korrektur);
	OCR1B = (speed>>speedstep_korrektur);
	pwm =   (speed>>speedstep_korrektur);
#endif

}

