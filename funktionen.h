/*
 * funktionen.h
 *
 *  Created on: 08.01.2011
 *      Author: Michael Brunnbauer
 */

#ifndef FUNKTIONEN_H_
#define FUNKTIONEN_H_


extern void spi_aus(void);
extern void spi_ein(void);
extern void SPI_FastMem2Write(unsigned char * buffer, unsigned int Datalenght);
extern void SPI_FastRead2Mem(unsigned char * buffer, unsigned int Datalenght);
extern void read_usb(void);
extern unsigned char read_rfid(void);
extern void init_uart(uint8_t uartnr, const unsigned int uartsetting);
//extern void setup_uart0(void);
//extern void setup_uart1(void);
//extern void setup_uart2(void);
//extern void setup_uart3(void);
extern void led_ein(void);
extern void led_aus(void);
extern void init_motorctrl(void);
extern void init_pwm(char freq_pwm);
extern void warte_ms(unsigned int wartezeit);
extern void remcrlf(char *crlftext);
extern unsigned int log_getc(void);
extern void log_putc(unsigned char data);
extern void log_puts(const char *s );
extern void log_puts_P(const char *progmem_s );
extern void ledcontrol_reset();
extern void ledcontrol_init();
extern void ledcontrol_setall();
extern void led_set_outputmode(unsigned int group_on, unsigned int group_blink);
extern void ledcontrol_setoutput();
extern void ledcontrol_setpwm();
extern void ledcontrol_led_setoutputmode(uint8_t lednumber, uint8_t value);
extern void ledcontrol_led_setpwm(uint8_t lednumber, uint8_t value);
extern void ledcontrol_allonoff(uint8_t on);
extern void system_power_on();
extern void system_power_off();
extern void set_powersource(uint8_t source);


#endif /* FUNKTIONEN_H_ */
