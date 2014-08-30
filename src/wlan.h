/*
 * wlan.h
 *
 *  Created on: 29.01.2011
 *      Author: Michael Brunnbauer
 */

#ifndef WLAN_H_
#define WLAN_H_

extern void wlan_on(unsigned char wlan_on);
extern void wlan_init(unsigned char wlan_typ);
extern void serial_cmd_ein(void);
extern void serial_cmd_aus(void);
extern void wlan_prepare_status(unsigned char type, char *text);
extern void wlan_read_cmd(char *text);
extern void wlan_command_query(const char *command, char *text);
extern void wlan_command_query2(const char *command, char *text);
extern void read_wlan(uint8_t wait_sek);
extern void wlan_status(void);
extern void wlan_tx_off(void);
extern void wlan_tx_on(void);
extern void wlan_set_ip(void);
extern void wlan_get_ip(void);
extern void wlanreset(void);
extern unsigned char setup_wlan(void);
extern void rn131_command_query(const char *command, const char *reply, char *text, int textlen);
extern void rn131_init(char *text, int textlen);
extern void rn131console();

#endif /* WLAN_H_ */
