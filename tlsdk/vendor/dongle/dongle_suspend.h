/*
 * dongle_suspend.h
 *
 *  Created on: 2014-7-10
 *      Author: Telink
 */

#ifndef DONGLE_POWER_H_
#define DONGLE_POWER_H_


void proc_suspend(void);
void usb_resume_host(void);
extern int resume_host_flg;


#endif /* DONGLE_POWER_H_ */
