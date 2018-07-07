/*
 * kb_info.h
 *
 *  Created on: 2015-1-21
 *      Author: Administrator
 */

#ifndef KB_INFO_H_
#define KB_INFO_H_


#define		PM_REG_MODE_LINK		0x0c
#define		PM_REG_CHECK		    0x0d
#define		PM_REG_LOCK		    	0x0e

#define		PM_REG_DONGLE_ID_START		0x19
#define		PM_REG_DONGLE_ID_END		0x1c


typedef struct{
	u32 dongle_id;
	u8	rsv;
    u8	channel;
	u8	mode;
	u8	poweron;
} kb_info_t;

extern void kb_info_load(void);
extern void kb_info_save(void);

#endif /* KB_INFO_H_ */
