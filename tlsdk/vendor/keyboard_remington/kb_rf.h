/*
 * kb_rf.h
 *
 *  Created on: 2015-1-21
 *      Author: Administrator
 */

#ifndef KB_RF_H_
#define KB_RF_H_



#define LINK_PIPE_CODE_OK  		0x01
#define LINK_RCV_DONGLE_DATA  	0x02
#define LINK_WITH_DONGLE_OK	    (LINK_PIPE_CODE_OK | LINK_RCV_DONGLE_DATA)


//kb_status.loop_cnt relative
#define KB_MANUAL_PARING_MOST	   (10000/KB_MAIN_LOOP_TIME_MS)  //手动配对最大时间
#define KB_NO_QUICK_SLEEP_CNT	   (20000/KB_MAIN_LOOP_TIME_MS)  //LINK最大时间
#define KB_PARING_POWER_ON_CNT      44    						 //上电自动配对包

#define HOST_NO_LINK        (kb_status.no_ack >= 400)
#define HOST_LINK_LOST		(kb_status.no_ack >= 300)

#define device_never_linked (rf_get_access_code1() == U32_MAX)

extern u8* kb_rf_pkt;
extern rf_packet_pairing_t	pkt_pairing;
extern rf_packet_keyboard_t	pkt_km;

extern int	km_dat_sending;

extern int kb_pairing_mode_detect(void);
extern void kb_paring_and_syncing_proc(void);

extern void kb_rf_proc(u32 key_scaned);
extern void kb_rf_init(void);

void irq_device_rx(void);
void irq_device_tx(void);


#define DO_TASK_WHEN_RF_EN      1
typedef void (*callback_rx_func) (u8 *);
typedef void (*task_when_rf_func) (void);
extern task_when_rf_func p_task_when_rf;

#endif /* KB_RF_H_ */
