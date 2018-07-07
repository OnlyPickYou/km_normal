/*
 * 8263_trace.h
 *
 *  Created on: 2015-7-9
 *      Author: Administrator
 */

#ifndef TRACE_H_
#define TRACE_H_


#define		LOG_MASK_BEGIN		0x40
#define		LOG_MASK_END		0x00
#define		LOG_MASK_TGL		0xC0
#define		LOG_MASK_DAT		0x80



#if(TRACE_DBG_ENABLE)


extern void trace_init(void);
extern void trace_log(u8 type_id,u32 data);

//void trace_write(int id, int type, u32 dat);

void trace_task_begin(int id);
void trace_task_end(int id);
void trace_event(int id);
void trace_data(int id, u32 dat);

#else

#define trace_init()
#define trace_write(id, type, dat)
#define trace_task_begin(id)
#define trace_task_end(id)
#define trace_event(id)
#define trace_data(id, dat)


#endif



#endif /* 8263_TRACE_H_ */
