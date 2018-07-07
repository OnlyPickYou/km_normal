/*
 * kb_emi.h
 *
 *  Created on: 2015-1-21
 *      Author: Administrator
 */

#ifndef KB_EMI_H_
#define KB_EMI_H_

typedef struct{
	u8  emi_start;

	u8	emi_lf_carry;
	u8	emi_lf_cd;
	u8	emi_lf_rx;
	u8	emi_lf_tx;

	u8	emi_mf_carry;
	u8	emi_mf_cd;
	u8	emi_mf_rx;
	u8	emi_mf_tx;

	u8	emi_hf_carry;
	u8	emi_hf_cd;
	u8	emi_hf_rx;
	u8	emi_hf_tx;

	u8	rsv[3];
}kb_emi_info_t;

extern void kb_emi_process(void);
#endif /* KB_EMI_H_ */
