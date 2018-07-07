/*
 * mouse_button.h
 *
 *  Created on: Feb 12, 2014
 *      Author: xuzhen
 */

#ifndef MOUSE_BUTTON_H_
#define MOUSE_BUTTON_H_


#ifndef MOUSE_BUTTON_FULL_FUNCTION
#define MOUSE_BUTTON_FULL_FUNCTION     1
#endif

void mouse_button_init(mouse_hw_t *mouse_hw);

u32 mouse_button_process(mouse_status_t * mouse_status);
u32 mouse_button_process_emi(s8 *chn_idx, u8 *test_mode_sel, u32 btn_pro_end);

u32 mouse_button_detect(mouse_status_t  * mouse_status, u32 detect_level);

#define MOUSE_BTN_HIGH    1
#define MOUSE_BTN_LOW     0

#define BTN_INTO_PAIRING        1
#define BTN_INTO_EMI            2
#define BTN_INTO_SPECL_MODE     4

#endif /* MOUSE_BUTTON_H_ */
