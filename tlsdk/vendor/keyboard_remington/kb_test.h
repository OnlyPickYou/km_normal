/*
 * kb_test.h
 *
 *  Created on: 2015-1-23
 *      Author: Administrator
 */

#ifndef KB_TEST_H_
#define KB_TEST_H_



extern void suspend_test(void);
extern void gpio_test(void);
extern void tx_test(void);
extern void rx_test(void);

int simu_key_data(void);
void clk_32k_rc_test(void);
void pm_test(void);
void adc_test(void);

#endif /* KB_TEST_H_ */
