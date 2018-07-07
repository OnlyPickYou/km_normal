/*
 * mouse_twheel.h
 *
 *  Created on: Feb 12, 2014
 *      Author: xuzhen
 */

#ifndef MOUSE_TWHEEL_H_
#define MOUSE_TWHEEL_H_

void mouse_twheel_init(mouse_hw_t *mouse_hw);

void mouse_twheel_process(mouse_status_t * mouse_status);

void mouse_twheel_detect(mouse_status_t  * mouse_status);

#endif /* MOUSE_TWHEEL_H_ */
