
#pragma once

#include "../common/types.h"

void i2c_sim_init(void);
void i2c_sim_write(u8 id, u8 addr, u8 dat);
u8 i2c_sim_read(u8 id, u8 addr);
void i2c_sim_burst_read(u8 id, u8 addr, u8 *p, u8 n);
void i2c_sim_burst_write(u8 id, u8 addr,u8 *p,u8 n);

void i2c_init (void);
int i2c_burst_write(u8 id, u16 adr, u8 * buff, int len);
int i2c_burst_read(u8 id, u16 adr, u8 * buff, int len);
void i2c_write(u8 id, u16 adr, u8 dat);
u8 i2c_read(u8 id, u16 adr);

