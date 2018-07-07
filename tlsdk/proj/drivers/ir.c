
#include "../tl_common.h"
#include "ir.h"

// infrared protocols
static void ir_stop_carrier(void){
#if(IR_USE_PWM)
    pwm_stop(IR_PWM_ID);
#else
	gpio_write(GPIO_IR, 1);
#endif
}

static void ir_out_carrier(u32 t_us){
#if(IR_USE_PWM)
    pwm_start(IR_PWM_ID);
#else
	gpio_write(GPIO_IR, 0);
#endif
	sleep_us(t_us);
	ir_stop_carrier();
}

static void ir_send_bit(int bit){
	if(bit){
		ir_out_carrier(IR_HIGH_CARR_TIME);
		sleep_us(IR_HIGH_NO_CARR_TIME);
	}else{
		ir_out_carrier(IR_LOW_CARR_TIME);
		sleep_us(IR_LOW_NO_CARR_TIME);
	}
}

static void ir_send_byte(u8 data){
	int mask = 0x01;
	foreach(i, 8){		//  send  LSB first
		ir_send_bit(mask & data);
		mask = mask << 1;
	}
}

static void ir_send_data(u8 dat){
	ir_send_byte(dat);
	ir_send_byte(~dat);
}

static void ir_send_intro(void){
	ir_out_carrier(IR_INTRO_CARR_TIME);
	sleep_us(IR_INTRO_NO_CARR_TIME);
}

void ir_send_repeat(void){
	u32 pst_time;
	u32 start_time = clock_time();
	ir_out_carrier(IR_INTRO_CARR_TIME);
	sleep_us(IR_REPEAT_NO_CARR_TIME);
	ir_end_transmit();
	pst_time = ((u32)(clock_time()- start_time))/CLOCK_SYS_CLOCK_1US;
	sleep_us(106*1000 - pst_time);
}

void ir_send_repeat2(void){
	u32 pst_time;
	u32 start_time = clock_time();
	ir_out_carrier(IR_INTRO_CARR_TIME);
	sleep_us(IR_REPEAT_NO_CARR_TIME);
	ir_end_transmit();
	pst_time = ((u32)(clock_time()- start_time))/CLOCK_SYS_CLOCK_1US;
	sleep_us(66*1000 - pst_time);
}

void ir_end_transmit(void){
	ir_out_carrier(IR_END_TRANS_TIME);
}

void ir_send_code(u8 addr, u8 cmd)
{
	u32 pst_time;
	u32 start_time = clock_time();
	ir_send_intro();
	ir_send_byte(addr);
	ir_send_byte(IR_CMD_CODE);
    ir_send_data(cmd);
    ir_end_transmit();
    pst_time = ((u32)(clock_time()- start_time))/CLOCK_SYS_CLOCK_1US;
    sleep_us(106*1000 - pst_time);
}

void ir_send_switch(u8 addr, u8 cmd){
	ir_send_intro();
    ir_send_byte(addr);
    ir_send_byte(cmd);
    ir_send_data(IR_SWITCH_CODE);
	ir_end_transmit();
    sleep_us(IR_REPEAT_INTERVAL_TIME);
	ir_send_repeat();
}

////////////////////////////////////////////////////////////////////////

#define IR_LEARN_CARR_GLITCH_MIN	(3*CLOCK_SYS_CLOCK_1US)
#define IR_LEARN_CARR_MIN			(7 * CLOCK_SYS_CLOCK_1US)
#define IR_LEARN_CARR_MAX			(20 * CLOCK_SYS_CLOCK_1US)
#define IR_LEARN_NONE_CARR_MIN		(80 * CLOCK_SYS_CLOCK_1US)
#define IR_LEARN_INTRO_MIN			(2300 * CLOCK_SYS_CLOCK_1US)		//  JVC is 2400us
#define IR_LEARN_MAX_TIME			(200000 * CLOCK_SYS_CLOCK_1US)

#define IR_LEARN_REPEAT_SPACE		(15000 * CLOCK_SYS_CLOCK_1US)		// nokia, < 20ms
#define IR_LEARN_REPEAT_INTRO		(8000 * CLOCK_SYS_CLOCK_1US)

#define IR_LEARN_SERIES_CNT			80

enum{
	IR_STAT_INIT,
	IR_STAT_INTRO_FOUND,
	IR_STAT_USER_ID,
	IR_STAT_SYS_CODE,
	IR_STAT_END,
};

typedef struct{
	int is_carr;
	int carr_found;
	u32 carr_check_cnt;		//  check carrier freq
	u32 carr_high_tm;
	u32 carr_low_tm;
	u32 carr_switch_start_tm;

	u32 series_cnt;
	u32 series_tm[IR_LEARN_SERIES_CNT];

	u32 time_interval;
	u32 last_trigger_tm;
	u32 curr_trigger_tm;
	u32 learn_timer_started;
	int ir_int_cnt;

}ir_learn_ctrl_t;

typedef struct{
	u32 carr_high_tm;
	u32 carr_low_tm;
	u32 intro_carr_tm;
	u32 intro_none_carr_tm;

	u32 digit0_carr_tm;
	u32 digit0_none_carr_tm;
	u32 digit1_carr_tm;
	u32 digit1_none_carr_tm;
}ir_learn_pattern_t;


#define IR_LEARN_KEY_COUNT_MAX	50
// STATIC_ASSERT_INT_DIV((IR_LEARN_KEY_COUNT_MAX+1), 4);
typedef struct{
	u8 key_count;
	u8 bits_cnt;
	u8 resv[2];
	u32 code[IR_LEARN_KEY_COUNT_MAX];
}ir_learn_code_t;

ir_learn_ctrl_t ir_learn_ctrl;
ir_learn_pattern_t ir_learn_pattern;
ir_learn_code_t	   ir_learn_code;

#define IR_ITT_PULSE_MIN		(10 * CLOCK_SYS_CLOCK_1US)

#define IR_NEC_INTRO_TIME_MIN	(4000*CLOCK_SYS_CLOCK_1US)	// some chip: intro == 4500us
#define IR_NEC_CARR_TIME_MAX	(1800*CLOCK_SYS_CLOCK_1US)

#define IR_RC5_CARR_TIME_MIN	(800*CLOCK_SYS_CLOCK_1US)
#define IR_RC5_CARR_TIME_MAX	(1000*CLOCK_SYS_CLOCK_1US)

void ir_learn_init(void){
	memset4(&ir_learn_ctrl, 0, sizeof(ir_learn_ctrl));
}

void ir_learn_pwm(int with_carr){
	int is_carr = 1;
	int cmd_received = 0;
	ir_learn_code.code[ir_learn_code.key_count] = 0;
	u32 last_duration = 0;
	foreach(i, ir_learn_ctrl.series_cnt){
		u32 duration = ir_learn_ctrl.series_tm[i];
		if(is_carr){
			// do not learn repeat code
			if(cmd_received && duration > IR_LEARN_REPEAT_INTRO){
				break;
			}
			if(0 == ir_learn_pattern.intro_carr_tm && duration > IR_LEARN_INTRO_MIN){
				ir_learn_pattern.intro_carr_tm = duration;
			}
			if(duration < IR_LEARN_INTRO_MIN){
				last_duration = duration;
			}
		}else{
			// do not learn repeat code
			if(cmd_received && duration > IR_LEARN_REPEAT_SPACE){
				break;
			}
			if(0 == ir_learn_pattern.intro_none_carr_tm && duration > IR_LEARN_INTRO_MIN){
				ir_learn_pattern.intro_none_carr_tm = duration;
			}
			if(duration < IR_LEARN_INTRO_MIN){
				cmd_received = 1;
				if(duration > (last_duration << 1)){
					ir_learn_code.code[ir_learn_code.key_count] |= (1 << ir_learn_code.bits_cnt);

					ir_learn_pattern.digit1_carr_tm = last_duration;
					ir_learn_pattern.digit1_none_carr_tm = duration;
				}else{
					ir_learn_pattern.digit0_carr_tm = last_duration;
					ir_learn_pattern.digit0_none_carr_tm = duration;
				}
				++ir_learn_code.bits_cnt;
			}
		}
		is_carr = !is_carr;
	}
}

void ir_learn_ppm(void){
}

void ir_learn(void){
	if(0 == ir_learn_ctrl.series_cnt)
		return;

	// !!! not cover sharp JVC, protocol yet
	if(ir_learn_ctrl.series_tm[0] > IR_LEARN_INTRO_MIN){
		ir_learn_pwm(1);
	}else if(ir_learn_ctrl.series_tm[0] < IR_ITT_PULSE_MIN){
		ir_learn_pwm(0);	// ITT
	}
	else{
		ir_learn_ppm();
	}
	++ir_learn_code.key_count;
}


#ifdef WIN32

typedef struct{
	int is_carr;
	int duration;
}IR_TEST_CTRL;

#define IR_ONE_CYCLE_TICK  (26.316 * CLOCK_SYS_CLOCK_1US)
#define IR_CARR_HIGH_TICK  (8.772 * CLOCK_SYS_CLOCK_1US)

void ir_learn_test(void){
	IR_TEST_CTRL ir_test_ctrl[] = {{0, 200}, {1, 9000}, {0, 4500}, {1, 560}, {0, 570}, {1, 570}, {0, 1700}, {1, 550}, {0, 1700}, {1, 560}, {0, 550}, {1, 560} /*end*/, {0, 18000}, {1, 9000}/*repeat*/};
	u32 ttl_cycle = 0;
	int trig_pol = 1;
	foreach(i, ARRAY_SIZE(ir_test_ctrl)){
		u32 cnt = ir_test_ctrl[i].duration * 38 / 1000;
		foreach(j, cnt){
			if(ir_test_ctrl[i].is_carr){
				ir_record((ttl_cycle * IR_ONE_CYCLE_TICK), trig_pol);
				trig_pol = trig_pol ? 0 : 1;
				ir_record((ttl_cycle * IR_ONE_CYCLE_TICK + IR_CARR_HIGH_TICK), trig_pol);
				trig_pol = trig_pol ? 0 : 1;
			}
			++ttl_cycle;
		}
	}
}

#endif

