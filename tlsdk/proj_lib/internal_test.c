#include "../proj/tl_common.h"

extern int main();

#if (MCU_CORE_TYPE == MCU_CORE_5330)
#define LEGAL_ID    0x8566
#elif (MCU_CORE_TYPE == MCU_CORE_5332)
#define LEGAL_ID    0x8666
#elif (MCU_CORE_TYPE == MCU_CORE_5328)
#define LEGAL_ID    0x8866
#else
#define LEGAL_ID    0xffff
#endif

static volatile unsigned int illegal_cnt;
static volatile unsigned int legal;
void init_main()
{
#if 0 //(__TL_LIB_5328__ || __TL_LIB_5320__ || __TL_LIB_5330__ || __TL_LIB_5332__)

	legal = read_reg16(0x80007e);
	if(legal==LEGAL_ID)
		main();
	else
	{
		while(1) illegal_cnt++;
	}
#else
	main();
#endif
}
