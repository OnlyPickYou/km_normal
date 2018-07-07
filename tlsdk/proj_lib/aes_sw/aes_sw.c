
#include "aes_sw.h"
#include "../../proj/tl_common.h"
#include "../../proj/mcu/compiler.h"

#ifndef ENC_IN_RAM
#define ENC_IN_RAM 0
#endif

#define HAS_ROTATE_SHIFT_BUG					1
#define PRE_CALCULATE_KEY_SCHEDULE		 		1			//  need  176 more RAM, but 30% faster
#define DECRYPT_FUNCTION_USED					0			//  if ccm,  then not used

typedef unsigned char           word8;	
typedef unsigned short          word16;	
typedef unsigned long           word32;
word8 aes_sw_S[256] = {
 99, 124, 119, 123, 242, 107, 111, 197,  48,   1, 103,  43, 254, 215, 171, 118, 
202, 130, 201, 125, 250,  89,  71, 240, 173, 212, 162, 175, 156, 164, 114, 192, 
183, 253, 147,  38,  54,  63, 247, 204,  52, 165, 229, 241, 113, 216,  49,  21, 
  4, 199,  35, 195,  24, 150,   5, 154,   7,  18, 128, 226, 235,  39, 178, 117, 
  9, 131,  44,  26,  27, 110,  90, 160,  82,  59, 214, 179,  41, 227,  47, 132, 
 83, 209,   0, 237,  32, 252, 177,  91, 106, 203, 190,  57,  74,  76,  88, 207, 
208, 239, 170, 251,  67,  77,  51, 133,  69, 249,   2, 127,  80,  60, 159, 168, 
 81, 163,  64, 143, 146, 157,  56, 245, 188, 182, 218,  33,  16, 255, 243, 210, 
205,  12,  19, 236,  95, 151,  68,  23, 196, 167, 126,  61, 100,  93,  25, 115, 
 96, 129,  79, 220,  34,  42, 144, 136,  70, 238, 184,  20, 222,  94,  11, 219, 
224,  50,  58,  10,  73,   6,  36,  92, 194, 211, 172,  98, 145, 149, 228, 121, 
231, 200,  55, 109, 141, 213,  78, 169, 108,  86, 244, 234, 101, 122, 174,   8, 
186, 120,  37,  46,  28, 166, 180, 198, 232, 221, 116,  31,  75, 189, 139, 138, 
112,  62, 181, 102,  72,   3, 246,  14,  97,  53,  87, 185, 134, 193,  29, 158, 
225, 248, 152,  17, 105, 217, 142, 148, 155,  30, 135, 233, 206,  85,  40, 223, 
140, 161, 137,  13, 191, 230,  66, 104,  65, 153,  45,  15, 176,  84, 187,  22, 
};

static const word8 aes_sw_Si[256] = {
 82,   9, 106, 213,  48,  54, 165,  56, 191,  64, 163, 158, 129, 243, 215, 251, 
124, 227,  57, 130, 155,  47, 255, 135,  52, 142,  67,  68, 196, 222, 233, 203, 
 84, 123, 148,  50, 166, 194,  35,  61, 238,  76, 149,  11,  66, 250, 195,  78, 
  8,  46, 161, 102,  40, 217,  36, 178, 118,  91, 162,  73, 109, 139, 209,  37, 
114, 248, 246, 100, 134, 104, 152,  22, 212, 164,  92, 204,  93, 101, 182, 146, 
108, 112,  72,  80, 253, 237, 185, 218,  94,  21,  70,  87, 167, 141, 157, 132, 
144, 216, 171,   0, 140, 188, 211,  10, 247, 228,  88,   5, 184, 179,  69,   6, 
208,  44,  30, 143, 202,  63,  15,   2, 193, 175, 189,   3,   1,  19, 138, 107, 
 58, 145,  17,  65,  79, 103, 220, 234, 151, 242, 207, 206, 240, 180, 230, 115, 
150, 172, 116,  34, 231, 173,  53, 133, 226, 249,  55, 232,  28, 117, 223, 110, 
 71, 241,  26, 113,  29,  41, 197, 137, 111, 183,  98,  14, 170,  24, 190,  27, 
252,  86,  62,  75, 198, 210, 121,  32, 154, 219, 192, 254, 120, 205,  90, 244, 
 31, 221, 168,  51, 136,   7, 199,  49, 177,  18,  16,  89,  39, 128, 236,  95, 
 96,  81, 127, 169,  25, 181,  74,  13,  45, 229, 122, 159, 147, 201, 156, 239, 
160, 224,  59,  77, 174,  42, 245, 176, 200, 235, 187,  60, 131,  83, 153,  97, 
 23,  43,   4, 126, 186, 119, 214,  38, 225, 105,  20,  99,  85,  33,  12, 125, 
};
  

#if	HAS_ROTATE_SHIFT_BUG
_attribute_no_inline_ static unsigned int rot(x, n){
	volatile unsigned int r = x >> n;
	volatile unsigned int l = x << (32 - n);
	return r | l;
}
#define rot1(x) 	rot(x, 8)
#define rot2(x) 	rot(x, 16)
#define rot3(x) 	rot(x, 24)
#else
#define rot1(x) (((x) << 24) | ((x) >> 8))
#define rot2(x) (((x) << 16) | ((x) >> 16))
#define rot3(x) (((x) <<  8) | ((x) >> 24))
#endif

/** AES round constants */
static const unsigned char Rcon[]= {
	0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36,
};

#define AES_BLOCKSIZE   16  /**< @brief Blocksize of aes encryption - 128bit */

const unsigned int ctx_iv[4] = {0x03020100,0x07060504,0x0b0a0908,0x0f0e0d0c};
#if PRE_CALCULATE_KEY_SCHEDULE
unsigned int ctx_ks[4 * 11];							/**< @brief hold the different keystates */
#else
unsigned int ctx_ks[4];									/**< @brief hold the different keystates */
#endif
static unsigned int ctx_iks[4 * 11];

/* ----- static functions ----- */
static void AES_cpy32(unsigned int * dst, unsigned int * src){
	*dst++ = *src++;
	*dst++ = *src++;
	*dst++ = *src++;
	*dst = *src;
}
/* Perform doubling in Galois Field GF(2^8) using the irreducible polynomial
   x^8+x^4+x^3+x+1 */
static unsigned char AES_xtime(unsigned int x){
	return (x&0x80) ? (x<<1)^0x1b : x<<1;
}

/** Set up AES with the key/iv and cipher size. */
/** Change a key for decryption. */
#define mt  0x80808080
#define mh  0xfefefefe
#define mm  0x1b1b1b1b

#if PRE_CALCULATE_KEY_SCHEDULE
void _rijndaelSetKey(unsigned char *key /*, int bothEncDec*/){  //sihui delete ramcode
	int i;
	const unsigned char * rtmp = Rcon;
	AES_cpy32((unsigned int *)(ctx_ks), (unsigned int *)key);
	for (i = 0; i < 4*10; i++){
		unsigned int tmp, tmp2;
		tmp = ctx_ks[i+3];
		if ((i % 4) == 0){
			tmp2 =(unsigned int)aes_sw_S[(tmp    )&0xff]<<24;
			tmp2|=(unsigned int)aes_sw_S[(tmp>> 8)&0xff];
			tmp2|=(unsigned int)aes_sw_S[(tmp>>16)&0xff]<<8;
			tmp2|=(unsigned int)aes_sw_S[(tmp>>24)     ]<<16;
			tmp=tmp2^(((unsigned int)*rtmp));
			rtmp++;
		}
		ctx_ks[i+4]=ctx_ks[i]^tmp;
	}
	#if DECRYPT_FUNCTION_USED
//	if(0 == bothEncDec) return;

	AES_cpy32((unsigned int *)(ctx_iks), (unsigned int *)ctx_ks);
	AES_cpy32(((unsigned int *)(ctx_iks)) + 10*4, ((unsigned int *)ctx_ks) + 10 * 4);

	unsigned int *k,*ik, w,t1,t2,t3,t4;
	k = ctx_ks + 4;
	ik = ctx_iks + 4;
	for (int i = 0; i < 10*4 - 4; i++){
		w= *k++;
		t1 = w & mt;
		t1 = ((w + w) & mh) ^ ((t1-(t1 >> 7)) & mm);
		t2 = t1 & mt;
		t2 = ((t1 + t1) & mh) ^ ((t2 - (t2 >> 7)) & mm);
		t3 = t2 & mt;
		t3 = ((t2 + t2) & mh) ^ ((t3 - (t3 >> 7)) & mm);
		t4 = w ^ t3;
		t3 = t1 ^ t2 ^ t3;
		t1 ^= t4;
		t2 ^= t4;
		t3 ^= rot1(t1);
		t3 ^= rot2(t2);
		*ik++ = t3 ^ rot3(t4);
	}
	#endif
}
#else
static void AES_set_next_key(unsigned int *ks, const unsigned char *rtmp){
	unsigned int tmp, tmp2;
	for(int i = 0; i < 4; ++i){
		tmp = ks[(i+3)%4];
		if ((i % 4) == 0){
			tmp2 =(unsigned int)aes_sw_S[(tmp    )&0xff]<<24;
			tmp2|=(unsigned int)aes_sw_S[(tmp>> 8)&0xff];
			tmp2|=(unsigned int)aes_sw_S[(tmp>>16)&0xff]<<8;
			tmp2|=(unsigned int)aes_sw_S[(tmp>>24)     ]<<16;
			tmp=tmp2^(((unsigned int)*rtmp));
		}
		ks[i%4]=ks[i%4]^tmp;	//  it is smaller than  ks[i]=ks[i]^tmp;
	}
}
void _rijndaelSetKey(unsigned char *key /*, int bothEncDec*/){
	AES_cpy32((unsigned int *)(ctx_ks), (unsigned int *)key);
	
	#if DECRYPT_FUNCTION_USED
//	if(0 == bothEncDec) return;

	unsigned int ks[4];
	AES_cpy32((unsigned int *)(ctx_iks), (unsigned int *)key);
	AES_cpy32((unsigned int *)(ks), (unsigned int *)key);
	const unsigned char * rtmp = Rcon;

	unsigned int *ik, w,t1,t2,t3,t4;
	ik = ctx_iks + 4;
	for (int i = 0; i < 10; i++){
		unsigned int *k = ks;
		AES_set_next_key((unsigned int *)(k), rtmp++);
		if(i == 9){
			AES_cpy32((unsigned int *)(ik), (unsigned int *)k);		//  last round
			break;
		}
		for(int j = 0; j < 4; ++j){
			w = *k++;
			t1 = w & mt;
			t1 = ((w + w) & mh) ^ ((t1-(t1 >> 7)) & mm);
			t2 = t1 & mt;
			t2 = ((t1 + t1) & mh) ^ ((t2 - (t2 >> 7)) & mm);
			t3 = t2 & mt;
			t3 = ((t2 + t2) & mh) ^ ((t3 - (t3 >> 7)) & mm);
			t4 = w ^ t3;
			t3 = t1 ^ t2 ^ t3;
			t1 ^= t4;
			t2 ^= t4;
			t3 ^= rot1(t1);
			t3 ^= rot2(t2);
			*ik++ = t3 ^ rot3(t4);
		}
	}
	#endif
}
#endif

/** Encrypt a single block (16 bytes) of data */
#if (ENC_IN_RAM)
_attribute_ram_code_
#endif
void _rijndaelEncrypt(unsigned char *d){
	int row;
	unsigned int *data = (unsigned int *)d;
#if PRE_CALCULATE_KEY_SCHEDULE
    const unsigned int *k = ctx_ks;
#else
	unsigned int ks[4];
	AES_cpy32(ks, (unsigned int *)(ctx_ks));

	unsigned int *k = ks;
	const unsigned char *rtmp = Rcon;
#endif
    /* Pre-round key addition */
    for (row = 0; row < 4; row++) 
		data[row] ^= *(k++);

    /* Encrypt one block. */
    for (int curr_rnd = 0; curr_rnd < 10; curr_rnd++){
	    unsigned int tmp[4];
        /* Perform ByteSub and ShiftRow operations together */
        for (row = 0; row < 4; row++){
		    unsigned int tmp1, old_a0, a0, a1, a2, a3;
            a0 = (unsigned int)aes_sw_S[(data[row%4])&0xFF];
            a1 = (unsigned int)aes_sw_S[(data[(row+1)%4]>>8)&0xFF];
            a2 = (unsigned int)aes_sw_S[(data[(row+2)%4]>>16)&0xFF]; 
            a3 = (unsigned int)aes_sw_S[(data[(row+3)%4]>>24)&0xFF];

            /* Perform MixColumn iff not last round */
            if (curr_rnd < (10 - 1)){
                tmp1 = a0 ^ a1 ^ a2 ^ a3;
                old_a0 = a0;
                a0 ^= tmp1 ^ AES_xtime(a0 ^ a1);
                a1 ^= tmp1 ^ AES_xtime(a1 ^ a2);
                a2 ^= tmp1 ^ AES_xtime(a2 ^ a3);
                a3 ^= tmp1 ^ AES_xtime(a3 ^ old_a0);
            }
            tmp[row] = ((a3 << 24) | (a2 << 16) | (a1 << 8) | a0);
        }
#if !PRE_CALCULATE_KEY_SCHEDULE
		k = ks; AES_set_next_key(k, rtmp++);
#endif
        /* KeyAddition - note that it is vital that this loop is separate from
           the MixColumn operation, which must be atomic...*/ 
        for (row = 0; row < 4; row++) 
			data[row] = tmp[row] ^ *(k++);
    }
}

#if (DECRYPT_FUNCTION_USED)
/** Decrypt a single block (16 bytes) of data */
void _rijndaelDecrypt(unsigned char *d)
{ 
    int row;
	unsigned int *data = (unsigned int *)d;
    const unsigned int *k = ctx_iks + ((10+1)*4);
    /* pre-round key addition */
    for (row=3; row >= 0;row--)
        data[row] ^= *(--k);

    /* Decrypt one block */
	for (int curr_rnd = 0; curr_rnd < 10; curr_rnd++){
	    unsigned int tmp[4];
		/* Perform ByteSub and ShiftRow operations together */
		for (row = 4; row > 0; row--){
		    unsigned int a0, a1, a2, a3;
			a0 = aes_sw_Si[(data[(row+3)%4])&0xFF];
			a1 = aes_sw_Si[(data[(row+2)%4]>>8)&0xFF];
			a2 = aes_sw_Si[(data[(row+1)%4]>>16)&0xFF];
			a3 = aes_sw_Si[(data[row%4]>>24)&0xFF];

			/* Perform MixColumn iff not last round */
			if (curr_rnd<(10-1)){
			    unsigned int xt0,xt1,xt2,xt3,xt4,xt5,xt6;
				/* The MDS cofefficients (0x09, 0x0B, 0x0D, 0x0E)
					are quite large compared to encryption; this 
					operation slows decryption down noticeably. */
				xt0 = AES_xtime(a0^a1);
				xt1 = AES_xtime(a1^a2);
				xt2 = AES_xtime(a2^a3);
				xt3 = AES_xtime(a3^a0);
				xt4 = AES_xtime(xt0^xt1);
				xt5 = AES_xtime(xt1^xt2);
				xt6 = AES_xtime(xt4^xt5);

				xt0 ^= a1^a2^a3^xt4^xt6;
				xt1 ^= a0^a2^a3^xt5^xt6;
				xt2 ^= a0^a1^a3^xt4^xt6;
				xt3 ^= a0^a1^a2^xt5^xt6;
				tmp[row-1] = ((xt3<<24)|(xt2<<16)|(xt1<<8)|xt0);
			}
			else
				tmp[row-1] = ((a3<<24)|(a2<<16)|(a1<<8)|a0);
		}
		for (row = 3; row >= 0; row--)
			data[row] = tmp[row] ^ *(--k);
	}
}
#endif

