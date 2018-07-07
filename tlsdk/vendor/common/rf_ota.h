/*
 * rf_ota.h
 *
 *  Created on: 2014-11-19
 *      Author: Telink
 */

#ifndef RF_OTA_H_
#define RF_OTA_H_

#define		FLG_RF_OTA_DATA			0xed
#define		RF_PROTO_OTA   		    0x69

typedef struct{
	u32 dma_len;

	u8  rf_len;
	u8	proto;
	u8  flag;
	u8  seq;

	u32	adr;
}rf_packet_ota_req_data_t;

typedef struct{
	u32 dma_len;

	u8  rf_len;
	u8	proto;
	u8  flag;
	u8  seq;

	u32	adr;

	u8	dat[32];
	u16	crc;
}rf_packet_ota_ack_data_t;



inline unsigned short crc16 (unsigned char *pD, int len)
{

    static unsigned short poly[2]={0, 0xa001};              //0x8005 <==> 0xa001
    unsigned short crc = 0xffff;
    unsigned char ds;
    int i,j;

    for(j=len; j>0; j--)
    {
        unsigned char ds = *pD++;
        for(i=0; i<8; i++)
        {
            crc = (crc >> 1) ^ poly[(crc ^ ds ) & 1];
            ds = ds >> 1;
        }
    }

     return crc;
}


#endif /* RF_OTA_H_ */
