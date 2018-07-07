#include "../../proj/tl_common.h"

///////////////////////////  ATT  protocol  ///////////////////////////
#include "ble_common.h"
#include "att.h"
#include "gatt_uuid.h"
#include "service.h"
#include "gatt_server.h"
#include "gap_const.h"
#include "blueLight.h"

/**********************************************************************
 * LOCAL VARIABLES
 */
const u16 primaryServiceUUID = GATT_UUID_PRIMARY_SERVICE;
static const u16 characterUUID = GATT_UUID_CHARACTER;

const u16 gapServiceUUID = SERVICE_UUID_GENERIC_ACCESS;
const u16 devNameUUID = GATT_UUID_DEVICE_NAME;
const u16 appearanceUIID = 0x2a01;
const u16 periConnParamUUID = 0x2a04;

// Device Name Characteristic Properties
static u8 devNameCharacter = 0x12;

// Appearance Characteristic Properties
static u8 appearanceCharacter = 0x02;

// Peripheral Preferred Connection Parameters Characteristic Properties
static u8 periConnParamChar = 0x02;

u16 appearance = GAP_APPEARE_UNKNOWN;

typedef struct
{
  /** Minimum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  u16 intervalMin;
  /** Maximum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  u16 intervalMax;
  /** Number of LL latency connection events (0x0000 - 0x03e8) */
  u16 latency;
  /** Connection Timeout (0x000A - 0x0C80 * 10 ms) */
  u16 timeout;
} gap_periConnectParams_t;

gap_periConnectParams_t periConnParameters = {20, 40, 0, 1000};

u8	ble_g_devName [] = {'b', 'l', 'u', 'e', 'L', 'i', 'g', 'h', 't'};

//////////////////////// SPP /////////////////////////////////////////////////////
#define TELINK_SPP_UUID_SERVICE   			0x1910		//!< TELINK_SPP service
#define TELINK_SPP_DATA_SERVER2CLIENT 		0x2B10 		//!< TELINK_SPP data from server to client
#define TELINK_SPP_DATA_CLIENT2SERVER 		0x2B11 		//!< TELINK_SPP data from client to server

const u16 TelinkSppServiceUUID       = TELINK_SPP_UUID_SERVICE;
const u16 TelinkSppDataServer2ClientUUID     = TELINK_SPP_DATA_SERVER2CLIENT;
const u16 TelinkSppDataClient2ServiceUUID     = TELINK_SPP_DATA_CLIENT2SERVER;


// Spp data from Server to Client characteristic variables
static u8 SppDataServer2ClientProp = CHAR_PROP_READ | CHAR_PROP_NOTIFY;

// Spp data from Client to Server characteristic variables
static u8 SppDataClient2ServerProp = CHAR_PROP_READ | CHAR_PROP_WRITE;

static u8 SppDataServer2ClientData[20] = {0xf0};

//#define		SppDataClient2ServerData		send_to_master
static u8 SppDataClient2ServerData[20];


// TM : to modify
attribute_t gAttributes_def[] = {
	{12,0,0,0,0,0},	//

	// gatt
	{7,2,2,2,(u8*)(&primaryServiceUUID), 	(u8*)(&gapServiceUUID), 0},
	{0,2,1,1,(u8*)(&characterUUID), 		(u8*)(&devNameCharacter), 0},
	{0,2,sizeof (ble_g_devName), sizeof (ble_g_devName),(u8*)(&devNameUUID), 			(u8*)(ble_g_devName), 0},
	{0,2,1,1,(u8*)(&characterUUID), 		(u8*)(&appearanceCharacter), 0},
	{0,2,sizeof (appearance), sizeof (appearance),(u8*)(&appearanceUIID), 	(u8*)(&appearance), 0},
	{0,2,1,1,(u8*)(&characterUUID), 		(u8*)(&periConnParamChar), 0},
	{0,2,sizeof (periConnParameters), sizeof (periConnParameters),(u8*)(&periConnParamUUID), 	(u8*)(&periConnParameters), 0},

	// spp
	{5,2,2,2,(u8*)(&primaryServiceUUID), 	(u8*)(&TelinkSppServiceUUID), 0},
	{0,2,1,1,(u8*)(&characterUUID), 		(u8*)(&SppDataServer2ClientProp), 0},				//prop
	{0,2,1,1,(u8*)(&TelinkSppDataServer2ClientUUID), 	(u8*)(SppDataServer2ClientData), 0},	//value
	{0,2,1,1,(u8*)(&characterUUID), 		(u8*)(&SppDataClient2ServerProp), 0},				//prop
	{0,2,16,16,(u8*)(&TelinkSppDataClient2ServiceUUID), 	(u8*)(SppDataClient2ServerData), 0},//value
};

void setSppWriteCB (att_readwrite_callback_t w)
{
	gAttributes_def[12].w = w;
}

attribute_t* gAttributes = 0;

void	blt_set_att_default ()
{
	gAttributes = gAttributes_def;
}

void blt_set_att_table (u8 *p)
{
	if (p) {
		gAttributes = (attribute_t *)p;
	}
}


void setSppAttTable (u8 *p)
{
	if (p) {
		gAttributes = (attribute_t *)p;
	}
	else {
		gAttributes = gAttributes_def;
	}
}

//u8 *pDevice2MasterData = SppDataClient2ServerData;

/////////////////////////////////////////////////////////////////////////
#define LL_FEATURE_REQ                              0x08
#define LL_FEATURE_RSP                              0x09
#define LL_VERSION_IND                              0x0C

// Hop & SCA mask
#define LL_CONNECT_REQ_HOP_MASK                     0x1F
#define LL_CONNECT_REQ_SCA_MASK                     0xE0

#define LL_RF_RESERVED_LEN                          4


// LL Header Bit Mask
#define LL_HDR_LLID_MASK                            0x03
#define LL_HDR_NESN_MASK                            0x04
#define LL_HDR_SN_MASK                              0x08
#define LL_HDR_MD_MASK                              0x10

#define LL_PDU_HDR_LLID_RESERVED                    0
#define LL_PDU_HDR_LLID_DATA_PKT_NEXT               1
#define LL_PDU_HDR_LLID_DATA_PKT_FIRST              2
#define LL_PDU_HDR_LLID_CONTROL_PKT                 3

// Macro to judgement the LL data type
#define IS_PACKET_LL_DATA(p)                        ((p & LL_HDR_LLID_MASK) != LL_PDU_HDR_LLID_CONTROL_PKT)
#define IS_PACKET_LL_CTRL(p)                        ((p & LL_HDR_LLID_MASK) == LL_PDU_HDR_LLID_CONTROL_PKT)
#define IS_PACKET_LL_INVALID(p)                     ((p & LL_HDR_LLID_MASK) == LL_PDU_HDR_LLID_RESERVED)

const rf_packet_feature_rsp_t	pkt_feature_rsp = {
		sizeof(rf_packet_feature_rsp_t) - 4,		// dma_len
		0x03,										// type
		sizeof(rf_packet_feature_rsp_t) - 6,		// rf_len
		LL_FEATURE_RSP,								// advA
		{1											// LL_FEATURE_MASK_CONNECTION_PARA_REQUEST_PROCEDURE
		,0,0,0,0,0,0,0}
};

const rf_packet_version_ind_t	pkt_version_ind = {
		sizeof(rf_packet_version_ind_t) - 4,		// dma_len
		0x03,										// type
		sizeof(rf_packet_version_ind_t) - 6,		// rf_len
		LL_VERSION_IND,								// advA
		2,
		0x544c,
		0x0008
};

rf_packet_att_errRsp_t pkt_errRsp = {
	sizeof(rf_packet_att_errRsp_t) - 4,		// dma_len
	0x02,									// type
	sizeof(rf_packet_att_errRsp_t) - 6,		// rf_len
	sizeof(rf_packet_att_errRsp_t) - 10,   	// l2cap_len
	4,
	ATT_OP_ERROR_RSP,
	0,0,ATT_ERR_ATTR_NOT_FOUND - ATT_ERR_START
};

const rf_packet_att_writeRsp_t pkt_writeRsp = {
	sizeof(rf_packet_att_writeRsp_t) - 4,		// dma_len
	0x02,										// type
	sizeof(rf_packet_att_writeRsp_t) - 6,		// rf_len
	sizeof(rf_packet_att_writeRsp_t) - 10,		// l2cap_len
	4, ATT_OP_WRITE_RSP,
};

const rf_packet_att_mtu_t pkt_mtu_rsp = {		//  spec 4.1 ,  3.4.7.1 Handle Value Notification
	sizeof(rf_packet_att_mtu_t) - 4,			// dma_len
	0x02,										// type
	sizeof(rf_packet_att_mtu_t) - 6,			// rf_len
	sizeof(rf_packet_att_mtu_t) - 10,			// rf_len
	4,
	ATT_OP_EXCHANGE_MTU_RSP,
	{L2CAP_MTU_SIZE,0}
};

rf_packet_att_readRsp_t rf_packet_att_rsp;

static inline int uuid_match(u8 uuidLen, u8* uuid1, u8* uuid2){
	if(2 == uuidLen  && uuid1[0] == uuid2[0] && uuid1[1] == uuid2[1]){
		return 1;
	}
	return 0;
}

static attribute_t* l2cap_att_search(u16 sh, u16 eh, u8 *attUUID, u16 *h){
	if(sh == gAttributes[0].attNum) return 0;	// ??????
	
	eh = eh < gAttributes[0].attNum ? eh : gAttributes[0].attNum;
	while(sh <= eh){
		attribute_t* pAtt = &gAttributes[sh];
		if(uuid_match(pAtt->uuidLen, pAtt->uuid, attUUID)){
			*h = sh;
			return pAtt;
		}
		++sh;
	}
	return 0;
}

u8 * l2cap_att_handler(u8 * p)
{
	if (!gAttributes)
		return 0;

	rf_packet_l2cap_req_t * req = (rf_packet_l2cap_req_t *)p;

	u8 * r = 0;
	if(IS_PACKET_LL_CTRL(req->type))  {
		if((req->l2capLen&0xff) == LL_VERSION_IND ){ 				// control packets
			return (u8 *)&pkt_version_ind;
		}
		else if ((req->l2capLen&0xff) == LL_FEATURE_REQ ){ 			// control packets
			return (u8 *)&pkt_feature_rsp;
		}
		else if ((req->l2capLen) == 0x1302 ){ 						// terminateion
			return -1;
		}
	}

	switch(req->opcode){
    case ATT_OP_READ_BY_GROUP_TYPE_REQ: {
		rf_packet_att_readByType_t *p = (rf_packet_att_readByType_t*)req;
		rf_packet_att_readByGroupTypeRsp_t *rsp = (rf_packet_att_readByGroupTypeRsp_t*)&rf_packet_att_rsp;
		attribute_t* pAtt;

		u16 sh = p->startingHandle, eh = p->endingHandle;
		u8 attUUID[2] = {p->attType[0], p->attType[1]};
		u16 i = 0;
		u8 attrLen = 0;
		while((pAtt = l2cap_att_search(sh, eh, attUUID, &sh))){
			if(attrLen && attrLen != pAtt->attrLen)		// 不能有两个不同的长度
				break;
			if((i*2) + pAtt->attrLen + 4 > ATT_MTU_SIZE)
				break;
			attrLen = pAtt->attrLen;

			rsp->data[i++] = sh;
			rsp->data[i++] = sh + (pAtt->attNum-1);
			memcpy(&rsp->data[i], pAtt->pAttrValue, pAtt->attrLen);
			i += (pAtt->attrLen / 2);
			sh += pAtt->attNum;
			if(sh > eh){
				break;
			}
		}
		if(i > 0){
			rsp->dma_len = (OFFSETOF(rf_packet_att_readByGroupTypeRsp_t, data) - 4) + (i*2);
			rsp->type = 2;
			rsp->rf_len = rsp->dma_len - 2;
			rsp->l2capLen = rsp->dma_len - 6;
			rsp->chanId = 4;
			rsp->opcode = ATT_OP_READ_BY_GROUP_TYPE_RSP;
			rsp->datalen = attrLen + 4;
			r = (u8 *)(rsp);
		}else{
			pkt_errRsp.errOpcode = ATT_OP_READ_BY_GROUP_TYPE_REQ;
			pkt_errRsp.errHandle = p->startingHandle;
//			pkt_errRsp.errReason = ATT_ERR_ATTR_NOT_FOUND - ATT_ERR_START;
			r = (u8 *)&pkt_errRsp;
		}
	}
	break;
	case ATT_OP_READ_BY_TYPE_REQ: {
		rf_packet_att_readByType_t *p = (rf_packet_att_readByType_t*)req;
		rf_packet_att_readByTypeRsp_t *rsp = (rf_packet_att_readByTypeRsp_t*)&rf_packet_att_rsp;
		attribute_t* pAtt;

		u16 sh = p->startingHandle, eh = p->endingHandle;
		u16 attUUID = (p->attType[1] << 8) | p->attType[0];
		u16 i = 0;
		u8 uuidLen = 0;
		switch(attUUID){
		case GATT_UUID_CHARACTER:
			while((pAtt = l2cap_att_search(sh, eh, (u8*)(&attUUID), &sh))){
				if(uuidLen && uuidLen != pAtt->uuidLen)		// 不能有两个不同的长度
					break;
				if(i + pAtt->uuidLen + 5 > ATT_MTU_SIZE)
					break;
				uuidLen = pAtt->uuidLen;

				rsp->data[i++] = sh++;
				rsp->data[i++] = 0;
				rsp->data[i++] = pAtt->pAttrValue[0];
				++pAtt;
				rsp->data[i++] = sh++;
				rsp->data[i++] = 0;
				memcpy(&rsp->data[i], pAtt->uuid, pAtt->uuidLen);
				i += pAtt->uuidLen;
			}
			break;
		case GATT_UUID_DEVICE_NAME:
			if((pAtt = l2cap_att_search(sh, eh, (u8*)(&attUUID), &sh))){
				rsp->data[0] = sh;
				rsp->data[1] = sh>>8;
				memcpy(&rsp->data[2], pAtt->pAttrValue, pAtt->attrLen);
				i = 2 + pAtt->attrLen;
			}
			break;
		}
		if(i > 0){
			rsp->dma_len = (OFFSETOF(rf_packet_att_readByGroupTypeRsp_t, data) - 4) + i;
			rsp->type = 2;
			rsp->rf_len = rsp->dma_len - 2;
			rsp->l2capLen = rsp->dma_len - 6;
			rsp->chanId = 4;
			rsp->opcode = ATT_OP_READ_BY_TYPE_RSP;
			rsp->datalen = uuidLen + 5;
			r = (u8 *)(rsp);
		}
		else{
			pkt_errRsp.errOpcode = ATT_OP_READ_BY_TYPE_REQ;
			pkt_errRsp.errHandle = sh;
//			pkt_errRsp.errReason = ATT_ERR_ATTR_NOT_FOUND - ATT_ERR_START;
			r = (u8 *)(&pkt_errRsp);
		}
	}
	break;
	case ATT_OP_FIND_INFO_REQ: {
		rf_packet_att_readByType_t *p = (rf_packet_att_readByType_t*)req;
		rf_packet_att_readByTypeRsp_t *rsp = (rf_packet_att_readByTypeRsp_t*)&rf_packet_att_rsp;
		attribute_t* pAtt;

		u16 sh = p->startingHandle;
		u16 eh = p->endingHandle < gAttributes[0].attNum ? p->endingHandle : gAttributes[0].attNum;
		int i = 0;
		while (sh <= eh ){
			if(i + 4 > ATT_MTU_SIZE)
				break;
			pAtt = &gAttributes[sh];
			rsp->data[i++] = sh;
			rsp->data[i++] = sh>>8;
			rsp->data[i++] = pAtt->uuid[0];
			rsp->data[i++] = pAtt->uuid[1];
			sh++;
		}
		if (i) {
			rsp->dma_len = 8 + i;
			rsp->type = 2;
			rsp->rf_len = rsp->dma_len - 2;			//10
			rsp->l2capLen = rsp->dma_len - 6;		//6
			rsp->chanId = 4;
			rsp->opcode = ATT_OP_FIND_INFO_RSP;
			rsp->datalen = 1;
			r = (u8 *)(rsp);
		}
		else{
			pkt_errRsp.errOpcode = ATT_OP_READ_BY_TYPE_REQ;
			pkt_errRsp.errHandle = sh;
//			pkt_errRsp.errReason = ATT_ERR_ATTR_NOT_FOUND - ATT_ERR_START;
			r = (u8 *)(&pkt_errRsp);
		}
	}
	break;
	case ATT_OP_WRITE_CMD:
	case ATT_OP_WRITE_REQ:{
		rf_packet_att_write_t *p = (rf_packet_att_write_t*)req;
		u8 h = p->handle;
		attribute_t *pAtt = &gAttributes[h];
		if(h <= gAttributes[0].attNum){
			if(ATT_OP_WRITE_REQ == req->opcode){
				r = (u8 *)(&pkt_writeRsp);
			}
			if(pAtt->w){
				pAtt->w(p);
			}else{
				if(p->l2capLen >= 3){
					u8 len = p->l2capLen - 3;
					pAtt->attrLen = len;
					memcpy(pAtt->pAttrValue, &p->value, len);
				}
			}
		}
	}
	break;
	case ATT_OP_READ_REQ:{
		rf_packet_att_read_t *p = (rf_packet_att_read_t*)req;
		u8 h = p->handle;
		attribute_t *pAtt = &gAttributes[h];
		if(h <= gAttributes[0].attNum){
			if(pAtt->r){
				pAtt->r(p);
			}else{
				rf_packet_att_readRsp_t *rsp = (rf_packet_att_readRsp_t*)&rf_packet_att_rsp;
				memcpy(rsp->value, pAtt->pAttrValue, pAtt->attrLen);
				rsp->type = 2;
				rsp->opcode = ATT_OP_READ_RSP;
				rsp->chanId = 4;
				rsp->l2capLen = pAtt->attrLen + 1;
				rsp->rf_len = rsp->l2capLen + 4;
				rsp->dma_len = rsp->rf_len + 2;
				r = (u8*)(rsp);
			}
		}
	}
	break;
	case ATT_OP_EXCHANGE_MTU_REQ:
		r = (u8*)(&pkt_mtu_rsp);
	break;
    default:
	break;
	}
	return r;
}
