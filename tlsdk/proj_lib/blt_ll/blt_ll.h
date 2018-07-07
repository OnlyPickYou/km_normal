
#include "../../proj/tl_common.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"

#include "ble_common.h"
#include "att.h"
#include "gatt_uuid.h"
#include "service.h"
#include "gatt_server.h"
#include "gap_const.h"
#include "blueLight.h"

#define TELINK_SPP_UUID_SERVICE   			0x1910		//!< TELINK_SPP service
#define TELINK_SPP_DATA_SERVER2CLIENT 		0x2B10 		//!< TELINK_SPP data from server to client
#define TELINK_SPP_DATA_CLIENT2SERVER 		0x2B11 		//!< TELINK_SPP data from client to server


#define			BLT_ENABLE_ADV_37			BIT(0)
#define			BLT_ENABLE_ADV_38			BIT(1)
#define			BLT_ENABLE_ADV_39			BIT(2)
#define			BLT_ENABLE_ADV_ALL			(BLT_ENABLE_ADV_37 | BLT_ENABLE_ADV_38 | BLT_ENABLE_ADV_39)

#define			BLT_LINK_STATE_ADV			0
#define			BLT_LINK_STATE_CONN			1
#define			BLT_LINK_STATE_CONNECTED	2


#define						BLT_BRX_CRC_ERROR			2
#define						BLT_BRX_CRC_OK				1
#define						BLT_BRX_TIMEOUT				0


extern u8				blt_state;
extern u32				blt_next_event_tick;

void	blt_init (u8 *p_mac, u8 *p_adv, u8 *p_rsp);

void	blt_adv_init ();

void	blt_set_channel (signed char chn);

int		blt_packet_tx_crc (u8 *pd, u8 *ps);

void	blt_get_crc_table (u8 *p, u32 *pt);

void	blt_connection_init (int crc, u8 * paccesscode);

void	blt_pn_init (int pn_init, int next);

void	blt_set_crc_init (int init);

void	blt_packet_tx_pn (u32 *pd, u32 *ps);

u8		blt_packet_rx (u32 timeout, int send_tx);

void	blt_set_adv (u8 *p, u8 *pr);

void	blt_set_adv_interval (u32 t_us);

u8*		blt_send_adv (int mask);

u8		blt_connect ();

u8		blt_brx ();

u8		blt_push_fifo (u8 *p);

void	blt_set_att_table (u8 *p);

void	blt_set_att_default ();
