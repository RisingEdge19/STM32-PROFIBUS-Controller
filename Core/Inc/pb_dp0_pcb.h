/*
 * pb_dp0_pcb.h
 *
 *  Created on: Jun 8, 2023
 *      Author: adamenko
 */

#ifndef INC_PB_DP0_PCB_H_
#define INC_PB_DP0_PCB_H_

/*	PROFIBUS DP0 PROTOCOL CONTROL BLOCK INTRODUCTION
 *  - HW support speed up to 12Mbit with default minTSDR
 * 	- Response time ~30 us
 * 	- flexible short type configuration with consistency across the entire module
 * 	- not support global control
 *	- master lost and station ready CB functions
 */

#include "main.h"
#include <stdio.h>
#include <string.h>

/*
 * 	This macros must be modify for other pin. This pin use to switch RS485 transceiver to RX/TX
 */

#define max485_tx					GPIOA->BSRR |= (1<<8)	// set
#define max485_rx					GPIOA->BSRR |= (1<<24)	// reset

#define	init_addr_mask		 		(uint8_t) 0x80
#define DEFAULT_MASTER_ADDR			(uint8_t) 0xFF
#define DEFAULT_minTSDR				(uint8_t) 11

/*	dev_cfg:
* 	7: 1 - Consistency across the entire module, 0 - Consistency across one byte or one word
* 	6: 1 - Word(s) of 16 bits, 0 - Bytes of 8 bits
* 	5..4: 01 - input
* 		  10 - output
* 		  11 - input & output
* 	3..0: length of data (1..16)
*/

typedef enum {

	// all values in BRR calculated for max core/AHB frequency (168/84 MHz)

	speed_9600,				// DIV: M = 546, F = 14, BRR = 0x222E
	speed_19200,			// DIV: M = 273, F = 7, BRR = 0x1117
	speed_93750,			// DIV: M = 56, F = 0, BRR = 0x0380
	speed_187500,			// DIV: M = 28, F = 0, BRR = 0x01C0
	speed_500K,				// DIV: M = 10, F = 8, BRR = 0x00A8
	speed_1500K,			// DIV: M = 3, F = 8, BRR = 0x0038
	speed_3M,				// DIV: M = 1, F = 12, BRR = 0x001C
	speed_6M,				// DIV: M = 0, F = 14, BRR = 0x000E
	speed_12M,				// DIV: M = 0, F = 7, BRR = 0x0007


} pb_speed_t;

typedef enum {

	/*
	 *  tBit for specific baud rate in ns
	 */

	tBit_9600		=	(uint32_t) 105000,
	tBit_19200		=	(uint32_t) 53000,
	tBit_93750		=	(uint32_t) 11000,
	tBit_187500		=	(uint32_t) 6000,
	tBit_500K		=	(uint32_t) 2000,
	tBit_1500K		=	(uint32_t) 700,
	tBit_3M			=	(uint32_t) 350,
	tBit_6M			=	(uint32_t) 170,
	tBit_12M		=	(uint32_t) 100,


} pb_tBit_t;

typedef enum {

	IN		 = (uint8_t) 0x10,
	OUT 	 = (uint8_t) 0x20,
	INOUT	 = (uint8_t) 0x30,
	WORD	 = (uint8_t) 0x40,
	BYTE	 = (uint8_t) 0x00,
	DEV_BASE = (uint8_t) 0x80,

} dev_cfg_t;

struct dev_config {

	dev_cfg_t dir;
	dev_cfg_t size;
	uint8_t len;

} dev_config;

struct slave_cfg {

	uint8_t addr;
	pb_speed_t speed;
	uint16_t ident;
	struct dev_config dev_cfg;

} slave_cfg;

struct pb_sd2_resp {

	uint8_t SD; 		// start delimiter
	uint8_t LE;			// length
	uint8_t LEr;		// length repeat
	uint8_t SDr;		// start delimiter repeat
	uint8_t DA;			// destination address
	uint8_t SA;			// source address
	uint8_t FC;			// functional code
	uint8_t PDU[246];	// PDU array with: PDU[0] - DSAP, PDU[1] - SSAP, FCS (checksum) and ED (end delimiter)

} pb_sd2_resp;

struct pb_sd1_resp {

	uint8_t SD;
	uint8_t DA;
	uint8_t SA;
	uint8_t FC;
	uint8_t FCS;
	uint8_t ED;

} pb_sd1_resp;

struct pb_short_resp { // short response

	uint8_t SC;

} pb_short_resp;

typedef enum {

	false,
	true,

} bool;

typedef enum {

	CHECKSUM_ERR,
	CHECKSUM_OK,
	UNKNOWN_TLGRM_TYPE,

} pb_pcb_t;

typedef enum {

	SD1 				= (uint8_t) 0x10, // telegram without data field
	SD2 				= (uint8_t) 0x68, // telegram with variable length
	SD3 				= (uint8_t) 0xA2, // telegram with fixed data length
	SD4 				= (uint8_t) 0xDC, // token telegram
	SC  				= (uint8_t) 0xE5, // short confirmation
	ED  				= (uint8_t) 0x16, // end telegram

} pb_telegram_t;

typedef enum {

	Ext_User_Prm 		= (uint8_t) 0x35, // master->slave: Ext. parameter, slave->master: SC
	Set_Slave_Adr 		= (uint8_t) 0x37, // master->slave: address, slave->master: SC
	Rd_Inp 				= (uint8_t) 0x38, // master->slave: blank (data), slave->master: input data
	Rd_Outp				= (uint8_t) 0x39, // master->slave: blank (data), slave->master: output data
	Global_Control		= (uint8_t) 0x3A, // master->slave: control, slave->master: no
	Get_Cfg				= (uint8_t) 0x3B, // master->slave: blank (data), slave->master: data (configuration)
	Slave_Diagnosis		= (uint8_t) 0x3C, // master->slave: blank (data), slave->master: data (diagnosis)
	Set_Prm				= (uint8_t) 0x3D, // master->slave: data (parameters), slave->master: SC
	Chk_Cfg				= (uint8_t) 0x3E, // master->slave: data (configuration), slave->master: SC


} pb_sap_t;

typedef enum { // use via "&"

	req_base			= (uint8_t) 0b01000000, // is request telegram
	FCV					= (uint8_t) 0b00010000, // alternating bit switched on
	TE					= (uint8_t) 0b01000000, // Time Event (Clock synchronization)
	SDA_LOW				= (uint8_t) 0x03, 		// Send Data Acknowledged - low priority
	SDN_LOW				= (uint8_t) 0x04, 		// Send Data Not acknowledged - low priority
	SDA_HIGH			= (uint8_t) 0x05, 		// Send Data Acknowledged - high priority
	SDN_HIGH			= (uint8_t) 0x06, 		// Send Data Not acknowledged
	MSRD				= (uint8_t) 0x07, 		// Send Request Data with Multicast Reply
	FDL					= (uint8_t) 0x09, 		// Request FDL Status
	SRD_LOW				= (uint8_t) 0x0C, 		// Send and Request Data, low
	SRD_HIGH			= (uint8_t) 0x0D, 		// Send and Request Data, high
	req_ident			= (uint8_t) 0x0E, 		// Request Ident with reply
	req_LSAP			= (uint8_t) 0x0F, 		// Request LSAP Status with reply 1

} pb_req_fc_t_mask;

typedef enum { // use via "set/ |="

	resp_base 			= (uint8_t) 0x00,
	slave				= (uint8_t) 0b00000000, // slave
	mstr_nrdy			= (uint8_t) 0b00010000, // master not ready
	mstr_rdy_ntok		= (uint8_t) 0b00100000, // master ready, without token
	mstr_rdy_tok		= (uint8_t) 0b00110000, // master ready, in token ring
	OK					= (uint8_t) 0x00, 		// OK
	UE					= (uint8_t) 0x01, 		// user error
	RR					= (uint8_t) 0x02, 		// no resources
	RS					= (uint8_t) 0x03, 		// sap not enabled
	DL					= (uint8_t) 0x08, 		// data Low (normal case with DP)
	NR					= (uint8_t) 0x09, 		// no response data ready
	DH					= (uint8_t) 0x0A, 		// data High (DP diagnosis pending)
	RDL					= (uint8_t) 0x0C, 		// data not received and Data Low
	RDH					= (uint8_t) 0x0D, 		// data not received and Data High

} pb_res_fc_t;

typedef enum { // use via mask "&"

	wdog_is_on			= (uint8_t) 0x08,		// If this bit is set to 0, watchdog monitoring will be disabled.
	freeze_req			= (uint8_t) 0x10,		// If this bit is set to 0, watchdog monitoring will be disabled, use in global control (not supported)
	sync_req			= (uint8_t) 0x20,		// This bit indicates to the DP slave that it is to be operated in Freeze-Mode,, use in global control (not supported)

	unlock_req			= (uint8_t) 0x40,		// The DP master sets this bit to 1 when access to a DP slave is to be enabled again for another DP master.
												// This bit has priority over bit 7/ Lock_Req.

	lock_req			= (uint8_t) 0x80,		// The DP master sets this bit to 1 when other masters are to be blocked from accessing a DP slave.

} pb_par_req_sta_status_mask;

typedef enum {

	status1_base		= (uint8_t) 0x00,
	sta_nrdy			= (uint8_t) 0b00000010, // Slave is not ready for data exchange.
												// This bit is set by the DP slave if the DP slave is not yet ready for data exchange.

	cfg_fault			= (uint8_t) 0b00000100, // Fault in the configuration telegram.
												// This bit is set by the DP slave,
												// as soon as the configuration data received most recently from the master
												// does not match that detected by the DP slave.

	ext_diag			= (uint8_t) 0b00001000, // An extended diagnosis follows in the telegram.
												// This bit indicates that further diagnostic blocks follow  starting from byte 7.

	not_sprtd			= (uint8_t) 0b00010000, // Requested function is not supported by slave.
												// This bit is set by a slave, as soon as a function is requested that is not supported by this slave.

	prm_fault			= (uint8_t) 0b01000000, // Fault in parameter telegram.
												// This bit is set by the DP slave if the last parameter telegram was faulty.


} diag_response_status1;

typedef enum {

	status2_base		= (uint8_t) 0x04,
	prm_req				= (uint8_t) 0b00000001, // Slave parameters must be reset.
												// If the DP slave sets this bit, its parameters must be reset followed by reconfiguration.
												// The bit remains set until valid parameters have been implemented

	stat_diag			= (uint8_t) 0b00000010, // Status diagnostics.
												// If the DP slave sets this bit, the DP master must continue fetching diagnostic data until this bit is deleted again.
												// he DP slave sets this bit when, for example, it is not able to provide any valid user data.

	wdog_on				= (uint8_t) 0b00001000, // Watchdog on.
												// If this bit is set to 1, watchdog monitoring is enabled.

	freeze_mode			= (uint8_t) 0b00010000, // Freeze command received.
												// This bit is set by the DP slave as soon as this DP slave receives the Freeze command

	sync_mode			= (uint8_t) 0b00100000, // Sync command received.
												// This bit is set by the DP slave as soon as this DP slave receives the Sync command.

	deactive			= (uint8_t) 0b10000000, // Slave is deactivated.
												// This bit is always set to zero by a slave.
												// Here a master notices that this slave has been deactivated and should therefore no longer be controlled cyclically.

} diag_response_status2;

typedef enum {

	SAP,
	CDXCH,
	TOKEN,


} pb_access_t;

struct pb_pcb {

	uint8_t dev_cfg;					// type of device
	uint8_t data_len;					// length of data for CDXCH
	uint8_t addr;						// DP slave address
	pb_speed_t speed;					// UART baud rate
	uint8_t ident_h;					// ID number high byte
	uint8_t ident_l;					// ID number low byte
	pb_tBit_t tBit;						// one bit period, ns

	bool frame_available;				// unblock PROFIBUS DP0 protocol control block
	bool req_rx;						// request to enable receiver. YOU MUST SET TRUE AFTER TX COMPLETE WITH/WITHOUT SPECIFIC DELAY (delay_factor_TXC)
	bool sta_ready;						// station ready for cycle data exchange
	bool sta_par;						// station successfully get parameters
	bool sta_cfg;						// station successfully get configuration
	bool wdog_on;						// enable/disable watchdog monitoring
	bool mstr_lock;						// if set true other masters are to be blocked from accessing a DP slave.

	pb_telegram_t telegram_type;		// type of available telegram (SD1, SD2...)
	pb_access_t access_type;			// access type to PCB (cycle data exchange, sap routing...)
	diag_response_status1 reason;		// err reason while init sequence

	uint8_t req_size;					// size of request telegram
	uint8_t req_checksum;				// actual calculated checksum from request
	uint8_t req_checksum_calc;			// calculated checksum from request
	uint8_t status1;					// diagnosis 1st byte
	uint8_t status2;					// diagnosis 2nd byte
	uint8_t checksum_resp;				// calculated checksum for response
	uint8_t last_master_addr;			// Address of master after setting parameters

	uint8_t minTSDR;					// This is the minimum time that the DP slave must wait
										// until it is allowed to send its response telegrams back to the DP master. Must 0/11..255 Tbit

	uint8_t WDOG_factor1;				// The values stored in the two bytes represent factors for adjusting the watchdog time (TWD).
	uint8_t WDOG_factor2;				// In a DP slave, watchdog monitoring ensures that, if the DP master goes down,
										// outputs will assume the secure state after this period has elapsed.
	uint32_t WDOG;						// Calc: WDOG = WDOG_factor1 * WDOG_factor2 * 10 ms. Unit ms, must 10 ms to 650 sec

	uint8_t req_buf[256];				// available request data buffer

	uint8_t* input_pData;				// pointer to input user data
	uint8_t* output_pData;				// pointer to output user data

	uint32_t delay_factor_TXC;			// delay time before switch RS485 to RX after transaction complete, us
	uint32_t delay_factor_minTSDR;		// delay time after start response (minTSDR), us

	void (*wdog_trig_callback)();		// watchdog triggered callback function for user notify
	void (*dev_CDXCH_callback)();		// device ready for cycle data exchange callback
	void (*data_ready_RW_callback)();	// one cycle data exchange complete

} pb_pcb;

uint8_t checksum_calc(uint8_t* ptr_buf, uint8_t size);
uint8_t checksum_check(pb_telegram_t type);
uint8_t pb_dp0_pcb_init(struct slave_cfg* cfg, uint8_t* input_pData, uint8_t* output_pData);
void pb_dp0_pcb();

/*
 * 	This function (transceiver_init) must be modify for other MCUs with next rules:
 * 	You must initialize UART in this function. This UART will be used in PROFIBUS protocol control block (pb_pcb)
 *
 */
void transceiver_init();

/*
 * 	This function (pb_tx_config) must be modify for other MCUs with next rules:
 * 	Pass the prepared buffer (pb_sd2_resp/pb_sd1_resp) via UART transmitter.
 * 	You can use DMA transaction or handled/ISR byte by byte transaction.
 */
void pb_tx_config(uint8_t * ptr_buf, uint8_t size);

/*
 * 	This function (pb_rx_config) must be modify for other MCUs with next rules:
 * 	Enable receiving bytes into the specific buffer (pb_pcb.req_buf) from the URAT receiver.
 * 	You can use DMA transaction or handled/ISR byte by byte receiving.
 */
void pb_rx_config();

/*
 * 	This function (delay_us) must be modify for other MCUs with next rules:
 * 	Block function for a while delay time in us.
 * 	You must use any timer with 1 tick period of 1 us.
 */
void delay_us(uint32_t us);

/*
 * 	This function (minTSDR_start) must be modify for other MCUs with next rules:
 * 	Start HW timer with tick rate in us.
 */
static __inline void minTSDR_start() {

	TIM2->CR1 &= 0xFFFE;
	TIM2->CNT = 0;
	TIM2->CR1 |= 0x0001;

}

/*
 * 	This function (minTSDR_stop) must be modify for other MCUs with next rules:
 * 	Block function for a while HW timer time elapsed.
 *
 */
static __inline void minTSDR_stop() {

	while(TIM2->CNT <= pb_pcb.delay_factor_minTSDR) {;;}

	TIM2->CR1 &= 0xFFFE;
	TIM2->CNT = 0;

}

/*
 * 	This function (WDOG_start) must be modify for other MCUs with next rules:
 * 	Start WDOG HW timer in one pulse mode.
 *
 */
static __inline void WDOG_start(uint32_t wdog_period) {

	TIM5->CR1 &= 0xFFFE;
	TIM5->CNT = 0;
	TIM5->DIER = 0;
	TIM5->DIER = 0x0001;
	TIM5->SR = 0;
	TIM5->ARR = wdog_period * 10;
	TIM5->CR1 |= 0x0001;

}

/*
 * 	This function (WDOG_stop) must be used for other MCUs with next rules:
 * 	This function notify that master lost.
 * 	This function must be use in timer update interrupt, in one pulse mode.
 *
 */
static __inline void WDOG_stop() {

	//pb_pcb.sta_ready = false;
	//pb_pcb.sta_par = false;
	pb_pcb.wdog_trig_callback();

}

/*
 * 	This function (WDOG_update) must be modify for other MCUs with next rules:
 * 	You must clear WDOG HW timer counter register
 *
 */
static __inline void WDOG_update() {

	TIM5->CNT = 0;

}

/*
 * 	This function (pb_dp0_pcb_unlock) must be used for other MCUs with next rules:
 * 	You must use this function after complete frame receiving.
 *	You can use IDLE interrupt with frame error and brake detection.
 */
static __inline void pb_dp0_pcb_unlock() {

	if( (pb_pcb.req_buf[0] == SD2) && (pb_pcb.req_buf[1] == pb_pcb.req_buf[2]) )  { // SD2 processing

		if(pb_pcb.req_buf[4] == pb_pcb.addr) { // cycle data exchange

			max485_tx;

			minTSDR_start();

			pb_pcb.telegram_type = SD2;
			pb_pcb.access_type = CDXCH;
			pb_pcb.frame_available = true;

		} else if(pb_pcb.req_buf[4] == pb_pcb.addr + init_addr_mask) { // SAP routing

			max485_tx;

			minTSDR_start();

			pb_pcb.telegram_type = SD2;
			pb_pcb.access_type = SAP;
			pb_pcb.frame_available = true;

		} else { // unknown address

			pb_rx_config();

		}

	} else if( (pb_pcb.req_buf[0] == SD1) && (pb_pcb.req_buf[1] == pb_pcb.addr) ) { // SD1 processing

		if( ((pb_pcb.req_buf[3] & FDL) == FDL) && ((pb_pcb.req_buf[3] & SRD_HIGH) != SRD_HIGH) ) { // token processing

//				pb_pcb.telegram_type = SD1;
//				pb_pcb.access_type = TOKEN;
//				pb_pcb.frame_available = true;

			pb_rx_config();

		} else if(pb_pcb.sta_ready == true) { // cycle data exchange

			max485_tx;

			minTSDR_start();

			pb_pcb.telegram_type = SD1;
			pb_pcb.access_type = CDXCH;
			pb_pcb.frame_available = true;

		}

	} else { // unknown telegram

		pb_rx_config();

	}

}


#endif /* INC_PB_DP0_PCB_H_ */
