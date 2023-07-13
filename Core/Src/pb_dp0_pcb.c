/*
 * pb_dp0_pcb.c
 *
 *  Created on: Jun 8, 2023
 *      Author: adamenko
 */

#include "pb_dp0_pcb.h"

uint8_t tmp = 0;

uint8_t checksum_calc(uint8_t* ptr_buf, uint8_t size) {

	uint8_t checksum = 0x00;

	for(int i = 0; i < size; i++) {

		checksum += ptr_buf[i];

	}

	return checksum;

}

uint8_t checksum_check(pb_telegram_t type) {

	pb_pcb.req_checksum = pb_pcb.req_buf[pb_pcb.req_size - 2]; /* - sum(ED, 1) */

	if(type == SD2) {

		pb_pcb.req_checksum_calc = checksum_calc((uint8_t*)&pb_pcb.req_buf[4], pb_pcb.req_size - 6 /* - sum(SD, LE, LEr, SDr, FCS, ED) */);

	} else if(type == SD1) {

		pb_pcb.req_checksum_calc = checksum_calc((uint8_t*)&pb_pcb.req_buf[1], pb_pcb.req_size - 3 /* - sum(SD, FCS, ED) */);

	}

	if(pb_pcb.req_checksum_calc == pb_pcb.req_checksum) {

		return CHECKSUM_OK;

	} else return CHECKSUM_ERR;

}

void pb_tx_config(uint8_t * ptr_buf, uint8_t size) {

	// dma tx config
	DMA2_Stream7->CR &= 0xFFFFFFFE;
	DMA2_Stream7->PAR = (uint32_t)&USART1->DR;
	DMA2_Stream7->M0AR = (uint32_t)ptr_buf;
	DMA2_Stream7->NDTR = size;

	DMA2->HIFCR |= 0xFFFFFFFF;
	DMA2->LIFCR |= 0xFFFFFFFF;

	DMA2_Stream7->CR |= 0x00000001; // TX EN

}

void pb_rx_config() {

	DMA2_Stream2->CR &= 0xFFFFFFFE;
	DMA2_Stream2->PAR = (uint32_t)&USART1->DR;
	DMA2_Stream2->M0AR = (uint32_t)&pb_pcb.req_buf;
	DMA2_Stream2->NDTR = sizeof(pb_pcb.req_buf);

	DMA2->HIFCR |= 0xFFFFFFFF;
	DMA2->LIFCR |= 0xFFFFFFFF;

	memset(pb_pcb.req_buf, 0, sizeof(pb_pcb.req_buf));

	DMA2_Stream2->CR |= 0x00000001; // RX EN

}


void pb_dp0_pcb() {

	if(pb_pcb.frame_available == true) {

		WDOG_update();

		switch(pb_pcb.telegram_type) {

			case SD2 : {

				pb_pcb.req_size = 256 - DMA2_Stream2->NDTR;

				if(checksum_check(SD2) == CHECKSUM_OK) {

					if((pb_pcb.access_type == CDXCH) && ((pb_pcb.req_buf[6] & SRD_HIGH) == SRD_HIGH) && ((pb_pcb.req_buf[6] & FCV) == FCV) ) {

						if( (pb_pcb.dev_cfg & INOUT) == INOUT ) {

							memcpy(pb_pcb.output_pData, &pb_pcb.req_buf[7], pb_pcb.data_len); // get data
							memcpy(pb_sd2_resp.PDU, pb_pcb.input_pData, pb_pcb.data_len); // send data

							pb_sd2_resp.LE = 3 + pb_pcb.data_len;
							pb_sd2_resp.LEr = 3 + pb_pcb.data_len;
							pb_sd2_resp.DA = pb_pcb.last_master_addr;
							pb_sd2_resp.SA = pb_pcb.addr;

							pb_pcb.checksum_resp = checksum_calc((uint8_t*)&pb_sd2_resp.DA, 3 + pb_pcb.data_len);

							pb_sd2_resp.PDU[pb_pcb.data_len] = pb_pcb.checksum_resp;
							pb_sd2_resp.PDU[pb_pcb.data_len + 1] = ED;

							// delay with new minTSDR
							minTSDR_stop();

							pb_rx_config();

							// start TX with size N + PDU byte ...
							pb_tx_config((uint8_t*)&pb_sd2_resp, 9 + pb_pcb.data_len);

							pb_pcb.data_ready_RW_callback();

						} else if( (pb_pcb.dev_cfg & OUT) == OUT ) {

							memcpy(pb_pcb.output_pData, &pb_pcb.req_buf[7], pb_pcb.data_len); // get data

							pb_sd1_resp.DA = pb_pcb.last_master_addr;
							pb_sd1_resp.SA = pb_pcb.addr;

							pb_sd1_resp.FCS = checksum_calc((uint8_t*)&pb_sd1_resp.DA, 3);

							// delay with new minTSDR
							minTSDR_stop();

							pb_rx_config();

							// start TX with size 6 byte ...
							pb_tx_config((uint8_t*)&pb_sd1_resp, 6);

							pb_pcb.data_ready_RW_callback();

						} else { // unknown configuration



						}

					} else if( (pb_pcb.access_type == SAP) && ((pb_pcb.req_buf[6] & SRD_HIGH) == SRD_HIGH) /*&& ((pb_pcb.req_buf[6] & FCV) == 0x00) */) {

						if( (pb_pcb.req_buf[7] == Slave_Diagnosis /* DSAP parse */) && (pb_pcb.req_buf[8] == Chk_Cfg /* SSAP parse */) ) { // get diagnostic request

							pb_sd2_resp.LE = 11;
							pb_sd2_resp.LEr = 11;
							pb_sd2_resp.DA = pb_pcb.req_buf[5];
							pb_sd2_resp.SA = pb_pcb.addr + init_addr_mask;
							pb_sd2_resp.PDU[0] = Chk_Cfg; // DSAP
							pb_sd2_resp.PDU[1] = Slave_Diagnosis; // SSAP

							pb_pcb.status1 = status1_base;
							pb_pcb.status2 = status2_base;

							if( (pb_pcb.sta_par == true) && (pb_pcb.sta_cfg == true) ) { // STA ready

								if(pb_pcb.wdog_on == true) {

									pb_pcb.status2 += wdog_on;
									WDOG_start(pb_pcb.WDOG);

								}

								pb_pcb.sta_ready = true;
								pb_pcb.dev_CDXCH_callback();

							} else {

								switch(pb_pcb.reason) {

									case prm_fault : { pb_pcb.status1 += prm_fault; } break;

									case cfg_fault : { pb_pcb.status1 += cfg_fault; } break;

									case not_sprtd : { pb_pcb.status1 += not_sprtd; } break;

									default : { pb_pcb.status1 += sta_nrdy; } break;

								}

								pb_pcb.status2 += prm_req;

							}

							pb_sd2_resp.PDU[2] = pb_pcb.status1;
							pb_sd2_resp.PDU[3] = pb_pcb.status2;
							pb_sd2_resp.PDU[4] = 0x00; // status3
							pb_sd2_resp.PDU[5] = pb_pcb.last_master_addr;
							pb_sd2_resp.PDU[6] = pb_pcb.ident_h;
							pb_sd2_resp.PDU[7] = pb_pcb.ident_l;

							pb_pcb.checksum_resp = checksum_calc((uint8_t*)&pb_sd2_resp.DA, 11);

							pb_sd2_resp.PDU[8] = pb_pcb.checksum_resp;
							pb_sd2_resp.PDU[9] = ED;

							// delay with default minTSDR = 11 Tbit
							minTSDR_stop();

							pb_rx_config();

							// start TX with size 17 byte ...
							pb_tx_config((uint8_t*)&pb_sd2_resp, 17);

						} else if( (pb_pcb.req_buf[7] == Set_Prm /* DSAP parse */) && (pb_pcb.req_buf[8] == Chk_Cfg /* SSAP parse */) ) { // Set Parameters Request

							pb_pcb.sta_par = true;
							pb_pcb.reason = status1_base;

							if( (pb_pcb.req_buf[9] & wdog_is_on) == wdog_is_on) pb_pcb.wdog_on = true;
							if( (pb_pcb.req_buf[9] & unlock_req) == unlock_req) pb_pcb.mstr_lock = false;
							else if( (pb_pcb.req_buf[9] & lock_req) == lock_req) pb_pcb.mstr_lock = true;
							else {

								pb_pcb.sta_par = false;
								pb_pcb.reason = not_sprtd;
							}

							pb_pcb.WDOG_factor1 = pb_pcb.req_buf[10];
							pb_pcb.WDOG_factor2 = pb_pcb.req_buf[11];

							pb_pcb.WDOG = pb_pcb.WDOG_factor1 * pb_pcb.WDOG_factor2 * 10;

							pb_pcb.minTSDR = pb_pcb.req_buf[12];
							pb_pcb.delay_factor_minTSDR = (pb_pcb.minTSDR * pb_pcb.tBit) / 1000;

							if( (pb_pcb.req_buf[13] != pb_pcb.ident_h) || (pb_pcb.req_buf[14] != pb_pcb.ident_l) ) {

								pb_pcb.sta_par = false;
								pb_pcb.reason = prm_fault;

							}

							if(pb_pcb.sta_par == true) pb_pcb.last_master_addr = pb_pcb.req_buf[5] & 0x7F;

							// delay with new minTSDR
							minTSDR_stop();

							pb_rx_config();

							// start TX with short response (SC) ...
							pb_tx_config((uint8_t*)&pb_short_resp, 1);

						} else if( (pb_pcb.req_buf[7] == Chk_Cfg /* DSAP parse */) && (pb_pcb.req_buf[8] == Chk_Cfg /* SSAP parse */) ) { // Configuration Request

							pb_pcb.sta_cfg = true;
							pb_pcb.reason = status1_base;

							if(pb_pcb.req_buf[9] != pb_pcb.dev_cfg) {

								pb_pcb.sta_cfg = false;
								pb_pcb.reason = cfg_fault;

							}

							// delay with minTSDR
							minTSDR_stop();

							pb_rx_config();

							// start TX with short response (SC) ...
							pb_tx_config((uint8_t*)&pb_short_resp, 1);

						} else { // unknown SAP



						}

					}

				} else { // CHECKSUM_ERR



				}

			} break;

			case SD1 : {

				pb_pcb.req_size = 256 - DMA2_Stream2->NDTR;

				if(checksum_check(SD1) == CHECKSUM_OK) {

					if( (pb_pcb.access_type == CDXCH) && ((pb_pcb.req_buf[3] & SRD_HIGH) == SRD_HIGH) && ((pb_pcb.req_buf[3] & FCV) == FCV) ) {

						if( (pb_pcb.dev_cfg & IN) == IN) {

							pb_sd2_resp.LE = 3 + pb_pcb.data_len;
							pb_sd2_resp.LEr = 3 + pb_pcb.data_len;
							pb_sd2_resp.DA = pb_pcb.last_master_addr;
							pb_sd2_resp.SA = pb_pcb.addr;

							memcpy(pb_sd2_resp.PDU, pb_pcb.input_pData, pb_pcb.data_len); // send data

							pb_pcb.checksum_resp = checksum_calc((uint8_t*)&pb_sd2_resp.DA, 3 + pb_pcb.data_len);

							pb_sd2_resp.PDU[pb_pcb.data_len] = pb_pcb.checksum_resp;
							pb_sd2_resp.PDU[pb_pcb.data_len + 1] = ED;

							// delay with new minTSDR
							minTSDR_stop();

							pb_rx_config();

							// start TX with size N + PDU byte ...
							pb_tx_config((uint8_t*)&pb_sd2_resp, 9 + pb_pcb.data_len);

							pb_pcb.data_ready_RW_callback();

						} else { // unknown configuration



						}

					} else if(pb_pcb.access_type == TOKEN) {



					}

				} else { // CHECKSUM_ERR



				}


			} break;

			default : {} break;

		}

		pb_pcb.frame_available = false;

	}

	if(pb_pcb.req_rx == true) {

		pb_pcb.req_rx = false;
		delay_us(pb_pcb.delay_factor_TXC);
		max485_rx;

	}

}

void transceiver_init() {

	USART1->CR1 |= (1<<4); // IDLE EN
	USART1->CR3 |= (1<<7)  // DMAT EN
		  	  	|  (1<<6); // DMAR EN

	DMA2->HIFCR |= 0xFFFFFFFF;
	DMA2->LIFCR |= 0xFFFFFFFF;

	DMA2_Stream7->CR |= (1<<4); // TCIE

}

/*
 * 	This function notify that master lost
 * 	You can use this function in your user code
 * 	STRONGLY RECOMENDED FAST REALISATION BECAUSE THIS FUCTION IS CALLED IN ISR!
 */

__weak void wdog_trig_cb() {



}

/*
 *	This function (dev_CDXCH_cb) notify that device ready for cycle data exchange
 * 	You can use this function in your user code
 * 	STRONGLY RECOMENDED FAST REALISATION BECAUSE THIS FUCTION IS CALLED IN RESPONSE FRAME PROCESSING!
 */

__weak void dev_CDXCH_cb() {



}

/*
 *	This function (data_ready_RW_cb) notify that one cycle data exchange complete
 * 	You can use this function in your user code
 * 	PROFIBUS PCB automatically copy input/output data to user added array,
 * 	this function notify that out data is valid and in this function you should push input data to user array.
 * 	You can push input data to user array any time, except interrupt.
 * 	If you want use interrupt you should check frame_available == false for push.
 * 	STRONGLY RECOMENDED FAST REALISATION BECAUSE THIS FUCTION IS CALLED IN RESPONSE FRAME PROCESSING!
 */

__weak void data_ready_RW_cb() {



}

uint8_t pb_dp0_pcb_init(struct slave_cfg* cfg, uint8_t* input_pData, uint8_t* output_pData) {

	if( (cfg->dev_cfg.len > 0) && (cfg->dev_cfg.len <= 16) ) {

		pb_pcb.minTSDR = DEFAULT_minTSDR;

		switch(cfg->speed) {

			/*
			 * 	For other MCUs or other MCU's speed you must calculate new division values for correctly UART work at any supported speeds
			 */

			case speed_9600 : {	USART1->BRR = 0x222E;
								pb_pcb.delay_factor_TXC = 2000;
								pb_pcb.tBit = tBit_9600;
								pb_pcb.delay_factor_minTSDR = (DEFAULT_minTSDR * tBit_9600) / 1000; } break;

			case speed_19200 : { USART1->BRR = 0x1117;
								 pb_pcb.delay_factor_TXC = 1270;
								 pb_pcb.delay_factor_minTSDR = (DEFAULT_minTSDR * tBit_19200) / 1000; } break;

			case speed_93750 : { USART1->BRR = 0x0380;
			 	 	 	 	 	 pb_pcb.delay_factor_TXC = 280;
								 pb_pcb.tBit = tBit_19200;
			 	 	 	 	 	 pb_pcb.delay_factor_minTSDR = (DEFAULT_minTSDR * tBit_93750) / 1000; } break;

			case speed_187500 : { USART1->BRR = 0x01C0;
				 	 	 	 	  pb_pcb.delay_factor_TXC = 145;
							 	  pb_pcb.tBit = tBit_187500;
				 	 	 	 	  pb_pcb.delay_factor_minTSDR = (DEFAULT_minTSDR * tBit_187500) / 1000; } break;

			case speed_500K: { USART1->BRR = 0x00A8;
			 	 	 	 	   pb_pcb.delay_factor_TXC = 65;
							   pb_pcb.tBit = tBit_500K;
			 	 	 	 	   pb_pcb.delay_factor_minTSDR = (DEFAULT_minTSDR * tBit_500K) / 1000; } break;

			case speed_1500K: { USART1->BRR = 0x0038;
								pb_pcb.delay_factor_TXC = 15;
								pb_pcb.tBit = tBit_1500K;
								pb_pcb.delay_factor_minTSDR = (DEFAULT_minTSDR * tBit_1500K) / 1000; } break;

			case speed_3M: { USART1->BRR = 0x001C;
							 pb_pcb.delay_factor_TXC = 10;
							 pb_pcb.delay_factor_minTSDR = (DEFAULT_minTSDR * tBit_3M) / 1000; } break;

			case speed_6M: { USART1->BRR = 0x000E;
			 	 	 	 	 pb_pcb.delay_factor_TXC = 5;
							 pb_pcb.tBit = tBit_6M;
			 	 	 	 	 pb_pcb.delay_factor_minTSDR = (DEFAULT_minTSDR * tBit_6M) / 1000; } break;

			case speed_12M: { USART1->BRR = 0x0007;
			 	 	 	 	  pb_pcb.delay_factor_TXC = 2;
			 	 	 	 	  pb_pcb.tBit = tBit_12M;
			 	 	 	 	  pb_pcb.delay_factor_minTSDR = (DEFAULT_minTSDR * tBit_12M) / 1000; } break;

		}

		transceiver_init();

		pb_pcb.wdog_trig_callback = &wdog_trig_cb;
		pb_pcb.dev_CDXCH_callback = &dev_CDXCH_cb;
		pb_pcb.data_ready_RW_callback = &data_ready_RW_cb;

		pb_pcb.input_pData = input_pData;
		pb_pcb.output_pData = output_pData;

		uint8_t dev_cfg = DEV_BASE;

		dev_cfg |= cfg->dev_cfg.dir;
		dev_cfg |= cfg->dev_cfg.size;
		dev_cfg |= (cfg->dev_cfg.len - 1);

		pb_pcb.dev_cfg = dev_cfg;
		if(cfg->dev_cfg.size == BYTE) pb_pcb.data_len = cfg->dev_cfg.len;
		else pb_pcb.data_len = cfg->dev_cfg.len * 2;

		pb_pcb.addr = cfg->addr;
		pb_pcb.speed = cfg->speed;
		pb_pcb.ident_l = cfg->ident;
		pb_pcb.ident_h = cfg->ident >> 8;

		pb_sd2_resp.SD = SD2;
		pb_sd2_resp.SDr = SD2;
		pb_sd2_resp.FC = DL;

		pb_sd1_resp.SD = SD1;
		pb_sd1_resp.FC = DL;
		pb_sd1_resp.ED = ED;

		pb_short_resp.SC = SC;

		pb_pcb.last_master_addr = DEFAULT_MASTER_ADDR;

		pb_rx_config();

		max485_rx;

		return 0;

	} else {

		return 1;

	}

}

void delay_us(uint32_t us) {

	TIM2->CR1 &= 0xFFFE;
	TIM2->CNT = 0;
	TIM2->CR1 |= 0x0001;

	while(TIM2->CNT <= us) {;;}

	TIM2->CR1 &= 0xFFFE;

}

