/*
 * can.h
 *
 *  Created on: 2021/11/10
 *      Author: Katte
 */

#ifndef CAN_H_
#define CAN_H_

/* function ----------------------------------------------------------*/
void can_filter_setting(void);
HAL_StatusTypeDef can1_transmit(void);
uint8_t can1_push_txmsg(uint16_t id, uint8_t *pdata);
/* define ------------------------------------------------------------*/
#define STORE_OK	0	// 送信バッファに格納成功
#define STORE_NG	1	// 送信バッファに格納失敗


#endif /* CAN */
