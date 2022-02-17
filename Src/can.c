/*
 * can.c
 *
 *  Created on: 2021/11/10
 *      Author: Katte
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "common_func.h"
#include "wheel.h"

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
/*
 * CAN���b�Z�[�W��M���ɃR�[���o�b�N�����֐�
 * ��M�f�[�^��ID���ƂɐU�蕪���ăX�g�A����
 */
	CAN_RxHeaderTypeDef   RxHeader;
	uint8_t	Rxdata_tmp[8] = {0,0,0,0,0,0,0,0};
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, Rxdata_tmp) == HAL_OK){
		switch(RxHeader.StdId){
//		�o�b�e���[�̏����@���������邽�߂Ɋ撣��Ȃ���
//		case 0x100:
//			memcpy(RxData_id100, Rxdata_tmp, sizeof(Rxdata_tmp));
//			break;
//		case 0x101:
//			memcpy(RxData_id101, Rxdata_tmp, sizeof(Rxdata_tmp));
//			break;

		case (SDO_RX_ID + L_WHEEL_ID):
			// �����s���[�^�[�h���C�o����̕ԓ�
			receive_wheel_motor_data(Rxdata_tmp, L_WHEEL_ID);
			break;

		case (SDO_RX_ID + R_WHEEL_ID):
			// �E���s���[�^�[�h���C�o����̕ԓ�
			receive_wheel_motor_data(Rxdata_tmp, R_WHEEL_ID);
			break;

		case (EMCY_ID + L_WHEEL_ID):
			// �����s���[�^�[�h���C�o����̃G�}�[�W�F���V�[���b�Z�[�W
			receive_wheel_motor_error_data(Rxdata_tmp, L_WHEEL_ID);
			break;

		case (EMCY_ID + R_WHEEL_ID):
			// �E���s���[�^�[�h���C�o����̃G�}�[�W�F���V�[���b�Z�[�W
			receive_wheel_motor_error_data(Rxdata_tmp, R_WHEEL_ID);
			break;

		case 0x701:
			// ���[�^�[�h���C�o����̃n�[�g�r�[�g
			break;
		}
	}
}

// CAN��M�t�B���^�̐ݒ�
void can_filter_setting(void) {

	// �SID����M����
	CAN_FilterTypeDef filter;
	filter.FilterIdHigh         = 0;                        // �t�B���^�[ID(���16�r�b�g)
	filter.FilterIdLow          = 0;                        // �t�B���^�[ID(����16�r�b�g)
	filter.FilterMaskIdHigh     = 0;                        // �t�B���^�[�}�X�N(���16�r�b�g)
	filter.FilterMaskIdLow      = 0;                        // �t�B���^�[�}�X�N(����16�r�b�g)
	filter.FilterScale          = CAN_FILTERSCALE_32BIT;    // �t�B���^�[�X�P�[��
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;         // �t�B���^�[�Ɋ��蓖�Ă�FIFO
	filter.FilterBank           = 0;                        // �t�B���^�[�o���NNo
	filter.FilterMode           = CAN_FILTERMODE_IDMASK;    // �t�B���^�[���[�h
	filter.SlaveStartFilterBank = 14;                       // �X���[�uCAN�̊J�n�t�B���^�[�o���NNo
	filter.FilterActivation     = ENABLE;                   // �t�B���^�[�����^�L��
	HAL_CAN_ConfigFilter(&hcan1, &filter);
}

// CAN���M�p�̊֐�
HAL_StatusTypeDef can1_transmit(uint16_t id, uint8_t *pdata) {

	static CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint32_t remain_box = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	HAL_StatusTypeDef result = HAL_OK;

	if (remain_box > 0) {
		// CAN�̑��MBOX�ɋ󂫂�����ꍇ�̂ݑ��M���s��
		// CAN�̐ݒ�
		TxHeader.StdId = id;			// CAN ID
		TxHeader.RTR = CAN_RTR_DATA;	// �f�[�^�t���[�����w��
		TxHeader.IDE = CAN_ID_STD;		// 11 bit ID (�W��ID)
		TxHeader.DLC = 8;				// Data Length 8Byte
		TxHeader.TransmitGlobalTime = DISABLE;

		result = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, pdata, &TxMailbox);
	} else {
		result = HAL_ERROR;
	}
	return result;
}

