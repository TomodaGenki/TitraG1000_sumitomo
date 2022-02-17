/*
 * wheel.c
 *
 *  Created on: 2021/03/28
 *      Author: Takumi
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "wheel.h"
#include "nuc.h"
#include "Cntrl_Proc.h"
#include "common_func.h"
#include "stdlib.h"
#include "stm32f4xx_hal_can.h"
#include "math.h"
#include "can.h"

/* External variables --------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
enum whl_sequence {
	Whl_PowerOn,
	Whl_Init,
	Whl_ParamSet,
	Whl_Run,
	Drv_Mode_Disable,
	Change_BrkMode_Manu,
	Whl_TorqueOff,
	Drv_Mode_Enable,
	Change_BrkMode_Auto,
	Whl_EmerStop,
};

enum Init_Setting {
	Set_DriveMode,
	Sw_On_Ready,
	Sw_On,
	Sw_On_Check,
	Drive_Enable,
	Drive_Enable_Check,
	Init_Num
};

enum Parameter_Setting {
	Set_Accel,
	Set_Decel,
	Set_Polarity,
	Param_Num
};

enum whl_can_data {
	Response,
	Index_Low,
	Index_High,
	Sub_Index,
	Data_Low,
	Data_Mdl_Low,
	Data_Mdl_High,
	Data_High,
	Whl_Data_Num
};

enum emcy_can_data {
	Code_Low,
	Code_High,
	Resister,
	Field_M0,
	Field_M1,
	Field_M2,
	Field_M3,
	Field_M4,
	Emcy_Data_Num
};

#define MAX_MOTOR_ROT		4500.0	// ���[�^�[��]���̍ő��]��4500rpm
#define MAX_VELOCTY_ORDER	30000.0	// 0 - 30000 �� 0 - 2.0m/s
#define MAX_VELOCTY			2000.0	// 2.0m/s
#define PULSE_PER_ROUND		16384.0
#define GEAR_RATIO_ECO_M	21.0
#define GEAR_RATIO_PRO_M	26.0
#define GEAR_RATIO			GEAR_RATIO_ECO_M
#define WHEEL_RAD			200.0

#define	DOWN_SLOPE		10
#define	LOW_SPDLIMIT	200
#define RESEND_WAIT		1000

#define MOTOR1_ALARM	0x01
#define MOTOR2_ALARM	0x02

#define ENC_VAL_MAX		2147483647

// CAN�֘A�̒�`

// Index���̒�`
#define DRIVE_MODE		0x6060		// �^�]���[�h
#define CNTRL_WORD		0x6040		// �R���g���[�����[�h
#define STATUS_WORD		0x6041		// �X�e�[�^�X���[�h
#define TARGET_SPEED	0x60FF		// �ڕW���x
#define ACCEL_PROFILE	0x6083		// �����x
#define DECEL_PROFILE	0x6084		// �����x
#define DEGITAL_IO		0x60FE
#define BRAKE_PARAM		0x3002		// �u���[�L�̃p�����[�^�ݒ�
#define ENCODER_OBJ		0x6064		// �G���R�[�_���
#define POLARITY_OBJ	0x607E		// �ɐ�

// Sub-Index���̒�`
#define SUB_INDEX_0		0x00
#define SUB_INDEX_1		0x01
#define SUB_INDEX_2		0x02
#define SUB_INDEX_3		0x03
#define SUB_INDEX_4		0x04
#define SUB_INDEX_5		0x05
#define SUB_INDEX_6		0x06
#define SUB_INDEX_7		0x07

// ���N�G�X�g���̒�`
#define WRITE_REQ_4B	0x23
#define WRITE_REQ_2B	0x2B
#define WRITE_REQ_1B	0x2F
#define WRITE_RSP		0x60
#define READ_REQ		0x40


// �p�����[�^�[�̒�`
#define SPD_PROFILE_MODE	0x03
#define SW_ON_READY			0x06
#define SW_ON				0x07
#define DRV_ENABLE			0x0F
#define DRV_DISABLE			0x07
#define ST_SW_ON			0x0002
#define ST_DRV_ENABLE		0x0004

#define ACCEL_RATE		((2387.0/60.0) * PULSE_PER_ROUND)
#define DECEL_RATE		((2387.0/60.0) * PULSE_PER_ROUND)
#define AUTO_BRAKE		0
#define MANUAL_BRAKE	1
#define BRAKE_ON		0x00000000
#define BRAKE_OFF		0x00000001
#define POLARITY_REV	0x40
#define READ_DATA		0

/* Private variables ---------------------------------------------------------*/
static WHL_MTR_DATA l_motor_data;
static WHL_MTR_DATA r_motor_data;
static uint8_t wheel_motor_alarm = 0;
static uint8_t wheel_seq = Whl_PowerOn;

/* Private functions -------------------------------------------------------- */
uint16_t ck_l_wheel_error(void);
uint16_t ck_r_wheel_error(void);

/******************************************************************************/
/*           Wheel motor GPIO control function								  */
/******************************************************************************/
void wheel_relay_off(void) {
	HAL_GPIO_WritePin(O_DriveRelay_GPIO_Port, O_DriveRelay_Pin, GPIO_PIN_RESET);// MOTOR POWER OFF
}

void wheel_relay_on(void) {
	HAL_GPIO_WritePin(O_DriveRelay_GPIO_Port, O_DriveRelay_Pin, GPIO_PIN_SET);// MOTOR POWER ON
}

void add_relay_on(void) {
	HAL_GPIO_WritePin(O_AddRelay_GPIO_Port, O_AddRelay_Pin, GPIO_PIN_SET);
}

void add_relay_off(void) {
	HAL_GPIO_WritePin(O_AddRelay_GPIO_Port, O_AddRelay_Pin, GPIO_PIN_RESET);
}

/******************************************************************************/
/*           Wheel motor alarm control function								  */
/******************************************************************************/
void scan_wheelmotor_alarm(void) {

	// motor1 driver alarm signal
	if(ck_l_wheel_error() != 0) {
		wheel_motor_alarm |= MOTOR1_ALARM;
	} else {
		wheel_motor_alarm &= (~MOTOR1_ALARM);
	}

	// motor2 driver alarm signal
	if(ck_r_wheel_error() != 0){
		wheel_motor_alarm |= MOTOR2_ALARM;
	} else {
		wheel_motor_alarm &= (~MOTOR2_ALARM);
	}
}

uint8_t ck_wheel1_motor_alarm(void) {
	return wheel_motor_alarm & MOTOR1_ALARM;
}

uint8_t ck_wheel2_motor_alarm(void) {
	return wheel_motor_alarm & MOTOR2_ALARM;
}

void reset_alarm(void){
	reset_motor1_alarm();
	reset_motor2_alarm();
}


void reset_motor1_alarm(void){
	HAL_GPIO_WritePin(O_Wheel1Reset_GPIO_Port, O_Wheel1Reset_Pin, GPIO_PIN_RESET);
}


void reset_motor2_alarm(void){
	HAL_GPIO_WritePin(O_Wheel2Reset_GPIO_Port, O_Wheel2Reset_Pin, GPIO_PIN_RESET);
}


void recover_alarm(void){
	recover_motor1_alarm();
	recover_motor2_alarm();
}

void recover_motor1_alarm(void) {
	HAL_GPIO_WritePin(O_Wheel1Reset_GPIO_Port, O_Wheel1Reset_Pin, GPIO_PIN_SET);
}


void recover_motor2_alarm(void) {
	HAL_GPIO_WritePin(O_Wheel2Reset_GPIO_Port, O_Wheel2Reset_Pin, GPIO_PIN_SET);
}

void reset_motor_alarm_auto(void){

}

/******************************************************************************/
/*           Wheel motor CAN control function								  */
/******************************************************************************/
HAL_StatusTypeDef transmit_motor_control_data(uint16_t id, uint8_t request, uint16_t index, uint8_t sub_index, int32_t data) {

	union LongByte tx_cnv;
	uint8_t tx_data[8];
	HAL_StatusTypeDef Result = HAL_OK;

	tx_cnv.l_val = 0;	//���p�̂̏�����

	// ���M�f�[�^�̐ݒ�
	tx_data[0] = request;

	tx_cnv.w_val[0] = index;
	tx_data[1] = tx_cnv.b_val[0];
	tx_data[2] = tx_cnv.b_val[1];

	tx_data[3] = sub_index;

	tx_cnv.l_val = data;
	tx_data[4] = tx_cnv.b_val[0];
	tx_data[5] = tx_cnv.b_val[1];
	tx_data[6] = tx_cnv.b_val[2];
	tx_data[7] = tx_cnv.b_val[3];

	Result = can1_transmit(id, tx_data);

	return Result;
}

// ���[�^�[�h���C�o����̎�M�f�[�^���e���[�^�[�̍\���̂֊i�[����
void receive_wheel_motor_data(uint8_t *receive_data, uint8_t l_r) {

	WHL_MTR_DATA *motor_data;
	union LongByte rx_cnv;
	uint16_t index;
	uint8_t sub_index;

	if (l_r == L_WHEEL_ID) {
		motor_data = &l_motor_data;
	} else {
		motor_data = &r_motor_data;
	}

	rx_cnv.l_val = 0;	//���p�̂̏�����

	rx_cnv.b_val[0] = receive_data[Index_Low];
	rx_cnv.b_val[1] = receive_data[Index_High];
	index = rx_cnv.w_val[0];
	sub_index = receive_data[Sub_Index];

	// �擾�����f�[�^�����p�̂Ɋi�[���Ă���
	for (int i = 0; i < 4; i++) {
		rx_cnv.b_val[i] = receive_data[Data_Low + i];
	}

	// ��M����index�ɂ���č\���̂ɏ����i�[����
	switch(index) {
	case STATUS_WORD:
		motor_data->whl_status = rx_cnv.w_val[0];
		break;

	case BRAKE_PARAM:
		if (sub_index == SUB_INDEX_5) {
			motor_data->brake_mode = rx_cnv.b_val[0];
		} else if (sub_index == SUB_INDEX_6) {
			motor_data->brake_status = rx_cnv.b_val[0];
		}
		break;

	case 0x60FE:

		break;

	case ENCODER_OBJ:
		motor_data->whl_encoder = rx_cnv.l_val;
		break;
	}
}

// ���[�^�[�h���C�o����̃G���[�����e���[�^�[�̍\���̂֊i�[����
void receive_wheel_motor_error_data(uint8_t *receive_data, uint8_t l_r) {

	WHL_MTR_DATA *motor_data;
	union LongByte rx_cnv;
	uint16_t err_code;

	if (l_r == L_WHEEL_ID) {
		motor_data = &l_motor_data;
	} else {
		motor_data = &r_motor_data;
	}

	rx_cnv.l_val = 0;	//���p�̂̏�����

	rx_cnv.b_val[0] = receive_data[Code_Low];
	rx_cnv.b_val[1] = receive_data[Code_High];
	err_code = rx_cnv.w_val[0];

	motor_data->whl_error = err_code;
}

uint16_t ck_l_wheel_switch_on(void) {
	return (l_motor_data.whl_status & ST_SW_ON);
}

uint16_t ck_r_wheel_switch_on(void) {
	return (r_motor_data.whl_status & ST_SW_ON);
}

uint16_t ck_l_wheel_drv_enable(void) {
	return (l_motor_data.whl_status & ST_DRV_ENABLE);
}

uint16_t ck_r_wheel_drv_enable(void) {
	return (r_motor_data.whl_status & ST_DRV_ENABLE);
}

uint8_t ck_l_wheel_brake_mode(void) {
	return l_motor_data.brake_mode;
}

uint8_t ck_r_wheel_brake_mode(void) {
	return r_motor_data.brake_mode;
}

uint8_t ck_l_wheel_brake_status(void) {
	return l_motor_data.brake_status;
}

uint8_t ck_r_wheel_brake_status(void) {
	return r_motor_data.brake_status;
}

uint16_t ck_l_wheel_error(void) {
	return l_motor_data.whl_error;
}

uint16_t ck_r_wheel_error(void) {
	return r_motor_data.whl_error;
}

uint32_t get_l_wheel_encoder(void) {
	return l_motor_data.whl_encoder;
}

uint32_t get_r_wheel_encoder(void) {
	return r_motor_data.whl_encoder;
}

HAL_StatusTypeDef set_drive_mode(uint8_t mode) {

	HAL_StatusTypeDef l_result = HAL_OK;
	HAL_StatusTypeDef r_result = HAL_OK;
	HAL_StatusTypeDef result = HAL_OK;

	l_result = transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_1B, DRIVE_MODE, SUB_INDEX_0, mode);
	r_result = transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_1B, DRIVE_MODE, SUB_INDEX_0, mode);

	if ((l_result != HAL_OK) || (r_result != HAL_OK)) {
		result = HAL_ERROR;
	}
	return result;
}

HAL_StatusTypeDef set_control_word(uint16_t mode) {

	HAL_StatusTypeDef l_result = HAL_OK;
	HAL_StatusTypeDef r_result = HAL_OK;
	HAL_StatusTypeDef result = HAL_OK;

	l_result = transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_2B, CNTRL_WORD, SUB_INDEX_0, mode);
	r_result = transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_2B, CNTRL_WORD, SUB_INDEX_0, mode);

	if ((l_result != HAL_OK) || (r_result != HAL_OK)) {
		result = HAL_ERROR;
	}
	return result;
}

void read_status_word(void) {
	transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), READ_REQ, STATUS_WORD, SUB_INDEX_0, READ_DATA);
	transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), READ_REQ, STATUS_WORD, SUB_INDEX_0, READ_DATA);
}

HAL_StatusTypeDef change_wheel_brake_mode(uint16_t mode) {
	// �u���[�L���[�h�̕ύX
	HAL_StatusTypeDef l_result = HAL_OK;
	HAL_StatusTypeDef r_result = HAL_OK;
	HAL_StatusTypeDef result = HAL_OK;

	l_result = transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_2B, BRAKE_PARAM, SUB_INDEX_5, mode);
	r_result = transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_2B, BRAKE_PARAM, SUB_INDEX_5, mode);

	if ((l_result != HAL_OK) || (r_result != HAL_OK)) {
		result = HAL_ERROR;
	}
	return result;
}

void read_brake_mode(void) {
	// �u���[�L���[�h�̊m�F
	transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), READ_REQ, BRAKE_PARAM, SUB_INDEX_5, READ_DATA);
	transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), READ_REQ, BRAKE_PARAM, SUB_INDEX_5, READ_DATA);
}

void reset_brake_mode(void) {
	// �u���[�L���[�h�� 0:�����A1:�蓮�̂��߁A�����I�Ƀ��Z�b�g���������߂�0xFF����
	l_motor_data.brake_mode = 0xFF;
	r_motor_data.brake_mode = 0xFF;
}

HAL_StatusTypeDef wheel_set_brake(uint32_t brake) {

	HAL_StatusTypeDef l_result = HAL_OK;
	HAL_StatusTypeDef r_result = HAL_OK;
	HAL_StatusTypeDef result = HAL_OK;

	l_result = transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_4B, DEGITAL_IO, SUB_INDEX_1, brake);
	r_result = transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_4B, DEGITAL_IO, SUB_INDEX_1, brake);

	if ((l_result != HAL_OK) || (r_result != HAL_OK)) {
		result = HAL_ERROR;
	}
	return result;
}

void read_brake_status(void) {
	// �u���[�L����������Ă��邩�m�F
	transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), READ_REQ, BRAKE_PARAM, SUB_INDEX_6, READ_DATA);
	transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), READ_REQ, BRAKE_PARAM, SUB_INDEX_6, READ_DATA);
}

void reset_brake_status(void) {
	// �u���[�L��Ԃ� 0:���b�N�A1:�A�����b�N�̂��߁A�����I�Ƀ��Z�b�g���������߂�0xFF����
	l_motor_data.brake_status = 0xFF;
	r_motor_data.brake_status = 0xFF;
}

/******************************************************************************/
/*           Wheel motor speed control function								  */
/******************************************************************************/
int32_t calc_wheel_speed(int16_t order) {

	double order_speed = ((double)order / MAX_VELOCTY_ORDER) * MAX_VELOCTY;	// 0 - 30000�̎w�߂�0 - 2000mm/s�̑��x�ɕϊ�
	double rotation_speed;
	double max_guard = (MAX_MOTOR_ROT/60.0) * PULSE_PER_ROUND;

	rotation_speed = ((order_speed / (WHEEL_RAD * M_PI)) * GEAR_RATIO);		// ���x(mm/s)�����[�^�[�̉�]��(round/sec)�ɕϊ�
	rotation_speed *= PULSE_PER_ROUND;	// r/s �����[�^�[�w�߂̒P��inc/s�ɕϊ�

	if (rotation_speed > max_guard) {
		// 4500rpm�ŏ���K�[�h(���[�^�[�̐��\���)
		rotation_speed = max_guard;
	}

	return (int32_t)rotation_speed;
}

void wheel_set_speed(int16_t left, int16_t right) {

	static int16_t left_old = 0xFFFF;
	static int16_t right_old = 0xFFFF;
	int32_t l_wheel_rot = calc_wheel_speed(left);
	int32_t r_wheel_rot = calc_wheel_speed(right);
	HAL_StatusTypeDef l_result = HAL_OK;
	HAL_StatusTypeDef r_result = HAL_OK;

	// �������e�̒ʐM�����x���s��Ȃ����߂̃K�[�h
	if (left != left_old) {
		l_result = transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_4B, TARGET_SPEED, SUB_INDEX_0, l_wheel_rot);
		if (l_result == HAL_OK) {
			left_old = left;
		}
	}

	if (right != right_old) {
		transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_4B, TARGET_SPEED, SUB_INDEX_0, r_wheel_rot);
		if (r_result == HAL_OK) {
			right_old = right;
		}
	}
}

/******************************************************************************/
/*           Wheel motor control function									  */
/******************************************************************************/
void monitor_wheel_encoder(void) {

	if (wheel_seq >= Whl_Run) {
		// �N�����������Ă��Ȃ��^�C�~���O�ł́A�G���R�[�_�[�����擾���ɂ����Ȃ�
		transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), READ_REQ, ENCODER_OBJ, SUB_INDEX_0, READ_DATA);
		transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), READ_REQ, ENCODER_OBJ, SUB_INDEX_0, READ_DATA);
	}
}

uint8_t wheel_initialize(void) {
	// DSP402�̏�ԋ@�B�ɏ]���A���s���[�^�[���N������

	static uint8_t init_set = Set_DriveMode;
	static uint8_t retry_cnt = 0;
	HAL_StatusTypeDef result = HAL_OK;
	uint8_t ret = 0;

	if (check_timer(CNT_WHEEL_WAIT) == TIME_UP) {
		set_utimer(CNT_WHEEL_WAIT, WHEEL_CAN_WAIT_TIME);

		if (init_set == Set_DriveMode) {
			// �^�]���[�h�𑬓x�v���t�@�C�����[�h�ɐݒ�
			result = set_drive_mode(SPD_PROFILE_MODE);
			if (result == HAL_OK) {
				init_set++;
			}
		} else if (init_set == Sw_On_Ready) {
			// �X�C�b�`ON���������̐ݒ�
			result = set_control_word(SW_ON_READY);
			if (result == HAL_OK) {
				init_set++;
			}
		} else if (init_set == Sw_On) {
			// �X�C�b�`ON�̐ݒ�
			result = set_control_word(SW_ON);
			if (result == HAL_OK) {
				init_set++;
			}
		} else if (init_set == Sw_On_Check) {
			//�X�C�b�`ON��Ԃ��m�F���邽�߂�Read�v���𑗐M
			read_status_word();
			if ((ck_l_wheel_switch_on()) && (ck_r_wheel_switch_on())) {
				init_set++;
				retry_cnt = 0;
			} else {
				retry_cnt++;
				if (retry_cnt > 5) {
					// ���񐔈ȏ�ǂݍ���ł��X�C�b�`ON��ԂƂȂ�Ȃ��ꍇ�A��蒼��
					init_set--;
					retry_cnt = 0;
				}
			}
		} else if (init_set == Drive_Enable) {
			// �^�]�L���̐ݒ�
			result = set_control_word(DRV_ENABLE);
			if (result == HAL_OK) {
				init_set++;
			}
		} else if (init_set == Drive_Enable_Check) {
			// �^�]�L����Ԃ��m�F���邽�߂�Read�v���𑗐M
			read_status_word();
			if ((ck_l_wheel_drv_enable()) && (ck_r_wheel_drv_enable())) {
				init_set++;
				retry_cnt = 0;
			} else {
				retry_cnt++;
				if (retry_cnt > 5) {
					// ���񐔈ȏ�ǂݍ���ł��X�C�b�`ON��ԂƂȂ�Ȃ��ꍇ�A��蒼��
					init_set--;
					retry_cnt = 0;
				}
			}
		} else if (init_set == Init_Num) {
			// �ݒ芮��
			ret = 1;
			init_set = Set_DriveMode;
		}
	}
	return ret;
}

// ���s���[�^�[�̃p�����[�^�[��ݒ肷��
uint8_t wheel_parameter(void) {

	static uint8_t parameter_set = Set_Accel;
	HAL_StatusTypeDef result = HAL_OK;
	uint8_t ret = 0;

	if (check_timer(CNT_WHEEL_WAIT) == TIME_UP) {
		set_utimer(CNT_WHEEL_WAIT, WHEEL_CAN_WAIT_TIME);

		if (parameter_set == Set_Accel) {
			// �����x��ݒ�
			transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_4B, ACCEL_PROFILE, SUB_INDEX_0, ACCEL_RATE);
			transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_4B, ACCEL_PROFILE, SUB_INDEX_0, ACCEL_RATE);
			parameter_set++;
		} else if (parameter_set == Set_Decel) {
			// �����x��ݒ�
			transmit_motor_control_data((SDO_TX_ID + L_WHEEL_ID), WRITE_REQ_4B, DECEL_PROFILE, SUB_INDEX_0, DECEL_RATE);
			transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_4B, DECEL_PROFILE, SUB_INDEX_0, DECEL_RATE);
			parameter_set++;
		} else if (parameter_set == Set_Polarity) {
			// �E���[�^�[�̋ɐ��𔽓]������
			result = transmit_motor_control_data((SDO_TX_ID + R_WHEEL_ID), WRITE_REQ_1B, POLARITY_OBJ, SUB_INDEX_0, POLARITY_REV);
			if (result == HAL_OK) {
				parameter_set++;
			}
		} else if (parameter_set == Param_Num) {
			// �ݒ芮��
			ret = 1;
			parameter_set = Set_Accel;
		}
	}
	return ret;
}

void wheel_init(void) {

}

void wheel_cntrl(int16_t left, int16_t right) {

	HAL_StatusTypeDef result = HAL_OK;

	switch(wheel_seq) {
	case Whl_PowerOn:
		// ���[�^�[�̋N���҂�
		if (ck_motor_active() != 0) {
			if (check_timer(CNT_WHEEL_WAIT) == TIME_UP) {
				wheel_seq = Whl_Init;
				set_utimer(CNT_WHEEL_WAIT, WHEEL_CAN_WAIT_TIME);
			}
		} else {
			// ���[�^�[�d��������ACAN���M���s���܂ő҂����Ԃ�����
			set_utimer(CNT_WHEEL_WAIT, WHEEL_MOTOR_WAKE_UP);
		}
		break;

	case Whl_Init:
		// ���[�^�[�̏�ԑJ�ڂɏ]���N�����s��
		if (wheel_initialize() != 0) {
			wheel_seq = Whl_ParamSet;
		}
		break;

	case Whl_ParamSet:
		// �p�����[�^�[�̐ݒ�
		if (wheel_parameter() != 0) {
			wheel_seq = Whl_Run;
		}
		break;

	case Whl_Run:
		// ����ԁ@���x�w���ɂ�蓮��
		if (ck_emerg_stop() == NOT_EMERGENCY) {
			if ((left == 0) && (right == 0)) {
				if (check_wheel_brake() != 0) {
					// �u���[�L�����X�C�b�`�������ꂽ�ꍇ�A�u���[�L���[�h���蓮�ɕύX���u���[�L����������
					// �^�]���[�h�𖳌��ɐݒ�
					result = set_control_word(DRV_DISABLE);
					if (result == HAL_OK) {
						set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
						wheel_seq = Drv_Mode_Disable;
					}
				}
			}
			wheel_set_speed(left, right);
		} else {
			wheel_set_speed(0, 0);
			wheel_seq = Whl_EmerStop;
		}
		break;

	case Drv_Mode_Disable:
		// �^�]���[�h�������ƂȂ��Ă��邱�Ƃ��m�F����
		if ((ck_l_wheel_drv_enable() == 0) && (ck_r_wheel_drv_enable() == 0)) {
			reset_brake_mode();
			// �u���[�L���[�h���蓮�ɐݒ�
			result = change_wheel_brake_mode(MANUAL_BRAKE);
			if (result == HAL_OK) {
				set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
				wheel_seq = Change_BrkMode_Manu;
			}
		} else {
			if (check_timer(CNT_WHEEL_WAIT) == TIME_UP) {
				// ��莞�Ԍo�߂��Ă��^�]���[�h�������ς��Ȃ��ꍇ�A�^�]���[�h�����w�߂��đ�����
				set_control_word(DRV_DISABLE);
				set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
			} else {
				read_status_word();
			}
		}
		break;

	case Change_BrkMode_Manu:
		// �u���[�L���[�h�������ς���Ă��邱�Ƃ��m�F
		if ((ck_l_wheel_brake_mode() == MANUAL_BRAKE) && (ck_r_wheel_brake_mode() == MANUAL_BRAKE)) {
			// �u���[�L���[�h���蓮�ɐݒ肳�ꂽ�̂��m�F��A�u���[�L����������
			reset_brake_status();
			result = wheel_set_brake(BRAKE_OFF);
			if (result == HAL_OK) {
				set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
				wheel_seq = Whl_TorqueOff;
			}
		} else {
			if (check_timer(CNT_WHEEL_WAIT) == TIME_UP) {
				// ��莞�Ԍo�߂��Ă��u���[�L���[�h�������ς��Ȃ��ꍇ�A�u���[�L���[�h�؂�ւ��w�߂��đ�����
				change_wheel_brake_mode(MANUAL_BRAKE);
				if (result == HAL_OK) {
					set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
				}
			} else {
				read_brake_mode();
			}
		}
		break;

	case Whl_TorqueOff:
		if (check_wheel_brake() == 0) {
			// �u���[�LON�w�߂������ꍇ�A�u���[�L���[�h�������ɐݒ肷��
			reset_brake_mode();
			result = change_wheel_brake_mode(AUTO_BRAKE);
			if (result == HAL_OK) {
				set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
				wheel_seq = Change_BrkMode_Auto;
			}
		} else if ((ck_l_wheel_brake_status() == BRAKE_OFF) && (ck_r_wheel_brake_status() == BRAKE_OFF)) {
			// �u���[�L�̉��������������ꍇ
			// ���ɉ������Ȃ�
		} else {
			if (check_timer(CNT_WHEEL_WAIT) == TIME_UP) {
				// ��莞�Ԍo�߂��Ă��u���[�L����������Ȃ��ꍇ�A�u���[�L�����w�߂��đ�����
				result = wheel_set_brake(BRAKE_OFF);
				if (result == HAL_OK) {
					set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
				}
			} else {
				read_brake_status();
			}
		}
		break;

	case Change_BrkMode_Auto:
		// �u���[�L���[�h�������ς���Ă��邱�Ƃ��m�F
		if ((ck_l_wheel_brake_mode() == AUTO_BRAKE) && (ck_r_wheel_brake_mode() == AUTO_BRAKE)) {
			// �^�]�L���̐ݒ�
			set_utimer(CNT_WHEEL_WAIT, WHEEL_CAN_WAIT_TIME);
			wheel_seq = Whl_Init;
		} else {
			if (check_timer(CNT_WHEEL_WAIT) == TIME_UP) {
				// ��莞�Ԍo�߂��Ă��u���[�L���[�h�������ς��Ȃ��ꍇ�A�u���[�L���[�h�؂�ւ��w�߂��đ�����
				result = change_wheel_brake_mode(AUTO_BRAKE);
				if (result == HAL_OK) {
					set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
				}
			} else {
				read_brake_mode();
			}
		}
		break;

	case Whl_EmerStop:
		if (ck_emerg_stop() == NOT_EMERGENCY) {
			wheel_seq = Whl_Run;
		} else if (check_wheel_brake() != 0) {
			result = set_control_word(DRV_DISABLE);
			if (result == HAL_OK) {
				set_utimer(CNT_WHEEL_WAIT, RESEND_WAIT);
				wheel_seq = Drv_Mode_Disable;
			}
		}
		break;
	}
}
