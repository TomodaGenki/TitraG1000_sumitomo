/*
 * turn.c
 *
 *  Created on: 2019/05/17
 *      Author: ayabe
 */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "conf.h"
#include "turn.h"
#include "nuc.h"
#include "Cntrl_Proc.h"
#include "lift.h"
#include "common_func.h"

/* External variables --------------------------------------------------------*/
extern DAC_HandleTypeDef hdac;

/* Private define ------------------------------------------------------------*/
#define TURN_LOGSIZE 		1300	// ���O�_��
#define TURN_LOG_CYCLE		5		// ���O�v������(msec)
#define TURN_TICK_MS		1
#define TURN_TICK_SEC		0.001
#define	DAC_RESOLT_T		4095.0			// DAC resolution 12bit
#define MAX_VELOCTY_T		30000.0			// indication of speed
#define	TRN_UPPER_LIMIT		30000
#define	TRN_LOWER_LIMIT		120
#define	TURN_MOTOR_ALARM	0x01
#define TURN_RIGHT			1
#define TURN_LEFT			-1
#define TURNTBL_VEL2VOLT(av)	2.496 * (av) //�^�[���e�[�u���p���x���w���d�� �ϊ���
#define TURNTBL_VOLT2DAC(volt)	(uint16_t)((double)(4095.0 / 5.0) * (volt)) //�w���d����DAC�w���l�ւ̕ϊ���
#define TIM_CYCLE_TTBL		1.2195122	//�^�C�}�[�J�E���g�A�b�v����_�^�[���e�[�u���p(sec��10^6�{)
#define STOP_WAIT			500		// �u���[�L��ҋ@����
#define SLOW_SPEED			0.2		// �����œ������Ƃ��̑��x(rad/s)
#define TRGT_ANG_SENS		4.5		// �Z���T������A�Z���^�[�ɗ���܂ł̊p�x(deg)
#define ANG_ACCURACY		1.5		// ��~���x�̖ڕW�i���̊p�x�𒴂����烊�J�o���[������{)(deg)
#define RECOVER_SPEED		0.05	// ���J�o���[���쎞�̉�]���x(rad/s)
#define BRAKE_ANG_RECOVER	0.8		// ���J�o���[���쎞�̃u���[�L�J�n�p(deg)
#define RECOVER_ITR			2		// ���J�o���[���삷���
#define TURN_TIME_OUT		20000	// �^�C���A�E�g���鎞��(msec)
// �^�[���e�[�u����������X�e�[�^�X
#define	TURN_VEL_RUN		0x00	// ���x���䃂�[�h
#define	TURN_ANG_INIT		0x01	// �p�x���䃂�[�h������
#define TURN_ANG_RUN		0x02	// �p�x���䃂�[�h���s��
#define TURN_ANG_RUN_SLOW	0x03	// �p�x���䃂�[�h�ᑬ��]��
#define TURN_ANG_STOP		0x04	// �p�x���䃂�[�h�u���[�L��
#define TURN_ANG_STOP_OTHER	0x05	// �p�x���䃂�[�h�^�[�Q�b�g�ʒu�ȊO�ł̃u���[�L
#define TURN_ANG_RECOVER	0x06	// �p�x���䃂�[�h�ڕW�p�Ɏ��܂�Ȃ����̃��J�o���[����
#define TURN_ANG_COMP		0x07	// �p�x���䃂�[�h����I��
#define TURN_ANG_ERRCOMP	0x08	// �p�x���䃂�[�h�ُ�I��
// feedback control parameter
#define	FB_ABLE_ORDER		3000
#define	ORDER_UPPER_LIMIT	2.3
#define	AVE_TIMES			20
#define	MOTOR_GEAR			30.0
#define	TABLE_GEAR			7.66
#define	PULSE_PER_ROUND		12666.7
#define	ORDER_TO_RPM		10.0		// If order is 30000, Turn motor rpm is 3000rpm
#define	MINUTE_TO_SECOND	60.0
#define	TURN_KP				1.12
#define	TURN_KI				0.0000
#define	TURN_KD				0.14
// angle control mode parameter
#define TTBL_VEL_FIR		0.02		// �^�[���e�[�u���p���x�ɂ�����LPF���萔(sec)
#define TURN_FF_GAIN		1.0			// FF���f���̃Q�C��
#define TURN_FF_TAU_RISE	0.15		// FF���f���̎��萔(������)
#define TURN_FF_TAU_FALL	0.15		// FF���f���̎��萔(������)
#define TURN_FB_KP			0.3			// FB���Q�C��
#define TURN_FB_TD			0.3			// FB��������
#define	TURN_FB_ETA			0.2			// FB�����W��

/* Private variables ---------------------------------------------------------*/
static TURN_CTRL_PRAM *pTurn_Pram;
static TURN_CTRL_PRAM turn_90l_pram = {
		// 90�x�ᑬ
		89,		//1.��]��(deg)
		0.6,	//2.�ō���(rad/s)
		1.5,	//3.��������(sec)
		2.3,	//4.��������(sec)
		0.7,	//5.�u���[�L�J�n�p(deg)
		0.07	//6.�A�v���[�`���x(rad/s)
};
static TURN_CTRL_PRAM turn_90m_pram = {
		// 90�x����(����͒ᑬ�Ɠ���)
		89,		//1.��]��(deg)
		0.75,	//2.�ō���(rad/s)
		1.15,	//3.��������(sec)
		2.0,	//4.��������(sec)
		0.7,	//5.�u���[�L�J�n�p(deg)
		0.07	//6.�A�v���[�`���x(rad/s)
};
static TURN_CTRL_PRAM turn_90h_pram = {
		// 90�x����
		89,		//1.��]��(deg)
		0.9,	//2.�ō���(rad/s)
		0.8,	//3.��������(sec)
		1.7,	//4.��������(sec)
		0.7,	//5.�u���[�L�J�n�p(deg)
		0.07	//6.�A�v���[�`���x(rad/s)
};

static TURN_CTRL_PRAM turn_180l_pram = {
		// 180�x�ᑬ
		179,	//1.�^�[����(deg)
		0.6,	//2.�ō���(rad/s)
		1.5,	//3.��������(sec)
		2.5,	//4.��������(sec)
		0.7,	//5.�u���[�L�J�n�p(deg)
		0.07	//6.�A�v���[�`���x(rad/s)
};
static TURN_CTRL_PRAM turn_180m_pram = {
		// 180�x����(����͒ᑬ�Ɠ���)
		179,	//1.�^�[����(deg)
		0.9,	//2.�ō���(rad/s)
		1.35,	//3.��������(sec)
		2.25,	//4.��������(sec)
		0.7,	//5.�u���[�L�J�n�p(deg)
		0.07	//6.�A�v���[�`���x(rad/s)
};
static TURN_CTRL_PRAM turn_180h_pram = {
		// 180�x����
		179,	//1.�^�[����(deg)
		1.2,	//2.�ō���(rad/s)
		1.2,	//3.��������(sec)
		2.0,	//4.��������(sec)
		0.7,	//5.�u���[�L�J�n�p(deg)
		0.07	//6.�A�v���[�`���x(rad/s)
};

static TURN_CTRL_PRAM turn_360l_pram = {
		// 360�x�ᑬ
		360,	//1.�^�[����(deg)
		0.6,	//2.�ō���(rad/s)
		1.5,	//3.��������(sec)
		2.5,	//4.��������(sec)
		0.7,	//5.�u���[�L�J�n�p(deg)
		0.07	//6.�A�v���[�`���x(rad/s)
};
static TURN_CTRL_PRAM turn_360m_pram = {
		// 360�x����(����͒ᑬ�Ɠ���)
		360,	//1.�^�[����(deg)
		0.6,	//2.�ō���(rad/s)
		1.5,	//3.��������(sec)
		2.5,	//4.��������(sec)
		0.7,	//5.�u���[�L�J�n�p(deg)
		0.07	//6.�A�v���[�`���x(rad/s)
};
static TURN_CTRL_PRAM turn_360h_pram = {
		// 360�x����
		360,	//1.�^�[����(deg)
		1.2,	//2.�ō���(rad/s)
		1.2,	//3.��������(sec)
		2.0,	//4.��������(sec)
		0.7,	//5.�u���[�L�J�n�p(deg)
		0.07	//6.�A�v���[�`���x(rad/s)
};

static double sum_dif = 0;
static double last_speed_dif = 0;
static double before_last_speed_dif = 0;
static int16_t turn_enc_buff[AVE_TIMES];
static int16_t last_turn_order = 0;
static uint16_t turn_enc_old = 0;
static uint8_t t_point = 0;
static uint8_t turn_motor_alarm = 0;
static uint8_t	g_turn_status = TURN_VEL_RUN;
static uint8_t	flg_turn_send_comp = 0;
static uint8_t	g_dump_req = 0;
static uint8_t	g_dump_comp = 0;

static TRUN_VARIABLE turn_var;

#if TURN_LOG_DUMP
// �J���p���O�ϐ�
static uint16_t g_t_log_cnt = 0;
static uint16_t	g_t_log_idx = 0;
static uint16_t	g_t_time_log[TURN_LOGSIZE] 			= {0};
static double	g_t_trgt_angle_log[TURN_LOGSIZE] 	= {0};
static double	g_t_trgt_vel_log[TURN_LOGSIZE] 		= {0};
static double	g_t_trgt_acc_log[TURN_LOGSIZE] 		= {0};
static double	g_t_act_angle_log[TURN_LOGSIZE] 	= {0};
static double	g_t_act_vel_log[TURN_LOGSIZE] 		= {0};
static double	g_t_act_vel_lpf_log[TURN_LOGSIZE]	= {0};
static double	g_t_err_angle[TURN_LOGSIZE]			= {0};
static double	g_t_err_vel[TURN_LOGSIZE]			= {0};
static double	g_t_ref_vel[TURN_LOGSIZE]			= {0};
static double	g_t_ref_vel_ff[TURN_LOGSIZE]		= {0};
static double	g_t_ref_vel_fb[TURN_LOGSIZE]		= {0};
#endif


/******************************************************************************/
/*           define GPIO control											  */
/******************************************************************************/
void t_set_brake(void){
	HAL_GPIO_WritePin(O_TurnBrake_GPIO_Port, O_TurnBrake_Pin, GPIO_PIN_RESET);
}

void t_release_brake(void){
	HAL_GPIO_WritePin(O_TurnBrake_GPIO_Port, O_TurnBrake_Pin, GPIO_PIN_SET);
}

void set_rotation_cw(void) {
	HAL_GPIO_WritePin(O_TurnCCW_GPIO_Port, O_TurnCCW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(O_TurnCW_GPIO_Port, O_TurnCW_Pin, GPIO_PIN_SET);
}

void set_rotation_ccw(void) {
	HAL_GPIO_WritePin(O_TurnCCW_GPIO_Port, O_TurnCCW_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(O_TurnCW_GPIO_Port, O_TurnCW_Pin, GPIO_PIN_RESET);
}

void turn_relay_on(void) {
	HAL_GPIO_WritePin(O_TurnRelay_GPIO_Port, O_TurnRelay_Pin, GPIO_PIN_SET);
}

void turn_relay_off(void) {
	HAL_GPIO_WritePin(O_TurnRelay_GPIO_Port, O_TurnRelay_Pin, GPIO_PIN_RESET);
}

void turn_alarm_reset(void) {
	HAL_GPIO_WritePin(O_TurnCCW_GPIO_Port, O_TurnCCW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(O_TurnCW_GPIO_Port, O_TurnCW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(O_TurnAlarmReset_GPIO_Port, O_TurnAlarmReset_Pin, GPIO_PIN_SET);
}

void turn_alarm_recover(void) {
	HAL_GPIO_WritePin(O_TurnAlarmReset_GPIO_Port, O_TurnAlarmReset_Pin, GPIO_PIN_RESET);
}

void select_turn_stop_mode(uint8_t mode) {
	if (mode == Deceleration) {
		HAL_GPIO_WritePin(O_TurnStopMode_GPIO_Port, O_TurnStopMode_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(O_TurnStopMode_GPIO_Port, O_TurnStopMode_Pin, GPIO_PIN_RESET);
	}
}

void turn_motor_stop(void) {
	HAL_GPIO_WritePin(O_TurnCCW_GPIO_Port, O_TurnCCW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(O_TurnCW_GPIO_Port, O_TurnCW_Pin, GPIO_PIN_RESET);
}

/******************************************************************************/
/*           turn photo sensor check									      */
/******************************************************************************/
void turn_sensor_check(void) {

	static uint8_t turn_sens_buff[2] = {0,0};

	turn_sens_buff[1] = turn_sens_buff[0];
	turn_sens_buff[0] = 0;

	//Turn0degSens
	if (HAL_GPIO_ReadPin(I_Turn0degSens_GPIO_Port, I_Turn0degSens_Pin) == GPIO_PIN_RESET) {
		turn_sens_buff[0] |= TURN_ANG0_SENSOR;
	}
	//Turn90degSens
	if (HAL_GPIO_ReadPin(I_Turn90degSens_GPIO_Port, I_Turn90degSens_Pin) == GPIO_PIN_RESET) {
		turn_sens_buff[0] |= TURN_ANG90_SENSOR;
	}
	//Turn180degSens
	if(HAL_GPIO_ReadPin(I_Turn180degSens_GPIO_Port, I_Turn180degSens_Pin) == GPIO_PIN_RESET) {
		turn_sens_buff[0] |= TURN_ANG180_SENSOR;
	}
	//Turn270degSens
	if(HAL_GPIO_ReadPin(I_Turn270degSens_GPIO_Port, I_Turn270degSens_Pin) == GPIO_PIN_RESET) {
		turn_sens_buff[0] |= TURN_ANG270_SENSOR;
	}

	if (((turn_sens_buff[0] & TURN_ANG0_SENSOR) == TURN_ANG0_SENSOR) && ((turn_sens_buff[1] & TURN_ANG0_SENSOR) == TURN_ANG0_SENSOR)) {
		set_TurnAngle0_state();
	} else if (((turn_sens_buff[0] & TURN_ANG0_SENSOR) != TURN_ANG0_SENSOR) && ((turn_sens_buff[1] & TURN_ANG0_SENSOR) != TURN_ANG0_SENSOR)) {
		reset_TurnAngle0_state();
	}

	if (((turn_sens_buff[0] & TURN_ANG90_SENSOR) == TURN_ANG90_SENSOR) && ((turn_sens_buff[1] & TURN_ANG90_SENSOR) == TURN_ANG90_SENSOR)) {
		set_TurnAngle90_state();
	} else if (((turn_sens_buff[0] & TURN_ANG90_SENSOR) != TURN_ANG90_SENSOR) && ((turn_sens_buff[1] & TURN_ANG90_SENSOR) != TURN_ANG90_SENSOR)) {
		reset_TurnAngle90_state();
	}

	if (((turn_sens_buff[0] & TURN_ANG180_SENSOR) == TURN_ANG180_SENSOR) && ((turn_sens_buff[1] & TURN_ANG180_SENSOR) == TURN_ANG180_SENSOR)) {
		set_TurnAngle180_state();
	} else if (((turn_sens_buff[0] & TURN_ANG180_SENSOR) != TURN_ANG180_SENSOR) && ((turn_sens_buff[1] & TURN_ANG180_SENSOR) != TURN_ANG180_SENSOR)) {
		reset_TurnAngle180_state();
	}

	if (((turn_sens_buff[0] & TURN_ANG270_SENSOR) == TURN_ANG270_SENSOR) && ((turn_sens_buff[1] & TURN_ANG270_SENSOR) == TURN_ANG270_SENSOR)) {
		set_TurnAngle270_state();
	} else if (((turn_sens_buff[0] & TURN_ANG270_SENSOR) != TURN_ANG270_SENSOR) && ((turn_sens_buff[1] & TURN_ANG270_SENSOR) != TURN_ANG270_SENSOR)) {
		reset_TurnAngle270_state();
	}
}

/******************************************************************************/
/*           Turn motor alarm control function								  */
/******************************************************************************/
void scan_turnmotor_alarm(void) {

	static uint8_t turn_alarm_tmp = 0;

	turn_alarm_tmp = turn_alarm_tmp << 0x01;

	if (HAL_GPIO_ReadPin(I_TurnAlm_GPIO_Port, I_TurnAlm_Pin) == GPIO_PIN_SET) {
//		turn_alarm_tmp |= BIT00;
	}

	if (turn_alarm_tmp == 0xFF) {
		turn_motor_alarm |= TURN_MOTOR_ALARM;
	} else if (turn_alarm_tmp == 0x00) {
		turn_motor_alarm &= (~TURN_MOTOR_ALARM);
	}
}

uint8_t ck_turn_motor_alarm(void) {
	return turn_motor_alarm & TURN_MOTOR_ALARM;
}

/******************************************************************************/
/*           feedback controller										      */
/******************************************************************************/
void clr_fb_parameter(void) {

	for(int i = 0; i < AVE_TIMES; i++) {
		turn_enc_buff[i] = 0;
	}

	sum_dif = 0.0;
	last_speed_dif = 0.0;
	before_last_speed_dif = 0.0;
	t_point = 0;
	last_turn_order = 0;
	turn_enc_old = 0;
}

double convert_enc_to_speed(int16_t encorder) {	// unit : [rad/sec]

	double ret_speed;

	ret_speed = ((double)encorder / (double)PULSE_PER_ROUND) * 2.0 * M_PI;
	ret_speed = ret_speed / ((double)AVE_TIMES/1000.0);

	return ret_speed;
}

double convert_order_to_speed(int16_t speed_order) {	// unit : [rad/sec]

	double ret_speed;

	ret_speed = ((double)speed_order / (double)TRN_UPPER_LIMIT) * ORDER_UPPER_LIMIT;

	return ret_speed;
}

double calc_feedback(double target, double current) {

	double ret;
	double diff;

	diff = target - current;
	sum_dif += diff;

	ret = diff*TURN_KP + sum_dif*TURN_KI - (diff - 2*last_speed_dif + before_last_speed_dif)*TURN_KD;

	ret = ((double)ret / (double)ORDER_UPPER_LIMIT) * (double)TRN_UPPER_LIMIT;

	before_last_speed_dif = last_speed_dif;
	last_speed_dif = diff;

	return ret;
}

int16_t turn_feedback(int16_t turn_order) {

	double feedback = 0.0;
	double ret_speed = 0.0;
	double now_speed = 0.0;
	double last_order_speed = 0.0;
	uint16_t turn_enc = get_turn_encoder();
	int16_t turn_enc_diff = turn_enc - turn_enc_old;
	int16_t turn_enc_sum = 0;

	turn_enc_old = turn_enc;

	turn_enc_buff[t_point] = turn_enc_diff;
	t_point++;

	if (t_point >= AVE_TIMES) {
		t_point = 0;
	}
	for (int i = 0; i < AVE_TIMES; i++) {
		turn_enc_sum += turn_enc_buff[i];	// sum for 20msec
	}

	if (abs(turn_order) >= FB_ABLE_ORDER) {
		now_speed = convert_enc_to_speed(turn_enc_sum);
		last_order_speed = convert_order_to_speed(last_turn_order);
		feedback = calc_feedback(last_order_speed, now_speed);
	}
	ret_speed = (double)last_turn_order + (double)feedback;

	// update last value
	last_turn_order = turn_order;

	return (int16_t)ret_speed;
}

/******************************************************************************/
/*           Turn controller for Velocity mode							      */
/******************************************************************************/
void turn_init(void) {
	select_turn_stop_mode(Immediate);
	turn_motor_stop();
	clr_fb_parameter();
	set_TurnComplete_state();
}

uint16_t get_speed(int16_t speed) {

	double dac_cnv;
	uint16_t ret;

	// Upper Limit
	if (speed > TRN_UPPER_LIMIT) {	// if too high val then change maximum val
		speed = TRN_UPPER_LIMIT;
	}

	// Lower Limit
	if (speed < TRN_LOWER_LIMIT) {	// if too low val then change minimum val
		speed = TRN_LOWER_LIMIT;
	}

	dac_cnv = speed;
	dac_cnv = dac_cnv * DAC_RESOLT_T / MAX_VELOCTY_T;
	ret = (uint16_t)dac_cnv;

	return ret;
}

void turn_set_speed(uint16_t dac) {
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac);
}

void tmtr_set(int16_t speed_order) {

	uint16_t dac = 0;

	if (speed_order == 0) {
		turn_motor_stop();
		set_TurnComplete_state();
		if (check_turn_brake() != 0) {
			t_release_brake();
		} else {
			t_set_brake();
		}
	} else {
		set_TurnActive_state();
		t_release_brake();
		if (speed_order < 0) {
			speed_order *= -1;
			set_rotation_ccw();
		} else {
			set_rotation_cw();
		}
		dac = get_speed(speed_order);
		turn_set_speed(dac);
	}
}


void turn_cntrl_vel(void) {

	static int16_t tmotor_last_data = 0;
	int16_t tmotor_data;

	tmotor_data = get_TurnTable_Speed();
	// emergency stop?
	if ((ck_emerg_stop() != NOT_EMERGENCY) || (tmotor_data == 0) || (ck_motor_active() == 0)) {
		tmotor_data = 0;
		clr_fb_parameter();
	} else if (tmotor_data != 0) {
		//PID�͖����ɂ���
		//tmotor_data = turn_feedback(tmotor_data);
	}

	// �蓮�ł̃u���[�L�����̂��߁A0�̎��͏펞�R�[������
	if ((tmotor_data != tmotor_last_data) || (tmotor_data == 0)){
		tmtr_set(tmotor_data);
		tmotor_last_data = tmotor_data;
	}
}

/******************************************************************************/
/*           Turn controller for Angle mode								      */
/******************************************************************************/
void set_turn_flg_send_comp(){
	// �^�[���e�[�u�����䊮���X�e�[�^�X�𑗐M�����t���O���Z�b�g
	flg_turn_send_comp = 1;
}

void reset_turn_flg_send_comp(){
	flg_turn_send_comp = 0;
}

uint8_t get_turn_flg_send_comp(){
	return flg_turn_send_comp;
}

uint8_t get_turn_status() {
	// ����X�e�[�^�X��NUC�ʒm�p�X�e�[�^�X�ɕϊ����ĕԂ��֐�
	uint8_t ret = 0;

	if (g_turn_status == TURN_VEL_RUN){
		ret = ST_TURN_VEL_RUN;
	}
	else if((g_turn_status >= TURN_ANG_INIT) && (g_turn_status <= TURN_ANG_RECOVER)){
		ret = ST_TURN_ANG_RUN;
	}
	else if(g_turn_status == TURN_ANG_COMP){
		ret = ST_TURN_ANG_COMP;
	}
	else if(g_turn_status == TURN_ANG_ERRCOMP){
		ret = ST_TURN_ANG_ERRCOMP;
	}
	return ret;
}


static uint8_t get_turn_sensor_pos_ANG(){
	// �������Ă���Z���T�ʒu��1~4�ŕԂ�   �������Ȃ�0
	uint8_t sens = get_turn_sensor_pos();
	uint8_t ret = 0;
	if(sens == TRUN_POSI_ANG0){
		ret = TURN_0;
	}
	else if(sens == TRUN_POSI_ANG90){
		ret = TURN_90;
	}
	else if(sens == TRUN_POSI_ANG180){
		ret = TURN_180;
	}
	else if(sens == TRUN_POSI_ANG270){
		ret = TURN_270;
	}
	else{
		ret = 0;
	}
	return ret;
}


static uint8_t get_TurnTable_Mode(void){
	uint8_t ret = 0;
	if (get_RxCommand() == TURN_ANG_MODE){
		ret = TURN_ANG_MODE;
	}
	else{
		ret = TURN_VEL_MODE;
	}
	return ret;
}


static uint8_t get_TurnTableTrgt(void){
	// NUC����̊p�x�w���R�}���h�����߂���֐�
	// tsp�̏��8�r�b�g���p�x�w���R�}���h
	// TURN_NO_REQ		0x00	// �p�x�w���Ȃ�
	// TURN_0			0x01	// 0�x�̈ʒu�։�]
	// TURN_90			0x02	// 90�x�̈ʒu�։�]
	// TURN_180			0x03	// 180�x�̈ʒu�։�]
	// TURN_270			0x04	// 270�x�̈ʒu�։�]

	uint8_t ret = get_TurnTable_Speed() >> 8;

	return ret;
}


static uint8_t get_TurnTableSpdKind(void){
	// NUC����̑��x��ʎw�������߂���֐�
	// tsp�̉���8�r�b�g���p�x�w�����[�h�ł̑��x���
	// LOW		0x00	// �ᑬ�i�g���^���Ԃ܂ł̑傫���Ȃ�񂹂鑬�x�j
	// MID		0x01	// �����i�����̂��߂ɗp�Ӂ@����͒ᑬ�Ɠ������x�j
	// HIGH		0x02	// �����i�W���I�܂ł̑傫���Ȃ�񂹂鑬�x�j

	uint8_t ret = get_TurnTable_Speed() & 0xFF;

	return ret;
}


static uint8_t ck_illg_turn_com(uint8_t _target, uint8_t _spd_king, uint8_t _mode_com) {
	// NUC����̊p�x�w���R�}���h���s�����`�F�b�N����֐�
	uint8_t ret = 0;

	if (_mode_com != TURN_ANG_MODE) {
		// �s��
		ret = 1;
	}
	else {
		if ((TURN_0 <= _target) && (_target <= TURN_270) &&
			(TURN_LOW <= _spd_king) && (_spd_king <= TURN_HIGH)) {
			// ����
			ret = 0;
		}
		else{
			// �s��
			ret = 1;
		}
	}
	return ret;
}


static uint8_t ck_time_out(uint8_t _mode_com){
	// �^�C���A�E�g����p�̊֐�
	static uint16_t timer = 0;
	uint8_t ret = 0;
	if(_mode_com == TURN_ANG_MODE){
		timer += TURN_TICK_MS;
	}
	else{
		timer = 0;
	}

	if(timer >= TURN_TIME_OUT){
		ret = 1;
	}

	return ret;
}


static void variable_init(void){
	// ��ԗʂ̏�����
	turn_var.time		= 0;
	turn_var.cruise_end_time = 0;
	turn_var.flg_cruise_end = 0;
	turn_var.trgt_angle	= 0.0;
	turn_var.trgt_vel	= 0.0;
	turn_var.trgt_acc	= 0.0;
	turn_var.act_angle	= 0.0;
	turn_var.act_vel	= 0.0;
	turn_var.act_vel_old= 0.0;
	turn_var.act_vel_lpf= 0.0;
	turn_var.err_angle	= 0.0;
	turn_var.err_vel	= 0.0;
	turn_var.err_vel_old= 0.0;
	turn_var.err_vel_lpf= 0.0;
	turn_var.ref_vel	= 0.0;
	turn_var.ref_vel_ff	= 0.0;
	turn_var.ref_vel_fb	= 0.0;
	turn_var.d_term		= 0.0;
	turn_var.sens_angle = 0.0;
	turn_var.init_deff = 0.0;
	turn_var.result_angle = 0.0;
	turn_var.recover_cnt = 0;
}


static int8_t set_dir(uint8_t _trgt_pos, uint8_t _sens_pos){
	// �^�[�Q�b�g�܂ł̉�]�������v�Z����֐�

	int8_t dir = 0;
	int8_t diff = _trgt_pos - _sens_pos;
	if((diff == 3) || (diff == -1)){
		dir = TURN_LEFT;
	}
	else{
		dir = TURN_RIGHT;
	}
	return dir;
}


static void set_calib(uint8_t _trgt_pos, uint8_t _sens_pos, uint8_t _turn_spd_kind){
	// �^�[�Q�b�g�܂ł̉�]�ʂɉ����ăL�����u���[�V�����l���Z�b�g����֐�

	int8_t diff = _trgt_pos - _sens_pos;
	if(abs(diff) == 1 || abs(diff) == 3){
		switch(_turn_spd_kind){
		case TURN_LOW:
			pTurn_Pram = &turn_90l_pram;
			break;
		case TURN_MID:
			pTurn_Pram = &turn_90m_pram;
			break;
		case TURN_HIGH:
		default:
			pTurn_Pram = &turn_90h_pram;
		}
	}
	else if(abs(diff) == 2){
		switch(_turn_spd_kind){
		case TURN_LOW:
			pTurn_Pram = &turn_180l_pram;
			break;
		case TURN_MID:
			pTurn_Pram = &turn_180m_pram;
			break;
		case TURN_HIGH:
		default:
			pTurn_Pram = &turn_180h_pram;
		}
	}
	else if(abs(diff) == 0){
		switch(_turn_spd_kind){
		case TURN_LOW:
			pTurn_Pram = &turn_360l_pram;
			break;
		case TURN_MID:
			pTurn_Pram = &turn_360m_pram;
			break;
		case TURN_HIGH:
		default:
			pTurn_Pram = &turn_360h_pram;
		}
	}
}


static void ck_turn_angle(int8_t _turn_dir, uint8_t _sens, uint8_t _reset){
	// �G���R�[�_����p�x�Ɗp���x���Z�o����֐�
	// _reset = 1 �ŏ�����

	// �p�x�̎Z�o
	static int16_t turn_enc = 0;
	static int16_t turn_enc_cnt = 0;
	static int16_t turn_enc_cnt_diff = 0;
	int16_t turn_enc_tmp = 0;
	uint16_t diff_t = 0;
	turn_enc_tmp = turn_enc;
	turn_enc = get_turn_encoder();
	if(_reset > 0){
		turn_enc_cnt = 0;
	}
	else{
		diff_t = turn_enc - turn_enc_tmp;
		// �񂷕����𐳂Ƃ��邽�߁A�����v��莞�͕����𔽓]
		if (_turn_dir == TURN_LEFT){
			diff_t *= -1;
		}
		turn_enc_cnt += (int16_t)diff_t;
	}
	turn_var.act_angle = (2.0 * PI * (double)turn_enc_cnt) / TURNTBL_ENC;

	// �p���X�����ɂ��p���x�̎Z�o�ƃ��[�p�X�t�B���^����
	double ttbl_angle_per_pulse = 2 * PI / PULSE_PER_ROUND;
	double turn_cnt = (double)get_turn_enc_cnt();
	turn_var.act_vel_old = turn_var.act_vel;
	if((_reset > 0) || (turn_cnt < 0.01) || (turn_var.act_angle < 0.001)){
		turn_var.act_vel = 0;
		turn_var.act_vel_old = 0;
	}
	else{
		turn_var.act_vel = ttbl_angle_per_pulse * 1.0e6 / (TIM_CYCLE_TTBL * turn_cnt);
	}
	turn_var.act_vel_lpf = LPF_1order(turn_var.act_vel, turn_var.act_vel_old, turn_var.act_vel_lpf, TTBL_VEL_FIR, TURN_TICK_SEC);

	// �Z���T������̊p�x���v�Z(���ʏo�͗p)
	if(_sens == 0){
		turn_enc_cnt_diff = 0;
		turn_var.result_angle = 0.0;
	}
	else{
		turn_enc_cnt_diff += (int16_t)diff_t;
		turn_var.result_angle = (2.0 * PI * (double)turn_enc_cnt_diff) / TURNTBL_ENC;
	}
}


static void ck_turn_err_angle(){
	// �ڕW�O�Ղƌ��݂̊p�x�E�p���x�Ƃ̕΍����v�Z����֐�

	turn_var.err_angle = turn_var.trgt_angle - turn_var.act_angle;
	turn_var.err_vel_old = turn_var.err_vel;
	turn_var.err_vel = turn_var.trgt_vel - turn_var.act_vel;
	turn_var.err_vel_lpf = turn_var.trgt_vel - turn_var.act_vel_lpf;
}


static uint8_t ck_turn_arrive(uint8_t _target_pos, uint8_t _turn_sens_pos){
	// �ڕW�̃t�H�g�Z���T���������Ă��邩���肷��֐�

	uint8_t ret = 0;
	if (_target_pos == _turn_sens_pos){
		ret = 1;
	}
	return ret;
}


static uint8_t ck_turn_brk(uint8_t _flg_turn_arv, uint8_t _flg_turn_arv_old, double _act_turn_angle){
	// �^�[���e�[�u����~�^�C�~���O�𔻒肷��֐�
	static uint8_t flg_turn_sens_aprch = 0;
	static double turn_start_sens = 0.0;
	double sens_diff;
	uint8_t ret = 0;

	if ((_flg_turn_arv == 1) && (_flg_turn_arv_old == 0)) {
		flg_turn_sens_aprch = 1;
		turn_start_sens = _act_turn_angle;
		turn_var.sens_angle = _act_turn_angle;
	}
	else if (flg_turn_sens_aprch == 1) {
		sens_diff = fabs(_act_turn_angle) - fabs(turn_start_sens);
		if (sens_diff > ((TRGT_ANG_SENS - pTurn_Pram->brake_angle) * PI / 180.0)) {
			ret = 1;
			flg_turn_sens_aprch = 0;
			turn_start_sens = 0;
		}
	}

	return ret;
}


static double ck_init_diff(uint8_t _sens_pos, uint8_t _start_sens_pos, double _now_angle){
	// ��]�J�n���̊p�x���t�H�g�Z���T�̒��S���牽�x�Y���Ă����������o
	// ��]�����ւ̃Y���𐳂Ƃ���
	static double diff = 0;
	static uint8_t _sens_pos_old = 1;

	if((_sens_pos == 0) && (_sens_pos_old == _start_sens_pos)){
		diff = TRGT_ANG_SENS * PI / 180.0 - _now_angle;
	}
	_sens_pos_old = _sens_pos;

	return diff;
}


static uint8_t ck_turn_stop_time(uint16_t _wait_time, uint8_t _reset){
	// �w�莞�Ԍo�߂�����1��Ԃ��֐�
	// _reset = 1 �ŏ�����

	static uint16_t timer = 0;
	uint8_t ret = 0;

	if (_reset > 0) {
		timer = 0;
	}
	else {
		if (timer >= _wait_time){
			ret = 1;
		}
		else{
			timer += TURN_TICK_MS;
		}
	}

	return ret;
}


static void calc_turn_table_locus(uint16_t _timer){
	// �ڕW���x�A�����x�A���x���v�Z����֐�(�����g�ɂ��S���쓮)

	// �^�[���ʂƃ^�[�����Ԃ��琳���g�̃p�����[�^���Z�o
	double omega_acc = PI / pTurn_Pram->t_acc;
	double omega_decel = PI / pTurn_Pram->t_decel;
	double acc_dist = 0.5 * pTurn_Pram->v_max * pTurn_Pram->t_acc;
	double decel_dist = 0.5 * pTurn_Pram->t_decel * (pTurn_Pram->v_max - pTurn_Pram->approach_vel) + pTurn_Pram->t_decel * pTurn_Pram->approach_vel;
	double cruise_dist = pTurn_Pram->reach_angle * PI/ 180.0 - turn_var.init_deff - acc_dist - decel_dist;

	// �ڕW�O�Ղ��v�Z
	if ((_timer * 0.001) < pTurn_Pram->t_acc){
		// ������� (�^�C�}�[�Ő���)
		turn_var.trgt_vel =  -1.0 * pTurn_Pram->v_max * 0.5 * mysin(omega_acc * _timer * 0.001 + PI * 0.5) + pTurn_Pram->v_max * 0.5;
		turn_var.trgt_acc = 0.5 * pTurn_Pram->v_max * omega_acc * mysin(omega_acc * _timer * 0.001);
	}
	else if((turn_var.act_angle <= (acc_dist + cruise_dist)) && (turn_var.flg_cruise_end == 0)){
		// �������(�G���R�[�_�p���K��̊p�x�ɓ��B����܂Ōp��)
		turn_var.trgt_vel = pTurn_Pram->v_max;
		turn_var.trgt_acc = 0.0;
		turn_var.cruise_end_time = (double)_timer * 0.001;
	}
	else if ((_timer * 0.001) < (turn_var.cruise_end_time + pTurn_Pram->t_decel)){
		// ������� (�^�C�}�[�Ő���)
		turn_var.flg_cruise_end = 1;
		turn_var.trgt_vel = 0.5 * (pTurn_Pram->v_max - pTurn_Pram->approach_vel) * mysin(omega_decel * (_timer * 0.001 - turn_var.cruise_end_time) + PI * 0.5) + 0.5 * (pTurn_Pram->v_max + pTurn_Pram->approach_vel);
		turn_var.trgt_acc = -0.5 * (pTurn_Pram->v_max - pTurn_Pram->approach_vel) * omega_decel * mysin(omega_decel * (_timer * 0.001 - turn_var.cruise_end_time));
	}
	else{
		// ���������
		turn_var.trgt_vel = pTurn_Pram->approach_vel;
		turn_var.trgt_acc = 0.0;
	}
	// �p�x�͐��l�ϕ��ŎZ�o
	turn_var.trgt_angle += turn_var.trgt_vel * TURN_TICK_SEC;
}


static double calc_FF(double _trgt_vel, double _trgt_acc){
	// �t���f��(1���x��n)�������ăt�B�[�h�t�H���[�h�����v�Z����֐�

	double ret = 0.0;
	if (_trgt_acc >= 0){
		ret = 1.0 / TURN_FF_GAIN * (TURN_FF_TAU_RISE * _trgt_acc + _trgt_vel);
	}
	else{
		ret = 1.0 / TURN_FF_GAIN * (TURN_FF_TAU_FALL * _trgt_acc + _trgt_vel);
	}

	return ret;
}


static double calc_d_term(double _d_term_old, double _err, double _err_old, double _Td, double _eta){
	// ���ǌ`�o�ꎟ�ϊ���p�����s���S�����ɂ��PID��D�������Z�o����
	// d_term_old:d���̑O��l
	// err:�΍��i���x�j�̍���l
	// err_old:�΍�(���x)�̑O��l
	// Td:��������(=D�Q�C��/P�Q�C��)
	// eta:�����W��(0.1~0.125����)

	double delta_t = TURN_TICK_SEC;	//�������[sec]
	double d_term = _d_term_old + (2 * _Td * (_err - _err_old)) / (delta_t + 2 * _eta * _Td)  - (2 * delta_t * _d_term_old) / (delta_t + 2 * _eta * _Td);

	return d_term;
}


static void turn_cntrl_angle(int8_t _dir){
	// �^�[���e�[�u�����p�x���䃂�[�h�œ��������̑��x�w�����v�Z����֐�

	turn_var.ref_vel_ff = calc_FF(turn_var.trgt_vel, turn_var.trgt_acc);
	turn_var.d_term = calc_d_term(turn_var.d_term, turn_var.err_vel, turn_var.err_vel_old, TURN_FB_TD, TURN_FB_ETA);
	turn_var.ref_vel_fb = (turn_var.err_vel_lpf * TURN_FB_KP + TURN_FB_KP * turn_var.d_term);
	turn_var.ref_vel = turn_var.ref_vel_ff + turn_var.ref_vel_fb;

	if (turn_var.ref_vel < 0){
		turn_var.ref_vel = 0.0;
	}

	volatile double turn_volt = TURNTBL_VEL2VOLT(turn_var.ref_vel);
	uint16_t turntbl_motor_dac = TURNTBL_VOLT2DAC(turn_volt);

	if (_dir == TURN_RIGHT){
		set_rotation_cw();
	}
	else{
		set_rotation_ccw();
	}

	turn_set_speed(turntbl_motor_dac);
}


static void turn_cntrl_slow(int8_t _dir){
	// �t�H�g�Z���T���������Ƀ^�[���e�[�u��������œ������֐�

	volatile double turn_volt = TURNTBL_VEL2VOLT(SLOW_SPEED);
	uint16_t turntbl_motor_dac = TURNTBL_VOLT2DAC(turn_volt);

	if (_dir == TURN_RIGHT){
		set_rotation_cw();
	}
	else{
		set_rotation_ccw();
	}

	turn_set_speed(turntbl_motor_dac);
}


static void turn_cntrl_recover(int8_t _dir, double _now_angle, double _sens_agnle){
	// ���J�o���[����Ń^�[���e�[�u��������œ������֐�

	// ��]�����w��
	if ((_now_angle - _sens_agnle) >= (TRGT_ANG_SENS * PI / 180.0)){
		// ��]�ߑ��Ȃ̂ŋt�]������
		_dir *= -1;
	}
	else{
		// ��]�ߏ��Ȃ̂Ő��]������
	}
	if (_dir == TURN_RIGHT){
		set_rotation_cw();
	}
	else{
		set_rotation_ccw();
	}

	// ���x�w��
	volatile double turn_volt = TURNTBL_VEL2VOLT(RECOVER_SPEED);
	uint16_t turntbl_motor_dac = TURNTBL_VOLT2DAC(turn_volt);
	turn_set_speed(turntbl_motor_dac);
}


static void turn_log_init(){
	// ���O�ϐ���������
#if TURN_LOG_DUMP
	g_t_log_cnt = 0;
	g_t_log_idx = 0;
	for(int i = 0; i < TURN_LOGSIZE; i++){
		g_t_time_log[i]			= 0;
		g_t_trgt_angle_log[i]	= 0.0;
		g_t_trgt_vel_log[i]		= 0.0;
		g_t_trgt_acc_log[i]		= 0.0;
		g_t_act_angle_log[i]	= 0.0;
		g_t_act_vel_log[i]		= 0.0;
		g_t_act_vel_lpf_log[i]	= 0.0;
		g_t_err_angle[i]		= 0.0;
		g_t_err_vel[i]			= 0.0;
		g_t_ref_vel[i]			= 0.0;
		g_t_ref_vel_ff[i]		= 0.0;
		g_t_ref_vel_fb[i]		= 0.0;
	}
#endif
}


static void turn_log_update(){
	// ���䒆�̏�ԗʂ�ۑ�(�J���p)
#if TURN_LOG_DUMP
	if(g_t_log_cnt % TURN_LOG_CYCLE == 0){
		g_t_time_log[g_t_log_idx]		= turn_var.time;
		g_t_trgt_angle_log[g_t_log_idx] = turn_var.trgt_angle;
		g_t_trgt_vel_log[g_t_log_idx]	= turn_var.trgt_vel;
		g_t_trgt_acc_log[g_t_log_idx]	= turn_var.trgt_acc;
		g_t_act_angle_log[g_t_log_idx]	= turn_var.act_angle;
		g_t_act_vel_log[g_t_log_idx]	= turn_var.act_vel;
		g_t_act_vel_lpf_log[g_t_log_idx]= turn_var.act_vel_lpf;
		g_t_err_angle[g_t_log_idx]		= turn_var.err_angle;
		g_t_err_vel[g_t_log_idx]		= turn_var.err_vel;
		g_t_ref_vel[g_t_log_idx]		= turn_var.ref_vel;
		g_t_ref_vel_ff[g_t_log_idx]		= turn_var.ref_vel_ff;
		g_t_ref_vel_fb[g_t_log_idx]		= turn_var.ref_vel_fb;

		if (g_t_log_idx < (TURN_LOGSIZE - 1)){
			g_t_log_idx++;
		}
	}
	g_t_log_cnt++;
#endif
}


void turn_log_dump() {
	extern void uart2_transmitte(char *p);

	// �����p�Ƀ^�[�����ʂ��o��
	char buf1[40];
	// �t�H�g�Z���T���������Ă��邩
	memset(buf1, 0x00, sizeof(buf1));
	if (get_turn_sensor_pos_ANG() == 0){
		sprintf(buf1, "Turn table photo sensor: NG\r\n");
	}
	else{
		sprintf(buf1, "Turn table photo sensor: OK\r\n");
	}

	uart2_transmitte(buf1);

	// ��]���x
	memset(buf1, 0x00, sizeof(buf1));
	int16_t result_diff = (int16_t)((turn_var.result_angle * 180.0/ PI - TRGT_ANG_SENS) * 100.0);
	sprintf(buf1, "Turn table accuracy: %d [1/100deg]\r\n", result_diff);
	uart2_transmitte(buf1);

	// �^�[���e�[�u�����䒆�̏�ԗʃ��O���o��(�J���p)
#if TURN_LOG_DUMP
	char buf[600];

	memset(buf, 0x00, sizeof(buf));
	sprintf(buf, "time,trgt_angle,trgt_vel,trgt_acc,act_angle,act_vel,act_vel_lpf,"
			"err_angle,err_vel,ref_vel,ref_vel_ff,ref_vel_fb"
			"\r\n");
	uart2_transmitte(buf);
	for(int i = 0; i < TURN_LOGSIZE; i++) {
		memset(buf, 0x00, sizeof(buf));
		//             1   2   3   4   5   6   7   8   9  10  11  12
		sprintf(buf, "%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n",
				g_t_time_log[i],			// 1
				g_t_trgt_angle_log[i],		// 2
				g_t_trgt_vel_log[i],		// 3
				g_t_trgt_acc_log[i],		// 4
				g_t_act_angle_log[i],		// 5
				g_t_act_vel_log[i],			// 6
				g_t_act_vel_lpf_log[i],		// 7
				g_t_err_angle[i],			// 8
				g_t_err_vel[i],				// 9
				g_t_ref_vel[i],				// 10
				g_t_ref_vel_ff[i],			// 11
				g_t_ref_vel_fb[i]			// 12
				);
		uart2_transmitte(buf);
		if (i != 0 && g_t_time_log[i+1] == 0){
			break;
		}
	}
#endif
	set_turn_dump_comp();
}

void set_turn_dump_req(){
	g_dump_req = 1;
}

void reset_turn_dump_req(){
	g_dump_req = 0;
}

uint8_t get_turn_dump_req(){
	return g_dump_req;
}

void set_turn_dump_comp(){
	g_dump_comp = 1;
}

void reset_turn_dump_comp(){
	g_dump_comp = 0;
}

uint8_t get_turn_dump_comp(){
	return g_dump_comp;
}
/******************************************************************************/
/*           Turn main sequence											      */
/******************************************************************************/

void turn_cntrl(void) {
	// �^�[���e�[�u������̃��C���֐�
	static uint8_t mode_com = 0;
	static int8_t turn_dir = 0;
	static uint8_t flg_turn_brk = 0;
	static uint8_t flg_turn_arv = 0;
	static uint8_t flg_turn_arv_other = 0;
	static uint8_t start_sens_pos = 0;
	uint8_t mode_com_old = mode_com;
	uint8_t target_pos = get_TurnTableTrgt();
	uint8_t	turn_spd_kind = get_TurnTableSpdKind();
	uint8_t turn_sens_pos = get_turn_sensor_pos_ANG();
	uint8_t flg_turn_arv_old = 0;

	mode_com = get_TurnTable_Mode();
	uint8_t flg_time_out = ck_time_out(mode_com);

	switch(g_turn_status){

	case TURN_VEL_RUN:
		if(mode_com == TURN_VEL_MODE){
			turn_cntrl_vel();
		}
		else if((mode_com == TURN_ANG_MODE) && (mode_com_old != TURN_ANG_MODE)){
			g_turn_status = TURN_ANG_INIT;
		}
		break;

	case TURN_ANG_INIT:
		t_release_brake();
		variable_init();
		turn_log_init();
		ck_turn_angle(turn_dir, turn_sens_pos, 1);
		ck_turn_stop_time(STOP_WAIT, 1);
		start_sens_pos = turn_sens_pos;
		turn_dir = set_dir(target_pos, turn_sens_pos);
		set_calib(target_pos, turn_sens_pos, turn_spd_kind);
		flg_turn_arv = ck_turn_arrive(target_pos, turn_sens_pos);
		flg_turn_brk = 0;
		flg_turn_arv_other = 0;

		if((ck_emerg_stop() != NOT_EMERGENCY) || (ck_illg_turn_com(target_pos, turn_spd_kind, mode_com) != 0) || (flg_time_out == 1)){
			g_turn_status = TURN_ANG_ERRCOMP;
		}
		else if(turn_sens_pos == 0){
			g_turn_status = TURN_ANG_RUN_SLOW;
		}
		else{
			g_turn_status = TURN_ANG_RUN;
		}
		break;

	case TURN_ANG_RUN:
		// ���݂̊p�x���Z�o
		ck_turn_angle(turn_dir, turn_sens_pos, 0);
		// �J�n���̊p�x���Z�o
		turn_var.init_deff = ck_init_diff(turn_sens_pos, start_sens_pos, turn_var.act_angle);
		//�@�ڕW�O�Ղ��Z�o
		calc_turn_table_locus(turn_var.time);
		// �΍����Z�o
		ck_turn_err_angle();
		// ���[�^�[����
		turn_cntrl_angle(turn_dir);
		// �I������
		flg_turn_arv_old = flg_turn_arv;
		flg_turn_arv = ck_turn_arrive(target_pos, turn_sens_pos);
		flg_turn_brk = ck_turn_brk(flg_turn_arv, flg_turn_arv_old, turn_var.act_angle);
		turn_var.time += TURN_TICK_MS;
		turn_log_update();
		if((ck_emerg_stop() != NOT_EMERGENCY) || (ck_illg_turn_com(target_pos, turn_spd_kind, mode_com) != 0) || (flg_time_out == 1)){
			g_turn_status = TURN_ANG_ERRCOMP;
		}
		else if (flg_turn_brk == 1){
			g_turn_status = TURN_ANG_STOP;
		}
		break;

	case TURN_ANG_RUN_SLOW:
		// ���݂̊p�x���Z�o
		ck_turn_angle(turn_dir, turn_sens_pos, 0);
		// ���[�^�[����
		turn_cntrl_slow(turn_dir);
		// �I������
		flg_turn_arv_old = flg_turn_arv;
		if(ck_turn_arrive(target_pos, turn_sens_pos) == 1){
			// �^�[�Q�b�g�̃t�H�g�Z���T�����������ꍇ
			flg_turn_arv = 1;
		}
		else if(turn_sens_pos > 0){
			// �^�[�Q�b�g�ȊO�̃t�H�g�Z���T�����������ꍇ
			flg_turn_arv = 1;
			flg_turn_arv_other = 1;
		}
		flg_turn_brk = ck_turn_brk(flg_turn_arv, flg_turn_arv_old, turn_var.act_angle);
		if((ck_emerg_stop() != NOT_EMERGENCY) || (ck_illg_turn_com(target_pos, turn_spd_kind, mode_com) != 0) || (flg_time_out == 1)){
			g_turn_status = TURN_ANG_ERRCOMP;
		}
		else if ((flg_turn_brk == 1) && (flg_turn_arv_other != 1)){
			g_turn_status = TURN_ANG_STOP;
		}
		else if ((flg_turn_brk == 1) && (flg_turn_arv_other == 1)){
			g_turn_status = TURN_ANG_STOP_OTHER;
		}
		break;

	case TURN_ANG_STOP:
		ck_turn_angle(turn_dir, turn_sens_pos, 0);
		tmtr_set(0);
		if((ck_emerg_stop() != NOT_EMERGENCY) || (ck_illg_turn_com(target_pos, turn_spd_kind, mode_com) != 0) || (flg_time_out == 1)){
			g_turn_status = TURN_ANG_ERRCOMP;
		}
		else if (ck_turn_stop_time(STOP_WAIT, 0) == 1){
			ck_turn_stop_time(STOP_WAIT, 1);
			double angle_from_sens = turn_var.act_angle - turn_var.sens_angle;
			if (turn_var.recover_cnt >= RECOVER_ITR){
				// ���J�o���[����K��񐔏I����
				if(turn_sens_pos != 0){
					g_turn_status = TURN_ANG_COMP;
				}
				else{
					g_turn_status = TURN_ANG_ERRCOMP;
				}
			}
			else if ((angle_from_sens <= (TRGT_ANG_SENS + ANG_ACCURACY) * PI / 180.0)
				&& (angle_from_sens >= (TRGT_ANG_SENS - ANG_ACCURACY) * PI / 180.0)){
				// �ڕW�̐��x�Œ�~�ł����ꍇ
				g_turn_status = TURN_ANG_COMP;
			}
			else{
				// �ڕW�̐��x�Œ�~�ł��Ȃ������ꍇ
				g_turn_status = TURN_ANG_RECOVER;
			}
		}
		break;

	case TURN_ANG_STOP_OTHER:
		ck_turn_angle(turn_dir, turn_sens_pos, 0);
		tmtr_set(0);
		if((ck_emerg_stop() != NOT_EMERGENCY) || (ck_illg_turn_com(target_pos, turn_spd_kind, mode_com) != 0) || (flg_time_out == 1)){
			g_turn_status = TURN_ANG_ERRCOMP;
		}
		else if (ck_turn_stop_time(STOP_WAIT, 0) == 1){
			g_turn_status = TURN_ANG_INIT;
		}
		break;

	case TURN_ANG_RECOVER:
		// ���݂̊p�x���Z�o
		ck_turn_angle(turn_dir, turn_sens_pos, 0);
		// ���[�^�[����
		turn_cntrl_recover(turn_dir, turn_var.act_angle, turn_var.sens_angle);
		// �I������
		if((ck_emerg_stop() != NOT_EMERGENCY) || (ck_illg_turn_com(target_pos, turn_spd_kind, mode_com) != 0) || (flg_time_out == 1)){
			g_turn_status = TURN_ANG_ERRCOMP;
		}
		else if(fabs(fabs(turn_var.act_angle - turn_var.sens_angle) - TRGT_ANG_SENS * PI / 180.0) <= BRAKE_ANG_RECOVER * PI / 180.0){
			g_turn_status = TURN_ANG_STOP;
			turn_var.recover_cnt++;
		}
		break;

	case TURN_ANG_COMP:
	case TURN_ANG_ERRCOMP:
		tmtr_set(0);
		if (get_turn_flg_send_comp() == 1){
			g_turn_status = TURN_VEL_RUN;
			reset_turn_flg_send_comp();
		}
		break;

	default: //�����͒ʂ�Ȃ������[�j���O����̂��߂ɋL�q
		break;
	}
}

