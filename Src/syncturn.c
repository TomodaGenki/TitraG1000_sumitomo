/*

 * syncturn.c
 *
 *  Created on: 2020/11/27
 *      Author: ROBO TAKUMI
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "Cntrl_Proc.h"
#include "syncturn.h"
#include "nuc.h"
#include "stm32f4xx_it.h"
#include "wheel.h"
#include "turn.h"
#include "common_func.h"
#include "conf.h"

// ��ʐݒ�
#define TimeIntSync_ms	10			//�������(msec)
#define TimeIntSync_sec	(double)TimeIntSync_ms / 1000.0	//�������(sec)
#define TIM_50MSEC	5
#define LOGSIZE  800 				// ���O�_��
#define	SYNC_ERROR_ANGLE 50 		// �V���N���^�[���������̋��e�덷(deg/10)
// ����
#define WHEEL_RATIO		4.9			// �z�C�[���x�[�X/���֒��a
// �^�[���e�[�u��
#define WHAT_TURNTBL_V(av)		2.344 * (av)	// �^�[���e�[�u���p���x���w���d�� �ϊ���
#define SYNC_TURNTBLIMPVOLT_DAC(volt)	(int32_t)((double)(4095 / 5.0) * (volt)) // �w���d����DAC�w���l�ւ̕ϊ���
#define TIM_CYCLE	24.39024		// �^�C�}�[�J�E���g�A�b�v����(sec��10^6�{)
#define TIM_CYCLE_TTBL	1.2195122	// �^�C�}�[�J�E���g�A�b�v����_�^�[���e�[�u���p(sec��10^6�{)
// �^�[����ʂɂ��Ȃ����ʃL�����u���[�V����
#if SYNC_RESULT_OUT == 1
#define WAIT_TIME 1000				// �u���[�L��ҋ@���� ms(�e�X�g�p)
#else
#define WAIT_TIME 250				// �u���[�L��ҋ@���� ms
#endif
#define STOP_PALSE_SYNC	4			// �u���[�L��������G���R�[�_�X�V��
#define FB_OFF_ANGLE 0				// �t�B�[�h�o�b�N�k�ފJ�n����p�x deg/10
#define FB_RELEASE_GRAD 0.0005 		// �t�B�[�h�o�b�N�k�ތ��z rad/s/10ms
#define APPROACH_VEL	0.05		// �^�[���I���ԍۂ̔��������x rad/s
#define BODY_VEL_FIR 0.05			// �ԑ̊p���x�ɂ�����LPF���萔(sec)
#define TTBL_VEL_FIR 0.05			// �^�[���e�[�u���p���x�ɂ�����LPF���萔(sec)
#define ERR_ANG_FIR 0.05			// �p�x�̕΍��ɂ�����LPF���萔(sec)
#define ERR_VEL_FIR 0.05			// �p���x�̕΍��ɂ�����LPF���萔(sec)
// �J���}���t�B���^�p�L�����u���[�V����
#define VER_COEF 0.1				// 0�ɋ߂��ق�P&F�̏����d���@0���傫��1��菬�����l��ݒ�
#define ALPHA_KL 0.0976				// P&F�Z���T�p�̒x��W��(rad/rad/s)
// �^�[����ʂ��Ƃ̃L�����u���[�V�����萔
static SYNC_CTRL_TBL *pTurnCalib;
//---LOW SPEED--- 90TURN
static SYNC_CTRL_TBL turn90_low_calib  = {
		(PI * 0.5 - 6.0 * PI / 180.0),	//1.�^�[���� (rad)
		3.2,				//2.�^�[������(sec)
		5,					//3.�u���[�L�J�n�ԑ̊p (deg/10)
		4.0 * PI / 180.0	//4.�e�[�u���Z���T�[������A�e�[�u����i�߂鋗�� rad
};
//---MID SPEED--- 90TURN
static SYNC_CTRL_TBL turn90_mid_calib  = {
		(PI * 0.5 - 6.0 * PI / 180.0),	//1.�^�[���� (rad)
		3.2,				//2.�^�[������(sec)
		5,					//3.�u���[�L�J�n�ԑ̊p (deg/10)
		4.0 * PI / 180.0	//4.�e�[�u���Z���T�[������A�e�[�u����i�߂鋗�� rad
};
//---HIGH SPEED--- 90TURN
static SYNC_CTRL_TBL turn90_high_calib  = {
		(PI * 0.5 - 6.0 * PI / 180.0),	//1.�^�[���� (rad)
		3.2,				//2.�^�[������(sec)
		5,					//3.�u���[�L�J�n�ԑ̊p (deg/10)
		4.0 * PI / 180.0	//4.�e�[�u���Z���T�[������A�e�[�u����i�߂鋗�� rad
};
//---LOW SPEED--- 180TURN
static SYNC_CTRL_TBL turn180_low_calib = {
		(PI - 9.0 * PI / 180.0),	//1.�^�[���� (rad)
		5.0,				//2.�^�[������(sec)
		3,					//3.�u���[�L�J�n�ԑ̊p (deg/10)
		4.0 * PI / 180.0	//4.�e�[�u���Z���T�[������A�e�[�u����i�߂鋗�� rad
};
//---MID SPEED--- 180TURN
static SYNC_CTRL_TBL turn180_mid_calib = {
		(PI - 9.0 * PI / 180.0),	//1.�^�[���� (rad)
		5.0,				//2.�^�[������(sec)
		3,					//3.�u���[�L�J�n�ԑ̊p (deg/10)
		4.0 * PI / 180.0	//4.�e�[�u���Z���T�[������A�e�[�u����i�߂鋗�� rad
};
//---HIGH SPEED--- 180TURN
static SYNC_CTRL_TBL turn180_high_calib = {
		(PI - 9.0 * PI / 180.0),	//1.�^�[���� (rad)
		5.0,				//2.�^�[������(sec)
		3,					//3.�u���[�L�J�n�ԑ̊p (deg/10)
		4.0 * PI / 180.0	//4.�e�[�u���Z���T�[������A�e�[�u����i�߂鋗�� rad
};

//�t�B�[�h�o�b�N�p�����[�^
static SYNC_FB_PRAM *pFB_Calib;
static SYNC_FB_PRAM fb_pram = {
		0.3,	//1.�ԑ�FB ���Q�C��Kp
		0.3,	//2.�ԑ�FB �ϕ�����Ti
		0.0,	//3.�ԑ�FB ��������Td
		0.5,	//4.�ԑ�FB �����W��
		0.3,	//5.�^�[���e�[�u��FB ���Q�C��Kp
		100.0,	//6.�^�[���e�[�u��FB �ϕ�����Ti
		0.2,	//7.�^�[���e�[�u��FB ��������Td
		0.2		//8.�^�[���e�[�u��FB �����W��
};

//�t���f���p�����[�^
static SYNC_INV_MDL *pInv_Model_Calib;
static SYNC_INV_MDL inv_model_param_90 = {
		0.9366,	//1.�ԑ�gain rise
		6.3421,	//2.�ԑ�omega rise
		0.1891,	//3.�ԑ�zeta rise
		0.9366,	//4.�ԑ�gain fall
		6.3421,	//5.�ԑ�omega fall
		0.1891,	//6.�ԑ�zeta fall
		0.9016,	//7.�^�[���e�[�u��gain rise
		5.9773,	//8.�^�[���e�[�u��omega rise
		0.5131,	//9.�^�[���e�[�u��zeta rise
		0.9016, //10.�^�[���e�[�u��gain fall
		6.004,	//11.�^�[���e�[�u��omega fall
		0.6411	//12.�^�[���e�[�u��zeta fall
};

static SYNC_INV_MDL inv_model_param_180 = {
		0.9396,	//1.�ԑ�gain rise
		6.6875,	//2.�ԑ�omega rise
		0.2038,	//3.�ԑ�zeta rise
		0.9396,	//4.�ԑ�gain fall
		6.6875,	//5.�ԑ�omega fall
		0.2038,	//6.�ԑ�zeta fall
		0.9021,	//7.�^�[���e�[�u��gain rise
		5.9787,	//8.�^�[���e�[�u��omega rise
		0.5152,	//9.�^�[���e�[�u��zeta rise
		0.9021, //10.�^�[���e�[�u��gain fall
		6.0048,	//11.�^�[���e�[�u��omega fall
		0.64	//12.�^�[���e�[�u��zeta fall
};


//�X�e�[�^�X
static uint8_t	g_syncturn_status = SYNC_TURN_READY;		//�V���N���^�[���X�e�[�^�X
//�t���O
static uint8_t 	g_flg_send_comp = 0;			//NUC�փV���N���^�[�������t���O�𑗐M�ς�1
static uint8_t	g_flg_approach	= 0;			//�ڕW�O�Ղ��A�v���[�`�ɓ�������1
static uint8_t 	g_flg_brk_angle_body = 0;		//�ԑ̃u���[�L�J�n�p�𒴂�����1
static uint8_t 	g_flg_brk_angle_ttbl = 0;		//�^�[���e�[�u���u���[�L�J�n�p�𒴂�����1
static uint8_t	g_flg_syncturn_start = 0;		//�V���N���^�[���J�n����1
static uint8_t	g_flg_illg_command = 0;			//NUC�����M�����R�}���h���s���Ȓl�̎���1

static uint8_t	g_flg_ttbl_sens_aprch = 0;		//�^�[���e�[�u�����Z���T�[�ɓ��B��1
static uint8_t	g_flg_brake_permission = 0;		//�u���[�L�������ėǂ��X�s�[�h�Ȃ�1
static uint8_t	g_flg_body_stp_jdg = 0;			//�p���X��~����(�ԗ�)
static uint8_t	g_flg_ttbl_stp_jdg = 0;			//�p���X��~����(�^�[���e�[�u��)
static uint8_t	g_flg_sync_dump_req = 0;		//���O�o�͗v�������1
static uint8_t	g_flg_sync_dump_comp = 0;		//���O�o�͏I����1
//NUC����̎w��
static uint8_t	g_command = 0;		//�V���N���^�[�����{�w��
static uint8_t	g_sync_spd = 0;		//�V���N���^�[�����x�w��
static uint8_t	g_sync_dir = 0;		//�V���N���^�[����]�����w��
//�^�C�}��
static uint32_t	g_time = 0;				//�J�n����̎���[ms]
static uint16_t	g_time_from_brk = 0; 	//�u���[�L��̌o�ߎ���
//P&F�Z���T
static uint16_t	g_pf_angle = 0;			//P&F�Z���T�p
static uint16_t g_pf_angle_old = 0;		//P&F�Z���T�p�̑O��l
static uint16_t	g_pf_start_angle = 0;	//�^�[���J�n����P&F�Z���T�p [deg/10]
static int16_t	g_diff_rad = 0;			//�����p�̂��� rad/1000
static int16_t	g_pf_angle_dist  = 0;	//�^�[���J�n����0�Ƃ���P&F�Z���T�p [deg/10]
static double	g_pf_angle_dist_ln = 0;	//P&F�Z���T���X�V���̕�ԏ�����[rad]
static int8_t	g_pf_nup_count	= 0;	//P&F�Z���T���X�V�񐔂̃J�E���^
static int8_t	g_pf_count = 0;			//P&F�Z���T��3600��0�̋��E���܂�������
static int16_t	g_target_dist = 0;		//�^�[���J�n������Ƃ����^�[�Q�b�g�p [deg/10]
//�V���N���^�[���p�G���R�[�_�J�E���^
static int16_t	g_wheel_enccnt_left = 0;
static int16_t	g_wheel_enccnt_right = 0;
static int16_t	g_turntbl_enccnt = 0;
static uint16_t g_wheel_encoder_left_old = 0;
static uint16_t g_wheel_encoder_right_old = 0;
static uint16_t g_ttbl_encoder_old = 0;
//�ڕW�O��
static double	g_trgt_angle = 0;			//�ڕW�p�x[rad]
static double	g_trgt_vel = 0;				//�ڕW�p���x[rad/s]
static double	g_trgt_acc = 0;				//�ڕW�p�����x[rad/s^2]
static double	g_trgt_jerk = 0;			//�ڕW���x[rad/s^3]
//�ԑ̊p���x�̎w���l
static double	g_ref_body_vel = 0;			//�ŏI�w���l[rad/s]
static double	g_ref_body_vel_ff = 0;		//�t�B�[�h�t�H���[�h��[rad/s]
static double	g_d_term_body = 0;
static double	g_d_term_body_old = 0;
//�^�[���e�[�u���p���x�̎w���l
static double	g_ref_ttbl_vel = 0;			//�ŏI�w���l[rad/s]
static double	g_ref_ttbl_vel_ff = 0;		//�t�B�[�h�t�H���[�h��[rad/s]
static double	g_d_term_ttbl = 0;
static double	g_d_term_ttbl_old = 0;
//�G���R�[�_����擾������ԗ�
static double	g_act_body_angle = 0;		//�ԑ̊p[rad]
static double	g_act_body_vel   = 0;		//�ԑ̊p���x[rad/s]
static double	g_act_body_vel_lpf   = 0;	//�ԑ̊p���x(LPF��)[rad/s]
static double	g_act_body_vel_lpf_old = 0;	//�ԑ̊p���x(LPF��)�O��l[rad/s]
static double	g_act_ttbl_angle = 0;		//�^�[���e�[�u���p[rad]
static double	g_act_ttbl_vel   = 0;		//�^�[���e�[�u���p���x[rad/s]
static double	g_act_ttbl_vel_lpf   = 0;	//�^�[���e�[�u���p���x(LPF��)[rad/s]
static double	g_act_ttbl_vel_lpf_old = 0;	//�^�[���e�[�u���p���x(LPF��)�O��l[rad/s]
static double	g_act_err_angle  = 0;		//�V���N������p[rad]
static double	g_act_err_angle_old  = 0;	//�V���N������p[rad]�O��l
static double	g_act_err_angle_lpf  = 0;	//�V���N������p[rad]LPF��
static double	g_act_err_vel		= 0;	//�V���N������p���x[rad/s]
static double	g_act_err_vel_old 	= 0;	//�V���N������p���x[rad/s]
static double	g_act_err_vel_lpf	= 0;	//�V���N������p���x(LPF��)[rad/s]
static double	g_ttbl_enc_sens		= 0;	//�t�H�g�Z���T�������������̊p�x[rad]
//�J���}���t�B���^
static double	g_p_body_angle		= 0;	//��ԗʐ���덷�̕��U
static double	g_est_body_angle	= 0;	//�J���}���t�B���^�ɂ�萄�肵���ԑ̊p[rad]
//�ڕW�O�ՂƎԑ́E�^�[���e�[�u���̕΍�
static double	g_act_body_err_angle	= 0;
static double	g_act_body_err_vel		= 0;
static double	g_act_body_err_vel_old	= 0;
static double	g_act_body_err_vel_lpf	= 0;
static double	g_act_body_err_acc		= 0;
static double	g_act_ttbl_err_angle	= 0;
static double	g_act_ttbl_err_vel		= 0;
static double	g_act_ttbl_err_vel_old	= 0;
static double	g_act_ttbl_err_vel_lpf	= 0;
static double	g_act_ttbl_err_acc		= 0;
//��]����
static int8_t	g_ttbl_dir = 0;				//�^�[���e�[�u����]�����F�E����
static int8_t	g_body_dir = 0;				//�ԑ̉�]�����F�E����
//�^�[�Q�b�g
static uint8_t	g_ttbl_sens_target_pos = 0;	//�^�[���e�[�u���̃^�[�Q�b�g�Z���T�[�ʒu
static uint16_t	g_target_pf_angle = 0;		//�ԑ̊p�x�^�[�Q�b�g
//�����p���O�ϐ�
static int16_t	g_result_max_err = 0;		//�V���N���덷�̍ő�l
static int16_t	g_result_ttbl_angle	= 0;	//�^�[���O��Ń^�[���e�[�u�����������p�x


#if SYNC_TURN_LOG_DUMP
// �J���p���O�ϐ�
uint16_t	g_log_idx = 0;
uint32_t	g_time_log[LOGSIZE] = {0};
uint16_t	g_pf_angle_log[LOGSIZE] = {0};
double		g_pf_angle_dist_ln_log[LOGSIZE] = {0};
uint8_t		g_flg_brk_angle_body_log[LOGSIZE] = {0};
uint8_t		g_flg_brk_angle_ttbl_log[LOGSIZE] = {0};
double		g_trgt_angle_log[LOGSIZE] = {0};
double		g_trgt_vel_log[LOGSIZE] = {0};
double		g_ref_body_vel_log[LOGSIZE] = {0};
double		g_ref_ttbl_vel_log[LOGSIZE] = {0};
double		g_act_body_angle_log[LOGSIZE] = {0};
double		g_act_body_vel_lpf_log[LOGSIZE] = {0};
double		g_act_ttbl_angle_log[LOGSIZE] = {0};
double		g_act_ttbl_vel_lpf_log[LOGSIZE] = {0};
double		g_act_err_angle_log[LOGSIZE] = {0};
double		g_act_err_vel_lpf_log[LOGSIZE] = {0};
double		g_est_body_angle_log[LOGSIZE] = {0};
#endif


/*
 * sycturn_direction
 * ��]������ݒ肷��B
 * int32_t dir�F�@��]����
 */
static void sycturn_direction(uint8_t dir, int32_t diffrad) {

	switch(dir) {
	case L90T:		//���X�O�x�^�[��
		//���^�C���t��]�A�E�^�C������]
		g_body_dir = -1;
//		wheel_set_dir(-1, 1, 0, 0);

		//�^�[���e�[�u���͉E����
		g_ttbl_dir = 1;
		set_rotation_cw();
		break;

	case R90T:		//�E�X�O�x�^�[��
		//���^�C������]�A�E�^�C���t��]
		g_body_dir = 1;
//		wheel_set_dir(1, -1, 0, 0);
		
		//�^�[���e�[�u���͍�����
		g_ttbl_dir = -1;
		set_rotation_ccw();
		break;

	case RL180T:
		if (diffrad < 0) {
			//���^�[��
			//���^�C���t��]�A�E�^�C������]
			g_body_dir = -1;
//			wheel_set_dir(-1, 1, 0, 0);
			//�^�[���e�[�u���͉E����
			g_ttbl_dir = 1;
			set_rotation_cw();
		}
		else {
			//�E�^�[��
			//���^�C������]�A�E�^�C���t��]
			g_body_dir = 1;
//			wheel_set_dir(1, -1, 0, 0);
			//�^�[���e�[�u���͍�����
			g_ttbl_dir = -1;
			set_rotation_ccw();
		}
		break;

	case STOP:
		//��~
//		wheel_set_dir(0, 0, 0, 0);
		turn_motor_stop();
	}
}

/*
 * conv_wheel_ref
 * ���s�p���[�^�[�̊p���x�w����NUC�w���l�ɕϊ�����
 */
static int16_t conv_wheel_ref(double speed){
	int16_t ref = 0;
	ref = (short)((double)WHEEL_DIAMETER * 0.5 * speed * (double)MAX_NUC_REF / (double)MAX_NUC_SPEED_WHEEL);
	return ref;
}


/*
 * conv_motervolt2dac
 * ���[�^�̎�ʂɉ����ē��͓d���l��DAC�w���l�ɕϊ�����
 * int kind:	���[�^���
 * double volt:	���͓d��
 * �߂�l�FDAC�w���l
 */
static int32_t conv_motervolt2dac(int kinds, double volt) {

	int32_t	retDAC;

	if (kinds == SYNC_WHEEL) {
		//TAG1000��wheel.c����đ��s�p���[�^�[�Ɏw�����o���̂�syncturn.c�œd���ϊ��͂��Ȃ�
	}
	else {
		retDAC = SYNC_TURNTBLIMPVOLT_DAC(volt);
	}

	return retDAC;
}


/*
 * turntbl_motor_ctrl
 * �^�[���e�[�u������
 */
static void turntbl_motor_ctrl(int32_t dacval) {
	turn_set_speed((uint16_t)dacval);
}


/*
 * calc_turn_locus
 * �ڕW�O�Ղ̌v�Z
 */
static void calc_turn_locus() {
	// �����p�ɉ����ă^�[���ʂ𑝌�
	double reach_angle_adj = 0;
	if (g_sync_dir == R90T || g_sync_dir == L90T){
		// 90�x�^�[��
		reach_angle_adj = (double)g_diff_rad * 0.001;
	}
	else{
		// 180�x�^�[��
		if(g_body_dir == 1){
			// �E�^�[��
			reach_angle_adj = -1.0 * (double)g_diff_rad * 0.001;
		}
		else{
			// ���^�[��
			reach_angle_adj = (double)g_diff_rad * 0.001;
		}
	}


	// �^�[���ʂƃ^�[�����Ԃ��琳���g�̃p�����[�^���Z�o
	double amp = (pTurnCalib->reach_angle + reach_angle_adj)/ pTurnCalib->turn_time - APPROACH_VEL/4.0;
	double amp_adj = APPROACH_VEL / 2.0;
	double omega = 2 * PI / pTurnCalib->turn_time;

	if ((omega * g_time * 0.001) < PI){
		// �������
		g_trgt_vel =  amp * mysin(omega * g_time * 0.001 - PI * 0.5) + amp;
		g_trgt_acc = amp * omega * mysin(omega * g_time * 0.001);
		g_trgt_jerk = -1.0 * amp * omega * omega * mysin(omega * g_time * 0.001 - PI * 0.5);
		g_flg_approach = 0;
	}
	else if ((omega * g_time * 0.001) < (2.0 * PI)){
		// �������
		g_trgt_vel =  amp * mysin(omega * g_time * 0.001 - PI * 0.5) + amp + amp_adj * mysin(omega * g_time * 0.001 + PI * 0.5) + amp_adj;
		g_trgt_acc = amp * omega * mysin(omega * g_time * 0.001) - amp_adj * omega * mysin(omega * g_time * 0.001);
		g_trgt_jerk = -1.0 * amp * omega * omega * mysin(omega * g_time * 0.001 - PI * 0.5) - 1.0 * amp_adj * omega * omega * mysin(omega * g_time * 0.001 + PI * 0.5);
		g_flg_approach = 0;
	}
	else{
		// ���������
		g_trgt_vel = APPROACH_VEL;
		g_trgt_acc = 0.0;
		g_trgt_jerk = 0.0;
		g_flg_approach = 1;
	}

	// �^�[�Q�b�g�p
	g_trgt_angle += g_trgt_vel * 0.01;
}


/*
 * calc_d_term
 * ���ǌ`�o�ꎟ�ϊ���p�����s���S�����ɂ��PID��D�������Z�o����
 */
static double calc_d_term(double d_term_old, double err, double err_old, double Td, double eta){
	//d_term_old:d���̑O��l
	//err:�΍��i���x�j�̍���l
	//err_old:�΍�(���x)�̑O��l
	//Td:��������(=D�Q�C��/P�Q�C��)
	//eta:�����W��(0.1~0.125����)

	double delta_t = 0.01;	//�������[sec]
	double d_term = d_term_old + (2 * Td * (err - err_old)) / (delta_t + 2 * eta * Td)  - (2 * delta_t * d_term_old) / (delta_t + 2 * eta * Td);

	return d_term;
}


/*
 * calc_FF_body
 * �ԑ̂̋t���f���������Ċp���x�w���l���v�Z����
 */
static double calc_FF_body(){

	static double ret = 0;
	double gain_body = 0;
	double omega_body = 0;
	double zeta_body = 0;

	if (g_trgt_acc >= 0){
		// �^�[�Q�b�g�p�����x����
		gain_body = pInv_Model_Calib->inv_mdl_body_gain_rise;
		omega_body = pInv_Model_Calib->inv_mdl_body_omega_rise;
		zeta_body = pInv_Model_Calib->inv_mdl_body_zeta_rise;
	}
	else{
		// �^�[�Q�b�g�p�����x����
		gain_body = pInv_Model_Calib->inv_mdl_body_gain_fall;
		omega_body = pInv_Model_Calib->inv_mdl_body_omega_fall;
		zeta_body = pInv_Model_Calib->inv_mdl_body_zeta_fall;
	}

	if (g_flg_approach == 0){
		ret = 1.0 / (gain_body * omega_body * omega_body) * (g_trgt_jerk + 2.0 * zeta_body * omega_body * g_trgt_acc + omega_body * omega_body * g_trgt_vel);
	}
	else{;}	// ���x�w���l�̒i�t���h�~�̂��߁A�A�v���[�`����FF�����X�V���Ȃ�

	return ret;
}


/*
 * calc_FF_ttbl
 * �^�[���e�[�u���̋t���f���������Ċp���x�w���l���v�Z����
 */
static double calc_FF_ttbl(){

	static double ret = 0;
	double gain_ttbl = 0.0;
	double omega_ttbl = 0;
	double zeta_ttbl = 0;

	if (g_trgt_acc >= 0){
		// �^�[�Q�b�g�p�����x����
		gain_ttbl = pInv_Model_Calib->inv_mdl_ttbl_gain_rise;
		omega_ttbl = pInv_Model_Calib->inv_mdl_ttbl_omega_rise;
		zeta_ttbl = pInv_Model_Calib->inv_mdl_ttbl_zeta_rise;
	}
	else{
		// �^�[�Q�b�g�p�����x����
		gain_ttbl = pInv_Model_Calib->inv_mdl_ttbl_gain_fall;
		omega_ttbl = pInv_Model_Calib->inv_mdl_ttbl_omega_fall;
		zeta_ttbl = pInv_Model_Calib->inv_mdl_ttbl_zeta_fall;
	}

	if (g_flg_approach == 0){
		ret = 1.0 / (gain_ttbl * omega_ttbl * omega_ttbl) * (g_trgt_jerk + 2.0 * zeta_ttbl * omega_ttbl * g_trgt_acc + omega_ttbl * omega_ttbl * g_trgt_vel);
	}
	else{;}	// ���x�w���l�̒i�t���h�~�̂��߁A�A�v���[�`����FF�����X�V���Ȃ�

	return ret;
}


/*
 * syncturn_result
 * �V���N���^�[���̌��ʂ𔻒�
 */
static uint8_t syncturn_result() {

	uint16_t diff;
	uint8_t ret = 0;
	uint8_t ttbl_pos = get_turn_sensor_pos();

	if ((g_target_pf_angle == 3600) || (g_target_pf_angle == 0)) {
		if (g_pf_angle < 180) {
			diff = abs(g_pf_angle - 0);
		} else {
			diff = abs(3600 - g_pf_angle);
		}
	} else {
		diff = abs(g_pf_angle - g_target_pf_angle);
	}

	// NUC�֒ʒm���邽�߂̃V���N���^�[�����ʔ���
	// return 1:�@�K����̊p�x�Ń^�[������, return 0�F �K��O�̊p�x�Ń^�[������
	if (((g_ttbl_sens_target_pos & ttbl_pos) != 0) && (diff < SYNC_ERROR_ANGLE)) {
		ret = 1;
	}

	// �^�[���J�n�O�ƏI����Ń^�[���e�[�u�����O���[�o�����W�n�ɑ΂��ĉ��x��������
	double angle_tmp = (double)g_pf_angle_dist / 10.0 - g_act_ttbl_angle * 180.0 / PI;
	g_result_ttbl_angle = (int16_t)(angle_tmp * 10.0);
	return ret;
}


/*
 * sync_motor_ctrl
 * �V���N���^�[�����[�^����
 */
static void sync_motor_ctrl() {
	///////////////////
	//���s�p���[�^�[�̐���//
	///////////////////
	double Kp_body = pFB_Calib->Kp_body;	// ���Q�C��
	double Ti_body = pFB_Calib->Ti_body;	// �ϕ�����
	double Td_body = pFB_Calib->Td_body;	// ��������
	double eta_body = pFB_Calib->eta_body;	// �����W��

	if(g_flg_brk_angle_body < 1){
		//FF���v�Z
		g_ref_body_vel_ff = calc_FF_body();
		//FB���v�Z
		g_d_term_body = calc_d_term(g_d_term_body, g_act_err_vel, g_act_err_vel_old, Td_body, eta_body);
		double ref_body_vel_fb = g_act_err_vel_lpf * Kp_body + Kp_body * g_d_term_body + Kp_body / Ti_body * g_act_err_angle;
		//FF����FB���𑫂�
		g_ref_body_vel = g_ref_body_vel_ff + ref_body_vel_fb;

		//�ԑ̊p���x�w�����^�C���p���x�w���ɕϊ�
		double wheel_angle_v = WHEEL_RATIO * g_ref_body_vel;
		//�^�C���p���x�w���l��NUC�w���l�ɕϊ�
		int16_t wheel_ref = conv_wheel_ref(wheel_angle_v);
		if (wheel_ref < 0){
			wheel_ref = 0;
		}
		else{;}
		//���[�^�[����
//		wheel_set_freerun(0);
//		wheel_set_brake(0);
//		wheel_set_dc_lock(0);
		wheel_set_speed(wheel_ref, wheel_ref);
	}
	else{						//�u���[�L���
//		wheel_set_freerun(0);
//		wheel_set_dc_lock(0);
		wheel_set_speed(0, 0);
		if(g_flg_brake_permission > 0){
			// ���x���������l�ȉ��ɂȂ��Ă���u���[�L��������
//			wheel_set_brake(1);
		}
		else{
//			wheel_set_brake(0);
		}
	}

	///////////////////////
	//�^�[���e�[�u�����[�^�[�̐���//
	///////////////////////
	double Kp_ttbl = pFB_Calib->Kp_ttbl;	// ���Q�C��
	double Ti_ttbl = pFB_Calib->Ti_ttbl;	// �ϕ�����
	double Td_ttbl = pFB_Calib->Td_ttbl;	// ��������
	double eta_ttbl = pFB_Calib->eta_ttbl;	// �����W��
	if(g_flg_brk_angle_ttbl < 1 ){
		//FF���v�Z
		g_ref_ttbl_vel_ff = calc_FF_ttbl();
		//FB���v�Z
		g_d_term_ttbl = calc_d_term(g_d_term_ttbl, g_act_err_vel, g_act_err_vel_old, Td_ttbl, eta_ttbl);
		double ref_ttbl_vel_fb = g_act_err_vel * Kp_ttbl + Kp_ttbl * g_d_term_ttbl + Kp_ttbl / Ti_ttbl * g_act_err_angle;
		//FF����FB���𑫂�
		g_ref_ttbl_vel = g_ref_ttbl_vel_ff - ref_ttbl_vel_fb;
		//�p���x����d���ɕϊ�
		volatile double turntable_input_volt = WHAT_TURNTBL_V(g_ref_ttbl_vel);
		if (turntable_input_volt < 0){
			turntable_input_volt = 0;
		}
		else{;}
		//�d����DAC�w���l�ɕϊ�
		int32_t turntbl_motor_dac = conv_motervolt2dac(TURN_TABLE, turntable_input_volt);
		turntbl_motor_ctrl(turntbl_motor_dac);
	}
	else{				//�u���[�L���
		turntbl_motor_ctrl(0);
		turn_motor_stop();
		t_set_brake();
	}
}


/*
 * calc_angle
 * �G���R�[�_�p���X�����猻�݂̊p�x���擾����
 */
static void calc_angle(){
	double body_angle_per_pulse = 2 * PI / (WHEEL_RATIO * PULS_WHEEL);	// �G���R�[�_1�p���X������̎ԑ̉�]�p(rad)
	int32_t wheel_angle_ave_l = (abs(g_wheel_enccnt_left) + abs(g_wheel_enccnt_right) + 1) / 2;
	g_act_body_angle = (double)wheel_angle_ave_l * body_angle_per_pulse;
	g_act_ttbl_angle = (2.0 * PI * (double)g_turntbl_enccnt) / TURNTBL_ENC *(double)g_ttbl_dir;
}


/*
 * calc_err_angle
 * �ڕW�O���ɑ΂���ԑ̊p�ƃ^�[���e�[�u���p�̍����v�Z
 */
static void calc_err_angle(){
	// �^�[���e�[�u���Ǝԑ̊p�̍����v�Z
	g_act_err_angle_old = g_act_err_angle;
	g_act_err_angle = g_act_ttbl_angle - g_est_body_angle;
	g_act_err_vel_old = g_act_err_vel;
	g_act_err_vel = g_act_ttbl_vel - g_act_body_vel; // [rad/s]
	// ���[�p�X�t�B���^����
	g_act_err_angle_lpf = LPF_1order(g_act_err_angle, g_act_err_angle_old, g_act_err_angle_lpf, ERR_ANG_FIR, TimeIntSync_sec); // [rad/s]
	g_act_err_vel_lpf = LPF_1order(g_act_err_vel, g_act_err_vel_old, g_act_err_vel_lpf, ERR_VEL_FIR, TimeIntSync_sec); // [rad/s]

	// �ڕW�O���ɑ΂��鍷���v�Z
	g_act_body_err_angle	= g_trgt_angle - g_act_body_angle;
	g_act_body_err_vel_old	= g_act_body_err_vel;
	g_act_body_err_vel		= g_trgt_vel   - g_act_body_vel;
	g_act_body_err_vel_lpf	= LPF_1order(g_act_body_err_vel, g_act_body_err_vel_old, g_act_body_err_vel_lpf, BODY_VEL_FIR, TimeIntSync_sec);

	g_act_ttbl_err_angle	= g_trgt_angle - g_act_ttbl_angle;
	g_act_ttbl_err_vel_old	= g_act_ttbl_err_vel;
	g_act_ttbl_err_vel		= g_trgt_vel   - g_act_ttbl_vel;
	g_act_ttbl_err_vel_lpf	= LPF_1order(g_act_ttbl_err_vel, g_act_ttbl_err_vel_old, g_act_ttbl_err_vel_lpf, TTBL_VEL_FIR, TimeIntSync_sec);
}


/*
 * calc_pf_dist
 * ������Ԃ���Ƃ���P&F�Z���T�̕ψʊp�����߂�
 */
static void calc_pf_dist(){
	if ((g_pf_angle != g_pf_angle_old) && (get_sync_start_flg() != 1)){
		// 0�x���ׂ����񐔂��J�E���g
		if((g_pf_angle_old > 2700) && (g_pf_angle < 900)){
			g_pf_count ++;
		}
		else if((g_pf_angle_old < 900) && (g_pf_angle > 2700)){
			g_pf_count --;
		}

		// �����p����Ƃ���P&F�Z���T�̕ψʊp���Z�o
		if (g_pf_count == 0){
			g_pf_angle_dist = (int16_t)g_pf_angle - (int16_t)g_pf_start_angle;
		}
		else if (g_pf_count > 0){
			g_pf_angle_dist = (3600 - (int16_t)g_pf_start_angle) + (int16_t)g_pf_angle + 3600 * (g_pf_count - 1);
		}
		else{
			g_pf_angle_dist = -1 * (int16_t)g_pf_start_angle - (3600 - (int16_t)g_pf_angle) - 3600 * (g_pf_count + 1);
		}

		// �^�[�������𐳂Ƃ���
		g_pf_angle_dist *= (int16_t)g_body_dir;
	}
	g_pf_angle_old = g_pf_angle;
}


/*
 * calc_pf_liner
 * P&F�Z���T���X�V���̕�ԏ���
 */
static void calc_pf_liner(){
	static int16_t	g_pf_angle_dist_old = 0;
	if((g_pf_angle_dist != g_pf_angle_dist_old) || ((g_pf_angle_dist == 0) && (g_pf_angle_dist_old == 0))){
		g_pf_nup_count = 0;
		g_pf_angle_dist_ln = (double)g_pf_angle_dist * 0.1 * M_PI / 180.0;
	}
	else{
		g_pf_nup_count ++;
		// ���X�V���͎ԗւ̃G���R�[�_�l���琄�肵���ԑ̊p���x���g���Đ��`��Ԃ���
		g_pf_angle_dist_ln = g_pf_angle_dist_ln + g_act_body_vel * 0.01;
	}
	g_pf_angle_dist_old = g_pf_angle_dist;
}


/*
 * calc_body_angle_kalman
 * �J���}���t�B���^�ɂ��ԑ̊p�𐄒肷��
 */
static void calc_body_angle_kalman(){
	double dt = 0.01;				// ���ԍ���(sec)
	double ver_v = 1.0 - VER_COEF;	// �V�X�e���m�C�Y�̕��U
	double ver_w = VER_COEF;		// �ϑ��m�C�Y�̕��U
	double est_body_angle_pre = g_est_body_angle + dt * g_act_body_vel;
	double p_body_angle_pre = g_p_body_angle + dt * dt  * ver_v;
	double kalman_gain = p_body_angle_pre / (p_body_angle_pre + ver_w);
	g_p_body_angle = (1.0 - kalman_gain) * p_body_angle_pre;
	g_est_body_angle = est_body_angle_pre + kalman_gain * (g_pf_angle_dist_ln - est_body_angle_pre + ALPHA_KL * g_act_body_vel);
}


/*
 * judge_ttbl_finish
 * �^�[���e�[�u�����ڕW�p(=�u���[�L�J�n�p)�ɓ��B���������肷��
 */
static void judge_ttbl_finish(uint8_t ttbl_pos) {

	double now_angle = g_act_ttbl_angle;
	double sens_diff;

	if (((g_ttbl_sens_target_pos & ttbl_pos) != 0) && (g_flg_ttbl_sens_aprch == 0)) {
		g_flg_ttbl_sens_aprch = 1;
		g_ttbl_enc_sens = now_angle;
	}

	if (g_flg_ttbl_sens_aprch == 1) {
		sens_diff = fabs(now_angle) - fabs(g_ttbl_enc_sens);
		if (sens_diff > pTurnCalib->brake_ttbl_angle) {
			g_flg_brk_angle_ttbl = 1;
		}
	}
}

/*
 * judge_body_finish
 * �ԑ̂��ڕW�p(=�u���[�L�J�n�p)�ɓ��B���������肷��
 */
static void judge_body_finish(void) {

	switch(g_sync_dir){
	case R90T:
	case L90T:
		g_target_dist = 900 + ((double)g_diff_rad * 0.001 * 180.0 / PI * 10);
		break;
	case RL180T:
		g_target_dist = 1800 - abs((double)g_diff_rad * 0.001 * 180.0 / PI * 10);
		break;
	}

	if(g_pf_angle_dist > (g_target_dist - pTurnCalib->brake_body_angle)){
		g_flg_brk_angle_body = 1;
	}
}

/*
 * brake_timer
 * �u���[�L�J�n��̌o�ߎ��Ԃ��J�E���g
 */
static void brake_timer(){
	if(g_flg_brk_angle_body == 1 && g_flg_brk_angle_ttbl == 1
	   && g_flg_brake_permission == 1){
		g_time_from_brk += TimeIntSync_ms;
	}
	else{
		g_time_from_brk = 0;
	}
}


/*
 * log_update
 * ���O�p�ϐ��Ɍ��݂̏�ԗʂ�ۑ�����
 */
static void log_update(){
	// �����p�Ƀ^�[�����̃V���N���덷�̍ő�l��ۑ�
	int16_t g_act_err_angle_temp = (int16_t)(g_act_err_angle * 180.0 / M_PI * 10);
	if(abs(g_act_err_angle_temp) > g_result_max_err){
		g_result_max_err = g_act_err_angle_temp;	// �p�x(deg)��10�{
	}

	// �^�[�����̏�ԗʂ�ۑ�(�J���p)
#if SYNC_TURN_LOG_DUMP
	g_time_log[g_log_idx] = g_time;

	g_pf_angle_log[g_log_idx] = g_pf_angle;
	g_pf_angle_dist_ln_log[g_log_idx] = g_pf_angle_dist_ln;
	g_flg_brk_angle_body_log[g_log_idx] = g_flg_brk_angle_body;
	g_flg_brk_angle_ttbl_log[g_log_idx] = g_flg_brk_angle_ttbl;

	g_trgt_angle_log[g_log_idx] = g_trgt_angle;
	g_trgt_vel_log[g_log_idx] = g_trgt_vel;

	g_ref_body_vel_log[g_log_idx] = g_ref_body_vel;
	g_ref_ttbl_vel_log[g_log_idx] = g_ref_ttbl_vel;

	g_act_body_angle_log[g_log_idx] = g_act_body_angle;
	g_act_body_vel_lpf_log[g_log_idx] = g_act_body_vel_lpf;
	g_act_ttbl_angle_log[g_log_idx] = g_act_ttbl_angle;
	g_act_ttbl_vel_lpf_log[g_log_idx] = g_act_ttbl_vel_lpf;

	g_act_err_angle_log[g_log_idx] = g_act_err_angle;
	g_act_err_vel_lpf_log[g_log_idx] = g_act_err_vel_lpf;
	g_est_body_angle_log[g_log_idx] = g_est_body_angle;

	if (g_log_idx < (LOGSIZE - 1)){
		g_log_idx++;
	}
#endif
}

/*
 * log_reset
 * ���O�ϐ��̃��Z�b�g
 */
static void log_reset(){
	g_result_max_err = 0;
#if SYNC_TURN_LOG_DUMP
	g_log_idx = 0;
	for(int i = 0; i < LOGSIZE; i++){
		g_time_log[i] = 0;
		g_pf_angle_log[i] = 0;
		g_pf_angle_dist_ln_log[i] = 0;
		g_flg_brk_angle_body_log[i] = 0;
		g_flg_brk_angle_ttbl_log[i] = 0;

		g_trgt_angle_log[i] = 0;
		g_trgt_vel_log[i] = 0;

		g_ref_body_vel_log[i] = 0;
		g_ref_ttbl_vel_log[i] = 0;

		g_act_body_angle_log[i] = 0;
		g_act_body_vel_lpf_log[i] = 0;
		g_act_ttbl_angle_log[i] = 0;
		g_act_ttbl_vel_lpf_log[i] = 0;

		g_act_err_angle_log[i] = 0;
		g_act_err_vel_lpf_log[i] = 0;

		g_est_body_angle_log[i] = 0;

	}
#endif
}

/*
 * get_diff_from_stdangele
 * ����P��F�Z���T�[�̊p�x���A0(360), 90, 180, 270�x��
 * �����ꂩ�炸��Ă���p�x�����߂�B
 */
static int32_t get_diff_from_stdangele(uint8_t dir, uint16_t pf_angle) {

	double min_angle_d;
	double rad_d;
	int32_t rad;
	int16_t diff;
	int16_t min_angle = 900;
	int16_t angle[5] = {0, 900, 1800, 2700, 3600};
	uint8_t i;
	uint8_t buf_save;

	for (i = 0; i < 5; i ++) {
		diff = pf_angle - angle[i];
		if (abs(min_angle) > abs(diff)) {
			min_angle = diff;
			buf_save = i;
		}
	}
	min_angle_d = (double)min_angle / 10.0;

	//���W�A���ɕϊ���A1000�{
	rad_d = (min_angle_d * M_PI) / 180.0;
	rad = rad_d * 1000;

	//��]�����ɂ���Đ�����ς���
	//��]�����ƌ��݂�P&F�Z���T�[�̊p�x����^�[�Q�b�g�p�x�����߂�
	switch (dir) {
	case L90T:	//��90�x��]�̏ꍇ�͂��̂܂�
		if (buf_save == 0) {
			buf_save = 4;
		}
		g_target_pf_angle = angle[buf_save - 1];
		break;
	case R90T:	//�E90�x��]�̏ꍇ�͐������]
		rad *= -1.0;
		if (buf_save == 4) {
			buf_save = 0;
		}
		g_target_pf_angle = angle[buf_save + 1];
		break;
	case RL180T:
		if (buf_save < 2) {
			g_target_pf_angle = angle[buf_save + 2];
		} else if ((buf_save > 2)) {
			g_target_pf_angle = angle[buf_save - 2];
		} else {
			// 180�x�t�߂ɂ��鎞�́A�ǂ���ɉ�]���邩�ɂ���ă^�[�Q�b�g��ς���
			if (rad < 0) {
				g_target_pf_angle = angle[0];
			} else {
				g_target_pf_angle = angle[4];
			}
		}
		break;
	}

	return rad;
}

/*
 * syncturn_init
 * �����ϐ��̃��Z�b�g
 */
static void syncturn_init() {

	reset_sync_flg_send_comp();
	g_flg_brk_angle_body = 0;
	g_flg_brk_angle_ttbl = 0;
	g_flg_ttbl_sens_aprch = 0;
	g_flg_brake_permission = 0;

	g_time = 0;
	g_diff_rad = 0;
	g_time_from_brk = 0;

	g_wheel_encoder_left_old = get_l_wheel_encoder();
	g_wheel_encoder_right_old = get_r_wheel_encoder();
	g_wheel_enccnt_left = 0;
	g_wheel_enccnt_right = 0;
	g_turntbl_enccnt = 0;
	g_ttbl_encoder_old = get_turn_encoder();

	g_pf_angle_old = 0;
	g_pf_angle_dist = 0;
	g_pf_angle_dist_ln = 0;
	g_pf_nup_count = 0;
	g_pf_count = 0;
	g_target_dist = 0;

	g_trgt_angle = 0;
	g_trgt_vel = 0;
	g_trgt_acc = 0;
	g_trgt_jerk = 0;

	g_ref_body_vel = 0;
	g_ref_body_vel_ff = 0;
	g_d_term_body = 0;
	g_d_term_body_old = 0;

	g_ref_ttbl_vel = 0;
	g_ref_ttbl_vel_ff = 0;
	g_d_term_ttbl = 0;
	g_d_term_ttbl_old = 0;

	g_act_body_angle = 0;
	g_act_body_vel   = 0;
	g_act_body_vel_lpf   = 0;
	g_act_body_vel_lpf_old = 0;

	g_act_ttbl_angle = 0;
	g_act_ttbl_vel   = 0;
	g_act_ttbl_vel_lpf   = 0;
	g_act_ttbl_vel_lpf_old = 0;

	g_act_err_angle		= 0;
	g_act_err_angle_old = 0;
	g_act_err_angle_lpf = 0;
	g_act_err_vel		= 0;
	g_act_err_vel_old	= 0;
	g_act_err_vel_lpf 	= 0;
	g_ttbl_enc_sens		= 0;

	g_p_body_angle		= 0;
	g_est_body_angle	= 0;

	g_act_body_err_angle	= 0;
	g_act_body_err_vel		= 0;
	g_act_body_err_vel_old	= 0;
	g_act_body_err_vel_lpf	= 0;
	g_act_body_err_acc		= 0;

	g_act_ttbl_err_angle	= 0;
	g_act_ttbl_err_vel		= 0;
	g_act_ttbl_err_vel_old	= 0;
	g_act_ttbl_err_vel_lpf	= 0;
	g_act_ttbl_err_acc		= 0;

	g_body_dir = 0;
	g_ttbl_dir = 0;

}
/*
 * syncturn_pram_set
 * �L�����u���[�V�����̃Z�b�g
 */
static void syncturn_pram_set(uint8_t spd, uint8_t dir) {
	//�L�����u���[�V�����l���Z�b�g
	if (dir == R90T || dir == L90T ){
		switch(spd){
		case LOW:
			pTurnCalib = &turn90_low_calib;
			break;
		case MID:
			pTurnCalib = &turn90_mid_calib;
			break;
		case HIGH:
			pTurnCalib = &turn90_high_calib;
			break;
		}
		pInv_Model_Calib = &inv_model_param_90;
		pFB_Calib = &fb_pram;
	}
	else if(dir == RL180T){
		switch(spd){
		case LOW:
			pTurnCalib = &turn180_low_calib;
			break;
		case MID:
			pTurnCalib = &turn180_mid_calib;
			break;
		case HIGH:
			pTurnCalib = &turn180_high_calib;
			break;
		}
		pInv_Model_Calib = &inv_model_param_180;
		pFB_Calib = &fb_pram;
	}
}


/*
 * calc_wheel_encoder
 * �ԗփG���R�[�_�p���X�X�V�����擾
 */
static void calc_wheel_encoder(void) {

	uint16_t diff_l;
	uint16_t diff_r;
	uint16_t l_wheel_encoder = get_l_wheel_encoder();
	uint16_t r_wheel_encoder = get_r_wheel_encoder();
	static uint8_t cnt_nochange = 0;

	if ( l_wheel_encoder == g_wheel_encoder_left_old){
		cnt_nochange ++;
	}
	else{
		cnt_nochange = 0;
	}

	if(cnt_nochange > 5){
		// �p���X��60ms�X�V�Ȃ���Β�~�����Ƃ݂Ȃ�
		g_flg_body_stp_jdg = 1;
	}
	else{
		g_flg_body_stp_jdg = 0;
	}

	diff_l = l_wheel_encoder - g_wheel_encoder_left_old;
	diff_r = r_wheel_encoder - g_wheel_encoder_right_old;

	g_wheel_enccnt_left += (int16_t)diff_l;
	g_wheel_enccnt_right += (int16_t)diff_r;

	g_wheel_encoder_left_old = l_wheel_encoder;
	g_wheel_encoder_right_old = r_wheel_encoder;
}


/*
 * ck_wheel_moving
 * ���փu���[�L�������ėǂ����x���m�F����
 */
static void ck_wheel_moving(void){
	static uint16_t encCtrW1_diff = 0;
	static uint16_t encCtrW2_diff = 0;
	static uint16_t encCtrW1_old = 0;
	static uint16_t encCtrW2_old = 0;
	static uint8_t timer_count = 0;
	uint16_t encCtrW1 = get_r_wheel_encoder();
	uint16_t encCtrW2 = get_l_wheel_encoder();

	timer_count++;
	if (timer_count >= TIM_50MSEC) {
		timer_count = 0;

		encCtrW1_diff = abs(encCtrW1 - encCtrW1_old);
		encCtrW2_diff = abs(encCtrW2 - encCtrW2_old);

		encCtrW1_old = encCtrW1;
		encCtrW2_old = encCtrW2;
	}

	if((encCtrW1_diff < STOP_PALSE_SYNC) && (encCtrW2_diff < STOP_PALSE_SYNC)){
		g_flg_brake_permission = 1;
	}
	else{
		g_flg_brake_permission = 0;
	}
}


/*
 * calc_wheel_enc_spd
 * �ԗփG���R�[�_�̃p���X�X�V�Ԋu����ԑ̊p���x���Z�o
 */
static void calc_wheel_enc_spd(void){
	double body_angle_per_pulse = 2 * PI / (WHEEL_RATIO * PULS_WHEEL);	// �G���R�[�_1�p���X������̎ԑ̉�]�p(rad)
	double ave_cnt = ((double)get_l_wheel_enc_cnt() + (double)get_r_wheel_enc_cnt()) * 0.5;

	double act_body_vel_old = g_act_body_vel;
	if ( ave_cnt != 0){
		g_act_body_vel = body_angle_per_pulse * 1.0e6 / (TIM_CYCLE * ave_cnt);
	}
	if(g_flg_body_stp_jdg > 0){
		g_act_body_vel = 0;
	}

	// ���[�p�X�t�B���^��������
	g_act_body_vel_lpf_old = g_act_body_vel_lpf;
	g_act_body_vel_lpf = LPF_1order(g_act_body_vel, act_body_vel_old, g_act_body_vel_lpf, BODY_VEL_FIR, TimeIntSync_sec);
}


/*
 * calc_turn_encoder
 * �^�[���e�[�u���G���R�[�_�̃p���X�X�V�����擾
 */
static void calc_turn_encoder(void) {

	uint16_t diff_t;
	uint16_t turn_enc = get_turn_encoder();
	static uint8_t cnt_nochange = 0;

	if ( turn_enc == g_ttbl_encoder_old){
		cnt_nochange ++;
	}
	else{
		cnt_nochange = 0;
	}

	if(cnt_nochange > 5){
		// �p���X��60ms�X�V�Ȃ���Β�~�����Ƃ݂Ȃ�
		g_flg_ttbl_stp_jdg = 1;
	}
	else{
		g_flg_ttbl_stp_jdg = 0;
	}

	diff_t = turn_enc - g_ttbl_encoder_old;
	g_turntbl_enccnt += (int16_t)diff_t;
	g_ttbl_encoder_old = turn_enc;
}


/*
 * calc_ttbl_enc_spd
 * �^�[���e�[�u���G���R�[�_�̃p���X�X�V�Ԋu����^�[���e�[�u���p���x���Z�o
 */
static void calc_ttbl_enc_spd(void){

	double ttbl_angle_per_pulse = 2 * PI / TURNTBL_ENC;	//�^�[���e�[�u���G���R�[�_1�p���X������̉�]�p
	double turn_cnt = (double)get_turn_enc_cnt();
	double act_ttbl_vel_old = g_act_ttbl_vel;
	if ( turn_cnt != 0){
		g_act_ttbl_vel = ttbl_angle_per_pulse * 1.0e6 / (TIM_CYCLE_TTBL * turn_cnt);
	}
	if(g_flg_ttbl_stp_jdg > 0){
		g_act_ttbl_vel = 0;
	}

	// ���[�p�X�t�B���^��������
	g_act_ttbl_vel_lpf_old = g_act_ttbl_vel_lpf;
	g_act_ttbl_vel_lpf = LPF_1order(g_act_ttbl_vel, act_ttbl_vel_old, g_act_ttbl_vel_lpf, TTBL_VEL_FIR, TimeIntSync_sec);
}

/*
 * set_ttbl_target_pos
 * �^�[�Q�b�g�Ƃ���^�[���e�[�u���t�H�g�Z���T�ʒu���v�Z
 */
uint8_t set_ttbl_target_pos(uint8_t start_pos, uint8_t dir) {

	uint8_t ret_pos = 0;

	switch(start_pos) {
	case TRUN_POSI_ANG0:
		if (dir == R90T) {
			ret_pos = TRUN_POSI_ANG270;
		} else if (dir == L90T) {
			ret_pos = TRUN_POSI_ANG90;
		} else if (dir == RL180T) {
			ret_pos = TRUN_POSI_ANG180;
		} else {
			ret_pos = start_pos;
		}
		break;

	case TRUN_POSI_ANG90:
		if (dir == R90T) {
			ret_pos = TRUN_POSI_ANG0;
		} else if (dir == L90T) {
			ret_pos = TRUN_POSI_ANG180;
		} else if (dir == RL180T) {
			ret_pos = TRUN_POSI_ANG270;
		} else {
			ret_pos = start_pos;
		}
		break;

	case TRUN_POSI_ANG180:
		if (dir == R90T) {
			ret_pos = TRUN_POSI_ANG90;
		} else if (dir == L90T) {
			ret_pos = TRUN_POSI_ANG270;
		} else if (dir == RL180T) {
			ret_pos = TRUN_POSI_ANG0;
		} else {
			ret_pos = start_pos;
		}

		break;

	case TRUN_POSI_ANG270:
		if (dir == R90T) {
			ret_pos = TRUN_POSI_ANG180;
		} else if (dir == L90T) {
			ret_pos = TRUN_POSI_ANG0;
		} else if (dir == RL180T) {
			ret_pos = TRUN_POSI_ANG90;
		} else {
			ret_pos = start_pos;
		}
		break;

	default:
		//�ǂ̃Z���T�[���������Ă��Ȃ��ꍇ�A�ŏ��ɔ��������Z���T�[�Ŏ~�߂�
		ret_pos = TURN_SENS_ALL;
		break;
	}
	return ret_pos;
}

/*
 * Is_SyncTurnning
 * �V���N���^�[���̏�Ԃ�Ԃ�
 * �߂�l�F	TRUE �V���N���^�[����
 * 			FALSE�@�V���N���^�[�����ł͂Ȃ�
 */
uint8_t Is_SyncTurnning() {

	uint8_t ret = FALSE;

	if (g_syncturn_status == SYNC_TURN_ING) {
		ret = TRUE;
	}
	return ret;
}

/*
 * get_syncturn_status
 * �V���N���^�[���̃X�e�[�^�X��Ԃ�
 */
uint8_t get_syncturn_status() {
	return g_syncturn_status;
}

/*
 * syncturn_stop
 * �V���N���^�[�����~����
 */
void syncturn_stop() {

//	wheel_set_freerun(0);
//	wheel_set_brake(1);
//	wheel_set_dc_lock(0);
	wheel_set_speed(0, 0);
	turn_motor_stop();
	t_set_brake();
	sycturn_direction(STOP, 0);
}



/*
 * set_sync_nuc_command
 * NUC����̃R�}���h���Z�b�g
 * �V���N���^�[������NUC����̃��b�Z�[�W��M�̓x�ɃR�[�������
 */
void set_sync_nuc_command(uint8_t cmd, uint8_t sync_spd, uint8_t sync_dir, uint16_t pf_angle) {

	uint8_t cmd_mon = 0;

	//�s���R�}���h���K�[�h
	if((cmd < NO_REQ_SYNC_TURN) || (cmd > REQ_SYNC_TURN)){
		cmd = NO_REQ_SYNC_TURN;
		cmd_mon++;
	}
	if((sync_spd < LOW) || (sync_spd > HIGH)){
		sync_spd = LOW;
		cmd_mon++;
	}
	if((sync_dir < R90T) || (sync_dir > RL180T)){
		sync_dir = R90T;
		cmd_mon++;
	}
	if((pf_angle < 0) || (pf_angle > 3600)){
		pf_angle = 0;
		cmd_mon++;
	}

	if (cmd_mon != 0) {
		set_illg_command_flg();
	} else {
		reset_illg_command_flg();
	}

	//�R�}���h�Z�b�g
	g_command = cmd;
	g_sync_spd = sync_spd;
	g_sync_dir = sync_dir;
	g_pf_angle = pf_angle;
}

/*
 * set_sync_flg_send_comp
 * �V���N���^�[���I���X�e�[�^�X���M�σt���O���Z�b�g
 */
void set_sync_flg_send_comp(){
	g_flg_send_comp = 1;
}

void reset_sync_flg_send_comp(){
	g_flg_send_comp = 0;
}

uint8_t get_sync_flg_send_comp(){
	return g_flg_send_comp;
}

/*
 * set_sync_start_flag
 * �V���N���^�[���J�n�t���O���Z�b�g
 */
void set_sync_start_flg(){
	g_flg_syncturn_start = 1;
}

void reset_sync_start_flg(){
	g_flg_syncturn_start = 0;
}

uint8_t get_sync_start_flg(){
	return g_flg_syncturn_start;
}

/*
 * set_illg_command_flg
 * �s���R�}���h��M�t���O���Z�b�g
 */
void set_illg_command_flg(){
	g_flg_illg_command = 1;
}

void reset_illg_command_flg(){
	g_flg_illg_command = 0;
}

uint8_t get_illg_command_flg(){
	return g_flg_illg_command;
}

void set_sync_dump_req(){
	g_flg_sync_dump_req = 1;
}

void reset_sync_dump_req(){
	g_flg_sync_dump_req = 0;
}

uint8_t get_sync_dump_req(){
	return g_flg_sync_dump_req;
}

void set_sync_dump_comp(){
	g_flg_sync_dump_comp = 1;
}

void reset_sync_dump_comp(){
	g_flg_sync_dump_comp = 0;
}

uint8_t get_sync_dump_comp(){
	return g_flg_sync_dump_comp;
}

/*
 * syncturn_log_dump
 * �V���N���^�[�����ʂ̃_���v���o�͂���B
 */
void syncturn_log_dump() {
	extern void uart2_transmitte(char *p);

	// �����p�Ƀ^�[�����ʂ��o��
	char buf1[40];
	memset(buf1, 0x00, sizeof(buf1));
	sprintf(buf1, "Table Angle: %d [deg/10]\r\n", g_result_ttbl_angle);
	uart2_transmitte(buf1);

	memset(buf1, 0x00, sizeof(buf1));
	sprintf(buf1, "Synchro Error: %d [deg/10]\r\n", g_result_max_err);
	uart2_transmitte(buf1);

	// �^�[�����̏�ԗʃ��O���o��(�J���p)
#if SYNC_TURN_LOG_DUMP
	char buf2[600];

	memset(buf2, 0x00, sizeof(buf2));
	sprintf(buf2, "g_time,g_pf_angle_dist_ln_log,"
			"g_trgt_angle,g_trgt_vel,"
			"g_ref_body_vel,g_ref_ttbl_vel,"
			"g_act_body_angle,g_act_body_vel_lpf,"
			"g_act_ttbl_angle,g_act_ttbl_vel_lpf,"
			"g_act_err_angle, g_act_err_vel_lpf,"
			"g_est_body_angle,"
			"g_flg_brk_angle_body,g_flg_brk_angle_ttbl"
			"\r\n");
	uart2_transmitte(buf2);
	for(int i = 0; i < LOGSIZE; i++) {
		memset(buf2, 0x00, sizeof(buf2));
		//             1   2   3   4   5   6   7   8   9   10  11  12  13  14  15
		sprintf(buf2, "%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d, %d\r\n",
				g_time_log[i],					// 1
				g_pf_angle_dist_ln_log[i],			// 2
				g_trgt_angle_log[i],			// 3
				g_trgt_vel_log[i],				// 4
				g_ref_body_vel_log[i],			// 5
				g_ref_ttbl_vel_log[i],			// 6
				g_act_body_angle_log[i],		// 7
				g_act_body_vel_lpf_log[i],		// 8
				g_act_ttbl_angle_log[i],		// 9
				g_act_ttbl_vel_lpf_log[i],		// 10
				g_act_err_angle_log[i],			// 11
				g_act_err_vel_lpf_log[i],		// 12
				g_est_body_angle_log[i],		// 13
				g_flg_brk_angle_body_log[i],	// 14
				g_flg_brk_angle_ttbl_log[i]		// 15
				);
		uart2_transmitte(buf2);
		if (i != 0 && g_time_log[i+1] == 0){
			break;
		}
	}
#endif
	set_sync_dump_comp();
}


/*
 * sync_status_ctrl
 * �V���N���^�[���X�e�[�^�X����
 */
static uint8_t sync_status_ctrl(uint8_t syncturn_status_z){

	uint8_t ret = syncturn_status_z;

	switch(syncturn_status_z){

	case SYNC_TURN_READY:
		if(g_command == REQ_SYNC_TURN){
			if ((ck_emerg_stop() == NOT_EMERGENCY) && (get_lift_pos() != LIFT_DOWN_STATE) && (get_illg_command_flg() == 0)){
				ret = SYNC_TURN_ING;
				set_sync_start_flg();
			}
			else{
				ret = SYNC_TURN_ILLG;
			}
		}
		break;

	case SYNC_TURN_ING:
		if((ck_emerg_stop() != NOT_EMERGENCY) || (get_illg_command_flg() != 0) || (g_command == NO_REQ_SYNC_TURN)){
			ret = SYNC_TURN_ILLG;
		}
		else if(g_time_from_brk >= WAIT_TIME) {
			if (syncturn_result() == 1){
			ret = SYNC_TURN_COMP;
			}
			else if (syncturn_result() != 1){
				ret = SYNC_TURN_ERR_COMP;
			}
		}
		break;

	case SYNC_TURN_COMP:
		if((g_command == NO_REQ_SYNC_TURN) && (get_sync_flg_send_comp() == 1)){
			ret = SYNC_TURN_READY;
		}
		break;

	case SYNC_TURN_ERR_COMP:
		if((g_command == NO_REQ_SYNC_TURN) && (get_sync_flg_send_comp() == 1)){
			ret = SYNC_TURN_READY;
		}
		break;

	case SYNC_TURN_ILLG:
		if((g_command == NO_REQ_SYNC_TURN) && (get_sync_flg_send_comp() == 1)){
			ret = SYNC_TURN_READY;
		}
		break;

	default: //�����͒ʂ�Ȃ������[�j���O����̂��߂ɋL�q
		ret = SYNC_TURN_READY;
		break;
	}
	return ret;
}


/*
 * syncturn_ctrl
 * �V���N���^�[���̃��C���֐�
 */
void syncturn_ctrl() {

	uint8_t ttbl_sens_pos = get_turn_sensor_pos();

	// �V���N���^�[���X�e�[�^�X�X�V
	g_syncturn_status = sync_status_ctrl(g_syncturn_status);

	switch(g_syncturn_status){
	case SYNC_TURN_READY:
		syncturn_init();
		break;

	case SYNC_TURN_ING:
		if (get_sync_start_flg() == 1){	//�@ING�Ɉڍs����1job�ڂ̂ݎ��s
			syncturn_pram_set(g_sync_spd, g_sync_dir);
			g_pf_start_angle = (short)g_pf_angle;
			g_diff_rad = get_diff_from_stdangele(g_sync_dir, g_pf_angle);
			sycturn_direction(g_sync_dir, g_diff_rad);
			g_ttbl_sens_target_pos = set_ttbl_target_pos(ttbl_sens_pos, g_sync_dir);
			log_reset();
			// �u���[�L����
//			wheel_set_brake(0);
//			wheel_set_dc_lock(0);
			t_release_brake();
			// ���[�^�[�h���C�o�̕��׊����ݒ�
//			wheel_set_inertia(INERTIA1);
		}
		calc_wheel_encoder();
		ck_wheel_moving();
		calc_wheel_enc_spd();
		calc_turn_encoder();
		calc_ttbl_enc_spd();
		calc_angle();
		calc_pf_dist();
		calc_pf_liner();
		calc_body_angle_kalman();
		calc_turn_locus();
		calc_err_angle();
		judge_ttbl_finish(ttbl_sens_pos);
		judge_body_finish();
		brake_timer();
		sync_motor_ctrl();
		log_update();
		reset_sync_start_flg();
		g_time += TimeIntSync_ms;
		break;

	case SYNC_TURN_COMP:
		syncturn_stop();
		break;

	case SYNC_TURN_ERR_COMP:
		syncturn_stop();
		break;

	case SYNC_TURN_ILLG:
		syncturn_stop();
		break;
	}

}
