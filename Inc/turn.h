/*
 * turn.h
 *
 *  Created on: 2019/05/31
 *      Author: ayabe
 */

#ifndef TURN_H_
#define TURN_H_

void t_set_brake(void);
void t_release_brake(void);
void set_rotation_cw(void);
void set_rotation_ccw(void);
void turn_cntrl(void);
void turn_init(void);
void turn_relay_on(void);
void turn_relay_off(void);
void turn_alarm_reset(void);
void turn_alarm_recover(void);
void turn_motor_stop(void);
void turn_sensor_check(void);
void scan_turnmotor_alarm(void);
void turn_set_speed(uint16_t dac);
uint8_t ck_turn_motor_alarm(void);
void set_turn_flg_send_comp(void);
void reset_turn_flg_send_comp(void);
uint8_t get_turn_flg_send_comp(void);
uint8_t get_turn_status(void);
void turn_log_dump(void);
void set_turn_dump_req(void);
void reset_turn_dump_req(void);
uint8_t get_turn_dump_req(void);
void set_turn_dump_comp(void);
void reset_turn_dump_comp(void);
uint8_t get_turn_dump_comp(void);

#define TURN_ANG0_SENSOR	0x01
#define TURN_ANG90_SENSOR	0x02
#define TURN_ANG180_SENSOR	0x04
#define TURN_ANG270_SENSOR	0x08

//NUC�ʒm�p�X�e�[�^�X
#define ST_TURN_VEL_RUN		0x00	// ���x���䃂�[�h
#define ST_TURN_ANG_RUN		0x01	// �p�x���䃂�[�h���s��
#define ST_TURN_ANG_COMP	0x02	// �p�x���䃂�[�h����I��
#define ST_TURN_ANG_ERRCOMP	0x03	// �p�x���䃂�[�h�ُ�I��
//NUC����̎w���R�}���h
#define TURN_VEL_MODE		0x00	// ���x�w�����[�h
#define TURN_ANG_MODE		0x12	// �p�x�w�����[�h
#define TURN_NO_REQ			0x00	// �p�x�w���Ȃ�
#define TURN_0				0x01	// 0�x�̈ʒu�։�]
#define TURN_90				0x02	// 90�x�̈ʒu�։�]
#define TURN_180			0x03	// 180�x�̈ʒu�։�]
#define TURN_270			0x04	// 270�x�̈ʒu�։�]
#define TURN_LOW			0x00	// �ᑬ�ŉ�]
#define TURN_MID			0x01	// �����ŉ�]
#define TURN_HIGH			0x02	// �����ŉ�]
//��]�ʂ̃p�����[�^
typedef struct turn_ctrl_pram{
	double		reach_angle;	// 1.��]��(deg)
	double		v_max;			// 2.�ō���(rad/s)
	double		t_acc;			// 3.��������(sec)
	double		t_decel;		// 4.��������(sec)
	double		brake_angle;	// 5.�u���[�L�J�n�p(deg)
	double		approach_vel;	// 6.�A�v���[�`���x(rad/s)
}TURN_CTRL_PRAM;

//��ԗ�
typedef struct turn_variable{
	uint16_t	time;
	uint8_t		flg_cruise_end;
	double		cruise_end_time;
	double		trgt_angle;
	double		trgt_vel;
	double		trgt_acc;
	double		act_angle;
	double		act_vel;
	double		act_vel_old;
	double		act_vel_lpf;
	double		err_angle;
	double		err_vel;
	double		err_vel_old;
	double		err_vel_lpf;
	double		ref_vel;
	double		ref_vel_ff;
	double		ref_vel_fb;
	double		d_term;
	double		init_deff;		// �J�n���̊p�x
	double		sens_angle;		// �^�[�Q�b�g�̃t�H�g�Z���T�������������̉�]�p
	double		result_angle;	// �t�H�g�Z���T������̊p�x(�^�[�����ʂ̏o�͗p)
	uint8_t		recover_cnt;	// ���J�o���[������{��
}TRUN_VARIABLE;

#endif /* TURN_H_ */
