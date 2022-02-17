/*
 * syncturn.h
 *
 *  Created on: 2020/11/27
 *      Author: ROBO TAKUMI
 */

#ifndef SYNCTURN_H_
#define SYNCTURN_H_

#define Bool int

//�V���N���^�[���X�e�[�^�X
#define	SYNC_TURN_READY	0x00		//�V���N���^�[���ҋ@��
#define	SYNC_TURN_ING	0x01		//�V���N���^�[����
#define SYNC_TURN_COMP	0x02		//�V���N���^�[������
#define SYNC_TURN_ERR_COMP	0x03	//�V���N���^�[���K��O����
#define SYNC_TURN_ILLG	0x07		//�V���N���^�[���ُ�I��
//NUC�v���@�V���N���^�[���R�}���h
#define NO_REQ_SYNC_TURN 0x00	//�V���N���^�[����v��
#define REQ_SYNC_TURN	 0x11 	//�V���N���^�[�����{�v��

//NUC�v���@��]�������
typedef enum  {
		R90T = 0x01,
		L90T,
		RL180T,
		STOP
} KIND_OF_TURN_DIR;

//NUC�v���@��]���x���
typedef enum  {
		LOW = 0x01,
		MID,
		HIGH
} KIND_OF_TURN_SPD;

//�f�o�C�X���
typedef enum motor_kind {
	WHEEL_RIGHT,
	WHEEL_LEFT,
	TURN_TABLE,
	SYNC_WHEEL
} MOTOR_KIND;

//�L�����u���[�V�����萔
typedef struct sync_ctrl_tbl {
	double		reach_angle;		//1.�^�[����(rad)
	double		turn_time;			//2.�^�[������(sec)
	int16_t		brake_body_angle;	//3.�u���[�L�J�n�ԑ̊p (angle/10)
	double		brake_ttbl_angle;	//4.�e�[�u���Z���T�[������A�e�[�u����i�߂鋗�� rad
} SYNC_CTRL_TBL;

//�t�B�[�h�o�b�N�p�����[�^
typedef struct sync_fb_pram{
	double		Kp_body;				//1.
	double		Ti_body;				//2.
	double		Td_body;				//3.
	double		eta_body;				//4.
	double		Kp_ttbl;				//5.
	double		Ti_ttbl;				//6.
	double		Td_ttbl;				//7.
	double		eta_ttbl;				//8.
}SYNC_FB_PRAM;

//�t���f���p�����[�^
typedef struct sync_inv_mdl{
	double		inv_mdl_body_gain_rise;		//1.
	double		inv_mdl_body_omega_rise;	//2.
	double		inv_mdl_body_zeta_rise;		//3.
	double		inv_mdl_body_gain_fall;		//4.
	double		inv_mdl_body_omega_fall;	//5.
	double		inv_mdl_body_zeta_fall;		//6.
	double		inv_mdl_ttbl_gain_rise;		//7.
	double		inv_mdl_ttbl_omega_rise;	//8.
	double		inv_mdl_ttbl_zeta_rise;		//9.
	double		inv_mdl_ttbl_gain_fall;		//10.
	double		inv_mdl_ttbl_omega_fall;	//11.
	double		inv_mdl_ttbl_zeta_fall;		//12.
}SYNC_INV_MDL;


//���f���p�����[�^
typedef struct sync_ttbl_mdl{
	double		mdl_ttbl_gain_90;	//1.90�x�^�[������gain
	double		mdl_ttbl_omega_90;	//2.90�x�^�[������omega
	double		mdl_ttbl_zeta_90;	//3.90�x�^�[������zeta
	double		mdl_ttbl_gain_180;	//4.180�x�^�[������gain
	double		mdl_ttbl_omega_180;	//5.180�x�^�[������omega
	double		mdl_ttbl_zeta_180;	//6.180�x�^�[������zeta
}SYNC_TTBL_MDL;


void syncturn_ctrl(void);
uint8_t Is_SyncTurnning(void);
void set_sync_nuc_command(uint8_t, uint8_t, uint8_t, uint16_t);
uint8_t get_syncturn_status(void);
void set_sync_flg_send_comp(void);
void reset_sync_flg_send_comp(void);
uint8_t get_sync_flg_send_comp(void);
void set_sync_start_flg(void);
void reset_sync_start_flg(void);
uint8_t get_sync_start_flg(void);
void set_illg_command_flg(void);
void reset_illg_command_flg(void);
uint8_t get_illg_command_flg(void);
void syncturn_log_dump(void);
void set_sync_dump_req(void);
void reset_sync_dump_req(void);
uint8_t get_sync_dump_req(void);
void set_sync_dump_comp(void);
void reset_sync_dump_comp(void);
uint8_t get_sync_dump_comp(void);

#endif /* SYNCTURN_H_ */
