/*
 * conf.h
 *
 *  Created on: 2019/09/12
 *      Author: yuki
 */

#ifndef CONF_H_
#define CONF_H_

// ----firmware version ----------------------
#define ver_length 8
// farmware_version	"SPC.004.000.000.000"
// �d������A���W���[�o�[�W�����A�}�C�i�[�o�[�W�����A�o�b�`�o�[�W�����̏��ɒ�`
// �o�[�W������� 000 ~ 999 ��2byte�̐����l�Ƃ��đ��M����
#define	DEST_HIGH	0x00
#define	DEST_LOW	0x04
#define MAJOR_HIGH	0x00
#define MAJOR_LOW	0x00
#define MINOR_HIGH	0x00
#define MINOR_LOW	0x00
#define BATCH_HIGH	0x00
#define BATCH_LOW	0x00


// �e��ݒ�؂�ւ��p�X�C�b�`

// �ߐڃZ���T�[�ݒ�p�X�C�b�`
// 0 : �ߐڃZ���T�[�s�g�p
// 1 : �O���ߐڃZ���T�[�̂ݎg�p
// 2 : ����ߐڃZ���T�[�̂ݎg�p
// 3 : �����̋ߐڃZ���T�[���g�p
#define LIDAR_USE 0

// �o�b�e���[�I���X�C�b�`
// 0 : NEC�o�b�e���[
// 1 : ���c�o�b�e���[
#define BATTERY_TYPE	0

// �u���[�L�������A���F�_�������邩�ǂ�����I������X�C�b�`
// 0 : �����@�@1 : �L��
#define USE_BRAKE_RELEASE	1

// ��m�@�̎g�p�I���X�C�b�`
// 0 : �����@�@1 : �L��
#define SOUND_USE	1

// ���`�����u�̎g�p�I���X�C�b�`
// 0 : �����@�@ 1 : �L��
#define OPTCOM_USE 0

// ���ח�����̑I���X�C�b�`
// 0 : �����@�@ 1 : �L��
#define USE_LOAD_MEASURE	0

// �V���N���^�[�����O�@�\�̑I���X�C�b�`
// 0 : �����@�@ 1 : �L��
#define SYNC_TURN_LOG_DUMP	0

// �^�[���e�[�u�����O�@�\�̑I���X�C�b�`
// 0 : �����@�@ 1 : �L��
#define TURN_LOG_DUMP	0

// ���s�p���[�^�[���O�@�\�̑I���X�C�b�`
// 0 : �����@�@ 1 : �L��
#define WHEEL_LOG_DUMP	0

// ���s�p���[�^�[�e�X�g�@�\�̑I���X�C�b�`
// 0 : �����@�@ 1 : �L��
#define WHEEL_TEST		1

// �V���N���^�[���������ʏo�͂̉ۃX�C�b�`
// 0 : �����@�@ 1 : �L��
#define SYNC_RESULT_OUT	0

// �~����
#define PI	3.1415926f

// ���s�p���[�^�[�̃M����I��
#define WHEEL_GEAR_RATIO	30

// ���ւ̒��a[m]
#define WHEEL_DIAMETER	0.155

// �ԗֈ��]������̃G���R�[�_�p���X��
#define PULS_WHEEL		900

// NUC����̑��x�w�ߍő�l
#define MAX_NUC_REF		30000

// ���x�w�ߍő厞�̎ԑ�[m/s]
#define MAX_NUC_SPEED_WHEEL 2.0

//�^�[���e�[�u��1��]�������̃p���X��
#define	TURNTBL_ENC		12666.7

#endif /* CONF_H_ */
