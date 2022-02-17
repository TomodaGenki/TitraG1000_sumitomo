/*
 * tbt.c
 *
 *	Time Base Timer
 *		base task procedure
 *
 *  Created on: 2019/05/13
 *      Author: ayabe
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "wheel.h"
#include "nuc.h"
#include "syncturn.h"

/* External variables --------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/******************************************************************************/
/*           TBT procedure    						      */
/******************************************************************************/

void tbt(void){
	enc_pulse_tim_cnt_reset();					// �G���R�[�_�p���X�p�^�C�}�̃I�[�o�[�t���[�h�~

	if (Is_SyncTurnning()) {
		return;
	}
}
