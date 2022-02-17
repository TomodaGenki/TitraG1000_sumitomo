/*
 * common_func.c
 *
 *  Created on: 2021/09/09
 *      Author: tomoda
 */
#include "main.h"
#include "common_func.h"
#include "conf.h"


#define TABLESIZE 128 //2�̏搔�ł��邱��
#define SUBBIT    8
#define SUBINDEX  (1 << SUBBIT)
#define I_PI      (TABLESIZE * SUBINDEX * 2)
#define I_HPI     (TABLESIZE * SUBINDEX)

const uint16_t sin_table[] = {
    0,   804,  1608,  2412,  3216,  4019,  4821,  5623,
 6424,  7223,  8022,  8820,  9616, 10411, 11204, 11996,
12785, 13573, 14359, 15142, 15924, 16703, 17479, 18253,
19024, 19792, 20557, 21319, 22078, 22834, 23586, 24334,
25079, 25820, 26557, 27291, 28020, 28745, 29465, 30181,
30893, 31600, 32302, 32999, 33692, 34379, 35061, 35738,
36409, 37075, 37736, 38390, 39039, 39682, 40319, 40950,
41575, 42194, 42806, 43411, 44011, 44603, 45189, 45768,
46340, 46905, 47464, 48014, 48558, 49095, 49624, 50145,
50659, 51166, 51664, 52155, 52638, 53113, 53580, 54039,
54490, 54933, 55367, 55794, 56211, 56620, 57021, 57413,
57797, 58171, 58537, 58895, 59243, 59582, 59913, 60234,
60546, 60850, 61144, 61429, 61704, 61970, 62227, 62475,
62713, 62942, 63161, 63371, 63571, 63762, 63943, 64114,
64276, 64428, 64570, 64703, 64826, 64939, 65042, 65136,
65219, 65293, 65357, 65412, 65456, 65491, 65515, 65530,
65535, 0
};

/*
 * mysin
 * ������Ԃ����C�u����
 * https://seesaawiki.jp/w/robolabo/d/%a5%b3%a5%f3%a5%d1%a5%af%a5%c8%bb%b0%b3%d1%b4%d8%bf%f4
 * ���́F�p�x[rad]
 */
double mysin(double x){
    int64_t ix, subix, sign, tval;

    ix = (int32_t)(x * (I_PI / PI));   //�P�ʕϊ�
    sign = ix & I_PI;              //��3,��4�ی��ł���
    ix &= (I_PI - 1);              //��1,��2�ی��Ɍ���
    if(ix > I_HPI) ix = I_PI - ix; //��1�ی��Ɍ���

    subix = ix & (SUBINDEX - 1);   //���`�⊮�ɗp����T�u�C���f�b�N�X
    ix >>= SUBBIT;                 //�e�[�u���ԍ��ɕϊ�

    //���`�⊮
    tval = ((int64_t)sin_table[ix]   * (SUBINDEX - subix)
          + (int64_t)sin_table[ix+1] * subix);
    return (sign ? -tval : tval) / (SUBINDEX * 65535.f);
}


/*
 * LPF_1order
 * �o�ꎟ�ϊ��ɂ�藣�U������1���x��t�B���^
 */
double LPF_1order(double input, double input_old, double output_old, double tau, double delta_t){
	//input:���͐M��
	//input_old:���͐M���̑O��l
	//output_old:�o�͐M���̑O��l
	//tau:���萔[sec]
	//delta_t:�������[sec]

	double output = 0;
	double coef_A = delta_t / (delta_t + 2 * tau);
	double coef_B = (delta_t - 2 * tau) / (delta_t + 2 * tau);

	output = coef_A * input + coef_A * input_old - coef_B * output_old;

	return output;
}

/*
 * CRC16 calculation
 * �I���G���^�����[�^�[�̃h���C�o�ŒʐM�̐������`�F�b�N�Ɏg�p�����CRC�̌v�Z�p�֐�
 */
uint16_t crc16_calc(uint8_t *buff, uint8_t sizeOfArray) {
	uint16_t crc16_registor = 0xFFFF;
	uint16_t tmp;
	int i;
	int shift_num;

	for (i = 0; i < sizeOfArray - 2; i++) {
		tmp = crc16_registor ^ buff[i];
		shift_num = 0;
		while (shift_num < 8) {
			if ((tmp & 1) == 1) {
				tmp = (tmp >> 1) ^ 0xA001;
			} else {
				tmp = (tmp >> 1);
			}
			shift_num++;
		}
		crc16_registor = tmp;
	}
	return crc16_registor;
}
