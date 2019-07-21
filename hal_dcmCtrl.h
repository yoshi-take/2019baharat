// *************************************************************************
//   ���{�b�g��	�F Baharat�i�o�n���b�g�j
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F ���[�^�[����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.11			TKR			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/
// ���d�R���p�C���h�~
#ifndef	_HAL_DCM_CTRL_H
#define	_HAL_DCM_CTRL_H

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>					// ��`
#include <common_define.h>				// ���ʒ�`
#include <iodefine.h>					// I/O
#include <stdio.h>						// �W�����o��


//**************************************************
// ��`�idefine�j
//**************************************************

//**************************************************
// �񋓑́ienum�j
//**************************************************
/* ���䓮��^�C�v */
typedef enum{
	CTRL_ACC,				// [00] ������(���i)
	CTRL_CONST,				// [01] ������(���i)
	CTRL_DEC,				// [02] ������(���i)

	CTRL_SKEW_ACC,			// [03] �΂߉�����(���i)
	CTRL_SKEW_CONST,		// [04] �΂ߓ�����(���i)
	CTRL_SKEW_DEC,			// [05] �΂ߌ�����(���i)
	

	CTRL_ACC_TURN,			// [06] ������(���M�n����)
	CTRL_CONST_TURN,		// [07] ������(���M�n����)
	CTRL_DEC_TURN,			// [08] ������(���M�n����)
	
	CTRL_HIT_WALL,			// [09] �ǂ��Đ���
	
	CTRL_ACC_SMOOTH,		// [10] ������(���i cos�ߎ�)
	CTRL_CONST_SMOOTH,		// [11] ������(���i cos�ߎ�)
	CTRL_DEC_SMOOTH,		// [12] ������(���i)

	CTRL_ACC_SLA,			// [13] ������(�X�����[��)
	CTRL_CONST_SLA,			// [14] ������(�X�����[��)
	CTRL_DEC_SLA,			// [15] ������(�X�����[��)
	
	CTRL_ENTRY_SLA,			// [16] �X�����[���O�̑O�i����(�X�����[��)
	CTRL_EXIT_SLA,			// [17] �X�����[����̑O�i����(�X�����[��)
	
	CTRL_MAX,

}enCTRL_TYPE;

//**************************************************
// �\���́istruct�j
//**************************************************
/* ����f�[�^ */
typedef struct{
	enCTRL_TYPE		en_type;		// ����^�C�v
	FLOAT			f_time;			// �ڕW���� [sec]
	FLOAT			f_acc;			// [���x����]   �����x[mm/s2]
	FLOAT			f_now;			// [���x����]   ���ݑ��x[mm/s]
	FLOAT			f_trgt;			// [���x����]   �ŏI���x[mm/s]
	FLOAT			f_nowDist;		// [��������]   ���݋���[mm]
	FLOAT			f_dist;			// [��������]   �ŏI����[mm]
	FLOAT			f_accAngleS;	// [�p���x����] �p�����x[rad/s2]
	FLOAT			f_nowAngleS;	// [�p���x����] ���݊p���x[rad/s]
	FLOAT			f_trgtAngleS;	// [�p���x����] �ŏI�p���x[rad/s]
	FLOAT			f_nowAngle;		// [�p�x����]   ���݊p�x[rad]
	FLOAT			f_angle;		// [�p������]   �ŏI�p�x[rad]
}stCTRL_DATA;


//**************************************************
// �O���[�o���ϐ�(extern)
//**************************************************
extern PUBLIC FLOAT			f_Time;							// ���쎞��[msec]
extern PUBLIC  volatile FLOAT  f_NowDist;        			// [��������]�@���݋���             �i1[msec]���ɍX�V�����j
extern PUBLIC FLOAT			f_TrgtSpeed;					// [���x����]   ���ݑ��x�iDCMC��FB�ڕW���x�j	�i1[msec]���ɍX�V�����j
extern PUBLIC  volatile FLOAT  f_NowAngle;			       	// [�p�x����]�@���݊p�x             �i1[msec]���ɍX�V�����j
extern PUBLIC FLOAT			f_TrgtAngleS;					// [�p���x����] ���݊p���x�iDCMC��FB�ڕW���x�j	�i1[msec]���ɍX�V�����j

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
//PUBLIC void CTRL_setData(stDCMC_DATA* p_data);
PUBLIC void CTRL_clrData(void);
PUBLIC void CTRL_stop(void);
PUBLIC void CTRL_polCtrl(void);
PUBLIC void CTRL_showLog(void);




#endif