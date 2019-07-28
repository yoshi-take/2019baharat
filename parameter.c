// *************************************************************************
//   ���{�b�g��	�F Baharat�i�o�n���b�g�j
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F parameter
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.5			TKR			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O
#include <stdio.h>							// �W�����o��
#include <common_define.h>					// common_define

#include <parameter.h>						// parameter

//**************************************************
// ��`�idefine�j
//**************************************************
/* �C���f�b�N�X���Z�Ɏg�p */
#define GET_INDEX_ST(i)			( i - PARAM_ST_TOP - 1 )		// ���i�p�̃C���f�b�N�X���擾
#define GET_INDEX_TURN(i)		( i - PARAM_TURN_TOP - 1 )		// ����p�̃C���f�b�N�X���擾
#define GET_INDEX_SLA(i)		( i - PARAM_SLA_TOP - 1 )		// �X�����[���p�̃C���f�b�N�X���擾

//**************************************************
// �\���́istruct�j
//**************************************************

//**************************************************
// �O���[�o���ϐ�
//**************************************************
PRIVATE	enPARAM_MOVE_SPEED	en_Speed_st		= PARAM_NORMAL;		// ���i���̈ړ����x�^�C�v
PRIVATE	enPARAM_MOVE_SPEED	en_Speed_turn	= PARAM_NORMAL;		// ���񎞂̈ړ����x�^�C�v
PRIVATE	enPARAM_MOVE_SPEED	en_Speed_sla	= PARAM_NORMAL;		// �X�����[�����̈ړ����x�^�C�v
PRIVATE	BOOL				bl_cntType		= false;			// �J�E���g�^�C�v(false:�T���Atrue:�ŒZ)

/* ============ */
/*  ���x�f�[�^  */
/* ============ */

	/* ���i���x�f�[�^ */
	PRIVATE	CONST stSPEED	f_StSpeedData[PARAM_MOVE_SPEED_MAX]	= {
		
		// �����x		�����x		�p���x		�p�����x
		{ 800,			1000,		0,			0			},		// ���ᑬ(PARAM_VERY_SLOW)
		{ 2000,			2000,		0,			0			},		// �ᑬ(PARAM_SLOW)
		{ 2000,			2500,		0,			0			},		// �ʏ�(PARAM_NORMAL)
		{ 2000,			2000,		0,			0			},		// ����(PARAM_FAST)
		{ 2000,			2000,		0,			0			}		// ������(PARAM_VERY_FAST)
	};
	
	/* ���i���x�f�[�^(cos�ߎ�) */
	PRIVATE	CONST stSPEED	f_StSpeedData_Smooth[PARAM_MOVE_SPEED_MAX]	= {
		
		// �ő�����x	�ő匸���x	�ő�p���x	�ő�p�����x
		{ 800,			1000,		0,			0			},		// ���ᑬ(PARAM_VERY_SLOW)
		{ 2500,			2500,		0,			0			},		// �ᑬ(PARAM_SLOW)
		{ 2000,			2500,		0,			0			},		// �ʏ�(PARAM_NORMAL)
		{ 3000,			3000,		0,			0			},		// ����(PARAM_FAST)
		{ 3200,			3200,		0,			0			}		// ������(PARAM_VERY_FAST)
	};
	
	/* ���񑬓x�f�[�^ */
	PRIVATE	CONST stSPEED	f_TurnSpeedData[PARAM_MOVE_SPEED_MAX]	= {
		
		// �����x		�����x		�p���x		�p�����x
		{ 0,			0,			2000,		3000		},		// ���ᑬ(PARAM_VERY_SLOW)
		{ 0,			0,			1800,		1800		},		// �ᑬ(PARAM_SLOW)
		{ 0,			0,			1800,		1800		},		// �ʏ�(PARAM_NORMAL)
		{ 0,			0,			1800,		1800		},		// ����(PARAM_FAST)
		{ 0,			0,			1800,		1800		}		// ������(PARAM_VERY_FAST)
	};

	/* �X�����[�����x�f�[�^ */
	PRIVATE CONST stSPEED f_SlaSpeedData[PARAM_MOVE_SPEED_MAX] = {
		
		//	�����x		�����x		�p�����x		�p�����x
		{ 1800,			1800,		1800,			1800,		},		// ���ᑬ(PARAM_VERY_SLOW)
		{ 1800,			1800,		1800,			1800,		},		// �ᑬ(PARAM_SLOW)
		{ 1800,			1800,		1800,			1800,		},		// �ʏ�(PARAM_NORMAL)
		{ 3000,			3000,		1800,			1800,		},		// ����(PARAM_FAST)
		{ 1800,			1800,		1800,			1800,		}		// ������(PARAM_VERY_FAST)
	};


/* ============== */
/*  �Q�C���f�[�^  */
/* ============== */
	// �y�A�h�o�C�X�z 
	//    �������Q�C���̃p�����[�^���𑝂₵�����ꍇ�́AstGAIN�̃����o�Ɓ��̃f�[�^�𑝂₷������OK�ł��B
	//    PARAM_getGain()�Ńp�����[�^�̃A�h���X���擾���āA�ǉ����������o���Q�Ƃ��ĉ������B

	/* ���i�Q�C���f�[�^ */
	PRIVATE CONST stGAIN f_StGainData[PARAM_MOVE_SPEED_MAX][PARAM_ST_MAX] = {
		
		/* ���ᑬ(PARAM_VERY_SLOW) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p���xki	��kp		��kd
			{0.1f,		1.0f,		0.0f,		0.0f,		0.08f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_ACC		
			{0.0f,		1.0f,		0.0f,		0.0f,		0.08f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_CONST
			{0.0f,		1.0f,		4.0f,		0.15f,		0.08f,		2.0f,		0.0f,		0.015f,		0.0f,	},		// PARAM_DEC
			{0.0f,		0.7f,		5.5f,		0.01f,		0.04f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.7f,		5.5f,		0.01f,		0.04f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.7f,		5.5f,		0.01f,		0.04f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_DEC
			{0.0f,		0.7f,		5.5f,		0.01f,		0.04f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.7f,		5.5f,		0.01f,		0.04f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.7f,		5.5f,		0.01f,		0.04f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_SKEW_DEC
			{0.12f,		0,			0,			0,			0,			0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.1f,		1.0f,		0.0f,		0.0f,		0.08f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		1.0f,		0.0f,		0.0f,		0.08f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		1.0f,		4.0f,		0.15f,		0.08f,		2.0f,		0.0f,		0.015f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* �ᑬ(PARAM_SLOW) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p���xki	��kp		��kd
			{0.05f,		2.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.0f,		0.0f,	},		// PARAM_ACC		
			{0.0f,		2.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.0f,		0.0f,	},		// PARAM_CONST
			{0.0f,		2.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.0f,		0.0f,	},		// PARAM_DEC
			{0.1f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.7f,		7.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_DEC
			{0.05f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.0f,		0.0f,	},		// PARAM_SKEW_ACC
			{0.0f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.0f,		0.0f,	},		// PARAM_SKEW_CONST
			{0.0f,		5.0f,		15.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.0f,		0.0f,	},		// PARAM_SKEW_DEC
			{0.12f,		0,			0,			0,			0,			0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.07f,		1.5f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* �ʏ�(PARAM_NORMAL) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p���xki	��kp		��kd
			{0.05f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.1f,	},		// PARAM_ACC		
			{0.0f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.1f,	},		// PARAM_CONST
			{0.0f,		5.0f,		15.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.1f,	},		// PARAM_DEC
			{0.1f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.7f,		7.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_DEC
			{0.1f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.7f,		7.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_DEC
			{0.12f,		0,			0,			0,			0,			0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.07f,		1.5f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* ����(PARAM_FAST) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p���xki	��kp		��kd
			{0.05f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_ACC		
			{0.0f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_CONST
			{0.0f,		5.0f,		15.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.2f,		0.0f,	},		// PARAM_DEC
			{0.1f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.7f,		7.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_DEC
			{0.1f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.7f,		7.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_DEC
			{0.12f,		0,			0,			0,			0,			0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.07f,		1.5f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* ������(PARAM_VERY_FAST) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p���xki	��kp		��kd
			{0.08f,		5.0f,		0.0f,		0.0f,		2.0f,		5.0f,		0.01f,		0.8f,		0.15f,	},		// PARAM_ACC		
			{0.0f,		5.0f,		0.0f,		0.0f,		2.0f,		5.0f,		0.01f,		0.8f,		0.15f,	},		// PARAM_CONST
			{0.0f,		5.0f,		15.0f,		0.0f,		1.0f,		5.0f,		0.01f,		0.4f,		0.15f,	},		// PARAM_DEC
			{0.1f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.7f,		7.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_DEC
			{0.1f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.7f,		7.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_DEC
			{0.12f,		0,			0,			0,			0,			0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.07f,		1.5f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
	};
	
	/* ����Q�C���f�[�^ */
	PRIVATE CONST stGAIN f_TurnGainData[PARAM_MOVE_SPEED_MAX][PARAM_TURN_MAX] = {
		
		/* ���ᑬ(PARAM_VERY_SLOW) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p�xki		��kp		��kd
			{0.06f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.5f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* �ᑬ(PARAM_SLOW) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p�xki		��kp		��kd
			{0.06f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.5f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* �ʏ�(PARAM_NORMAL) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p�xki		��kp		��kd
			{0.06f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.5f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* ����(PARAM_FAST) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p�xki		��kp		��kd
			{0.06f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.5f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* ���ᑬ(PARAM_VERY_FAST) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p�xki		��kp		��kd
			{0.06f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.5f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
	};
	
	/* �X�����[���Q�C���f�[�^ */
	PRIVATE CONST stGAIN f_SlaGainData[PARAM_MOVE_SPEED_MAX][PARAM_SULA_MAX] = {
		
		/* ���ᑬ(PARAM_VERY_SLOW) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p���xki	��kp		��kd
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f	},		// PARAM_ENTRY_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.34f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.34f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.34f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_EXIT_SLA
		},
		
		/* �ᑬ(PARAM_SLOW) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p���xki	��kp		��kd
			{0.0f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f	},		// PARAM_ENTRY_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		2.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		2.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		2.0f,		4.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_EXIT_SLA
		},
		
		/* �ʏ�(PARAM_NORMAL) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p���xki	��kp		��kd
			{0.0f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f	},		// PARAM_ENTRY_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		2.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		2.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		2.0f,		4.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_EXIT_SLA
		},
		
		/* ����(PARAM_FAST) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p���xki	��kp		��kd
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f	},		// PARAM_ENTRY_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.34f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.34f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.34f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_EXIT_SLA
		},
		
		/* ������(PARAM_VERY_FAST) */
		{	// FF		���xkp		�ʒukp		�ʒuki		�p���xkp	�p�xkp		�p���xki	��kp		��kd
			{0.0f,		5.0f,		0.0f,		0.0f,		1.0f,		5.0f,		0.0f,		0.8f,		0.2f	},		// PARAM_ENTRY_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		1.8f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		1.8f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		1.8f,		5.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		1.0f,		5.0f,		0.0f,		0.8f,		0.2f,	},		// PARAM_EXIT_SLA
		},
	};
	
/* ================= */
/*  �X�����[���̋��� */
/* ================= */
	PRIVATE	CONST FLOAT f_SlaDistData[PARAM_MOVE_SPEED_MAX][SLA_CORR_DIST_MAX]	= {
		
		//�i������		�ޔ����� 
		{ 	2,			2,		},		// ���ᑬ(PARAM_VERY_SLOW)
		{ 	2,			2,		},		// �ᑬ(PARAM_SLOW)
		{ 	2,			2,		},		// �ʏ�(PARAM_NORMAL)
		{ 	2,			2,		},		// ����(PARAM_FAST)
		{ 	-2,			-2,		}		// ������(PARAM_VERY_FAST)
	};

// *************************************************************************
//   �@�\		�F ������@�ɑΉ��������쑬�x��ݒ肷��
//   ����		�F ����O�ɂ��炩���ߐݒ肵�Ă���
//   ����		�F ���x�l��Q�C���l���擾����ۂɁA�x�̓��쑬�x�̃p�����[�^���擾���邩�����肷�邽�߂Ɏg�p����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.3.11			�g�c			�V�K
// *************************************************************************/
PUBLIC void PARAM_setSpeedType( enPARAM_MODE en_mode, enPARAM_MOVE_SPEED en_speed ){
	switch( en_mode ){
		
		case PARAM_ST:
			en_Speed_st = en_speed;
			break;
		
		case PARAM_TURN:
			en_Speed_turn = en_speed;
			break;
		
		case PARAM_SLA:
			en_Speed_sla = en_speed;
			break;
			
		default:
			printf("�ݒ肵�����x�̃p�����[�^�^�C�v������܂��� \n\r");
			break;
	}
}


// *************************************************************************
//   �@�\		�F ���x�f�[�^�̃|�C���^���擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.4.3			�g�c			�V�K
// *************************************************************************/
PUBLIC	CONST stSPEED* PARAM_getSpeed( enPARAM_MODE en_mode ){
	
	const stSPEED*	p_adr;
	
	switch( en_mode ){
		
		case PARAM_ST:													// ���i
		case PARAM_ACC:													// ������(���i)
		case PARAM_CONST:												// ������(���i)
		case PARAM_DEC:													// ������(���i)
		case PARAM_BACK_ACC:											// ������(��i)
		case PARAM_BACK_CONST:											// ������(��i)
		case PARAM_BACK_DEC:											// ������(��i)
		case PARAM_SKEW_ACC:											// ������(�΂�)
		case PARAM_SKEW_CONST:											// ������(�΂�)
		case PARAM_SKEW_DEC:											// ������(�΂�)
		case PARAM_HIT_WALL:											// �ǂ��Đ���		
			p_adr = &f_StSpeedData[en_Speed_st];
			break;
			
		case PARAM_ACC_SMOOTH:											// ������(���i cos�ߎ�)
		case PARAM_CONST_SMOOTH:										// ������(���i cos�ߎ�)
		case PARAM_DEC_SMOOTH:											// ������(���i cos�ߎ�)
			p_adr = &f_StSpeedData_Smooth[en_Speed_st];
			break;
			
		case PARAM_TURN:												// ����
		case PARAM_ACC_TURN:											// ������(���n�M����)
		case PARAM_CONST_TURN:											// ������(���n�M����)
		case PARAM_DEC_TURN:											// ������(���n�M����)
			p_adr = &f_TurnSpeedData[en_Speed_turn];
			break;
			
		case PARAM_SLA:													// �X�����[��
		case PARAM_ENTRY_SURA:											// �X�����[���O�̑O�i����(�X�����[��)
		case PARAM_ACC_SURA:											// ������(�X�����[��)
		case PARAM_CONST_SURA:											// ������(�X�����[��)
		case PARAM_DEC_SURA:											// ������(�X�����[��)
		case PARAM_EXIT_SURA:											// �X�����[����̑O�i����(�X�����[��)
			p_adr = &f_SlaSpeedData[en_Speed_sla];
			break;
			
		default:
			printf("�ݒ肵�����x�^�C�v������܂���\n\r");
			p_adr = &f_SlaSpeedData[en_Speed_sla];
			break;
	}
	
	return p_adr;

}


// *************************************************************************
//   �@�\		�F ���x�f�[�^�̃|�C���^���擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.4.3			�g�c			�V�K
// *************************************************************************/
PUBLIC	CONST stGAIN* PARAM_getGain( enPARAM_MODE en_mode ){
	
	const stGAIN*	p_adr;
	
	switch( en_mode ){
		
		case PARAM_ACC:													// ������(���i)
		case PARAM_CONST:												// ������(���i)
		case PARAM_DEC:													// ������(���i)
		case PARAM_BACK_ACC:											// ������(��i)
		case PARAM_BACK_CONST:											// ������(��i)
		case PARAM_BACK_DEC:											// ������(��i)
		case PARAM_SKEW_ACC:											// ������(�΂�)
		case PARAM_SKEW_CONST:											// ������(�΂�)
		case PARAM_SKEW_DEC:											// ������(�΂�)
		case PARAM_HIT_WALL:											// �ǂ��Đ���
		case PARAM_ACC_SMOOTH:											// ������(���i cos�ߎ�)
		case PARAM_CONST_SMOOTH:										// ������(���i cos�ߎ�)
		case PARAM_DEC_SMOOTH:											// ������(���i cos�ߎ�)
			p_adr = &f_StGainData[en_Speed_st][GET_INDEX_ST( en_mode )];
			break;
			
		case PARAM_ACC_TURN:											// ������(���n�M����)
		case PARAM_CONST_TURN:											// ������(���n�M����)
		case PARAM_DEC_TURN:											// ������(���n�M����)
			p_adr = &f_TurnGainData[en_Speed_turn][GET_INDEX_TURN( en_mode )];
			break;
		
		case PARAM_ENTRY_SURA:											// �X�����[���O�̑O�i����(�X�����[��)
		case PARAM_ACC_SURA:											// ������(�X�����[��)
		case PARAM_CONST_SURA:											// ������(�X�����[��)
		case PARAM_DEC_SURA:											// ������(�X�����[��)
		case PARAM_EXIT_SURA:											// �X�����[����̑O�i����(�X�����[��)
			p_adr = &f_SlaGainData[en_Speed_sla][GET_INDEX_SLA( en_mode )];
			break;
		
		default:														// Err�A�Ƃ肠�����E�E�E�i�������j���h�����߁j
			printf("�ݒ肵���Q�C���^�C�v������܂��� \n\r");
			p_adr = &f_SlaGainData[en_Speed_sla][GET_INDEX_SLA( en_mode )];
			break;
	}
	
	return p_adr;
}	
			

// *************************************************************************
//   �@�\		�F �J�E���g��������[mm]�ɕϊ�����
//   ����		�F �������Ȃǂ̃I�[�o�[�w�b�h�����邽�߁A1step���𒲐����ĒT�����͋�捇�킹���s���B
//   ����		�F ADJ_1STEP_SEARCH�͌������킹
//   ����		�F �Ȃ�
//   �Ԃ�l		�F [mm]
// **************************    ��    ��    *******************************
// 		v1.0		2018.3.28			�g�c			�V�K
//		v2.0		2018.9.9			�g�c			1717�d�l�ɕύX
// *************************************************************************/
PUBLIC FLOAT F_CNT2MM( LONG l_cnt ){
	
	/* �T�� */
	if( bl_cntType == false ){
		return (FLOAT)l_cnt * DIST_1STEP( ADJ_1STEP_SEARCH ) * ENC_CONV;
	
	/* �ŒZ */
	}else{
		return (FLOAT)l_cnt * DIST_1STEP( ADJ_1STEP_DIRECT ) * ENC_CONV;
	
	}
}


// *************************************************************************
//   �@�\		�F 1step������̃J�E���g���s���ꍇ�̃^�C�v��ݒ肷��
//   ����		�F �������Ȃǂ̃I�[�o�[�w�b�h�����邽�߁A1step���𒲐����ĒT�����͋�捇�킹���s���B
//   ����		�F ���x�l��Q�C���l���擾����ۂɁA�x�̓��쑬�x�̃p�����[�^���擾���邩�����肷�邽�߂Ɏg�p����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.3.25			�g�c			�V�K
// *************************************************************************/
PUBLIC void PARAM_setCntType( BOOL bl_type )
{
	bl_cntType = bl_type;
}

// *************************************************************************
//   �@�\		�F �X�����[���̑ޔ������Ɛi�������̕␳
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ���x�^�C�v�C�i��or�ޔ�
//   �Ԃ�l		�F �␳����[mm]
// **************************    ��    ��    *******************************
// 		v1.0		2018.11.30			�g�c			�V�K
// *************************************************************************/
PUBLIC FLOAT PARAM_getSlaCorrDist( enPARAM_MOVE_SPEED en_speed , enSlaCorrDist en_dist)
{
	return f_SlaDistData[en_speed][en_dist];	
}

// *************************************************************************
//   �@�\		�F �X�����[���̑��s�p�����[�^�̊i�[��A�h���X���擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �X�����[���^�C�v
//   �Ԃ�l		�F �X�����[���f�[�^�A�h���X
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.01			�O��			�V�K
// *************************************************************************/
#if 0
PUBLIC stSLA* PARAM_getSra(enSLA_TYPE en_mode)
{
	return &st_Sla[en_mode];
}
#endif