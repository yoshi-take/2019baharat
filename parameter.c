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
#include <math.h>							// ���l�v�Z

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
PRIVATE stSLA				st_Sla[SLA_TYPE_MAX];				// �X�����[�����̑��s�p�����[�^
PRIVATE	BOOL				bl_cntType		= false;			// �J�E���g�^�C�v(false:�T���Atrue:�ŒZ)

/* ============ */
/*  ���x�f�[�^  */
/* ============ */

	/* ���i���x�f�[�^ */
	PRIVATE	CONST stSPEED	f_StSpeedData[PARAM_MOVE_SPEED_MAX]	= {
		
		// �����x		�����x		�p���x		�p�����x
		{ 2000,			2000,		0,			0			},		// ���ᑬ(PARAM_VERY_SLOW)
		{ 2000,			2000,		0,			0			},		// �ᑬ(PARAM_SLOW)
		{ 2500,			3500,		0,			0			},		// �ʏ�(PARAM_NORMAL)
		{ 10000,		10000,		0,			0			},		// ����(PARAM_FAST)
		{ 12000,		10000,		0,			0			}		// ������(PARAM_VERY_FAST)
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
		{ 0,			0,			2000,		2000		},		// ���ᑬ(PARAM_VERY_SLOW)
		{ 0,			0,			2200,		2200		},		// �ᑬ(PARAM_SLOW)
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

	/* �΂ߑ��s���x�f�[�^ */
	PRIVATE	CONST stSPEED f_SkewSpeedData[PARAM_MOVE_SPEED_MAX] = {
		
		//	�����x		�����x		�p�����x	 �p�����x
		{ 2000,			2000,		0,			0,		},		// ���ᑬ(PARAM_VERY_SLOW)
		{ 2000,			2000,		0,			0,		},		// �ᑬ(PARAM_SLOW)
		{ 2000,			2000,		0,			0,		},		// �ʏ�(PARAM_NORMAL)
		{ 2000,			2000,		0,			0,		},		// ����(PARAM_FAST)
		{ 2000,			2000,		0,			0,		}		// ������(PARAM_VERY_FAST)
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
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.07f,		0.0f,		0.0f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.1f,		0.2f,		0.0f,	},		// PARAM_ACC		
			{0.28f,		0.0f,		0.0f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.1f,		0.2f,		0.0f,	},		// PARAM_CONST
			{0.0f,		0.0f,		0.0f,		0.0f,		1.7f,		0.0f,		1.0f,		0.2f,		0.08f,		0.0f,		3.0f,		0.1f,		0.2f,		0.0f,	},		// PARAM_DEC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_DEC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_SKEW_DEC
			{0.12f,		0.0f,		0.0f,		0.0f,		0,			0.0f,		0,			0,			0,			0.0f,		0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.1f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		4.0f,		0.15f,		0.08f,		0.0f,		2.0f,		0.0f,		0.015f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* �ᑬ(PARAM_SLOW) */
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.055f,	0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_ACC		
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_CONST
			{0.0f,		0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_DEC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_DEC
			{0.055f,	0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.0f,		0.0f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.0f,		0.0f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.0f,		0.0f,	},		// PARAM_SKEW_DEC
			{0.12f,		0.0f,		0.0f,		0.0f,		0,			0.0f,		0,			0,			0,			0.0f,		0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.1f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		4.0f,		0.15f,		0.08f,		0.0f,		2.0f,		0.0f,		0.015f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* �ʏ�(PARAM_NORMAL) */
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.055f,	0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_ACC		
			{0.0f,		0.23f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_CONST
			{0.0f,		0.15f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_DEC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_DEC
			{0.055f,	0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_DEC
			{0.12f,		0.0f,		0.0f,		0.0f,		0,			0.0f,		0,			0,			0,			0.0f,		0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.1f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		4.0f,		0.15f,		0.08f,		0.0f,		2.0f,		0.0f,		0.015f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* ����(PARAM_FAST) */
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.050f,	0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_ACC		
			{0.0f,		0.24f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_CONST
			{0.0f,		0.13f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_DEC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_DEC
			{0.055f,	0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_DEC
			{0.12f,		0.0f,		0.0f,		0.0f,		0,			0.0f,		0,			0,			0,			0.0f,		0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.1f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		4.0f,		0.15f,		0.08f,		0.0f,		2.0f,		0.0f,		0.015f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* ������(PARAM_VERY_FAST) */
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.05f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.0f,		0.0f,	},		// PARAM_ACC		
			{0.0f,		0.25f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.0f,		0.0f,	},		// PARAM_CONST
			{0.0f,		0.1f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.0f,		0.0f,	},		// PARAM_DEC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_DEC
			{0.055f,	0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_DEC
			{0.12f,		0.0f,		0.0f,		0.0f,		0,			0.0f,		0,			0,			0,			0.0f,		0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.1f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		4.0f,		0.15f,		0.08f,		0.0f,		2.0f,		0.0f,		0.015f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
	};
	
	/* ����Q�C���f�[�^ */
	PRIVATE CONST stGAIN f_TurnGainData[PARAM_MOVE_SPEED_MAX][PARAM_TURN_MAX] = {
		
		/* ���ᑬ(PARAM_VERY_SLOW) */
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.0f,		0.0f,		0.065f,		0.0f,		0.0f,		0.5f,		0.0f,		0.0f,		2.0f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.0f,		1.5f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.0f,		1.0f,		0.0f,		1.0f,		0.1f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* �ᑬ(PARAM_SLOW) */
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.0f,		0.0f,		0.065f,		0.0f,		0.0f,		0.5f,		0.0f,		0.0f,		2.0f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.0f,		1.5f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.0f,		1.0f,		0.0f,		1.0f,		0.1f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* �ʏ�(PARAM_NORMAL) */
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.0f,		0.0f,		0.06f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.5f,		0.0f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* ����(PARAM_FAST) */
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.0f,		0.0f,		0.06f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.5f,		0.0f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* ���ᑬ(PARAM_VERY_FAST) */
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.0f,		0.0f,		0.06f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.5f,		0.0f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
	};
	
	/* �X�����[���Q�C���f�[�^ */
	PRIVATE CONST stGAIN f_SlaGainData[PARAM_MOVE_SPEED_MAX][PARAM_SULA_MAX] = {
		
		/* ���ᑬ(PARAM_VERY_SLOW) */
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.0f,		0.0f,	},		// PARAM_ENTRY_SLA
			{0.0f,		0.3f,		0.01f,		0.18f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		0.3f,		0.0f,		0.2f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.0f,		0.0f,	},		// PARAM_EXIT_SLA
		},
		
		/* �ᑬ(PARAM_SLOW) */
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_ENTRY_SLA
			{0.0f,		0.3f,		0.013f,		0.13f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		0.3f,		0.0f,		0.20f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		0.3f,		0.0002f,	0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_EXIT_SLA
		},
		
		/* �ʏ�(PARAM_NORMAL) */
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_ENTRY_SLA
			{0.0f,		0.3f,		0.013f,		0.13f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		0.3f,		0.0f,		0.20f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		0.3f,		0.0002f,	0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_EXIT_SLA
		},
		
		/* ����(PARAM_FAST) */
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		4.0f,		0.0f,		0.48f,		0.01f	},		// PARAM_ENTRY_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.34f,		0.0f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.34f,		0.0f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.34f,		0.0f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_EXIT_SLA
		},
		
		/* ������(PARAM_VERY_FAST) */
		{// FF_�����x	FF_���x		FF_�p�����x	 FF_�p���x	  ���xkp	  ���xki	  �ʒukp	  �ʒuki	  �p���xkp	   �p���xki		�p�xkp		�p�xki		 ��kp	 	��kd
			{0.0f,		0.0f,		0.0f,		0.0f,		5.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		5.0f,		0.0f,		0.8f,		0.2f	},		// PARAM_ENTRY_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		5.0f,		0.0f,		0.0f,		0.0f,		1.8f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		5.0f,		0.0f,		0.0f,		0.0f,		1.8f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		5.0f,		0.0f,		0.0f,		0.0f,		1.8f,		0.0f,		5.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		5.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		5.0f,		0.0f,		0.8f,		0.2f,	},		// PARAM_EXIT_SLA
		},
	};
	
/* ================= */
/*  �X�����[���̋��� */
/* ================= */
	PRIVATE	CONST FLOAT f_SlaDistData[PARAM_MOVE_SPEED_MAX][SLA_CORR_DIST_MAX]	= {
		
		//�i������		�ޔ����� 
		{ 	0,			0,		},		// ���ᑬ(PARAM_VERY_SLOW)
		{ 	0,			0,		},		// �ᑬ(PARAM_SLOW)
		{ 	0,			0,		},		// �ʏ�(PARAM_NORMAL)
		{ 	0,			0,		},		// ����(PARAM_FAST)
		{ 	0,			0,		}		// ������(PARAM_VERY_FAST)
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
		case PARAM_HIT_WALL:											// �ǂ��Đ���		
			p_adr = &f_StSpeedData[en_Speed_st];
			break;
		
		case PARAM_SKEW_ACC:											// ������(�΂�)
		case PARAM_SKEW_CONST:											// ������(�΂�)
		case PARAM_SKEW_DEC:											// ������(�΂�)
			p_adr = &f_SkewSpeedData[en_Speed_st];
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
PUBLIC stSLA* PARAM_getSra(enSLA_TYPE en_mode)
{
	return &st_Sla[en_mode];
}

// *************************************************************************
//   �@�\		�F �X�����[���̑��s�p�����[�^���쐬����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �i�����x[mm/s]�C�p�����x[rad/s^2]�C��G[mm/s^2]�C�X�����[���^�C�v
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.9.8			TKR			�V�K
// *************************************************************************/
PUBLIC void PARAM_makeSla( FLOAT f_speed, FLOAT f_angAcc, FLOAT f_g, enSLA_TYPE en_mode, enPARAM_MOVE_SPEED en_speed){
	
	FLOAT	f_start_x;					// �J�nx�ʒu [mm]
	FLOAT	f_start_y;					// �J�ny�ʒu [mm]
	FLOAT	f_final_x;					// �ŏIx�ʒu [mm]
	FLOAT	f_final_y;					// �ŏIy�ʒu [mm]
	FLOAT	f_final_ang;				// �p�������̍ŏI�p�x [rad]	
	FLOAT	f_maxAngleV		= 0;		// �ő�p���x[rad/s]
	FLOAT	f_timeAcc		= 0;		// ��������[s]
	FLOAT	f_accAngle		= 0;		// �����p�x[rad]
	FLOAT	f_timeConst		= 0;		// ��������[s]
	FLOAT	f_constAngle	= 0;		// �����p�x[rad]
	FLOAT	f_ang			= 0;		// ���Z�p�A�p�x [rad]
	FLOAT	f_time			= 0;		// ���Z�p�A���� [s]
	FLOAT	f_x;						// ���Z�px�ʒu [mm]
	FLOAT	f_y;						// ���Z�py�ʒu [mm]
	USHORT	i = 0;						// ���[�v�p
	stSLA	*p_adr 			= &st_Sla[en_mode];	// �L�^���鑖�s�f�[�^

	//�f�o�b�O�p
	FLOAT	f_x_acc = 0;
	FLOAT	f_y_acc = 0;
	FLOAT	f_x_const = 0;
	FLOAT	f_y_const = 0;
	FLOAT	f_x_dec = 0;
	FLOAT	f_y_dec = 0;

	/* �X�����[���ɉ������ݒ�l����X�����[���ɕK�v�ȃp�����[�^�����Z���� */
	switch(en_mode){

		case SLA_90:
			f_start_x   = HALF_BLOCK;
			f_start_y   = 0.0f;
			f_final_x   = BLOCK;
			f_final_y   = HALF_BLOCK;
			f_final_ang = 90.0f* DEG_TO_RAD;
			break;

		case SLA_45:
			f_start_x   = HALF_BLOCK;
			f_start_y   = 0.0f;
			f_final_x   = BLOCK * 0.75f;
			f_final_y   = BLOCK * 0.75f;
			f_final_ang = 45.0f * DEG_TO_RAD;
			break;
			
		case SLA_N90:
			f_start_x   = HALF_BLOCK * 0.5f * 1.4142f;
			f_start_y   = 0.0f;
			f_final_x   = HALF_BLOCK * 1.4142f;
			f_final_y   = HALF_BLOCK * 0.5f * 1.4142f;
			f_final_ang = 90.0f * DEG_TO_RAD;
			break;
			
		case SLA_135:
			f_start_x   = HALF_BLOCK;
			f_start_y   = 0.0f;
			f_final_x   = BLOCK * 1.25f;
			f_final_y   = BLOCK * 0.25;
			f_final_ang = 135.0f * DEG_TO_RAD;
			break;
	}
#ifdef TEST
	printf("f_speed = %f[mm/s]\n\r",f_speed);
	printf("f_angAcc = %f[rad/s]\n\r",f_angAcc);	
	printf("f_g = %f[mm/s]\n\r",f_g);	
#endif

	/* �������p�x�̎Z�o */
	f_maxAngleV		= f_g / f_speed;								// �ő�p���x[rad/s] �i��[rad/s] = g[mm/s^2] / v[mm/s] �j
	f_timeAcc		= f_maxAngleV / f_angAcc;						// �ő�̊p���x�ɂȂ�܂ł̉�������[s]
	f_accAngle		= 0.5f * f_angAcc * f_timeAcc * f_timeAcc;		// �����������Ԃ̊p�x[rad] (��[rad] = 1/2 * a[rad/s^2] * t[s]^2 )
	f_constAngle	= f_final_ang - f_accAngle * 2;					// ���p���x�̋�Ԃ̊p�x[rad] (��[rad] = Total�p�x - �����p�x + �����p�x )
	f_timeConst		= f_constAngle / f_maxAngleV;					// �ő�̊p���x�œ��삷�鎞��[s]�i t[s] = ��[rad] / ��[rad/s] �j
	
#ifdef TEST	
	printf("f_maxAngleV = %f[rad/s]\n\r",f_maxAngleV);
	printf("f_timeAcc = %f[s]\n\r",f_timeAcc);
	printf("f_accAngle = %f[rad]\n\r",f_accAngle);
	printf("f_constAngle = %f[rad]\n\r",f_constAngle);
	printf("f_timeConst = %f[s]\n\r",f_timeConst);
#endif

	/* -------------------------------- */
	/*  �X�����[���������̈ʒu�����߂�  */
	/* -------------------------------- */
	/* ���W�J�n�ʒu */
	f_x		= f_start_x;
	f_y		= f_start_y;

	/* �������̍��W���Z */
	for( i=0; i<(USHORT)(f_timeAcc*1000); i++ ){				// [msec]
	
		f_time	=  0.001f * (FLOAT)i;							// ����[s]
		f_ang	=  0.5f * f_angAcc * f_time * f_time;			// �p�x[rad] (��[rad] = 1/2 * a[rad/s^2] * t[s]^2 )
		f_x		+= f_speed * (FLOAT)sin( f_ang ) * 0.001f;		// X���W[mm] 
		f_y		+= f_speed * (FLOAT)cos( f_ang ) * 0.001f;		// Y���W[mm]
	}
	
	f_x_acc = f_x;
	f_y_acc = f_y;
#ifdef TEST	
	printf("f_x_acc = %f\n\r",f_x_acc);
	printf("f_y_acc = %f\n\r",f_y_acc);
#endif
	/* �������̍��W���Z */
	for( i=0; i<(USHORT)(f_timeConst*1000); i++ ){				// [msec]
	
		f_time	 = 0.001f * (FLOAT)i;							// ����[s]
		f_ang	 = f_accAngle + f_maxAngleV * f_time;			// �p�x[rad] (��[rad] = ��[rad/s] * t[s] )
		f_x		+= f_speed * (FLOAT)sin( f_ang ) * 0.001f;		// X���W[mm] 
		f_y		+= f_speed * (FLOAT)cos( f_ang ) * 0.001f;		// Y���W[mm]
	}

	f_x_const = f_x ;
	f_y_const = f_y ;
#ifdef	TEST
	printf("f_x_const = %f\n\r",f_x_const);
	printf("f_y_const = %f\n\r",f_y_const);
#endif
	/* �������̍��W���Z */
	for( i=0; i<(USHORT)(f_timeAcc*1000); i++ ){				// [msec]
	
		f_time	 = 0.001f * (FLOAT)i;							// ����[s]
		f_ang	 = f_accAngle + f_constAngle +0.5f * f_angAcc * f_time * f_time;	// �p�x[rad] (��[rad] = 1/2 * a[rad/s^2] * t[s]^2 )
		f_x		+= f_speed * (FLOAT)sin( f_ang ) * 0.001f;		// X���W[mm] 
		f_y		+= f_speed * (FLOAT)cos( f_ang ) * 0.001f;		// Y���W[mm]
	}
	
	f_x_dec = f_x ;
	f_y_dec = f_y ;
#ifdef TEST	
	printf("f_x_dec = %f\n\r",f_x_dec);
	printf("f_y_dec = %f\n\r",f_y_dec);
#endif
	/* ---------------------------- */
	/*  �X�����[���p�p�����[�^�쐬  */
	/* ---------------------------- */
    	p_adr->f_speed						= f_speed;										// �i�����x[mm/s]
    	p_adr->f_angAcc						= f_angAcc * RAD_TO_DEG ;						// �p�����x[deg/s]
    	p_adr->f_angvel						= f_maxAngleV * RAD_TO_DEG;						// �ő�p���x���Z�o �ő�p���x[deg/s]
    	p_adr->us_accAngvelTime				= (USHORT)( f_timeAcc * 1000.0f );				// �p��������[msec]
    	p_adr->us_constAngvelTime			= (USHORT)( f_timeConst * 1000.0f );			// ���p������[msec]
    	p_adr->f_ang_AccEnd					= f_accAngle * RAD_TO_DEG;						// �p���������p�x[deg]
    	p_adr->f_ang_ConstEnd				= ( f_accAngle + f_constAngle ) * RAD_TO_DEG;	// ���p���x�����p�x[deg]
		p_adr->f_ang_Total					= f_final_ang * RAD_TO_DEG;						// �S�ړ��p�x[deg]
#ifdef TEST	
	printf("\n========PARAMETER======\n\r");
	printf("f_speed = %f[mm/s]\n\r",f_speed);
	printf("f_angAcc = %f[deg/s]\n\r",f_angAcc*180.0f/3.1416f);
	printf("us_accAngvelTime = %f[ms]\n\r",f_timeAcc*1000);
	printf("us_constAngvelTime = %f[ms]\n\r",f_timeConst*1000);
	printf("f_ang_AccEnd = %f[deg]\n\r",f_accAngle*180.0f/3.1416f);
	printf("f_ang_ConstEnd = %f[deg/s]\n\r",( f_accAngle + f_constAngle )*180.0f/3.1416f);
	printf("f_ang_Total = %f[deg]\n\r",f_final_ang*180.0f/3.1416f);
#endif	
	/* �K�v�Ȑi���Ƒޏo�̋������Z�o���� */
	switch(en_mode){
		case SLA_90:
			p_adr->f_escapeLen = f_final_x - f_x + PARAM_getSlaCorrDist(en_speed,SLA_ESCAPE_ADD);
			p_adr->f_entryLen  = f_final_y - f_y + PARAM_getSlaCorrDist(en_speed,SLA_ENTRY_ADD);
			break;

		case SLA_45:
			p_adr->f_escapeLen = 1.4142f * ( f_final_x - f_x );
			p_adr->f_entryLen  = f_final_y - f_y - ( f_final_x - f_x );
			break;

		case SLA_N90:
			p_adr->f_escapeLen = f_final_x - f_x;
			p_adr->f_entryLen  = f_final_y - f_y;
			break;

		case SLA_135:
			p_adr->f_escapeLen = 1.4142f * ( f_final_x - f_x );
			p_adr->f_entryLen  = f_final_y - f_y + ( f_final_x - f_x );
			break;
	}
	
}