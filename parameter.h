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

// ���d�R���p�C���}�~
#ifndef _PARAMETER_H
#define _PARAMETER_H

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>					// ��`
#include <common_define.h>				// ���ʒ�`
#include <iodefine.h>					// I/O
#include <stdio.h>						// �W�����o��
		

//**************************************************
// ��`�i�@�\�I���j                               
//**************************************************
//#define FUNC_DIST_AUTO_THRESH											// �Z���T臒l�`���[�j���O�@�\�i��`�F�L���A����`�F�����i�Z���T�̌Œ�l���g�p�����j�j

//**************************************************
// ��`�idefine�j
//**************************************************
/* �񒲐��p�����[�^ */
#define PI					( 3.14159f )			// ��
#define	ENC_CONV			( 0.2222f )				// �s�j�I��8T�C�X�p�[36T

/* �����p�����[�^ */
/*�� hal.c����ڐA*/
#define TIRE_R						( 25.0f )								// �^�C�����a [mm]
#define ROTATE_PULSE				( 2048 )								// ���[�^�[1���̃p���X��

#define ADJ_1STEP_SEARCH			( 1 )									// 1step�̒����Q�C���A�T�����s�p  (������グ��Ƒ�R�i��)
#define ADJ_1STEP_DIRECT			( 1 )								// 1step�̒����Q�C���ADrive���s�p  (������グ��Ƒ�R�i��)
#define DIST_1STEP(adj)				( PI * TIRE_R / ROTATE_PULSE * adj)		// 1�p���X�Ői�ދ��� [mm]

#define ENTRY_ADD					(2)										// �i�������̒ǉ�4
#define ESCAPE_ADD					(2)									// �ޔ������̒ǉ�5

/* ���H���ł̑҂����� */
#define MAP_TURN_WAIT				( 100 )								// ���M�n����T���̓���؂�ւ��҂�����
#define MAP_SLA_WAIT				( 150 )								// �X�����[���T���̓���؂�ւ��҂�����

/* ���s���x */
#define SEN_BACK_CHK_SPEED			( 180 ) 							// �Z���T�`���[�j���O�̂��߂̈ړ��ő呬�x[mm/s]

/* ���� */
#define MOT_BACK_SEN_ADJ			( 73.0f )							// �ǁ`��+a�܂Łi�Z���T�I�[�g�`���[�j���O�Ɏg�p����j

/* �A�h���X */
#define	ADR_MAP						( 0x00100000 )						// ���H�o�b�N�A�b�v�p�A�h���X
#define	ADR_SEN						( 0x00101000 )						// �Z���T�p�f�[�^�t���b�V���A�h���X

/* �����Z���T(���ω��ȊO) */
#define SEN_WAIT_CNT				( 175 )								// �Z���T�̔�������҂��i�����l�j
#define DIST_NO_WALL_DIV_FILTER		( 30 )								// �ǂȂ��Ƃ��鍷���l
#define DIST_REF_UP					( 400 )								// �ǂȂ��Ɣ��f����ۂɊ�l�ɉ��Z����l
#define DIST_NEAR_WALL				( 800 )								// �N�����̃`���[�j���O������O�ǂ�臒l

/* �����Z���T(���ω�) */
#define R_FRONT_WALL_GAIN			( 1.0f )							// �E�O�ǃQ�C���i��l�ɑ΂��āj�A�ǌ��m�l
#define L_FRONT_WALL_GAIN			( 1.0f )							// ���O�ǃQ�C���i��l�ɑ΂��āj�A�ǌ��m�l
//#define R_45_WALL_GAIN				( 1.0f )							// �E45�x�Q�C���i��l�ɑ΂��āj�A�ǌ��m�l
#define R_SIDE_WALL_GAIN			( 1.0f )							// �E���ǃQ�C���i��l�ɑ΂��āj�A�ǌ��m�l
//#define L_45_WALL_GAIN				( 1.0f )							// ��45�x�Q�C���i��l�ɑ΂��āj�A�ǌ��m�l
#define L_SIDE_WALL_GAIN			( 1.0f )							// �����ǃQ�C���i��l�ɑ΂��āj�A�ǌ��m�l
#define R_FRONT_WALL_CTRL_GAIN		( 1.0f )							// �E�O�ǃQ�C���i��l�ɑ΂��āj�A����ȏ�߂��Ɛ��䂷��l
#define L_FRONT_WALL_CTRL_GAIN		( 1.0f )							// ���O�ǃQ�C���i��l�ɑ΂��āj�A����ȏ�߂��Ɛ��䂷��l
#define R_FRONT_WALL_NO_CTRL_GAIN	( 1.0f )							// �E�O�ǃQ�C���i��l�ɑ΂��āj�A����ȏ�߂��Ɛ��䂵�Ȃ��l
#define L_FRONT_WALL_NO_CTRL_GAIN	( 1.0f )							// ���O�ǃQ�C���i��l�ɑ΂��āj�A����ȏ�߂��Ɛ��䂵�Ȃ��l
#define R_FRONT_WALL_HIT_GAIN		( 1.0f )							// �E�O�ǃQ�C���i��l�ɑ΂��āj�A�ǂɓ������Ă��Ă����������Ȃ��l�i�O�ǂƃ}�E�X�Ԃ���2mm�̎��̒l�j
#define L_FRONT_WALL_HIT_GAIN		( 1.0f )							// ���O�ǃQ�C���i��l�ɑ΂��āj�A�ǂɓ������Ă��Ă����������Ȃ��l�i�O�ǂƃ}�E�X�Ԃ���2mm�̎��̒l�j
#define R_FRONT_SKEW_ERR1_GAIN		( 0.0f )							// �E�O�ǁA�΂ߑ��s���̕␳臒l1
#define R_FRONT_SKEW_ERR2_GAIN		( 0.0f )							// �E�O�ǁA�΂ߑ��s���̕␳臒l2
#define R_FRONT_SKEW_ERR3_GAIN		( 0.0f )							// �E�O�ǁA�΂ߑ��s���̕␳臒l3
#define L_FRONT_SKEW_ERR1_GAIN		( 0.0f )							// ���O�ǁA�΂ߑ��s���̕␳臒l1
#define L_FRONT_SKEW_ERR2_GAIN		( 0.0f )							// ���O�ǁA�΂ߑ��s���̕␳臒l2
#define L_FRONT_SKEW_ERR3_GAIN		( 0.0f )							// ���O�ǁA�΂ߑ��s���̕␳臒l3

/* ���̃Z���T�l�́AFUNC_DIST_AUTO_THRESH���L���Ȃ��FALSH�̃f�[�^�ŏ㏑������āA�����Ȃ�ΐ����l�Ƃ��Ďg�p����� */
#define R_FRONT_REF					( 379 )							// �E�O�ǁA��l
#define L_FRONT_REF					( 300 )							// ���O�ǁA��l
//#define R_45_REF					( 580 )							// �E45�x�A��l
//#define L_45_REF					( 440 )							// ��45�x�A��l
#define R_SIDE_REF					( 173 )							// �E���ǁA��l
#define L_SIDE_REF					( 206 )							// �����ǁA��l
#define R_FRONT_WALL				( 43 )							// �E�O�ǁA�ǌ��m�l
#define L_FRONT_WALL				( 53 )							// ���O�ǁA�ǌ��m�l
//#define R_45_WALL					( 270 )							// �E45�x�A�ǌ��m�l
#define R_SIDE_WALL					( 63 )							// �E���ǁA�ǌ��m�l
//#define L_45_WALL					( 180 )							// ��45�x�A�ǌ��m�l
#define L_SIDE_WALL					( 81 )							// �����ǁA�ǌ��m�l
#define R_FRONT_WALL_CTRL			( 82 )							// �E�O�ǁA����ȏ�߂��Ɛ��䂷��l
#define L_FRONT_WALL_CTRL			( 100 )							// ���O�ǁA����ȏ�߂��Ɛ��䂷��l
#define R_FRONT_WALL_NO_CTRL		( 390 )							// �E�O�ǁA����ȏ�߂��Ɛ��䂵�Ȃ��l
#define L_FRONT_WALL_NO_CTRL		( 320 )							// ���O�ǁA����ȏ�߂��Ɛ��䂵�Ȃ��l
#define R_FRONT_WALL_HIT			( 1050 )						// �E�O�ǁA�ǂɓ������Ă��Ă����������Ȃ��l�i�O�ǂƃ}�E�X�Ԃ���2mm�̎��̒l�j
#define L_FRONT_WALL_HIT			( 1550 )						// ���O�ǁA�ǂɓ������Ă��Ă����������Ȃ��l�i�O�ǂƃ}�E�X�Ԃ���2mm�̎��̒l�j

/* ���O */
#define CTRL_LOG				( 1800 )							// 1msec���ɋL�^���鐧�䃍�O�̌�
#define CTRL_LOG_CYCLE			( 2 )								// ���̋L�^����[msec]�i1��菬�����l��NG�j
#define SET_LOG					( 100 )								// �ݒ肵������f�[�^�̃��O��
#define DIST_LOG				( 5 )								// �����Z���T�̃��O�̌�
#define POS_LOG					( 5 )								// 1msec���ɋL�^����ʒu��񃍃O�̌�
#define POS_LOG_INTERVAL		( 5 )								// ��msec���ɋL�^

#define GYRO_SCALE_FACTOR			(16.4f)							
#define TEMP_SCALE_FACTOR			(333.87f)	
#define ACC_SCALE_FACTOR			(8192.0f)						

//**************************************************
// �O���[�o���ϐ�
//**************************************************


//**************************************************
// �񋓑́ienum�j
//**************************************************

/* ������@ */
typedef enum{
	
	/* ========================================== */ 
	/*  �p�����[�^���擾����ۂɎg�p����V���{��  */ 
	/* ========================================== */ 
	/* ---------- */
	/*  ���i����  */
	/* ---------- */
	PARAM_ST_TOP = 0,				// �J�E���g�p
	// �� �����ǉ�����ꍇ�ɂ͂��̊ԂɋL��

		PARAM_ACC,					// ������(���i)
		PARAM_CONST,				// ������(���i)
		PARAM_DEC,					// ������(���i)
		PARAM_BACK_ACC,				// ������(��i)
		PARAM_BACK_CONST,			// ������(��i)
		PARAM_BACK_DEC,				// ������(��i)
		PARAM_SKEW_ACC,				// ������(�΂�)
		PARAM_SKEW_CONST,			// ������(�΂�)
		PARAM_SKEW_DEC,				// ������(�΂�)
		PARAM_HIT_WALL,				// �ǂ��Đ���
		PARAM_ACC_SMOOTH,			// ������(���i cos�ߎ�)
		PARAM_CONST_SMOOTH,			// ������(���i cos�ߎ�)
		PARAM_DEC_SMOOTH,			// ������(���i cos�ߎ�)

	// ��  �����ǉ�����ꍇ�ɂ͂��̊ԂɋL��
	PARAM_ST_BTM,					// �J�E���g�p
	
	/* -------- */
	/*  �^�[��  */
	/* -------- */
	PARAM_TURN_TOP,					// �J�E���g�p
	// ��  �����ǉ�����ꍇ�ɂ͂��̊ԂɋL��

		PARAM_ACC_TURN,				// ������(���n�M����)
		PARAM_CONST_TURN,			// ������(���n�M����)
		PARAM_DEC_TURN,				// ������(���n�M����)

	// ��  �����ǉ�����ꍇ�ɂ͂��̊ԂɋL��
	PARAM_TURN_BTM,					// �J�E���g�p
	
	/* ------------ */
	/*  �X�����[��  */
	/* ------------ */
	PARAM_SLA_TOP,					// �J�E���g�p
	// ��  �����ǉ�����ꍇ�ɂ͂��̊ԂɋL��

		PARAM_ENTRY_SURA,			// �X�����[���O�̑O�i����(�X�����[��)
		PARAM_ACC_SURA,				// ������(�X�����[��)
		PARAM_CONST_SURA,			// ������(�X�����[��)
		PARAM_DEC_SURA,				// ������(�X�����[��)
		PARAM_EXIT_SURA,			// �X�����[����̑O�i����(�X�����[��)

	// ��  �����ǉ�����ꍇ�ɂ͂��̊ԂɋL��
	PARAM_SLA_BTM,					// �J�E���g�p
	
	
	/* ===================================================================== */ 
	/*  PARAM_setGainType()�ɂă��[�h�����߂�ۂɈ����Ƃ��Ďg�p����V���{��  */ 
	/* ===================================================================== */ 
	PARAM_ST,						// ���i����
	PARAM_TURN,						// ���񐧌�
	PARAM_SLA,						// �X�����[������
	
	
	/* ====================================================== */ 
	/*  �쐬����f�[�^�����J�E���g���邽�߂Ɏg�p����V���{��  */ 
	/* ====================================================== */ 
	PARAM_ST_MAX		= PARAM_ST_BTM   - PARAM_ST_TOP,		// ���i�ő吔
	PARAM_TURN_MAX		= PARAM_TURN_BTM - PARAM_TURN_TOP,		// ����ő吔
	PARAM_SULA_MAX		= PARAM_SLA_BTM  - PARAM_SLA_TOP,		// �X�����[���ő吔
	
	
	PARAM_NC = 0xff,
	
}enPARAM_MODE;


/* ���쑬�x */
typedef enum{
	
	PARAM_VERY_SLOW = 0,	// ���ᑬ
	PARAM_SLOW,				// �ᑬ
	PARAM_NORMAL,			// �ʏ�
	PARAM_FAST,				// ����
	PARAM_VERY_FAST,		// ������
	
	PARAM_MOVE_SPEED_MAX
	
}enPARAM_MOVE_SPEED;

typedef enum{
	SLA_ENTRY_ADD = 0,
	SLA_ESCAPE_ADD,
	SLA_CORR_DIST_MAX,
}enSlaCorrDist;

//**************************************************
// �\���́istruct�j
//**************************************************
/* ���x�f�[�^ */
typedef struct{
	FLOAT			f_acc;					// �����x�i�������j
	FLOAT			f_dec;					// �����x�i�������j
	FLOAT			f_accAngle;				// �p�����x�i�������j
	FLOAT			f_decAngle;				// �p�����x�i�������j
}stSPEED;

/* �Q�C�� */
typedef struct{
	FLOAT			f_FF;					// �t�B�[�h�t�H���[�h
	FLOAT 			f_FB_speed_kp;			// �t�B�[�h�o�b�N�A���x ��ᐧ��
	FLOAT			f_FB_dist_kp;			// �t�B�[�h�o�b�N�A���� ��ᐧ��
	FLOAT 			f_FB_dist_ki;			// �t�B�[�h�o�b�N�A���� �ϕ�����
	FLOAT			f_FB_angleS_kp;			// �t�B�[�h�o�b�N�A�p���x ��ᐧ��
	FLOAT			f_FB_angle_kp;			// �t�B�[�h�o�b�N�A�p�x ��ᐧ��
	FLOAT			f_FB_angle_ki;			// �t�B�[�h�o�b�N�A�p�x �ϕ�����
	FLOAT			f_FB_wall_kp;			// �t�B�[�h�o�b�N�A�� ��ᐧ��
	FLOAT			f_FB_wall_kd;			// �t�B�[�h�o�b�N�A�� ��������
}stGAIN;

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
PUBLIC 	void 	PARAM_setCntType( BOOL bl_type );
PUBLIC 	FLOAT 	F_CNT2MM( LONG l_cnt );
PUBLIC	CONST 	stGAIN* PARAM_getGain( enPARAM_MODE en_mode );
PUBLIC	CONST 	stSPEED* PARAM_getSpeed( enPARAM_MODE en_mode );
PUBLIC 	FLOAT PARAM_getSlaCorrDist( enPARAM_MOVE_SPEED en_speed , enSlaCorrDist en_dist);
//PUBLIC stSLA* PARAM_getSra(enSLA_TYPE en_mode);

#endif