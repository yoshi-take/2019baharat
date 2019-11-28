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
// ��`�idefine�j                               
//**************************************************
//#define FUNC_DIST_AUTO_THRESH											// �Z���T臒l�`���[�j���O�@�\�i��`�F�L���A����`�F�����i�Z���T�̌Œ�l���g�p�����j�j

//**************************************************
// ��`�idefine�j	 �񒲐��p�����[�^ 
//**************************************************
#define PI					( 3.14159f )			// ��
#define	ENC_CONV			( 0.2f )				// �s�j�I��8T�C�X�p�[40T
#define DEG_TO_RAD	 	 	( 3.1416f/180.0f );		// �x�����ʓx
#define RAD_TO_DEG  		( 180.0f/3.1416f );		// �ʓx���x��

/* ���H���@ */
#define HALF_BLOCK					( 90.0f )							// ����� [mm]
#define BLOCK						( 180.0f )							// �P��� [mm]
#define HALF_BLOCK_SKEW				( 127.28f )							// �΂ߔ���� [mm]
#define BLOCK_SKEW					( 254.56f )							// �΂߂P��� [mm]

/* �A�h���X */
#define	ADR_MAP						( 0x00100000 )						// ���H�o�b�N�A�b�v�p�A�h���X
#define	ADR_SEN						( 0x00101000 )						// �Z���T�p�f�[�^�t���b�V���A�h���X

/* �W���C���C���x�C�����x�̃X�P�[�� */
#define GYRO_SCALE_FACTOR			(16.4f)								// �}2000[dps]
#define TEMP_SCALE_FACTOR			(333.87f)							// 
#define ACC_SCALE_FACTOR			(8192.0f)							// �}4[g]

//**************************************************
// ��`�idefine�j	�`���[�j���O���K�v�ȃp�����[�^ 
//**************************************************
/* ���H�T�C�Y */
#define GOAL_MAP_X					( 7 )								// �S�[����X��搔�i�������j [���]
#define GOAL_MAP_Y					( 4 )								// �S�[����Y��搔�i�c�����j [���]
#define MAP_X_SIZE					( 16 )								// ���H��X��搔�i�������j [���]
#define MAP_Y_SIZE					( 16 )								// ���H��Y��搔�i�c�����j [���]

#define MAP_X_SIZE_REAL				( 16 )								// ���H�̎�X��搔�i�������j [���]
#define MAP_Y_SIZE_REAL				( 16 )								// ���H�̎�Y��搔�i�c�����j [���]

#define LIST_NUM					( 4096 )							// �R�}���h���s�̃��X�g��

/* ���J�l */
#define TIRE_R						( 23.03f )								// �^�C�����a [mm]
#define ROTATE_PULSE				( 2048 )								// ���[�^�[1���̃p���X��

#define ADJ_1STEP_SEARCH			( 1 )									// 1step�̒����Q�C���A�T�����s�p  (������グ��Ƒ�R�i��)
#define ADJ_1STEP_DIRECT			( 1 )									// 1step�̒����Q�C���ADrive���s�p  (������グ��Ƒ�R�i��)
#define DIST_1STEP(adj)				( PI * TIRE_R / ROTATE_PULSE * adj)		// 1�p���X�Ői�ދ��� [mm]

#define MOVE_BACK_DIST				( 0.27f ) 							// �Ǔ��ē���Ō�ނ������� [���]
#define MOVE_BACK_DIST_SURA			( 0.27f ) 							// �Ǔ��ē���Ō�ނ������� [���]

/* �T�����̐K���Ď��� */
#define MAP_SLA_NUM_MAX				( 20 )								// �ő�A���X�����[��������܂ŋ����邩
#define MAP_TURN_NUM_MAX			( 1 )								// �ő�A�����������܂ŋ����邩

/* ���H���ł̑҂����� */
#define MAP_TURN_WAIT				( 100 )								// ���M�n����T���̓���؂�ւ��҂�����
#define MAP_SLA_WAIT				( 150 )								// �X�����[���T���̓���؂�ւ��҂�����

/* ���s���x */
#define MAP_SEARCH_SPEED			( 500 ) 							// �T�����s�̍ő呬�x[mm/s]
#define MAP_KNOWN_ACC_SPEED			( 1200 )							// ���m��ԉ�������Ƃ��̖ڕW���x[mm/s]
#define SEN_BACK_CHK_SPEED			( 180 ) 							// �Z���T�`���[�j���O�̂��߂̈ړ��ő呬�x[mm/s]

/* �t�B���^ */
#define SW_GYRO_FILTER_VAL_MIN		( -1.5f )							// �t�B���^�ŏ��l[dps]�i�ŏ��l�`�ő�l�̊Ԃ̓t�B���^��������j
#define SW_GYRO_FILTER_VAL_MAX		( 1.5f )							// �t�B���^�ő�l[dps]
#define SW_ACC_FILTER_VAL_MIN		( -0.008f )							// �t�B���^�ŏ��l[g]�i�ŏ��l�`�ő�l�̊Ԃ̓t�B���^��������j
#define SW_ACC_FILTER_VAL_MAX		( 0.008f )							// �t�B���^�ő�l[g]

/* �肩�������s��臒l */
#define	EXE_THRESH_R				( 1300 )							// �E����臒l
#define	EXE_THRESH_L				( 800 )							// ������臒l

/* �t�F�C���Z�[�t��臒l */
#define	FAIL_THRESH_ACC				( -35.0f )							// �����x�i���̒l�ȉ��Ŕ����j

/* ���� */
#define MOT_BACK_SEN_ADJ			( 73.0f )							// �ǁ`��+a�܂Łi�Z���T�I�[�g�`���[�j���O�Ɏg�p����j
#define MOT_WALL_EDGE_DIST			( 33.0f )							// �ǐ؂�Z���TOFF�`�ǂ܂�

/* �����Z���T(���ω��ȊO) */
#define SEN_WAIT_CNT				( 175 )								// �Z���T�̔�������҂��i�����l�j
#define DIST_NO_WALL_DIV_FILTER		( 30 )								// �ǂȂ��Ƃ��鍷���l
#define DIST_REF_UP					( 30 )								// �ǂȂ��Ɣ��f����ۂɊ�l�ɉ��Z����l
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
#define R_FRONT_REF					( 2450 )							// �E�O�ǁA��l
#define L_FRONT_REF					( 2280 )							// ���O�ǁA��l
//#define R_45_REF					( 580 )							// �E45�x�A��l
//#define L_45_REF					( 440 )							// ��45�x�A��l
#define R_SIDE_REF					( 928 )							// �E���ǁA��l
#define L_SIDE_REF					( 790 )							// �����ǁA��l
#define R_FRONT_WALL				( 656 )							// �E�O�ǁA�ǌ��m�l
#define L_FRONT_WALL				( 697 )							// ���O�ǁA�ǌ��m�l
//#define R_45_WALL					( 270 )							// �E45�x�A�ǌ��m�l
#define R_SIDE_WALL					( 437 )							// �E���ǁA�ǌ��m�l
//#define L_45_WALL					( 180 )							// ��45�x�A�ǌ��m�l
#define L_SIDE_WALL					( 560 )							// �����ǁA�ǌ��m�l
#define R_FRONT_WALL_CTRL			( 3334 )							// �E�O�ǁA����ȏ�߂��Ɛ��䂷��l
#define L_FRONT_WALL_CTRL			( 3158 )							// ���O�ǁA����ȏ�߂��Ɛ��䂷��l
#define R_FRONT_WALL_NO_CTRL		( 3386 )							// �E�O�ǁA����ȏ�߂��Ɛ��䂵�Ȃ��l
#define L_FRONT_WALL_NO_CTRL		( 2598 )							// ���O�ǁA����ȏ�߂��Ɛ��䂵�Ȃ��l
#define R_FRONT_WALL_HIT			( 1050 )						// �E�O�ǁA�ǂɓ������Ă��Ă����������Ȃ��l�i�O�ǂƃ}�E�X�Ԃ���2mm�̎��̒l�j
#define L_FRONT_WALL_HIT			( 1550 )						// ���O�ǁA�ǂɓ������Ă��Ă����������Ȃ��l�i�O�ǂƃ}�E�X�Ԃ���2mm�̎��̒l�j

/* ���O */
#define CTRL_LOG				( 700 )								// �L�^���鐧�䃍�O�̌�
#define CTRL_LOG_CYCLE			( 5 )								// ���̋L�^����[msec]�i1��菬�����l��NG�j
#define SET_LOG					( 100 )								// �ݒ肵������f�[�^�̃��O��
#define DIST_LOG				( 5 )								// �����Z���T�̃��O�̌�
#define POS_LOG					( 5 )								// �L�^����ʒu��񃍃O�̌�
#define POS_LOG_INTERVAL		( 5 )								// ��msec���ɋL�^

/* ���̑� */
#define TIME_THRE_WAIT			(2)									// ���̎��Ԉȏソ�ƌ������[�v���甲����
		


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
typedef struct {
	FLOAT			f_FF_speed_acc;			// �t�B�[�h�t�H���[�h
	FLOAT			f_FF_speed;				// �t�B�[�h�t�H���[�h
	FLOAT			f_FF_angleS_acc;		// �t�B�[�h�t�H���[�h
	FLOAT			f_FF_angleS;			// �t�B�[�h�t�H���[�h

	FLOAT 			f_FB_speed_kp;			// �t�B�[�h�o�b�N�A���x ��ᐧ��
	FLOAT 			f_FB_speed_ki;			// �t�B�[�h�o�b�N�A���x �ϕ�����
	FLOAT			f_FB_dist_kp;			// �t�B�[�h�o�b�N�A���� ��ᐧ��
	FLOAT 			f_FB_dist_ki;			// �t�B�[�h�o�b�N�A���� �ϕ�����
	FLOAT			f_FB_angleS_kp;			// �t�B�[�h�o�b�N�A�p���x ��ᐧ��
	FLOAT			f_FB_angleS_ki;			// �t�B�[�h�o�b�N�A�p���x �ϕ�����
	FLOAT			f_FB_angle_kp;			// �t�B�[�h�o�b�N�A�p�x ��ᐧ��
	FLOAT			f_FB_angle_ki;			// �t�B�[�h�o�b�N�A�p�x �ϕ�����
	FLOAT			f_FB_wall_kp;			// �t�B�[�h�o�b�N�A�� ��ᐧ��
	FLOAT			f_FB_wall_kd;			// �t�B�[�h�o�b�N�A�� ��������
}stGAIN;

/* �X�����[���f�[�^ */
typedef struct{
	FLOAT	f_speed;
	FLOAT	f_angAcc;
	FLOAT	f_angvel;
	FLOAT	f_entryLen;
	FLOAT	f_escapeLen;
	USHORT	us_accAngvelTime;
	USHORT	us_constAngvelTime;
	FLOAT	f_ang_AccEnd;
	FLOAT	f_ang_ConstEnd;
	FLOAT	f_ang_Total;
}stSLA;

/* �X�����[���^�C�v */
typedef enum{
	SLA_90,
	SLA_45,	
	SLA_135,
	SLA_N90,				// �΂� �� 90���� �΂�
	SLA_TYPE_MAX
}enSLA_TYPE;

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
PUBLIC void PARAM_setCntType( BOOL bl_type );
PUBLIC FLOAT F_CNT2MM( LONG l_cnt );
PUBLIC CONST stGAIN* PARAM_getGain( enPARAM_MODE en_mode );
PUBLIC CONST stSPEED* PARAM_getSpeed( enPARAM_MODE en_mode );
PUBLIC FLOAT PARAM_getSlaCorrDist( enPARAM_MOVE_SPEED en_speed , enSlaCorrDist en_dist);
PUBLIC void PARAM_setSpeedType( enPARAM_MODE en_mode, enPARAM_MOVE_SPEED en_speed );
PUBLIC stSLA* PARAM_getSra(enSLA_TYPE en_mode);
PUBLIC void PARAM_makeSla( FLOAT f_speed, FLOAT f_angAcc, FLOAT f_g, enSLA_TYPE en_mode, enPARAM_MOVE_SPEED en_speed);

#endif