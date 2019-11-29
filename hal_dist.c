// *************************************************************************
//   ���{�b�g��	�F Baharat�i�o�n���b�g�j
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F dist
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
#include <string.h>							// �����񑀍�imemset�p�j
#include <common_define.h>					// common_define

#include <hal_led.h>		// LED
#include <hal_dist.h>		// DIST
#include <hal_sci.h>		// SCI
#include <parameter.h>		// PARAMETER
#include <hal_flash.h>		// FLASH


//**************************************************
// ��`�idefine�j
//**************************************************
#define	SEN_FLASH_SIZE		( 64 )												// �Z���T�p�f�[�^�t���b�V���̃f�[�^��

/* �����Z���T */
#define		LED_DIST_RF		( PORTA.PODR.BIT.B4 )			// �r�b�g�A�N�Z�X
#define		LED_DIST_RS		( PORTA.PODR.BIT.B6 )			// �r�b�g�A�N�Z�X
#define		LED_DIST_LF		( PORTA.PODR.BIT.B1 )			// �r�b�g�A�N�Z�X
#define		LED_DIST_LS		( PORT4.PODR.BIT.B6 )			// �r�b�g�A�N�Z�X


//**************************************************
// �񋓑́ienum�j
//**************************************************

//**************************************************
// �\���́istruct�j
//**************************************************
/* �����Z���T���i�O�ǂ̂݁A�f�[�^�t���b�V���p�\���̂Ƃ��Ă��g�p����j */
typedef struct{
	SHORT		s_wallHit;					///< @var : �ǂɓ������Ă��Ă����������Ȃ��l         ( AD �l ) �i�O�ǂƃ}�E�X�Ԃ���2mm�̎��̒l�j
	SHORT		s_skewErr1;					///< @var : �΂ߑ��s���̕␳臒l1                    ( AD �l )
	SHORT		s_skewErr2;					///< @var : �΂ߑ��s���̕␳臒l2                    ( AD �l )
	SHORT		s_skewErr3;					///< @var : �΂ߑ��s���̕␳臒l3                    ( AD �l )
}stDIST_FRONT_SEN;

/* �����Z���T���i�S�Z���T���ʁj */
typedef struct{
	SHORT		s_now;						// LED �_�����̋����Z���T�̌��ݒl           ( AD �l )
	SHORT		s_old;						// LED �_�����̋����Z���T��1�O�̒l        ( AD �l )
	SHORT		s_limit;					// �����Z���T��臒l                         ( AD �l ) ( ���̒l���傫���ꍇ�A�ǂ���Ɣ��f���� )
	SHORT		s_ref;						// ���̒��S�ɒu�������̋����Z���T�̊�l ( AD �l )
	SHORT		s_offset;					// LED �������̋����Z���T�̒l               ( AD �l )
	SHORT		s_ctrl;						// ����L��������ۂ�臒l                   ( AD �l ) ��ɑO�ǂŎg�p
	SHORT		s_noCtrl;					// �ǂɋ߂����邽�ߐ��䖳��������ۂ�臒l   ( AD �l ) ��ɑO�ǂŎg�p
}stDIST_SEN;

/* �����Z���T���i�S�Z���T���ʁA�f�[�^�t���b�V���p�\���݂̂̂Ɏg�p�j */
typedef struct{
	SHORT		s_ref;						///< @var : ���̒��S�ɒu�������̋����Z���T�̊�l ( AD �l )
	SHORT		s_limit;					///< @var : �����Z���T��臒l                         ( AD �l ) ( ���̒l���傫���ꍇ�A�ǂ���Ɣ��f���� )
	SHORT		s_ctrl;						///< @var : ����L��������ۂ�臒l                   ( AD �l ) ��ɑO�ǂŎg�p
	SHORT		s_noCtrl;					///< @var : �ǂɋ߂����邽�ߐ��䖳��������ۂ�臒l   ( AD �l ) ��ɑO�ǂŎg�p
}stDIST_SEN_DATA;

/* �f�[�^�t���b�V���̃o�b�N�A�b�v or ���A�p */
typedef struct{
	stDIST_SEN_DATA		st_common[DIST_SEN_MAX];			// �Z���T�f�[�^(�S�Z���T����)
	stDIST_FRONT_SEN	st_front[DIST_SEN_L_FRONT+1];		// �Z���T�f�[�^(�O�ǂ̂�)
}stDIST_FLASH;

/* �ǃZ���T�̃��O�p */
typedef struct {
	SHORT		s_frontR;
	SHORT		s_frontL;
	SHORT		s_sideR;
	SHORT		s_sideL;
}stDIST_SEN_LOG;


//**************************************************
// �O���[�o���ϐ�
//**************************************************
PRIVATE stDIST_SEN			st_sen[DIST_SEN_MAX];				// �����Z���T
PRIVATE	stDIST_FRONT_SEN	st_senF[DIST_SEN_L_FRONT+1];		// �����Z���T(�O�ǂ̂�)

/* ���O */
PRIVATE	stDIST_SEN_LOG		st_DistSenLog[DIST_LOG];			// ���O
PRIVATE	USHORT				us_SenLogPt		= 0;				// ���O�ʒu
PRIVATE BOOL				bl_senlog		= false;			// ���O�擾�����ifalse:�֎~�@true:���j
static	UCHAR				uc_sencycle		= 0;				// ���O�L�^�̃T�C�N��

//PRIVATE	USHORT				us_Log[DIST_LOG];					// ���O
//PRIVATE	USHORT				us_LogPt = 0;						// ���O�ʒu
//PRIVATE UCHAR				us_LogSta = 0;						// ���O�J�n/��~�i0:��~�A1:�J�n�j
PRIVATE BOOL				bl_NoChkErr = false;				// �G���[�ʂ̃`�F�b�N�����邩�iFALSE:�G���[�ʃ`�F�b�N����ATRUE�F�G���[�ʃ`�F�b�N���Ȃ��j

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
extern PUBLIC void TIME_wait(ULONG ul_time);
extern PUBLIC void TIME_waitFree(ULONG ul_cnt);

// *************************************************************************
//   �@�\		�F �����Z���T������������
//   ����		�F Storage_Save���g�����߂ɂ́A1�x�ǂݍ���ł����K�v�����邽�߁A�����A�h���X���N������Storage_Load���Ă���B(���ケ�̏������폜������P�̗]�n����)
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.5			TKR			�V�K
// *************************************************************************/
PUBLIC void DIST_init( void ){
	
	//stDIST_FLASH	st_dummy;
	
	//Storage_Load( (const void*)&st_dummy, SEN_FLASH_SIZE, ADR_SEN );	// �f�[�^���[�h(dummy) ���ꂪ�Ȃ��Ǝ��̂P���ڂ�save�����܂������Ȃ�
	
	/* �����ϐ��̏����� */
	memset( st_sen, 0, sizeof(st_sen) );				// �����Z���T�i�S�Z���T���ʁj
	memset( st_senF, 0, sizeof(st_senF) );				// �����Z���T�i�O�ǂ̂݁j
	//memset( us_Log, 0, sizeof(us_Log[DIST_LOG]) );
}

// *************************************************************************
//   �@�\		�F �S�Z���T��臒l��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.11.4			TKR			�V�K
// *************************************************************************/
PUBLIC void DIST_setThresh_AllSenData( void )
{	
	stDIST_FLASH	st_FlashData;					// �f�[�^�t���b�V��
	ULONG			ul_sum[DIST_SEN_MAX] = {0};
	UCHAR			i,j;

	for( i = 0; i < 100; i++){		// 100��T���v�����O
		for( j = 0; j < DIST_SEN_MAX; j++){
			ul_sum[j]	+= (ULONG)st_sen[j].s_now;	// ���v
		}
		TIME_wait(1);
	}
	for( j = 0; j < DIST_SEN_MAX; j++){
		st_FlashData.st_common[j].s_ref		= (SHORT)( ul_sum[j] / 100 );
	}
	
	/* ���t�@�����X�l�����Ɋe臒l���Z�o */
	st_FlashData.st_common[DIST_SEN_R_FRONT].s_limit  	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_R_FRONT].s_ref * R_FRONT_WALL_GAIN );
	st_FlashData.st_common[DIST_SEN_L_FRONT].s_limit  	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_L_FRONT].s_ref * L_FRONT_WALL_GAIN );
	st_FlashData.st_common[DIST_SEN_R_SIDE].s_limit   	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_R_SIDE].s_ref  * R_SIDE_WALL_GAIN );
	st_FlashData.st_common[DIST_SEN_L_SIDE].s_limit   	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_L_SIDE].s_ref  * L_SIDE_WALL_GAIN );
	st_FlashData.st_common[DIST_SEN_R_FRONT].s_ctrl   	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_R_FRONT].s_ref * R_FRONT_WALL_CTRL_GAIN );
	st_FlashData.st_common[DIST_SEN_L_FRONT].s_ctrl   	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_L_FRONT].s_ref * L_FRONT_WALL_CTRL_GAIN );
	st_FlashData.st_common[DIST_SEN_R_FRONT].s_noCtrl 	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_R_FRONT].s_ref * R_FRONT_WALL_NO_CTRL_GAIN );
	st_FlashData.st_common[DIST_SEN_L_FRONT].s_noCtrl 	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_L_FRONT].s_ref * L_FRONT_WALL_NO_CTRL_GAIN );
	st_FlashData.st_front[DIST_SEN_R_FRONT].s_wallHit 	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_R_FRONT].s_ref * R_FRONT_WALL_HIT_GAIN );
	st_FlashData.st_front[DIST_SEN_L_FRONT].s_wallHit 	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_L_FRONT].s_ref * L_FRONT_WALL_HIT_GAIN );
	st_FlashData.st_front[DIST_SEN_R_FRONT].s_skewErr1 	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_R_FRONT].s_ref * R_FRONT_SKEW_ERR1_GAIN );
	st_FlashData.st_front[DIST_SEN_L_FRONT].s_skewErr1 	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_L_FRONT].s_ref * L_FRONT_SKEW_ERR1_GAIN );
	st_FlashData.st_front[DIST_SEN_R_FRONT].s_skewErr2 	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_R_FRONT].s_ref * R_FRONT_SKEW_ERR2_GAIN );
	st_FlashData.st_front[DIST_SEN_L_FRONT].s_skewErr2 	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_L_FRONT].s_ref * L_FRONT_SKEW_ERR2_GAIN );
	st_FlashData.st_front[DIST_SEN_R_FRONT].s_skewErr3 	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_R_FRONT].s_ref * R_FRONT_SKEW_ERR3_GAIN );
	st_FlashData.st_front[DIST_SEN_L_FRONT].s_skewErr3 	= (SHORT)( (FLOAT)st_FlashData.st_common[DIST_SEN_L_FRONT].s_ref * L_FRONT_SKEW_ERR3_GAIN );

	printf("st_FlashData:%d\n\r",sizeof(st_FlashData));

	//�����Ńf�[�^�Z�[�u����
}

// *************************************************************************
//   �@�\		�F �S�Z���T��臒l���ړ����Đݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �f�[�^�t���b�V���ƕ������킹
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.5			TKR			�V�K
// *************************************************************************/
#if 0
PUBLIC void DIST_setThresh_AllSenData_Move( void ){
	
	/* ��𗣂����Ԃ�݂��� */
	TIME_wait(1000);
	
	/* �ړ� */
	bl_NoChkErr = true;			// �G���[�ʂ��`�F�b�N���Ȃ�
	
	MOT_setTrgtSpeed( (FLOAT)SEN_BACK_CHK_SPEED );			// �ڕW���x�ݒ��Ⴍ����
	PARAM_setSpeedType( PARAM_ST, PARAM_VERY_SLOW );		// [���i]���x���ᑬ
	
	//MOT_goBack_Const( MOT_BACK_SEN_ADJ );	����ō��
	
	MOT_setTrgtSpeed( (FLOAT)MAP_SEARCH_SPEED );			// �ڕW���x�ݒ��Ⴍ����
	PARAM_setSpeedType( PARAM_ST, PARAM_NORMAL );		// [���i]���x����
	
	bl_NoChkErr = false;		// �G���[�ʂ��`�F�b�N����
	
	DIST_setThresh_AllSenData();

}
#endif

// *************************************************************************
//   �@�\		�F �����Z���T���`���[�j���O����
//   ����		�F �Ȃ�
//   ����		�F main�֐��Ŏ��s
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.5			TKR			�V�K
// *************************************************************************/
PUBLIC void DIST_adj( void ){
	
	stDIST_FLASH st_FlashData	= {0};					// �f�[�^�t���b�V���̃o�b�N�A�b�v�f�[�^
	
	/* �����l�Ƃ��āA�܂��͌Œ�l�������� */
	st_sen[DIST_SEN_R_FRONT].s_ref       = R_FRONT_REF;
	st_sen[DIST_SEN_L_FRONT].s_ref       = L_FRONT_REF;
//	st_sen[DIST_SEN_R_45].s_ref          = R_45_REF;
//	st_sen[DIST_SEN_L_45].s_ref          = L_45_REF;
	st_sen[DIST_SEN_R_SIDE].s_ref        = R_SIDE_REF;
	st_sen[DIST_SEN_L_SIDE].s_ref        = L_SIDE_REF;
	
	st_sen[DIST_SEN_R_FRONT].s_limit     = R_FRONT_WALL;
	st_sen[DIST_SEN_L_FRONT].s_limit     = L_FRONT_WALL;
//	st_sen[DIST_SEN_R_45].s_limit        = R_45_WALL;
//	st_sen[DIST_SEN_L_45].s_limit        = L_45_WALL;
	st_sen[DIST_SEN_R_SIDE].s_limit      = R_SIDE_WALL;
	st_sen[DIST_SEN_L_SIDE].s_limit      = L_SIDE_WALL;
	
	st_sen[DIST_SEN_R_FRONT].s_ctrl      = R_FRONT_WALL_CTRL;
	st_sen[DIST_SEN_L_FRONT].s_ctrl      = L_FRONT_WALL_CTRL;
	
	st_sen[DIST_SEN_R_FRONT].s_noCtrl    = R_FRONT_WALL_NO_CTRL;
	st_sen[DIST_SEN_L_FRONT].s_noCtrl    = L_FRONT_WALL_NO_CTRL;
	
	/* �O�ǂ̂� */
	st_senF[DIST_SEN_R_FRONT].s_wallHit  = R_FRONT_WALL_HIT;
	st_senF[DIST_SEN_L_FRONT].s_wallHit  = L_FRONT_WALL_HIT;
	
	st_senF[DIST_SEN_R_FRONT].s_skewErr1 = R_FRONT_SKEW_ERR1;
	st_senF[DIST_SEN_L_FRONT].s_skewErr1 = L_FRONT_SKEW_ERR1;
	
	st_senF[DIST_SEN_R_FRONT].s_skewErr2 = R_FRONT_SKEW_ERR2;
	st_senF[DIST_SEN_L_FRONT].s_skewErr2 = L_FRONT_SKEW_ERR2;

	st_senF[DIST_SEN_R_FRONT].s_skewErr3 = R_FRONT_SKEW_ERR3;
	st_senF[DIST_SEN_L_FRONT].s_skewErr3 = L_FRONT_SKEW_ERR3;

	
#ifdef FUNC_DIST_AUTO_THRESH

	/* �N�����ɑO�ǂ��߂���΃`���[�j���O���[�V��������� */

	if( ( st_sen[DIST_SEN_R_FRONT].s_now > DIST_NEAR_WALL ) ||
		( st_sen[DIST_SEN_L_FRONT].s_now > DIST_NEAR_WALL )
	){
		
		DIST_setThresh_AllSenData_Move();		// �`���[�j���O���s��
		printf("�`���[�j���O \n\r" );
	}

	/* �N�����ɑO�ǂȂ���΁A�f�[�^�t���b�V������ȑO�ɕۑ������Z���T�̒l�����[�h���� */
	else{
		
		/* �f�[�^�t���b�V������̒l�����[�h */
		Storage_Load( (const void*)&st_FlashData, SEN_FLASH_SIZE, ADR_SEN );			// �f�[�^���[�h
		TIME_wait(10);

		/* �S�Z���T���� */
		st_sen[DIST_SEN_R_FRONT].s_ref       = st_FlashData.st_common[DIST_SEN_R_FRONT].s_ref;
		st_sen[DIST_SEN_L_FRONT].s_ref       = st_FlashData.st_common[DIST_SEN_L_FRONT].s_ref;
		st_sen[DIST_SEN_R_45].s_ref          = st_FlashData.st_common[DIST_SEN_R_45].s_ref;
		st_sen[DIST_SEN_L_45].s_ref          = st_FlashData.st_common[DIST_SEN_L_45].s_ref;
		st_sen[DIST_SEN_R_SIDE].s_ref        = st_FlashData.st_common[DIST_SEN_R_SIDE].s_ref;
		st_sen[DIST_SEN_L_SIDE].s_ref        = st_FlashData.st_common[DIST_SEN_L_SIDE].s_ref;

		st_sen[DIST_SEN_R_FRONT].s_limit     = st_FlashData.st_common[DIST_SEN_R_FRONT].s_limit;
		st_sen[DIST_SEN_L_FRONT].s_limit     = st_FlashData.st_common[DIST_SEN_L_FRONT].s_limit;
//		st_sen[DIST_SEN_R_45].s_limit        = st_FlashData.st_common[DIST_SEN_R_45].s_limit;
//		st_sen[DIST_SEN_L_45].s_limit        = st_FlashData.st_common[DIST_SEN_L_45].s_limit;
		st_sen[DIST_SEN_R_SIDE].s_limit      = st_FlashData.st_common[DIST_SEN_R_SIDE].s_limit;
		st_sen[DIST_SEN_L_SIDE].s_limit      = st_FlashData.st_common[DIST_SEN_L_SIDE].s_limit;

		st_sen[DIST_SEN_R_FRONT].s_ctrl      = st_FlashData.st_common[DIST_SEN_R_FRONT].s_ctrl;
		st_sen[DIST_SEN_L_FRONT].s_ctrl      = st_FlashData.st_common[DIST_SEN_L_FRONT].s_ctrl;

		st_sen[DIST_SEN_R_FRONT].s_noCtrl    = st_FlashData.st_common[DIST_SEN_R_FRONT].s_noCtrl;
		st_sen[DIST_SEN_L_FRONT].s_noCtrl    = st_FlashData.st_common[DIST_SEN_L_FRONT].s_noCtrl;

		/* �O�ǂ̂� */
		st_senF[DIST_SEN_R_FRONT].s_wallHit  = st_FlashData.st_front[DIST_SEN_R_FRONT].s_wallHit;
		st_senF[DIST_SEN_L_FRONT].s_wallHit  = st_FlashData.st_front[DIST_SEN_L_FRONT].s_wallHit;

		st_senF[DIST_SEN_R_FRONT].s_skewErr1 = st_FlashData.st_front[DIST_SEN_R_FRONT].s_skewErr1;
		st_senF[DIST_SEN_L_FRONT].s_skewErr1 = st_FlashData.st_front[DIST_SEN_L_FRONT].s_skewErr1;

		st_senF[DIST_SEN_R_FRONT].s_skewErr2 = st_FlashData.st_front[DIST_SEN_R_FRONT].s_skewErr2;
		st_senF[DIST_SEN_L_FRONT].s_skewErr2 = st_FlashData.st_front[DIST_SEN_L_FRONT].s_skewErr2;

		st_senF[DIST_SEN_R_FRONT].s_skewErr3 = st_FlashData.st_front[DIST_SEN_R_FRONT].s_skewErr3;
		st_senF[DIST_SEN_L_FRONT].s_skewErr3 = st_FlashData.st_front[DIST_SEN_L_FRONT].s_skewErr3;
		printf("�ʏ탍�[�h \n\r" );
	}
#endif
}

// *************************************************************************
//   �@�\		�F �����Z���T�̒l���擾����
//   ����		�F �Ȃ�
//   ����		�F ��
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.5			TKR			�V�K
// *************************************************************************/
PUBLIC SHORT DIST_getNowVal( enDIST_SEN_ID en_id )
{
	return st_sen[en_id].s_now;
}

// *************************************************************************
//   �@�\		�F �����Z���T�̒l��\������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑��@�@	�F hal_dist
//**************************    ��    ��    *******************************
// 		v1.0		2019.5.5			TKR			�V�K
// *************************************************************************/
PUBLIC void DIST_Check( void ){
	
	LED_offAll();	// �C���W�Q�[�^����

	while(1){

		printf(" WallSensor [R_F]%5d [L_F]%5d [R_S]%5d [L_S]%5d \r", 
			(int)DIST_getNowVal(DIST_SEN_R_FRONT),
			(int)DIST_getNowVal(DIST_SEN_L_FRONT),
			(int)DIST_getNowVal(DIST_SEN_R_SIDE),
			(int)DIST_getNowVal(DIST_SEN_L_SIDE)
			);
		
		TIME_wait( 300 );
	}
}


// *************************************************************************
//   �@�\		�F �O�ǂ̗L���𔻒f����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑��@�@	�F hal_dist
//**************************    ��    ��    *******************************
// 		v1.0		2019.5.5			TKR			�V�K
// *************************************************************************/
PUBLIC BOOL DIST_isWall_FRONT( void )
{
	BOOL bl_res 		= false;
	
	if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_sen[DIST_SEN_R_FRONT].s_limit ) ||
		( st_sen[DIST_SEN_L_FRONT].s_now > st_sen[DIST_SEN_L_FRONT].s_limit ) ){
	//if( st_sen[DIST_SEN_R_FRONT].s_now > st_sen[DIST_SEN_R_FRONT].s_limit ){
		bl_res = true;
	}
	
	return bl_res;
}

// *************************************************************************
//   �@�\		�F �E�ǂ̗L���𔻒f����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑��@�@	�F hal_dist
//**************************    ��    ��    *******************************
// 		v1.0		2019.5.5			TKR			�V�K
// *************************************************************************/
PUBLIC BOOL DIST_isWall_R_SIDE( void )
{
	BOOL bl_res 		= false;
	
	if( st_sen[DIST_SEN_R_SIDE].s_now > st_sen[DIST_SEN_R_SIDE].s_limit ){
		bl_res = true;
	}
	
	return bl_res;
}

// *************************************************************************
//   �@�\		�F ���ǂ̗L���𔻒f����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑��@�@	�F hal_dist
//**************************    ��    ��    *******************************
// 		v1.0		2019.5.5			TKR			�V�K
// *************************************************************************/
PUBLIC BOOL DIST_isWall_L_SIDE( void )
{
	BOOL bl_res 		= false;
	
	if( st_sen[DIST_SEN_L_SIDE].s_now > st_sen[DIST_SEN_L_SIDE].s_limit ){
		bl_res = true;
	}
	
	return bl_res;
}

// *************************************************************************
//   �@�\		�F �O�ǂɐڋ߂��Ă��邩
//   ����		�F �Ȃ�
//   ����		�F �t�F�[���Z�[�t�p�H
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
//   ���̑��@�@	�F hal_dist
//**************************    ��    ��    *******************************
// 		v1.0		2019.5.5			TKR			�V�K
// *************************************************************************/
PUBLIC BOOL DIST_isNearWall_FRONT( void )
{
	BOOL bl_res 		= false;
	
	if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_senF[DIST_SEN_R_FRONT].s_wallHit ) ||
		( st_sen[DIST_SEN_L_FRONT].s_now > st_senF[DIST_SEN_L_FRONT].s_wallHit )
	){
		bl_res = true;
	}
	
	return bl_res;
}

// *************************************************************************
//   �@�\		�F ��l�ƌ��ݒl�̕΍����擾����
//   ����		�F �Ȃ�
//   ����		�F �ǂ̐؂�ڕ␳�Ή����{
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �΍�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
// *************************************************************************/
PUBLIC void DIST_getErr( LONG* p_err )
{
	VSHORT	s_threshold_R = 0;		// �E�Z���T��臒l
	VSHORT	s_threshold_L = 0;		// ���Z���T��臒l
	SHORT	s_temp;

#if 0
	/* �G���[�`�F�b�N���Ȃ� */
	if( bl_NoChkErr == TRUE ){
		*p_err = 0;		// �N���A
		return SUCCESS;
	}
#endif	
	/* ---------- */
	/*  �E�ǐ���  */
	/* ---------- */
	/* �ǂ̐؂�ڑ΍� */
	// �}���ɃZ���T�̒l���ω������ꍇ�́A�ǂ̗L���̊�l��臒l�ɕύX����
	s_temp = st_sen[DIST_SEN_R_SIDE].s_now - st_sen[DIST_SEN_R_SIDE].s_old;
	if( ( s_temp < -1 * DIST_NO_WALL_DIV_FILTER ) || ( DIST_NO_WALL_DIV_FILTER < s_temp )
	){
		s_threshold_R = st_sen[DIST_SEN_R_SIDE].s_ref + DIST_REF_UP;		// ��l�{����ǂ̑��݂���臒l�ɂ���
		
	}
	else{
		s_threshold_R = st_sen[DIST_SEN_R_SIDE].s_limit;		// �ʏ�ʂ�
		
	}

	/* ---------- */
	/*  ���ǐ���  */
	/* ---------- */
	/* �ǂ̐؂�ڑ΍� */
	// �}���ɃZ���T�̒l���ω������ꍇ�́A�ǂ̗L���̊�l��臒l�ɕύX����
	s_temp = st_sen[DIST_SEN_L_SIDE].s_now - st_sen[DIST_SEN_L_SIDE].s_old;
	if( ( s_temp < -1 * DIST_NO_WALL_DIV_FILTER ) || ( DIST_NO_WALL_DIV_FILTER < s_temp )
	){
		s_threshold_L = st_sen[DIST_SEN_L_SIDE].s_ref + DIST_REF_UP;		// ��l�{����ǂ̑��݂���臒l�ɂ���
		
	}
	else{
		s_threshold_L = st_sen[DIST_SEN_L_SIDE].s_limit;		// �ʏ�ʂ�
	}
	
	/* ------------ */
	/*  ����l�Z�o  */
	/* ------------ */
	*p_err = 0;		// �N���A
	
	/* �O�ǂ����̂������߂��� */
	if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_sen[DIST_SEN_R_FRONT].s_noCtrl ) &&
		( st_sen[DIST_SEN_L_FRONT].s_now > st_sen[DIST_SEN_L_FRONT].s_noCtrl )
	){
//		printf("[Val]%6d �O�ǂ����̂������߂� 	\n\r", *p_err);
		*p_err = 0;
	}
	/* �O�� */
	else if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_sen[DIST_SEN_R_FRONT].s_ctrl ) &&
			 ( st_sen[DIST_SEN_L_FRONT].s_now > st_sen[DIST_SEN_L_FRONT].s_ctrl )
	){
		*p_err = ( st_sen[DIST_SEN_L_FRONT].s_now - st_sen[DIST_SEN_L_FRONT].s_ref ) -
				 ( st_sen[DIST_SEN_R_FRONT].s_now - st_sen[DIST_SEN_R_FRONT].s_ref );
		
		
		//printf("[Val]%6d �O�ǐ��� 	\n\r", *p_err);
	}
	/* �E�ǂƍ��ǂ��� */
	else if( ( s_threshold_R < st_sen[DIST_SEN_R_SIDE].s_now ) && ( s_threshold_L < st_sen[DIST_SEN_L_SIDE].s_now )
	){
		*p_err = ( st_sen[DIST_SEN_R_SIDE].s_now - st_sen[DIST_SEN_R_SIDE].s_ref ) + 
				 ( st_sen[DIST_SEN_L_SIDE].s_ref - st_sen[DIST_SEN_L_SIDE].s_now );
		//printf("[Val]%6d ���ǐ��� 	\n\r", *p_err);
	}
	/* �E�ǂ��� */
	else if( s_threshold_R < st_sen[DIST_SEN_R_SIDE].s_now ){
		*p_err = ( st_sen[DIST_SEN_R_SIDE].s_now - st_sen[DIST_SEN_R_SIDE].s_ref ) * 2;
		//printf("[Val]%6d �E�ǐ��� 	\n\r", *p_err);
	}
	
	/* ���ǂ��� */
	else if( s_threshold_L < st_sen[DIST_SEN_L_SIDE].s_now ){
		
		*p_err = ( st_sen[DIST_SEN_L_SIDE].s_ref - st_sen[DIST_SEN_L_SIDE].s_now ) * 2;
		//printf("[Val]%6d ���ǐ��� 	\n\r", *p_err);
	}
	
}

// *************************************************************************
//   �@�\		�F ���ɋ߂�������FB
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �����
// **************************    ��    ��    *******************************
// 		v1.0		2019.11.29			TKR			�V�K
// *************************************************************************/
PUBLIC void DIST_getErrSkew( LONG* p_err ){

	*p_err	= 0;
	
	if( st_sen[DIST_SEN_R_SIDE].s_now > R_SIDE_PILL ){
		*p_err	= 307;
		LED_on(LED4);
	}else if(st_sen[DIST_SEN_L_SIDE].s_now > L_SIDE_PILL ){
		*p_err	= -200;
		LED_on(LED0);
	}else{
		// ���䂵�Ȃ�
		LED_offAll();
	}
#if 0
	if( st_sen[DIST_SEN_R_SIDE].s_now > st_senF[DIST_SEN_R_FRONT].s_skewErr3 ){
		*p_err	= 300;
	
	}else if(st_sen[DIST_SEN_L_SIDE].s_now > st_senF[DIST_SEN_L_FRONT].s_skewErr3 ){
		*p_err	= -300;

	}else if(st_sen[DIST_SEN_R_SIDE].s_now > st_senF[DIST_SEN_R_FRONT].s_skewErr2 ){
		*p_err	= 200;

	}else if(st_sen[DIST_SEN_L_SIDE].s_now > st_senF[DIST_SEN_L_FRONT].s_skewErr2 ){
		*p_err	= -200;

	}else if(st_sen[DIST_SEN_R_SIDE].s_now > st_senF[DIST_SEN_R_FRONT].s_skewErr1 ){
		*p_err	= 100;

	}else if(st_sen[DIST_SEN_L_SIDE].s_now > st_senF[DIST_SEN_L_FRONT].s_skewErr1 ){
		*p_err	= -100;
	
	}else{

	}
#endif
}


// *************************************************************************
//   �@�\		�F �O�ǂ̊�l�ƌ��ݒl�̕΍����擾����
//   ����		�F �Ȃ�
//   ����		�F �O�Ǖ␳�p
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �΍�
// **************************    ��    ��    *******************************
// 		v1.0		2019.10.22			TKR			�V�K
// *************************************************************************/
PUBLIC void DIST_getErrFront( LONG* p_err )
{
#if 0
	/* �G���[�`�F�b�N���Ȃ� */
	if( bl_NoChkErr == TRUE ){
		*p_err = 0;		// �N���A
		return SUCCESS;
	}
#endif	
	
	/* ------------ */
	/*  ����l�Z�o  */
	/* ------------ */
	*p_err = 0;		// �N���A
		
	/* �O�� */
	if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_sen[DIST_SEN_R_FRONT].s_ctrl ) &&
	    ( st_sen[DIST_SEN_L_FRONT].s_now > st_sen[DIST_SEN_L_FRONT].s_ctrl )
	){
		*p_err = ( st_sen[DIST_SEN_L_FRONT].s_now - st_sen[DIST_SEN_L_FRONT].s_ref ) -
				 ( st_sen[DIST_SEN_R_FRONT].s_now - st_sen[DIST_SEN_R_FRONT].s_ref );
			
		//printf("[Val]%6d �O�Ǖ␳\n\r", *p_err);
	}
	
}

// *************************************************************************
//   �@�\		�F �����Z���T�p�i�O�ǁj�|�[�����O�֐�
//   ����		�F �Ȃ�
//   ����		�F ���荞�݂�����s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.5			TKR			�V�K
//		v1.5		2019.11.29			TKR			���O�ǉ�
// *************************************************************************/
PUBLIC void DIST_Pol_Front( void )
{
	/* ���������̒l�擾 */
	S12AD.ADANS0.WORD 		= 0x06;			// AN1/2 �ϊ��Ώېݒ�
	S12AD.ADCSR.BIT.ADST	= 1;			// AD�ϊ��J�n
	while(S12AD.ADCSR.BIT.ADST == 1);		// AD�ϊ��҂�
	st_sen[DIST_SEN_R_FRONT].s_offset = (SHORT)S12AD.ADDR2;
	st_sen[DIST_SEN_L_FRONT].s_offset = (SHORT)S12AD.ADDR1;
	
	/* �O��LED�_�� */
	LED_DIST_RF = ON;
	LED_DIST_LF = ON;
	
	/* ��������҂� */
	TIME_waitFree( SEN_WAIT_CNT );

	/* �������̒l�Ɩ��������̒l�ō������擾 */
	S12AD.ADANS0.WORD 		= 0x06;			// AN1/2 �ϊ��Ώېݒ�
	S12AD.ADCSR.BIT.ADST	= 1;			// AD�ϊ��J�n
	while(S12AD.ADCSR.BIT.ADST == 1);		// AD�ϊ��҂�
	st_sen[DIST_SEN_R_FRONT].s_old = st_sen[DIST_SEN_R_FRONT].s_now;		// �o�b�t�@�����O
	st_sen[DIST_SEN_L_FRONT].s_old = st_sen[DIST_SEN_L_FRONT].s_now;		// �o�b�t�@�����O
	st_sen[DIST_SEN_R_FRONT].s_now = (SHORT)S12AD.ADDR2 - st_sen[DIST_SEN_R_FRONT].s_offset;		// ���ݒl��������
	st_sen[DIST_SEN_L_FRONT].s_now = (SHORT)S12AD.ADDR1 - st_sen[DIST_SEN_L_FRONT].s_offset;		// ���ݒl��������
	
	/* ���O */
	if(us_SenLogPt != DIST_LOG){
		if( bl_senlog == true ){
			uc_sencycle++;
		}

		/* ���O�L�^ */
		if( uc_sencycle == DIST_LOG_CYCLE ){
			st_DistSenLog[us_SenLogPt].s_frontR	= st_sen[DIST_SEN_R_FRONT].s_now;	// �E�O��
			st_DistSenLog[us_SenLogPt].s_frontL	= st_sen[DIST_SEN_L_FRONT].s_now;	// ���O��
		}
	}

	/* �O��LED���� */
	LED_DIST_RF = OFF;
	LED_DIST_LF = OFF;
}

// *************************************************************************
//   �@�\		�F �����Z���T�p�i���ǁj�|�[�����O�֐�
//   ����		�F �Ȃ�
//   ����		�F ���荞�݂�����s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.02			�O��			�V�K
//		v1.5		2019.11.29			TKR				���O�ǉ�
// *************************************************************************/
PUBLIC void DIST_Pol_Side( void )
{
	/* ���������̒l�擾 */
	S12AD.ADANS0.WORD 		= 0x09;			// AN0/3 �ϊ��Ώېݒ�
	S12AD.ADCSR.BIT.ADST	= 1;			// AD�ϊ��J�n
	while(S12AD.ADCSR.BIT.ADST == 1);		// AD�ϊ��҂�
	st_sen[DIST_SEN_R_SIDE].s_offset = (SHORT)S12AD.ADDR3;
	st_sen[DIST_SEN_L_SIDE].s_offset = (SHORT)S12AD.ADDR0;
	
	/* ����LED�_�� */
	LED_DIST_RS = ON;
	LED_DIST_LS = ON;
	
	/* ��������҂� */
	TIME_waitFree( SEN_WAIT_CNT );

	/* �������̒l�Ɩ��������̒l�ō������擾 */
	/* ���������̒l�擾 */
	S12AD.ADANS0.WORD 		= 0x09;			// AN0/3 �ϊ��Ώېݒ�
	S12AD.ADCSR.BIT.ADST	= 1;			// AD�ϊ��J�n
	while(S12AD.ADCSR.BIT.ADST == 1);		// AD�ϊ��҂�
	st_sen[DIST_SEN_R_SIDE].s_old = st_sen[DIST_SEN_R_SIDE].s_now;		// �o�b�t�@�����O
	st_sen[DIST_SEN_L_SIDE].s_old = st_sen[DIST_SEN_L_SIDE].s_now;		// �o�b�t�@�����O
	st_sen[DIST_SEN_R_SIDE].s_now = (SHORT)S12AD.ADDR3 - st_sen[DIST_SEN_R_SIDE].s_offset;		// ���ݒl��������
	st_sen[DIST_SEN_L_SIDE].s_now = (SHORT)S12AD.ADDR0 - st_sen[DIST_SEN_L_SIDE].s_offset;		// ���ݒl��������
	
	/* ���O */
	if(us_SenLogPt != DIST_LOG){
		
		/* ���O�L�^ */
		if( uc_sencycle == DIST_LOG_CYCLE ){
			st_DistSenLog[us_SenLogPt].s_sideR	= st_sen[DIST_SEN_R_SIDE].s_now;	// �E����
			st_DistSenLog[us_SenLogPt].s_sideL	= st_sen[DIST_SEN_L_SIDE].s_now;	// ������
			uc_sencycle	= 0;
			us_SenLogPt++;
			if(us_SenLogPt == DIST_LOG)bl_senlog = FALSE;
		}
	}

	/* ����LED���� */
	LED_DIST_RS = OFF;
	LED_DIST_LS = OFF;
}

// *************************************************************************
//   �@�\		�F �o�b�N�A�b�v�����ǃZ���T�̒l�����f�[�^�ɔ��f����
//   ����		�F �Ȃ�
//   ����		�F ���荞�݂�����s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.11.4			TKR			�V�K
// *************************************************************************/
PUBLIC void DIST_LoadData( void ){

	SHORT	i;
	USHORT	*dist_add;
//	dist_add = (USHORT*)&;

	// �ǃf�[�^��RAM�ɃR�s�[
	for( i= 0; i<128; i++ ){
		FLASH_Read( (USHORT*)(ADR_SEN+i*2), dist_add );
		dist_add++;
	}

	printf("SENSOR Load Complete\n\r");

}

// *************************************************************************
//   �@�\		�F �o�b�N�A�b�v���H�����o�b�N�A�b�v����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.11.4			TKR			�V�K
// *************************************************************************/
PUBLIC void DIST_SaveData( void )
{

	SHORT	i;
	USHORT	*dist_add;
//	dist_add	= (USHORT*)g_sysMap;

	// �f�[�^�t���b�V���̃C���[�X
	for( i=0; i<8; i++ ){
		FLASH_Erase((ULONG)(ADR_SEN+i*32));
	}

	// MAP�f�[�^���f�[�^�t���b�V���ɏ�������
	for( i=0; i<128; i++){
		FLASH_WriteEE((ULONG)(ADR_SEN+i*2),dist_add);
		dist_add++;
		printf("0x%x\n\r",(ULONG)(ADR_SEN+i*2));
	}

	printf("Sensor Save Complete\n\r");
}

// *************************************************************************
//   �@�\		�F �o�b�N�A�b�v���H�����N���A����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.11.4			TKR			�V�K
// *************************************************************************/
PUBLIC void Dist_ClearSensorData( void )
{
	SHORT	i;
	
	/* �f�[�^�t���b�V���C���[�X */
	for( i=0; i<8; i++){
		FLASH_Erase((ULONG)(ADR_SEN+i*32));
	}

	printf("Sensor Erase Complete\n\r");
}

// *************************************************************************
//   �@�\		�F ���O�̋��i�ǃZ���T�j
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.11.4			TKR			�V�K
// *************************************************************************/
PUBLIC void DIST_LogSta( void ){
	
	bl_senlog = TRUE;

}

// *************************************************************************
//   �@�\		�F ���O�̏o�́i�ǃZ���T�j
//   ����		�F �Ȃ�
//   ����		�F TeraTerm�ɏo�͂��Ccsv�ɂ��ĕۑ����CMATLAB�ɏo��
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.8.6			TKR			�V�K
// *************************************************************************/
PUBLIC void DIST_showLog( void ){
	
	USHORT	i;
	
	printf("index,[R_F],[L_F],[R_S],[L_S]\n\r");
	for(i=0; i<DIST_LOG; i++){
		printf("%4d,%4d,%4d,%4d,%4d\n\r",
				i,(int)st_DistSenLog[i].s_frontR,
				(int)st_DistSenLog[i].s_frontL,
				(int)st_DistSenLog[i].s_sideR,
				(int)st_DistSenLog[i].s_sideL);
	}

}