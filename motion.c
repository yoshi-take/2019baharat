// *************************************************************************
//   ���{�b�g��	�F Baharat�i�o�n���b�g�j
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F �}�E�X�̓���
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.25			TKR			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O
#include <stdio.h>							// �W�����o��
#include <math.h>							// math

#include <hal_dcm.h>						// DCM
#include <hal_dcmCtrl.h>					// DCM_CTRL
#include <hal_gyro.h>						// GYRO

#include <motion.h>                         // motion
#include <parameter.h>						// parameter

#include <mode.h>							// mode
#include <hal_led.h>						// LED
#include <hal_spk.h>						// SPK

//**************************************************
// ��`�idefine�j
//**************************************************
#define A1_MIN						( 20 )			// ��1�Œ�ړ��p�x
#define A2_MIN						( 30 )			// ��2�Œ�ړ��p�x
#define A3_MIN						( 20 )			// ��3�Œ�ړ��p�x

#define ANGLE_OFFSET_R90			( 0 )			// �p�x�̃I�t�Z�b�g�l�i�o�b�t�@�����O�ɂ��덷�𖄂߂邽�߂̒l�j
#define ANGLE_OFFSET_L90			( 0 )			// �p�x�̃I�t�Z�b�g�l�i�o�b�t�@�����O�ɂ��덷�𖄂߂邽�߂̒l�j
#define ANGLE_OFFSET_R180			( 0 )			// �p�x�̃I�t�Z�b�g�l�i�o�b�t�@�����O�ɂ��덷�𖄂߂邽�߂̒l�j
#define ANGLE_OFFSET_L180			( 0 )			// �p�x�̃I�t�Z�b�g�l�i�o�b�t�@�����O�ɂ��덷�𖄂߂邽�߂̒l�j

#define MOT_MOVE_ST_THRESHOLD		( 45 )			// ���i�ړ�������臒l[mm]
#define MOT_MOVE_ST_MIN				( 20 )          // ���i�ړ������̍Œ�ړ���[mm]

//**************************************************
// �񋓑́ienum�j
//**************************************************

//**************************************************
// �\���́istruct�j
//**************************************************
/* ������ */
typedef struct{

	FLOAT			f_time;			// ����					[msec]

	/* ���x���� */
	FLOAT			f_acc1;			// �����x1				[mm/s2]
	FLOAT			f_acc3;			// �����x3				[mm/s2]
	FLOAT			f_now;			// ���ݑ��x				[mm/s]
	FLOAT			f_trgt;			// ������̖ڕW���x		[mm/s]
	FLOAT			f_last;			// ������̍ŏI���x		[mm/s]

	/* �������� */
	FLOAT			f_dist;			// �ړ�����				[mm]
	FLOAT			f_l1;			// ��1�ړ�����			[mm]
	FLOAT			f_l1_2;			// ��1+2�ړ�����		[mm]

	/* �p���x���� */
	FLOAT			f_accAngleS1;	// �p�����x1			[rad/s2]
	FLOAT			f_accAngleS3;	// �p�����x3			[rad/s2]
	FLOAT			f_nowAngleS;	// ���݊p���x			[rad/s]
	FLOAT			f_trgtAngleS;	// ������̖ڕW�p���x	[rad/s]
	FLOAT			f_lastAngleS;	// ������̍ŏI�p���x	[rad/s]

	/* �p�x���� */
	FLOAT			f_angle;		// �ړ��p�x				[rad]
	FLOAT			f_angle1;		// ��1�ړ��p�x			[rad]
	FLOAT			f_angle1_2;		// ��1+2�ړ��p�x		[rad]
}stMOT_DATA;

//**************************************************
// �ϐ�
//**************************************************
/* ���� */
PRIVATE FLOAT 				f_MotNowSpeed 			= 0.0f;		// ���ݑ��x
PRIVATE FLOAT 				f_MotTrgtSpeed 			= 300.0f;	// �ڕW���x
PRIVATE FLOAT 				f_MotSlaStaSpeed 		= 0.0f;		// �X�����[���J�n���x
PUBLIC	FLOAT				f_MotTrgtSkewSpeed		= 0.0f;		// �ڕW���x�i�΂ߗp�j
PRIVATE	stMOT_DATA 			st_Info;							// �V�[�P���X�f�[�^
PRIVATE BOOL				bl_failsafe				= false;	// �t�F�C���Z�[�t�iTRUE�F����	FALSE�F�����Ȃ��j

/* �ǐ؂�֌W */
PRIVATE enMOT_WALL_EDGE_TYPE		en_WallEdge 		= MOT_WALL_EDGE_NONE;	//�ǐ؂�␳
PRIVATE BOOL						bl_IsWallEdge		= false;				//�ǐ؂ꌟ�m(true:���m�@false:�񌟒m)
PRIVATE FLOAT						f_WallEdgeAddDist	= 0;					//�ǐ؂�␳��̈ړ�����

/* ���̑� */
extern PUBLIC	VUCHAR		uc_CntSec;		// �������v[sec]

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************


// *************************************************************************
//   �@�\		�F �����x���擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �����x[mm/s^2]
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PUBLIC  FLOAT   MOT_getAcc1( void ){

    return 	PARAM_getSpeed( PARAM_ST ) -> f_acc;

}

// *************************************************************************
//   �@�\		�F �����x���擾����
//   ����		�F ���̒l(��Βl)�Ŏw��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �����x[mm/s^2]
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PUBLIC  FLOAT   MOT_getAcc3( void ){

    return 	PARAM_getSpeed( PARAM_ST ) -> f_dec;

}

// *************************************************************************
//   �@�\		�F �����x���擾����i�΂ߑ��s�p�j
//   ����		�F �Ȃ�
//   ����		�F ���MOT_getAcc1�֐��ƃ}�[�W
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �����x[mm/s^2]
// **************************    ��    ��    *******************************
// 		v1.0		2019.11.28			TKR			�V�K
// *************************************************************************/
PUBLIC  FLOAT   MOT_getAcc1Skew( void ){

    return 	PARAM_getSpeed( PARAM_SKEW_ACC ) -> f_acc;

}

// *************************************************************************
//   �@�\		�F �����x���擾����i�΂ߑ��s�p�j
//   ����		�F ���̒l(��Βl)�Ŏw��
//   ����		�F ���MOT_getAcc3�֐��ƃ}�[�W
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �����x[mm/s^2]
// **************************    ��    ��    *******************************
// 		v1.0		2019.11.28			TKR			�V�K
// *************************************************************************/
PUBLIC  FLOAT   MOT_getAcc3Skew( void ){

    return 	PARAM_getSpeed( PARAM_SKEW_DEC ) -> f_dec;

}


// *************************************************************************
//   �@�\		�F �����x���擾����(cos�ߎ�)
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �ő�����x[mm/s2]
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			�g�c			�V�K
// *************************************************************************/
PUBLIC FLOAT MOT_getAcc1_Smooth( void )
{	
	return 	PARAM_getSpeed( PARAM_ACC_SMOOTH ) -> f_acc;
}


// *************************************************************************
//   �@�\		�F �����x���擾����(cos�ߎ�)
//   ����		�F �v���X�Ŏw��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �ő�����x[mm/s2]
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			�g�c			�V�K
// *************************************************************************/
PUBLIC FLOAT MOT_getAcc3_Smooth( void )
{	
	return	PARAM_getSpeed( PARAM_DEC_SMOOTH ) -> f_dec;
}

// *************************************************************************
//   �@�\		�F �p�����x���擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �p�����x[rad/s2]
// **************************    ��    ��    *******************************
// 		v1.0		2017.4.26			�g�c			�V�K
// *************************************************************************/
PUBLIC FLOAT MOT_getAccAngle1( void )
{
	return	PARAM_getSpeed( PARAM_TURN )->f_accAngle;
}

// *************************************************************************
//   �@�\		�F �p�����x���擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �p�����x[rad/s2]
// **************************    ��    ��    *******************************
// 		v1.0		2018.4.26			�g�c			�V�K
// *************************************************************************/
PUBLIC FLOAT MOT_getAccAngle3( void )
{
	return	PARAM_getSpeed( PARAM_TURN ) -> f_decAngle;
}

// *************************************************************************
//   �@�\		�F ���i����
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2013.12.14			�O��			�V�K
//		v2.0		2019.11.3			TKR				�������Ƀ��[�v���甲���o���Ȃ��Ȃ����Ƃ��̑΍��ǉ�		
// *************************************************************************/
PRIVATE void MOT_goBlock_AccConstDec( FLOAT f_fin, enMOT_ST_TYPE en_type, enMOT_GO_ST_TYPE en_goType )
{
	stCTRL_DATA		st_data;					// ����f�[�^
	
	f_WallEdgeAddDist = 0;
	/* ================ */
	/*      ������      */
	/* ================ */
	/* ------ */
	/*  ����  */
	/* ------ */
	if( ( en_type != MOT_CONST_DEC ) && ( en_type != MOT_CONST_DEC_CUSTOM ) && ( en_type != MOT_CONST_DEC_SMOOTH ) && ( en_type != MOT_CONST_DEC_SMOOTH_CUSTOM ) ){
			
		if( MOT_GO_ST_NORMAL == en_goType ){
			st_data.en_type		= CTRL_ACC;
		
		}else if(MOT_GO_ST_SMOOTH == en_goType){
			st_data.en_type		= CTRL_ACC_SMOOTH;
			
		}else{
			st_data.en_type		= CTRL_SKEW_ACC;
		}
		
		st_data.f_acc			= st_Info.f_acc1;			// �����x�w��
		st_data.f_now			= st_Info.f_now;			// ���ݑ��x
		st_data.f_trgt			= st_Info.f_trgt;			// �ڕW���x
		st_data.f_nowDist		= 0;					// �i��ł��Ȃ�
		st_data.f_dist			= st_Info.f_l1;				// ��������
		st_data.f_accAngleS		= 0;					// �p�����x
		st_data.f_nowAngleS		= 0;					// ���݊p���x
		st_data.f_trgtAngleS	= 0;						// �ڕW�p�x
		st_data.f_nowAngle		= 0;					// ���݊p�x
		st_data.f_angle			= 0;					// �ڕW�p�x
		st_data.f_time 			= 0;					// �ڕW���� [sec] �� �w�肵�Ȃ�
		CTRL_clrData();								// �ݒ�f�[�^���N���A
		CTRL_setData( &st_data );						// �f�[�^�Z�b�g
		DCM_staMotAll();							// ���[�^ON
			
		while( f_NowDist < st_Info.f_l1 ){					// �w�苗�����B�҂�
			
		#ifdef	TEST
			/*�����f�o�b�O*/
			if(f_NowDist>=st_Info.f_l1*0.25){
				LED_on(LED0);
				if(f_NowDist>=st_Info.f_l1*0.5){
					LED_on(LED1);
					if(f_NowDist>=st_Info.f_l1*0.75){
						LED_on(LED2);
					}
				}
			}
		#endif

			/*�E�o*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// �����~
				break;
			}
			
			/* �t�F�C���Z�[�t */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE ){
				return;
			}
			
			MOT_setWallEdgeDIST();		// �ǐ؂�␳�����s���鋗����ݒ�
			
		}
		
	}
	
	/* ------ */
	/*  ����  */
	/* ------ */
	if( MOT_GO_ST_NORMAL == en_goType ){
		st_data.en_type		= CTRL_CONST;
		
	}else if(MOT_GO_ST_SMOOTH == en_goType){
		st_data.en_type		= CTRL_CONST_SMOOTH;
				
	}else{
		st_data.en_type		= CTRL_SKEW_CONST;
	}
	
	st_data.f_acc			= 0;						// �����x�w��
	st_data.f_now			= st_Info.f_trgt;				// ���ݑ��x
	st_data.f_trgt			= st_Info.f_trgt;				// �ڕW���x
	st_data.f_nowDist		= st_Info.f_l1;					// ���݈ʒu
	st_data.f_dist			= st_Info.f_l1_2;				// ���������ʒu
	st_data.f_accAngleS		= 0;						// �p�����x
	st_data.f_nowAngleS		= 0;						// ���݊p���x
	st_data.f_trgtAngleS	= 0;						// �ڕW�p�x
	st_data.f_nowAngle		= 0;						// ���݊p�x
	st_data.f_angle			= 0;						// �ڕW�p�x
	st_data.f_time 			= 0;						// �ڕW���� [sec] �� �w�肵�Ȃ�

	if( ( en_type == MOT_CONST_DEC ) || ( en_type == MOT_CONST_DEC_CUSTOM ) || ( en_type == MOT_CONST_DEC_SMOOTH ) || ( en_type == MOT_CONST_DEC_SMOOTH_CUSTOM ) ){
		CTRL_clrData();										// �ݒ�f�[�^���N���A
	}
	
	CTRL_setData( &st_data );							// �f�[�^�Z�b�g
	
	while( f_NowDist < (st_Info.f_l1_2 ) ){				// �w�苗�����B�҂�	
		
	#ifdef	TEST
		/*�����f�o�b�O*/
		/*if(f_NowDist>=st_Info.f_l1){
			LED8=0x04;
			if(f_NowDist>=((st_Info.f_l1_2-st_Info.f_l1)*0.25+st_Info.f_l1)){
				LED8=0x08;
				if(f_NowDist>=((st_Info.f_l1_2-st_Info.f_l1)*0.5+st_Info.f_l1)){
					LED8=0x10;
					if(f_NowDist>=((st_Info.f_l1_2-st_Info.f_l1)*0.75+st_Info.f_l1)){
						LED8=0x20;
					}
				}
			}
		}*/
	#endif	
		
		/*�E�o*/
		if(SW_ON == SW_INC_PIN){
			CTRL_stop();				// �����~
			break;
		}

		/* �t�F�C���Z�[�t */
		MOT_Failsafe(&bl_failsafe);
		if( bl_failsafe == TRUE )return;

		MOT_setWallEdgeDIST();	// �ǐ؂�␳�����s���鋗����ݒ�
	}
	
	/* ------ */
	/*  ����  */
	/* ------ */
	if( ( en_type != MOT_ACC_CONST ) && ( en_type != MOT_ACC_CONST_CUSTOM ) && ( en_type != MOT_ACC_CONST_SMOOTH ) && ( en_type != MOT_ACC_CONST_SMOOTH_CUSTOM ) ){	
			
		if( MOT_GO_ST_NORMAL == en_goType ){
			st_data.en_type		= CTRL_DEC;
		
		}else if(MOT_GO_ST_SMOOTH == en_goType){
			st_data.en_type		= CTRL_DEC_SMOOTH;
	
		}else{
			st_data.en_type		= CTRL_SKEW_DEC;
		}
		
		st_data.f_acc			= st_Info.f_acc3;			// ����
		st_data.f_now			= st_Info.f_trgt;			// ���ݑ��x
		st_data.f_trgt			= st_Info.f_last;			// �ŏI���x
		st_data.f_nowDist		= st_Info.f_l1_2;			// ���������ʒu
		st_data.f_dist			= st_Info.f_dist;			// �S�ړ������ʒu
		st_data.f_accAngleS		= 0;						// �p�����x
		st_data.f_nowAngleS		= 0;						// ���݊p���x
		st_data.f_trgtAngleS	= 0;						// �ڕW�p�x
		st_data.f_nowAngle		= 0;						// ���݊p�x
		st_data.f_angle			= 0;						// �ڕW�p�x
		st_data.f_time 			= 0;						// �ڕW���� [sec] �� �w�肵�Ȃ�
		CTRL_setData( &st_data );							// �f�[�^�Z�b�g

		uc_CntSec	= 0;	// ���[�v�ɂ��鎞�Ԃ𐔂���p					
		while( f_NowDist < ( st_Info.f_dist ) ){		// ������

		#ifdef	TEST
			/*if(f_NowDist>=st_Info.f_l1_2){
				LED8=0x80;
			}*/
			/*�����f�o�b�O*/
			/*
			if( f_NowDist < (st_Info.f_dist-st_Info.f_l1_2-0.02)*0.25+st_Info.f_l1_2){
				LED8=0x81;
				if( f_NowDist < (st_Info.f_dist-st_Info.f_l1_2-0.02)*0.5+st_Info.f_l1_2){
					LED8=0x82;
					if( f_NowDist < (st_Info.f_dist-st_Info.f_l1_2-0.02)*0.75+st_Info.f_l1_2){
						LED8=0x84;
					}
				}
			}
			*/
		#endif
			
			/*�E�o*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// �����~
				break;
			}
			
			/* �t�F�C���Z�[�t */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;

			MOT_setWallEdgeDIST();		// �ǐ؂�␳�����s���鋗����ݒ�

			if( uc_CntSec >= TIME_THRE_WAIT )break;		// ��莞�Ԍo�ƃ��[�v���甲����
		}

	}

	/* ------------------ */
	/*  ����(�ǂ̐؂��)  */
	/* ------------------ */
	/* �ǐ؂ꂪ�܂�������Ȃ���ԁi�ǐ؂�ݒ�����Ă���̂ɁA�G�b�W�������Ă��Ȃ��j */
#if 1
	if( ( en_WallEdge != MOT_WALL_EDGE_NONE)  && ( bl_IsWallEdge == false) ){
		
		st_data.en_type			= CTRL_CONST;
		st_data.f_acc			= 0;					// �����x�w��
		st_data.f_now			= st_Info.f_last;		// ���ݑ��x
		st_data.f_trgt			= st_Info.f_last;		// �ڕW���x
		st_data.f_nowDist		= f_NowDist;			// ���݈ʒu
		st_data.f_dist			= f_NowDist + 180.0f;	// ���������ʒu�i180.0f�F�ǐ؂���ǂ��܂ŋ~�����̋����j
		st_data.f_accAngleS		= 0;					// �p�����x
		st_data.f_nowAngleS		= 0;					// ���݊p���x
		st_data.f_trgtAngleS	= 0;					// �ڕW�p���x
		st_data.f_nowAngle		= 0;					// ���݊p�x
		st_data.f_angle			= 0;					// �ڕW�p�x
		st_data.f_time			= 0;					// �ڕW����[sec]�@���w�肵�Ȃ�
		
		CTRL_clrData();									// �}�E�X�̌��݈ʒu/�p�x���N���A
		CTRL_setData( &st_data );						// �f�[�^�Z�b�g
		
		while( f_NowDist < st_data.f_dist ){			// �w�苗�����B�҂�
			
			/* �t�F�C���Z�[�t */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;

			if( MOT_setWallEdgeDIST_LoopWait() == true ) break;			// �ǐ؂�␳�����s���鋗����ݒ�
			
		}
	}
		
	/* �ǐ؂�܂Œ��i������s�� */
	if( ( MOT_GO_ST_NORMAL == en_goType) &&
		( f_WallEdgeAddDist != 0.0f ) &&
		( f_fin != 0.0f )
	){
		st_data.en_type			= CTRL_CONST;
		st_data.f_acc			= 0;					// �����x�w��
		st_data.f_now			= st_Info.f_last;		// ���ݑ��x
		st_data.f_trgt			= st_Info.f_last;		// �ڕW���x
		st_data.f_nowDist		= 0;					// ���݈ʒu
		st_data.f_dist			= f_WallEdgeAddDist;	// ���������ʒu
		st_data.f_accAngleS		= 0;					// �p�����x
		st_data.f_nowAngleS		= 0;					// ���݊p���x
		st_data.f_trgtAngleS	= 0;					// �ڕW�p���x
		st_data.f_nowAngle		= 0;					// ���݊p�x
		st_data.f_angle			= 0;					// �ڕW�p�x
		st_data.f_time			= 0;					// �ڕW����[sec]�@���w�肵�Ȃ�
		
		CTRL_clrData();									// �}�E�X�̌��݈ʒu/�p�x���N���A
		CTRL_setData( &st_data );						// �f�[�^�Z�b�g
		
		while( f_NowDist < st_data.f_dist ){			// �w�苗�����B�҂�
			
			/* �t�F�C���Z�[�t */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;

		}
		
	}
#endif

	/* ��~ */
	if( 0.0f == f_fin ){
		TIME_wait(500);				// ����҂�
	 	CTRL_stop();				// �����~
		DCM_brakeMot( DCM_R );	// �u���[�L
		DCM_brakeMot( DCM_L );	// �u���[�L
	}	
	
	f_MotNowSpeed = f_fin;			// ���ݑ��x�X�V
}

// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i��`�����j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PRIVATE void MOT_setData_ACC_CONST_DEC( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_l3;						// ��3�ړ�����[mm]
	FLOAT			f_1blockDist;				// 1���̋���[mm]
		
	/* 1���̋��� */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}
	
	/* �����x */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̑��s
		st_Info.f_acc1 		= MOT_getAcc1();													// �����x1[mm/s^2]
		st_Info.f_acc3 		= MOT_getAcc3();													// �����x3[mm/s^2]
	}else{									// �΂ߑ��s
		st_Info.f_acc1 		= MOT_getAcc1Skew();												// �����x1[mm/s^2]
		st_Info.f_acc3 		= MOT_getAcc3Skew();												// �����x3[mm/s^2]
	}

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_trgt		= f_MotTrgtSpeed;													// �ڕW���x
	st_Info.f_last		= f_fin;															// �ŏI���x

	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist;												// �ړ�����[mm]
	st_Info.f_l1		= ( f_MotTrgtSpeed * f_MotTrgtSpeed - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// ��1�ړ�����[mm]
	f_l3			= ( f_fin * f_fin - f_MotTrgtSpeed * f_MotTrgtSpeed ) / ( ( st_Info.f_acc3 * -1 ) * 2 );			// ��3�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist - f_l3;											// ��1+2�ړ�����[mm]

#ifdef	TEST
	//printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	//printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	//printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	//printf("st_Info.f_l1:%5.4f \n\r",st_Info.f_l1);
	//printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
	//printf("f_l3%5.4f \n\r",f_l3);
#endif
}

// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i��`�����i�����j�j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST_DEC_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_l3;						// ��3�ړ�����[mm]
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	FLOAT check;
	check = MOT_MOVE_ST_MIN;
	
	/* 1���̋��� */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* �����x */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̑��s
		st_Info.f_acc1 		= MOT_getAcc1();													// �����x1[mm/s^2]
		st_Info.f_acc3 		= MOT_getAcc3();													// �����x3[mm/s^2]
	}else{									// �΂ߑ��s
		st_Info.f_acc1 		= MOT_getAcc1Skew();												// �����x1[mm/s^2]
		st_Info.f_acc3 		= MOT_getAcc3Skew();												// �����x3[mm/s^2]
	}

	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist;												// �ړ�����[mm]

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_last		= f_fin;		// �ŏI���x
	
	st_Info.f_trgt		= sqrt( 1 / ( ( st_Info.f_acc3 * -1 ) - st_Info.f_acc1 ) *					// �ڕW���x
								( 2 * st_Info.f_acc1 * ( st_Info.f_acc3 * -1 ) * ( st_Info.f_dist - MOT_MOVE_ST_MIN ) + 
								 ( st_Info.f_acc3 * -1 ) * f_MotNowSpeed * f_MotNowSpeed - st_Info.f_acc1 * f_fin * f_fin ) );
								 
	st_Info.f_l1		= ( st_Info.f_trgt * st_Info.f_trgt - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// ��1�ړ�����[mm]
	f_l3				= ( f_fin * f_fin - st_Info.f_trgt * st_Info.f_trgt ) / ( ( st_Info.f_acc3  * -1 ) * 2 );		// ��3�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist - f_l3;											// ��1+2�ړ�����[mm]

#ifdef	TEST	
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("MOT_MOVE_ST_MIN:%5.4f \n\r",check);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("f_now:%5.4f \n\r",st_Info.f_now);
	printf("f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("f_last:%5.4f \n\r",st_Info.f_last);
	printf("f_l1:%5.4f \n\r",st_Info.f_l1);
	printf("f_l1_2 - f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
	printf("f_l3:%5.4f \n\r",f_l3);
#endif

}

// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i��`���� cos�ߎ��j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
//		v1.0		2019.4.26			TKR			cos�ߎ��ǉ�
// *************************************************************************/
PRIVATE void MOT_setData_ACC_CONST_DEC_SMOOTH( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_l3;						// ��3�ړ�����[mm]
	FLOAT			f_1blockDist;				// 1���̋���[mm]
		
	/* 1���̋��� */
	if( MOT_GO_ST_SMOOTH == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}
	
	/* �����x */
	st_Info.f_acc1 		= MOT_getAcc1_Smooth();													// �ő�����x1[mm/s^2]
	st_Info.f_acc3 		= MOT_getAcc3_Smooth();													// �ő�����x3[mm/s^2]

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_trgt		= f_MotTrgtSpeed;													// �ڕW���x
	st_Info.f_last		= f_fin;															// �ŏI���x
	
	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist;												// �ړ�����[mm]
	
	st_Info.f_l1		= ( ( f_MotTrgtSpeed * f_MotTrgtSpeed - f_MotNowSpeed * f_MotNowSpeed )* 3.1416f)/ ( 4*st_Info.f_acc1 );			// ��1�ړ�����[mm]
	f_l3				= ( ( f_fin * f_fin - f_MotTrgtSpeed * f_MotTrgtSpeed )* 3.1416f) / ( ( st_Info.f_acc3 * -1 ) * 4 );				// ��3�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist - f_l3;											// ��1+2�ړ�����[mm]
	
#ifdef	TEST	
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_l1:%5.4f \n\r",st_Info.f_l1);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
	printf("f_l3:%5.4f \n\r",f_l3);
#endif	

}

// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i��`�����i�����j cos�ߎ��j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
//		v1.0		2019.4.26			TKR			cos�ߎ��ǉ�
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST_DEC_SMOOTH_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_l3;						// ��3�ړ�����[mm]
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	FLOAT check;
	check = MOT_MOVE_ST_MIN;
	
	/* 1���̋��� */
	if( MOT_GO_ST_SMOOTH == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* �����x */
	st_Info.f_acc1 		= MOT_getAcc1_Smooth();													// �ő�����x1[mm/s^2]
	st_Info.f_acc3 		= MOT_getAcc3_Smooth();													// �ő�����x3[mm/s^2]


	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist;												// �ړ�����[mm]

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_last		= f_fin;															// �ŏI���x
	
	st_Info.f_trgt		= sqrt( ( 2*st_Info.f_acc1*(st_Info.f_dist - MOT_MOVE_ST_MIN) ) / (3.1416f) + ( ( f_MotNowSpeed * f_MotNowSpeed + f_fin * f_fin ) / 2 ) );					// �ڕW���x
							
	st_Info.f_l1		= ( ( st_Info.f_trgt * st_Info.f_trgt - f_MotNowSpeed * f_MotNowSpeed ) *3.1416f) / ( st_Info.f_acc1 * 4 );			// ��1�ړ�����[mm]
	f_l3				= ( (f_fin * f_fin - st_Info.f_trgt * st_Info.f_trgt ) *3.1416f) / ( ( st_Info.f_acc3  * -1 ) * 4 );				// ��3�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist - f_l3;											// ��1+2�ړ�����[mm]
	
#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("MOT_MOVE_ST_MIN:%5.4f \n\r",check);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_l1:%5.4f \n\r",st_Info.f_l1);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
	printf("f_l3:%5.4f \n\r",f_l3);
#endif

}

// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i�����{�����j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* �����x */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̑��s
		st_Info.f_acc1 		= MOT_getAcc1();												// �����x1[mm/s^2]
		st_Info.f_acc3 		= 0;															// �����x3[mm/s^2](���g�p)
	}else{									// �΂ߑ��s
		st_Info.f_acc1 		= MOT_getAcc1Skew();											// �����x1[mm/s^2]
		st_Info.f_acc3 		= 0;															// �����x3[mm/s^2]
	}

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_trgt		= f_fin;															// �ڕW���x
	st_Info.f_last		= 0;																// �ŏI���x(���g�p)

	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist;												// �ړ�����[mm]
	st_Info.f_l1		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// ��1�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist;													// ��1+2�ړ�����[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_l1_2:%5.4f \n\r",st_Info.f_l1_2);
	printf("st_Info.f_l1:%5.4f \n\r",st_Info.f_l1);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
#endif

}

// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i�����{�����i�����j�j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_trgt		= f_fin;															// �ڕW���x
	st_Info.f_last		= 0;																// �ŏI���x(���g�p)

	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist;												// �ړ�����[mm]

	/* �����x */
	st_Info.f_acc1 		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_dist - MOT_MOVE_ST_MIN ) * 2.0f );	// �����x1[mm/s^2]�i�����I�ɏ��������j
	st_Info.f_acc3 		= 0;																// �����x3[mm/s^2](���g�p)

	/* ���� */
	st_Info.f_l1		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// ��1�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist;													// ��1+2�ړ�����[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_l1_2:%5.4f \n\r",st_Info.f_l1_2);
	printf("st_Info.f_l1:%5.4f \n\r",st_Info.f_l1);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
#endif

}

// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i�����{�����j cos�ߎ�
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
//		v1.0		2019.4.26			TKR			cos�ߎ��ǉ�
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST_SMOOTH( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_SMOOTH == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* �����x */
	st_Info.f_acc1 		= MOT_getAcc1_Smooth();													// �ő�����x1[mm/s^2]
	st_Info.f_acc3 		= 0;																// �����x3[mm/s^2](���g�p)

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_trgt		= f_fin;															// �ڕW���x
	st_Info.f_last		= 0;																// �ŏI���x(���g�p)

	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist;												// �ړ�����[mm]
	st_Info.f_l1		= ( ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) *3.1416f) / ( st_Info.f_acc1 * 4 );			// ��1�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist;													// ��1+2�ړ�����[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_l1_2:%5.4f \n\r",st_Info.f_l1_2);
	printf("st_Info.f_l1:%5.4f \n\r",st_Info.f_l1);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
#endif

}

// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i�����{�����i�����j cos�ߎ� �j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
//		v1.0		2019.4.26			TKR			cos�ߎ��ǉ�
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST_SMOOTH_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_SMOOTH == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_trgt		= f_fin;															// �ڕW���x
	st_Info.f_last		= 0;																// �ŏI���x(���g�p)

	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist;												// �ړ�����[mm]

	/* �����x */
	st_Info.f_acc1 		= ( ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) *3.1416f) / ( ( st_Info.f_dist - MOT_MOVE_ST_MIN ) * 4.0f );	// �ő�����x1[mm/s^2]�i�����I�ɏ��������j
	st_Info.f_acc3 		= 0;																// �����x3[mm/s^2](���g�p)

	/* ���� */
	st_Info.f_l1		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 4 );			// ��1�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist;													// ��1+2�ړ�����[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_l1_2:%5.4f \n\r",st_Info.f_l1_2);
	printf("st_Info.f_l1:%5.4f \n\r",st_Info.f_l1);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
#endif

}

// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i�����{�����j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PRIVATE void MOT_setData_MOT_CONST_DEC( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* �����x */
	if( MOT_GO_ST_NORMAL == en_type ){
		st_Info.f_acc1 		= 0;															// �����x1[mm/s^2](���g�p)
		st_Info.f_acc3 		= MOT_getAcc3();												// �����x3[mm/s^2]
	}else{
		st_Info.f_acc1 		= 0;															// �����x1[mm/s^2](���g�p)
		st_Info.f_acc3 		= MOT_getAcc3Skew();												// �����x3[mm/s^2]
	}

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_trgt		= f_MotNowSpeed;													// �ڕW���x
	st_Info.f_last		= f_fin;															// �ŏI���x(���g�p)

	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist;												// �ړ�����[mm]
	st_Info.f_l1		= 0;																// ��1�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist - ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_acc3 * -1 ) * 2 );			// ��1-2�ړ�����[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
	printf("st_Info.f_l1_2:%5.4f \n\r",st_Info.f_l1_2);
#endif

}

// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i�����i�����j�{�����j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PRIVATE void MOT_setData_MOT_CONST_DEC_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_NORMAL == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_trgt		= f_MotNowSpeed;													// �ڕW���x
	st_Info.f_last		= f_fin;															// �ŏI���x

	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist;												// �ړ�����[mm]

	/* �����x */
	st_Info.f_acc1 		= 0;																// �����x1[mm/s^2](���g�p)
	st_Info.f_acc3 		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_dist - MOT_MOVE_ST_MIN ) * 2.0f ) * -1;	// �����x3[mm/s^2]�i�����I�ɏ��������j

	/* ���� */
	st_Info.f_l1		= 0;																// ��1�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist - ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_acc3 * -1 ) * 2 );			// ��1-2�ړ�����[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
	printf("st_Info.f_l1_2:%5.4f \n\r",st_Info.f_l1_2);
#endif

}

// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i�����{���� cos�ߎ��j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
//		v1.0		2019.4.26			TKR			cos�ߎ��ǉ�
// *************************************************************************/
PRIVATE void MOT_setData_MOT_CONST_DEC_SMOOTH( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_SMOOTH == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* �����x */
	st_Info.f_acc1 		= 0;																// �ő�����x1[mm/s^2](���g�p)
	st_Info.f_acc3 		= MOT_getAcc3_Smooth();													// �ő�����x3[mm/s^2]

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_trgt		= f_MotNowSpeed;													// �ڕW���x
	st_Info.f_last		= f_fin;															// �ŏI���x(���g�p)

	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist;												// �ړ�����[mm]
	st_Info.f_l1		= 0;																// ��1�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist - ( ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) *3.1416f) / ( ( st_Info.f_acc3 * -1 ) * 4 );			// ��1-2�ړ�����[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
	printf("st_Info.f_l1_2:%5.4f \n\r",st_Info.f_l1_2);
#endif

}

// *************************************************************************
//   �@�\		�F ��� �O�i�A����f�[�^�쐬�i�����i�����j�{���� cos�ߎ��j
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
//		v1.0		2019.4.26			TKR			cos�ߎ��ǉ�
// *************************************************************************/
PRIVATE void MOT_setData_MOT_CONST_DEC_SMOOTH_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1���̋���[mm]
	
	/* 1���̋��� */
	if( MOT_GO_ST_SMOOTH == en_type ){		// �ʏ�̒��i
		f_1blockDist = BLOCK;
	}
	else{									// �΂߂̒��i
		f_1blockDist = BLOCK_SKEW;
	}

	/* ���x */
	st_Info.f_now		= f_MotNowSpeed;													// ���ݑ��x	
	st_Info.f_trgt		= f_MotNowSpeed;													// �ڕW���x
	st_Info.f_last		= f_fin;															// �ŏI���x

	/* ���� */
	st_Info.f_dist		= f_num * f_1blockDist;												// �ړ�����[mm]

	/* �����x */
	st_Info.f_acc1 		= 0;																// �����x1[mm/s^2](���g�p)
	st_Info.f_acc3 		= ( ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) *3.1416f) / ( ( st_Info.f_dist - MOT_MOVE_ST_MIN ) * 4.0f ) * -1;	// �ő�����x3[mm/s^2]�i�����I�ɏ��������j

	/* ���� */
	st_Info.f_l1		= 0;																// ��1�ړ�����[mm]
	st_Info.f_l1_2		= st_Info.f_dist - ( ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) *3.1416f) / ( ( st_Info.f_acc3 * -1 ) * 4 );			// ��1-2�ړ�����[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
#endif

}

// *************************************************************************
//   �@�\		�F �O�i�̃^�C�v���擾����
//   ����		�F MOT_go_FinSpeed�̂ݎ��s�\
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B�΂ߑ��s�Ƌ��ʏ����B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �O�i�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.01			TKR			�V�K
// *************************************************************************/
PRIVATE enMOT_ST_TYPE MOT_getStType( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT f_v1Div;
	FLOAT f_v3Div;
	FLOAT f_acc1;
	FLOAT f_acc3;
	FLOAT f_t1;
	FLOAT f_t3;
	FLOAT f_l1;
	FLOAT f_l3;
	FLOAT f_total;							// �ړ�����[mm]
		
	/* 1���̋��� */
	if( (MOT_GO_ST_NORMAL == en_type) || (MOT_GO_ST_SMOOTH == en_type) ){		// �ʏ�̒��i or cos�ߎ�
		f_total	= f_num * BLOCK;

	}
	else{									// �΂߂̒��i
		f_total	= f_num * BLOCK_SKEW;
	}

	if((MOT_GO_ST_NORMAL == en_type) || (MOT_GO_ST_SKEW == en_type)){			// �ʏ�̒��i or �΂�
		
		/* ================ */
		/*  �����{��������  */
		/* ================ */
		f_v1Div		= f_fin - f_MotNowSpeed;
		
		if( MOT_GO_ST_NORMAL == en_type ){		// �����x1[mm/s^2]
			f_acc1		= MOT_getAcc1();		// ���i		
		}else{
			f_acc1		= MOT_getAcc1Skew();	// �΂�
		}

		f_t1		= f_v1Div / f_acc1;

		f_l1 = ( f_MotNowSpeed + f_fin ) * 0.5f * f_t1;

		/* �����{�������� */
		if( f_total <= ( f_l1 + MOT_MOVE_ST_THRESHOLD ) ){

			/* �������ŏI���x�ɑ΂��Ċ������Ȃ� */
			if( f_total < ( f_l1 + MOT_MOVE_ST_MIN ) ){
	//			printf("�p�^�[��4\n\r");
				return MOT_ACC_CONST_CUSTOM;		// �p�^�[��4�i�����I�ɉ����x��ύX����j
			}
			else{
	//			printf("�p�^�[��3\n\r");
				return MOT_ACC_CONST;				// �p�^�[��3�i�����{�����j
			}
		}

		/* ================ */
		/*  �����{��������  */
		/* ================ */
		f_v3Div		= f_fin - f_MotNowSpeed;

		if( MOT_GO_ST_NORMAL ){		// �����x3[mm/s^2]
			f_acc3		= MOT_getAcc3();			// ���i
		}else{
			f_acc3		= MOT_getAcc3Skew();		// �΂�
		}

		f_t3		= f_v3Div / ( f_acc3 * -1 );

		f_l3 = ( f_MotNowSpeed + f_fin ) * 0.5f * f_t3;

		//printf("f_l3�F%5.4f \n\r",f_l3);
		//printf("f_total�F%5.4f \n\r",f_total);
		
		/* �����{�������� */
		if( f_total <= ( f_l3 + MOT_MOVE_ST_THRESHOLD ) ){

			/* �������ŏI���x�ɑ΂��Ċ������Ȃ� */
			if( f_total < ( f_l3 + MOT_MOVE_ST_MIN ) ){
	//			printf("�p�^�[��6\n\r");
				return MOT_CONST_DEC_CUSTOM;		// �p�^�[��6�i�����I�ɉ����x��ύX����j
			}
			else{
	//			printf("�p�^�[��5\n\r");
				return MOT_CONST_DEC;				// �p�^�[��5�i�����{�����j
			}
		}

		/* ========== */
		/*  ��`����  */
		/* ========== */
		f_v1Div		= f_MotTrgtSpeed - f_MotNowSpeed;					// ��`���̑��x��
		f_t1		= f_v1Div / f_acc1;
		f_l1		= ( f_MotNowSpeed + f_MotTrgtSpeed ) * 0.5f * f_t1;

		f_v3Div		= f_fin - f_MotTrgtSpeed;							// ��`���̑��x��

		if( MOT_GO_ST_NORMAL ){		// �����x3[mm/s^2]
			f_acc3		= MOT_getAcc3();			// ���i
		}else{
			f_acc3		= MOT_getAcc3Skew();		// �΂�
		}

		f_t3		= -1.0f * f_v3Div / f_acc3;							// �������̏��v����
		f_l3		= ( f_MotTrgtSpeed + f_fin ) * 0.5f * f_t3;

		/* �ʏ�̑�`���� */
		if( ( f_total - f_l1 - f_l3 - MOT_MOVE_ST_MIN) >= 0 ){

		//printf("%5.4f\n\r",f_total - f_l1 - f_l3 - MOT_MOVE_ST_MIN);	
		//printf("�p�^�[��1\n\r");
			return MOT_ACC_CONST_DEC;				// �p�^�[��1�i�ʏ�j
		}
		/* �����l��ύX���� */
		else{
			//printf("�p�^�[��2\n\r");
			return MOT_ACC_CONST_DEC_CUSTOM;		// �p�^�[��2�i�ڕW���x��ύX�j
		}
	
	}else if( (MOT_GO_ST_SMOOTH == en_type) ){				// cos�ߎ�
	
		/* ========================= */
		/*  �����{��������(cos�ߎ�)  */
		/* ========================= */
		f_v1Div		= f_fin - f_MotNowSpeed;
		f_acc1		= MOT_getAcc1_Smooth();						// �ő�����x1[mm/s^2]
		f_t1		= ( 3.1416f * f_v1Div) / (2.0f *f_acc1);	// ��������
				
		f_l1 = ( ( f_fin*f_fin - f_MotNowSpeed*f_MotNowSpeed ) * 3.1416f ) / ( 4*f_acc1 );	
		
		//printf("f_total= %f\n\r",f_total);
		
		/* �����{�������� */
		if( f_total <= ( f_l1 + MOT_MOVE_ST_THRESHOLD ) ){

			/* �������ŏI���x�ɑ΂��Ċ������Ȃ� */
			if( f_total < ( f_l1 + MOT_MOVE_ST_MIN ) ){
				//printf("�p�^�[��10\n\r");
				return MOT_ACC_CONST_SMOOTH_CUSTOM;		// �p�^�[��10�i�����I�ɉ����x��ύX����j
			}
			else{
				//printf("�p�^�[��9\n\r");
				return MOT_ACC_CONST_SMOOTH;				// �p�^�[��9�i�����{�����j
			}
		}

		/* ======================== */
		/*  �����{��������(cos�ߎ�) */
		/* ======================== */
		f_v3Div		= f_fin - f_MotNowSpeed;
		f_acc3		= MOT_getAcc3_Smooth();				// �ő�����x3[mm/s^2]
		f_t3		= ( 3.1416f * f_v3Div) / ( 2*f_acc3 * -1 );

		f_l3 = ( ( f_fin*f_fin - f_MotNowSpeed*f_MotNowSpeed) * 3.1416f) / ( 4*f_acc3*-1 );

		//printf("f_l3�F%5.4f \n\r",f_l3);
		//printf("f_total�F%5.4f \n\r",f_total);
		
		/* �����{�������� */
		if( f_total <= ( f_l3 + MOT_MOVE_ST_THRESHOLD ) ){

			/* �������ŏI���x�ɑ΂��Ċ������Ȃ� */
			if( f_total < ( f_l3 + MOT_MOVE_ST_MIN ) ){
				//printf("�p�^�[��12\n\r");
				return MOT_CONST_DEC_SMOOTH_CUSTOM;		// �p�^�[��12�i�����I�ɉ����x��ύX����j
			}
			else{
				//printf("�p�^�[��11\n\r");
				return MOT_CONST_DEC_SMOOTH;				// �p�^�[��11�i�����{�����j
			}
		}

		/* ================== */
		/*  ��`����(cos�ߎ�) */
		/* ================== */
		f_v1Div		= f_MotTrgtSpeed - f_MotNowSpeed;		// ��`���̑��x��
		f_acc1		= MOT_getAcc1_Smooth();						// �ő�����x1[mm/s^2]
		f_t1		= ( 3.1416f * f_v1Div) / (2 *f_acc1);	// �������̏��v����

		f_l1 		= ( ( f_MotTrgtSpeed*f_MotTrgtSpeed  - f_MotNowSpeed*f_MotNowSpeed ) * 3.1416f ) / ( 4*f_acc1 );

		f_v3Div		= f_fin - f_MotTrgtSpeed;							// ��`���̑��x��
		f_acc3		= MOT_getAcc3_Smooth();									// �����x3[mm/s^2]
		f_t3		= ( 3.1416f * f_v3Div) / ( 2*f_acc3 * -1 );			// �������̏��v����
		
		f_l3 		= ( ( f_fin*f_fin - f_MotTrgtSpeed*f_MotTrgtSpeed) * 3.1416f) / ( 4*f_acc3*-1 );

		/* �ʏ�̑�`���� */
		if( ( f_total - f_l1 - f_l3 - MOT_MOVE_ST_MIN) >= 0 ){

			//printf("%5.4f\n\r",f_total - f_l1 - f_l3 - MOT_MOVE_ST_MIN);	
			//printf("�p�^�[��7\n\r");
			return MOT_ACC_CONST_DEC_SMOOTH;				// �p�^�[��1�i�ʏ�j
		}
		/* �����l��ύX���� */
		else{
			//printf("�p�^�[��8\n\r");
			return MOT_ACC_CONST_DEC_SMOOTH_CUSTOM;		// �p�^�[��2�i�ڕW���x��ύX�j
		}
	
	}
	
}

// *************************************************************************
//   �@�\		�F ���O�i
//   ����		�F �Ȃ�
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B
//   ����		�F ��搔�A�ŏI���x�A���i�^�C�v�i�ʏ�or�΂߁j
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			TKR			�V�K
// *************************************************************************/
PRIVATE void MOT_go_FinSpeed( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_goStType )
{
	enMOT_ST_TYPE 		en_type 		= MOT_getStType( f_num, f_fin, en_goStType );			// ����p�^�[���擾
	

	/* �ړ������Ǝw��l�ɉ����œ����ς��� */
	switch( en_type ){
	
		case MOT_ACC_CONST_DEC:			// [01] ��`����
			MOT_setData_ACC_CONST_DEC( f_num, f_fin, en_goStType );					// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
	
		case MOT_ACC_CONST_DEC_CUSTOM:	// [02] ��`�����i�����j
			MOT_setData_MOT_ACC_CONST_DEC_CUSTOM( f_num, f_fin, en_goStType );		// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
	
		case MOT_ACC_CONST:				// [03] �����{����
			MOT_setData_MOT_ACC_CONST( f_num, f_fin, en_goStType );					// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
	
		case MOT_ACC_CONST_CUSTOM:		// [04] �����{�����i�����j
			MOT_setData_MOT_ACC_CONST_CUSTOM( f_num, f_fin, en_goStType );			// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, MOT_GO_ST_NORMAL );			// ����
			break;

		case MOT_CONST_DEC:				// [05] �����{����
			MOT_setData_MOT_CONST_DEC( f_num, f_fin, en_goStType );					// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
	
		case MOT_CONST_DEC_CUSTOM:		// [06] �����{�����i�����l�ύX�j
			MOT_setData_MOT_CONST_DEC_CUSTOM( f_num, f_fin, en_goStType );			// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
			
		/* cos�ߎ� */
		case MOT_ACC_CONST_DEC_SMOOTH:			// [07] ��`����
			MOT_setData_ACC_CONST_DEC_SMOOTH( f_num, f_fin, en_goStType );					// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
	
		case MOT_ACC_CONST_DEC_SMOOTH_CUSTOM:	// [08] ��`�����i�����j
			MOT_setData_MOT_ACC_CONST_DEC_SMOOTH_CUSTOM( f_num, f_fin, en_goStType );		// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
	
		case MOT_ACC_CONST_SMOOTH:				// [09] �����{����
			MOT_setData_MOT_ACC_CONST_SMOOTH( f_num, f_fin, en_goStType );					// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
	
		case MOT_ACC_CONST_SMOOTH_CUSTOM:		// [10] �����{�����i�����j
			MOT_setData_MOT_ACC_CONST_SMOOTH_CUSTOM( f_num, f_fin, en_goStType );			// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;

		case MOT_CONST_DEC_SMOOTH:				// [11] �����{����
			MOT_setData_MOT_CONST_DEC_SMOOTH( f_num, f_fin, en_goStType );					// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
	
		case MOT_CONST_DEC_SMOOTH_CUSTOM:		// [12] �����{�����i�����l�ύX�j
			MOT_setData_MOT_CONST_DEC_SMOOTH_CUSTOM( f_num, f_fin, en_goStType );			// ����f�[�^�쐬
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// ����
			break;
	
		default:
			break;
	}

}

// *************************************************************************
//   �@�\		�F ���O�i�i�ʏ�j
//   ����		�F �Ȃ�
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B��
//   ����		�F ��搔�A�ŏI���x
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			TKR			�V�K
// *************************************************************************/
PUBLIC void MOT_goBlock_FinSpeed( FLOAT f_num, FLOAT f_fin )
{	
	MOT_go_FinSpeed( f_num, f_fin, MOT_GO_ST_NORMAL );		// �ʏ�̒��i	
}
	
// *************************************************************************
//   �@�\		�F ���O�i�icos�ߎ��j
//   ����		�F �Ȃ�
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B��
//   ����		�F ��搔�A�ŏI���x
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.5.1			TKR			�V�K
// *************************************************************************/
PUBLIC void MOT_goBlock_FinSpeed_Smooth( FLOAT f_num, FLOAT f_fin){
	
	MOT_go_FinSpeed( f_num, f_fin, MOT_GO_ST_SMOOTH );		// cos�ߎ�����
	 
}

// *************************************************************************
//   �@�\		�F ���O�i�i�΂ߑ��s�j
//   ����		�F �Ȃ�
//   ����		�F ��搔��1.5���Ȃǂ̎w����������ꍇ�́A"1.5"���w�肷��B��
//   ����		�F ��搔�A�ŏI���x
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.11.24			TKR			�V�K
// *************************************************************************/
PUBLIC void MOT_goSkewBlock_FinSpeed( FLOAT f_num, FLOAT f_fin){
	
	MOT_go_FinSpeed( f_num, f_fin, MOT_GO_ST_SKEW );		// cos�ߎ�����
	 
}

// *************************************************************************
//   �@�\		�F ���M�n����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			TKR			�V�K
// *************************************************************************/
PUBLIC void MOT_turn( enMOT_TURN_CMD en_type){
	volatile stMOT_DATA	st_info;			//�V�[�P���X�f�[�^
	stCTRL_DATA		st_data;				//����f�[�^
	FLOAT			f_angle2	=A2_MIN;	//�Œ��2�ړ��p�x[deg]
	FLOAT			f_angle3;
	FLOAT			us_trgtAngleS;			//�ڕW�p���x[deg/s]

	us_trgtAngleS = 300;	
	
	/* -------------- */
	/* ����f�[�^�v�Z */
	/* -------------- */
	/*�@�p�����x�@*/
	st_info.f_accAngleS1 = MOT_getAccAngle1();		
	st_info.f_accAngleS3 = MOT_getAccAngle3();		
	
	/* �p���x*/
	st_info.f_nowAngleS = 0;
	st_info.f_trgtAngleS = (FLOAT)us_trgtAngleS;
	st_info.f_lastAngleS = 0;
	
	/* �p�x */
	switch(en_type){	
		case MOT_R90:	
			st_info.f_angle =  -90 + ANGLE_OFFSET_R90;
			break;
			
		case MOT_L90:	
			st_info.f_angle =   90 + ANGLE_OFFSET_L90 ;
			break;
			
		case MOT_R180:
			st_info.f_angle = -180 + ANGLE_OFFSET_R180 ;
			break;
			
		case MOT_L180:
			st_info.f_angle =  180 + ANGLE_OFFSET_L180;	
			break;
			
		case MOT_R360:
			st_info.f_angle = -360 ;
			break;
			
		case MOT_L360:
			st_info.f_angle =  360 ;		
		break;
	}
	
	f_angle3 = (st_info.f_trgtAngleS - st_info.f_lastAngleS) / 2 * (st_info.f_trgtAngleS - st_info.f_lastAngleS) / st_info.f_accAngleS3;	//��3�ړ��p�x
	
	if((en_type == MOT_R90) || (en_type == MOT_R180) || (en_type == MOT_R360)){								//-����
		st_info.f_trgtAngleS 		*= -1;												//��]�������t�ɂ���
		f_angle2			*= -1;												//��]�������t�ɂ���
		f_angle3			*= -1;												//��]�������t�ɂ���
		st_info.f_angle1		= st_info.f_angle - f_angle3 - f_angle2;							//��1�ړ��p�x[deg]
		st_info.f_angle1_2		= st_info.f_angle - f_angle3;									//��1+2�ړ��p�x[deg]
		
		/* �ŏ��ړ��������㏑�� */
		if( st_info.f_angle1 > (A1_MIN * -1)){
			st_info.f_angle1 = A1_MIN * -1;
		}
		
	}else{
		st_info.f_angle1	= st_info.f_angle - f_angle3 - f_angle2;
		st_info.f_angle1_2	= st_info.f_angle - f_angle3;
		
		/* �ŏ��ړ��������㏑�� */
		if( st_info.f_angle < A1_MIN ){
			st_info.f_angle1 = A1_MIN;
		}
		
	}
	
	//GYRO_staErrChkAngle();			//�G���[���o�J�n(�܂�)
#ifndef TEST	
	printf("st_info.f_trgtAngleS:%5.4f \n\r",st_info.f_trgtAngleS);
	printf("f_angle3:%5.4f \n\r",f_angle3);
	printf("st_info.f_angle1:%5.4f \n\r",st_info.f_angle1);
	printf("st_info.f_angle1_2:%5.4f \n\r",st_info.f_angle1_2);
#endif
	
	/* ================ */
	/*�@�@ ������ �@�@�@*/
	/* ================ */
	/* -----*/
	/* ���� */
	/* -----*/
	st_data.en_type			= CTRL_ACC_TURN;
	st_data.f_acc			= 0;						//�����x�w��
	st_data.f_trgt			= 0;						// �ڕW���x
	st_data.f_nowDist		= 0;						// �i��ł��Ȃ�
	st_data.f_dist			= 0;						// ��������
	st_data.f_accAngleS		= st_info.f_accAngleS1;		// �p�����x
	st_data.f_nowAngleS		= 0;						// ���݊p���x
	st_data.f_trgtAngleS	= st_info.f_trgtAngleS;		// �ڕW�p���x
	st_data.f_nowAngle		= 0;						// ���݊p�x
	st_data.f_angle			= st_info.f_angle1;			// �ڕW�p�x
	st_data.f_time 			= 0;						// �ڕW���� [sec] �� �w�肵�Ȃ�
	
	CTRL_clrData();										// �}�E�X�̌��݈ʒu/�p�x���N���A
	CTRL_setData( &st_data );							// �f�[�^�Z�b�g
	DCM_staMotAll();									// ���[�^ON	
	//printf("��1+2�ړ��p�x�F%5.4f \n\r",st_info.f_angle1_2);
	if(( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 )){		//-����
		while( f_NowAngle > st_info.f_angle1 ){				//�w��p�x���B�҂�(�E��])
			
			/*�E�o*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// �����~
				break;
			}
			
			//if( SYS_isOUTOfCtrl() == true ) break;		//�r���Ő���s�\�ɂȂ���
		}
	}else{
		while( f_NowAngle < st_info.f_angle1){				//�w��p�x���B�҂�(����])
		
			/*�E�o*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// �����~
				break;
			}
			//if( SYS_isOutOfCtrl() == true ) break;		//�r���Ő���s�\�ɂȂ���
		}
	}
	
	//printf("��������(*�L���M*)\n\r");
	//LED_SYS = 0;
	/* ---- */
	/* ���� */
	/* ---- */
	if(( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 )){
		f_angle3		= ( f_TrgtAngleS - st_info.f_lastAngleS ) / 2 * ( f_TrgtAngleS - st_info.f_lastAngleS) / st_info.f_accAngleS3;		//��3�ړ��p�x
		f_angle3		= -1*f_angle3;
		if( f_angle3 > A3_MIN*-1 ) f_angle3 = A3_MIN*-1;			//�����Œ�p�x�ɏ�������
		st_info.f_angle1_2	= st_info.f_angle - f_angle3;			//��1+2�ړ��p�x[rad]
	}else{
		f_angle3		= ( f_TrgtAngleS - st_info.f_lastAngleS ) / 2 * ( f_TrgtAngleS - st_info.f_lastAngleS ) / st_info.f_accAngleS3;		// ��3�ړ��p�x
		if( f_angle3 < A3_MIN ) f_angle3 = A3_MIN;					//�����Œ�p�x�ɏ�������											
		st_info.f_angle1_2	= st_info.f_angle - f_angle3;			//��1+2�ړ��p�x[rad]												
	}
	//printf("��1+2�ړ��p�x�F%5.4f \n\r",st_info.f_angle1_2);
	st_data.en_type			= CTRL_CONST_TURN;
	st_data.f_acc			= 0;					//�����x�w��
	st_data.f_now			= 0;					//���ݑ��x
	st_data.f_trgt			= 0;					//�ڕW���x
	st_data.f_nowDist		= 0;					//�i��ł��Ȃ�
	st_data.f_dist			= 0;					//���������ʒu
	st_data.f_accAngleS		= 0;					//�p�����x
	st_data.f_nowAngleS		= f_TrgtAngleS;			//���݊p���x
	st_data.f_trgtAngleS	= f_TrgtAngleS;			//�ڕW�p���x
	st_data.f_nowAngle		= st_info.f_angle1;		//���݊p�x
	st_data.f_angle			= st_info.f_angle1_2;	//�ڕW�p�x
	st_data.f_time			= 0;					//�ڕW����[sec]���w�肵�Ȃ�
	
	CTRL_setData( &st_data );				//�f�[�^�Z�b�g
	
	if(( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 )){		//-����
		while( f_NowAngle > st_info.f_angle1_2 ){		//�w��p�x���B�҂�(����])
			
			/*�E�o*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// �����~
				break;
			}
		
			//if( SYS_isOUTOfCtrl() == true ) break;		//�r���Ő���s�\�ɂȂ���
		}
	}else{
		while( f_NowAngle < st_info.f_angle1_2){		//�w��p�x���B�҂�(�E��])
			/*�E�o*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// �����~
				break;
			}
			//if( SYS_isOutOfCtrl() == true ) break;		//�r���Ő���s�\�ɂȂ���
		}
	}

#ifndef TEST	
	printf("st_info.f_angle:%5.4f \n\r",st_info.f_angle);
	printf("f_angle3:%5.4f \n\r",f_angle3);
	printf("st_info.f_angle1:%5.4f \n\r",st_info.f_angle1);
	printf("st_info.f_angle1_2:%5.4f \n\r",st_info.f_angle1_2);
#endif
	/* ---- */
	/* ���� */
	/* ---- */
	st_data.en_type			= CTRL_DEC_TURN;
	st_data.f_acc			= 0;					//�����x�w��
	st_data.f_now			= 0;					//���ݑ��x
	st_data.f_trgt			= 0;					//�ŏI���x
	st_data.f_nowDist		= 0;					//���������ʒu
	st_data.f_dist			= 0;					//�S�ړ������ʒu
	st_data.f_accAngleS		= st_info.f_accAngleS3;	//�p�����x
	st_data.f_nowAngleS		= f_TrgtAngleS;			//���݊p���x
	st_data.f_trgtAngleS	= 0;					//�ڕW�p���x
	st_data.f_nowAngle		= st_info.f_angle1_2;	//���݊p�x
	st_data.f_angle			= st_info.f_angle;		//�ڕW�p�x
	st_data.f_time			= 0;					//�ڕW����[sec]���w�肵�Ȃ�
	
	CTRL_setData( &st_data );						// �f�[�^�Z�b�g
	
	if(( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 )){	//-����
		while( f_NowAngle > ( st_info.f_angle + 1)){		//�w��p�x���B�҂�(�E��])
			
			/*�E�o*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// �����~
				break;
			}
		
			//if( SYS_isOutOfCtrl() == true ) break;		//�r���Ő���s�\�ɂȂ���
		}
	}else{
		while( f_NowAngle < (st_info.f_angle - 1)){		//�w��p�x���B�҂�(����])
		
			/*�E�o*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// �����~
				break;
			}
			//if( SYS_isOutOfCtrl() == true ) break;		//�r���Ő���s�\�ɂȂ���
		}
	
	}
	//LED8=0xff;
	//printf("���񊮗�");
	/* ��~ */
	TIME_wait(200);		//����҂�
	CTRL_stop();		// �����~	
	
}

// *************************************************************************
//   �@�\		�F ���M�n����i�ڕW�p���x�ύX�\�j
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ����̎�ށC�ڕW�p���x[deg/s]
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.9.17			TKR			�V�K
// *************************************************************************/
PUBLIC void MOT_turn2( enMOT_TURN_CMD en_type, FLOAT f_trgtAngleS ){
	volatile stMOT_DATA	st_info;			//�V�[�P���X�f�[�^
	stCTRL_DATA		st_data;				//����f�[�^
	FLOAT			f_angle3;

	/* -------------- */
	/* ����f�[�^�v�Z  */
	/* -------------- */
	FLOAT			f_acc1;
	FLOAT			f_acc3;
	FLOAT			f_theta1;
	FLOAT			f_theta3;
	FLOAT			f_total;	

	f_acc1		= MOT_getAccAngle1();
	f_theta1	= f_trgtAngleS * f_trgtAngleS / ( f_acc1 * 2);

	f_acc3		= MOT_getAccAngle3();
	f_theta3	=  f_trgtAngleS * f_trgtAngleS /( f_acc3 * 2 );

	/* �p���x*/
	st_info.f_nowAngleS = 0;
	st_info.f_lastAngleS = 0;

	/* �p�x */
	switch(en_type){	
		case MOT_R90:
			st_info.f_angle	= -90 + ANGLE_OFFSET_R90;
			f_total			= st_info.f_angle * (-1);
			break;
			
		case MOT_L90:
			st_info.f_angle	= 90 + ANGLE_OFFSET_L90;
			f_total 		= st_info.f_angle;
			break;
			
		case MOT_R180:
			st_info.f_angle	= -180 + ANGLE_OFFSET_R180;
			f_total 		= st_info.f_angle * (-1);
			break;
			
		case MOT_L180:
			st_info.f_angle	= 180 + ANGLE_OFFSET_L180;
			f_total 		= st_info.f_angle;	
			break;
			
		case MOT_R360:
			st_info.f_angle	= -360;
			f_total = st_info.f_angle * (-1);
			break;
			
		case MOT_L360:
			st_info.f_angle	= 360;
			f_total = st_info.f_angle;		
			break;
	}

	/* ��`����̎�ޔ��� */
	if( ( f_total - f_theta1 - f_theta3 -  A2_MIN) >= 0 ){		// �ʏ�̑�`����

		/* �p�����x */
		st_info.f_accAngleS1		= MOT_getAccAngle1();
		st_info.f_accAngleS3		= MOT_getAccAngle3();

		/* �p���x */
		st_info.f_nowAngleS 		= 0;
		st_info.f_lastAngleS 		= 0;
		st_info.f_trgtAngleS		= f_trgtAngleS;
		
		/* �p�x */
		st_info.f_angle1	= f_trgtAngleS * f_trgtAngleS /( st_info.f_accAngleS1 * 2 );
		f_angle3			= f_trgtAngleS * f_trgtAngleS /( st_info.f_accAngleS3 * 2 );
		st_info.f_angle1_2	= f_total - f_angle3;

	}else{		// �ڕW�p���x��ύX

		/* �p�����x */
		st_info.f_accAngleS1		= MOT_getAccAngle1();
		st_info.f_accAngleS3		= MOT_getAccAngle3();

		/* �p���x */
		st_info.f_nowAngleS 		= 0;
		st_info.f_lastAngleS 		= 0;
		st_info.f_trgtAngleS	= sqrt( 1 / ( ( st_info.f_accAngleS3 * -1 ) - st_info.f_accAngleS1 ) *					// �ڕW�p���x��ύX
									( 2 * st_info.f_accAngleS1 * ( st_info.f_accAngleS3 * -1 ) * ( f_total - MOT_MOVE_ST_MIN ) ) );
		
		/* �p�x */
		st_info.f_angle1	= st_info.f_trgtAngleS * st_info.f_trgtAngleS /( st_info.f_accAngleS1 * 2 );
		f_angle3			= st_info.f_trgtAngleS * st_info.f_trgtAngleS /( st_info.f_accAngleS3 * 2 );
		st_info.f_angle1_2	= f_total - f_angle3;
	
	}

	/* �������� */
	if((en_type == MOT_R90) || (en_type == MOT_R180) || (en_type == MOT_R360)){
		st_info.f_trgtAngleS 		*= -1;
		st_info.f_angle1			*= -1;
		st_info.f_angle1_2			*= -1;
		f_angle3					*= -1;
	}


#ifndef TEST	
	printf("st_info.f_trgtAngleS:%f \n\r",st_info.f_trgtAngleS);
	printf("f_angle3:%f \n\r",f_angle3);
	printf("st_info.f_angle1:%f \n\r",st_info.f_angle1);
	printf("st_info.f_angle1_2:%f \n\r",st_info.f_angle1_2);
#endif
	
	/* ================ */
	/*�@�@ ������ �@�@�@*/
	/* ================ */
	/* -----*/
	/* ���� */
	/* -----*/
	st_data.en_type			= CTRL_ACC_TURN;
	st_data.f_acc			= 0;						//�����x�w��
	st_data.f_trgt			= 0;						// �ڕW���x
	st_data.f_nowDist		= 0;						// �i��ł��Ȃ�
	st_data.f_dist			= 0;						// ��������
	st_data.f_accAngleS		= st_info.f_accAngleS1;		// �p�����x
	st_data.f_nowAngleS		= 0;						// ���݊p���x
	st_data.f_trgtAngleS	= st_info.f_trgtAngleS;		// �ڕW�p���x
	st_data.f_nowAngle		= 0;						// ���݊p�x
	st_data.f_angle			= st_info.f_angle1;			// �ڕW�p�x
	st_data.f_time 			= 0;						// �ڕW���� [sec] �� �w�肵�Ȃ�
	
	CTRL_clrData();										// �}�E�X�̌��݈ʒu/�p�x���N���A
	CTRL_setData( &st_data );							// �f�[�^�Z�b�g
	DCM_staMotAll();									// ���[�^ON	
	
	if(( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 )){		//-����
		while( f_NowAngle > st_info.f_angle1 ){				//�w��p�x���B�҂�(�E��])
			
			/*�E�o*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// �����~
				break;
			}
			
			//if( SYS_isOUTOfCtrl() == true ) break;		//�r���Ő���s�\�ɂȂ���
		}
	}else{
		while( f_NowAngle < st_info.f_angle1){				//�w��p�x���B�҂�(����])
		
			/*�E�o*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// �����~
				break;
			}
			//if( SYS_isOutOfCtrl() == true ) break;		//�r���Ő���s�\�ɂȂ���
		}
	}
	
	//printf("��������(*�L���M*)\n\r");

	/* ---- */
	/* ���� */
	/* ---- */
	//printf("��1+2�ړ��p�x�F%5.4f \n\r",st_info.f_angle1_2);
	st_data.en_type			= CTRL_CONST_TURN;
	st_data.f_acc			= 0;					//�����x�w��
	st_data.f_now			= 0;					//���ݑ��x
	st_data.f_trgt			= 0;					//�ڕW���x
	st_data.f_nowDist		= 0;					//�i��ł��Ȃ�
	st_data.f_dist			= 0;					//���������ʒu
	st_data.f_accAngleS		= 0;					//�p�����x
	st_data.f_nowAngleS		= f_TrgtAngleS;			//���݊p���x
	st_data.f_trgtAngleS	= f_TrgtAngleS;			//�ڕW�p���x
	st_data.f_nowAngle		= st_info.f_angle1;		//���݊p�x
	st_data.f_angle			= st_info.f_angle1_2;	//�ڕW�p�x
	st_data.f_time			= 0;					//�ڕW����[sec]���w�肵�Ȃ�
	
	CTRL_setData( &st_data );				//�f�[�^�Z�b�g
	
	if(( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 )){		//-����
		while( f_NowAngle > st_info.f_angle1_2 ){		//�w��p�x���B�҂�(����])
			
			/*�E�o*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// �����~
				break;
			}
		
			//if( SYS_isOUTOfCtrl() == true ) break;		//�r���Ő���s�\�ɂȂ���
		}
	}else{
		while( f_NowAngle < st_info.f_angle1_2){		//�w��p�x���B�҂�(�E��])
			/*�E�o*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// �����~
				break;
			}
			//if( SYS_isOutOfCtrl() == true ) break;		//�r���Ő���s�\�ɂȂ���
		}
	}

#ifndef TEST	
	printf("st_info.f_angle:%5.4f \n\r",st_info.f_angle);
	printf("st_info.f_angle1:%5.4f \n\r",st_info.f_angle1);
	printf("st_info.f_angle1_2:%5.4f \n\r",st_info.f_angle1_2);
#endif
	/* ---- */
	/* ���� */
	/* ---- */
	st_data.en_type			= CTRL_DEC_TURN;
	st_data.f_acc			= 0;					//�����x�w��
	st_data.f_now			= 0;					//���ݑ��x
	st_data.f_trgt			= 0;					//�ŏI���x
	st_data.f_nowDist		= 0;					//���������ʒu
	st_data.f_dist			= 0;					//�S�ړ������ʒu
	st_data.f_accAngleS		= st_info.f_accAngleS3;	//�p�����x
	st_data.f_nowAngleS		= f_TrgtAngleS;			//���݊p���x
	st_data.f_trgtAngleS	= 0;					//�ڕW�p���x
	st_data.f_nowAngle		= st_info.f_angle1_2;	//���݊p�x
	st_data.f_angle			= st_info.f_angle;		//�ڕW�p�x
	st_data.f_time			= 0;					//�ڕW����[sec]���w�肵�Ȃ�
	
	CTRL_setData( &st_data );						// �f�[�^�Z�b�g
	
	if(( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 )){	//-����
		while( f_NowAngle > ( st_info.f_angle + 1)){		//�w��p�x���B�҂�(�E��])
			
			/*�E�o*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// �����~
				break;
			}
		
			//if( SYS_isOutOfCtrl() == true ) break;		//�r���Ő���s�\�ɂȂ���
		}
	}else{
		while( f_NowAngle < (st_info.f_angle - 1)){		//�w��p�x���B�҂�(����])
		
			/*�E�o*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// �����~
				break;
			}
			//if( SYS_isOutOfCtrl() == true ) break;		//�r���Ő���s�\�ɂȂ���
		}
	
	}

	//printf("���񊮗�");
	/* ��~ */
	TIME_wait(200);		//����҂�
	CTRL_stop();		// �����~	
	
}


// *************************************************************************
//   �@�\		�F �������@�O�i
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ��搔
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			�g�c			�V�K
// *************************************************************************/
PUBLIC void MOT_goBlock_Const( FLOAT f_num){
	volatile stMOT_DATA	st_info;			//�V�[�P���X�f�[�^
	stCTRL_DATA		st_data;			//����f�[�^
		
	/*----------------*/
	/* ����f�[�^�v�Z */
	/*----------------*/
	/* ���� */
	st_info.f_dist	= f_num * BLOCK;
	
	/*------*/
	/* ���� */
	/*------*/
	st_data.en_type		= CTRL_CONST;
	st_data.f_acc		= 0;			//�����x�w��
	st_data.f_now		= f_MotNowSpeed;	//���ݑ��x
	st_data.f_trgt		= f_MotNowSpeed;	//�ڕW���x
	st_data.f_nowDist	= 0;			//���݈ʒu
	st_data.f_dist		= st_info.f_dist;	//���������ʒu
	st_data.f_accAngleS	= 0;			//�p�����x
	st_data.f_nowAngleS	= 0;			//���݊p���x
	st_data.f_trgtAngleS	= 0;			//�ڕW�p�x
	st_data.f_nowAngle	= 0;			//���݊p�x
	st_data.f_angle		= 0;			//�ڕW�p�x
	st_data.f_time		= 0;			//�ڕW���� [sec] ���w�肵�Ȃ�
	
	CTRL_clrData();					//�ݒ�f�[�^���N���A
	CTRL_setData( &st_data );			//�f�[�^�Z�b�g
	f_TrgtSpeed		= f_MotNowSpeed;	//�ڕW���x

	//printf("st_info.f_dist:%5.4f \n\r",st_info.f_dist);

	while( f_NowDist < st_info.f_dist  ){		//�w�苗�����B�҂�
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE ){
				return;
			}

	}
	
}

// *************************************************************************
//   �@�\		�F �ǂ��Đ���
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			TKR			�V�K
// *************************************************************************/
PUBLIC void MOT_goHitBackWall(void){
	
	volatile stMOT_DATA	st_info;			//�V�[�P���X�f�[�^
	stCTRL_DATA		st_data;			//����f�[�^
	
	/*----------------*/
	/* ����f�[�^�v�Z */
	/*----------------*/
	st_info.f_acc1	= 800;
	
	/*--------*/
	/* �o�b�N */
	/*--------*/
	st_data.en_type			= CTRL_HIT_WALL;
	st_data.f_acc			= st_info.f_acc1 * 1.0;		// �����x�w��
	st_data.f_now			= 0;				// ���ݑ��x
	st_data.f_trgt			= 0;				// �ڕW���x
	st_data.f_nowDist		= 0;				// �i��ł��Ȃ�
	st_data.f_dist			= 0;				// ��������
	st_data.f_accAngleS		= 0;				// �p�����x
	st_data.f_nowAngleS		= 0;				// ���݊p���x
	st_data.f_trgtAngleS	= 0;					// �ڕW�p�x
	st_data.f_nowAngle		= 0;				// ���݊p�x
	st_data.f_angle			= 0;				// �ڕW�p�x
	st_data.f_time 			= 0;				// �ڕW���� [sec] �� �w�肵�Ȃ�
	
	CTRL_clrData();							// �}�E�X�̌��݈ʒu/�p�x���N���A
	CTRL_setData( &st_data );					// �f�[�^�Z�b�g
	DCM_staMotAll();						// ���[�^ON
	
	TIME_wait(550);
	
	/* ��~ */
	CTRL_stop();		// �����~
	DCM_brakeMot( DCM_R );	// �u���[�L
	DCM_brakeMot( DCM_L );	// �u���[�L
	TIME_wait(300);
	
	f_MotNowSpeed = 0.0f;		//���ݑ��x�X�V
	
}

// *************************************************************************
//   �@�\		�F �X�����[��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �X�����[���R�}���h�C�X�����[���f�[�^
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			TKR			�V�K
// *************************************************************************/
PUBLIC void MOT_goSla( enMOT_SULA_CMD en_type, stSLA *p_sla){
	
	volatile stMOT_DATA	st_info;			//�V�[�P���X�f�[�^
	stCTRL_DATA		st_data;			//����f�[�^
	
	FLOAT			f_entryLen;			
	FLOAT			f_escapeLen;
		
	/* -------------- */
	/* ����f�[�^�v�Z */
	/* -------------- */
	/* �����x */
	st_info.f_acc1 		= 0;					//�����x1[m/s^2]
	st_info.f_acc3		= 0;					//�����x3[m/s^2]
	
	/* ���x */
	st_info.f_now		= p_sla -> f_speed;			//���ݑ��x
	st_info.f_trgt		= p_sla -> f_speed;			//�ڕW���x
	st_info.f_last		= p_sla -> f_speed;			//�ŏI���x
	
	/* ���� */
	st_info.f_dist		= 0;
	st_info.f_l1		= 0;
	st_info.f_l1_2		= 0;
	
	/* �p�����x */
	st_info.f_accAngleS1	= p_sla -> f_angAcc;
	st_info.f_accAngleS3	= p_sla -> f_angAcc;
	
	/* �p���x */
	st_info.f_nowAngleS 	= 0;
	st_info.f_trgtAngleS 	= p_sla -> f_angvel;
	st_info.f_lastAngleS	= 0;
	
	/* �p�x */
	st_info.f_angle		= p_sla -> f_ang_Total;			//����p�x[deg]
	st_info.f_angle1	= p_sla -> f_ang_AccEnd;		//��1�ړ��p�x[deg]
	st_info.f_angle1_2	= p_sla -> f_ang_ConstEnd;		//��1+2�ړ��p�x[deg]


	/* �����ɉ����ĕ�����ύX */
	if( ( en_type == MOT_R90S ) ||
	    ( en_type == MOT_R45S_S2N ) || ( en_type == MOT_R45S_N2S )|| 
	    ( en_type == MOT_R90S_N ) ||
	    ( en_type == MOT_R135S_S2N ) || (en_type == MOT_R135S_N2S)
	){
		st_info.f_accAngleS1	*= -1;
		st_info.f_trgtAngleS	*= -1;
		st_info.f_angle		*= -1;
		st_info.f_angle1	*= -1;
		st_info.f_angle1_2	*= -1;
	}
	else{
		st_info.f_accAngleS3	*= -1;
	}
	
	
	/* �΂߂̃^�C�v�ɉ����āA�X�����[���O�̋����ƃX�����[����̑ޔ����������ւ��� */
		if( ( en_type == MOT_R45S_N2S ) || ( en_type == MOT_L45S_N2S ) || ( en_type == MOT_R135S_N2S ) || ( en_type == MOT_L135S_N2S ) ){	//�t�ɂ������
		f_entryLen	= p_sla -> f_escapeLen;
		f_escapeLen	= p_sla -> f_entryLen;
	}
	else{	//�ʏ�
	
		f_entryLen	= p_sla -> f_entryLen;
		f_escapeLen	= p_sla -> f_escapeLen;
	}
	
	/* ========== */
	/* �@������@ */
	/* ========== */
	/* ---------------------- */
	/* �X�����[���O�̑O�i���� */
	/* ---------------------- */
	st_data.en_type			= CTRL_ENTRY_SLA;
	st_data.f_acc			= 0;				// �����x�w��
	st_data.f_now			= st_info.f_now;	// ���ݑ��x
	st_data.f_trgt			= st_info.f_now;	// �ڕW���x
	st_data.f_nowDist		= 0;				// �i��ł��Ȃ�
	st_data.f_dist			= f_entryLen;		// �X�����[���O�̑O�i����
	st_data.f_accAngleS		= 0;				// �p�����x
	st_data.f_nowAngleS		= 0;				// ���݊p���x
	st_data.f_trgtAngleS	= 0;				// �ڕW�p�x
	st_data.f_nowAngle		= 0;				// ���݊p�x
	st_data.f_angle			= 0;				// �ڕW�p�x
	st_data.f_time 			= 0;				// �ڕW���� [sec] �� �w�肵�Ȃ�
	
	CTRL_clrData();					// �}�E�X�̌��݈ʒu/�p�x���N���A
	CTRL_setData( &st_data );			// �f�[�^�Z�b�g
	DCM_staMotAll();				// ���[�^ON

//	LED_onAll();
	while( f_NowDist < f_entryLen ){				// �w�苗�����B�҂�
			/* �t�F�C���Z�[�t */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;
	}
	
	
	/* -------- */
	/* �@�����@ */
	/* -------- */
	st_data.en_type			= CTRL_ACC_SLA;	
	st_data.f_acc			= 0;						// �����x�w��
	st_data.f_now			= st_info.f_now;			// ���ݑ��x
	st_data.f_trgt			= st_info.f_now;			// �ڕW���x
	st_data.f_nowDist		= f_entryLen;	
	st_data.f_dist			= f_entryLen + st_info.f_now * p_sla->us_accAngvelTime * 0.001;		//��������
	st_data.f_accAngleS		= st_info.f_accAngleS1;		// �p�����x
	st_data.f_nowAngleS		= 0;						// ���݊p���x
	st_data.f_trgtAngleS	= st_info.f_trgtAngleS;		// �ڕW�p���x
	st_data.f_nowAngle		= 0;						// ���݊p�x
	st_data.f_angle			= st_info.f_angle1;			// �ڕW�p�x
	st_data.f_time			= p_sla->us_accAngvelTime * 0.001;		//[msec] �� [sec]
	
	CTRL_setData( &st_data );			// �f�[�^�Z�b�g
	
	if( IS_R_SLA( en_type ) == true ){	// -����
		while( ( f_NowAngle > st_info.f_angle1 ) ){
		//while( ( f_NowAngle > st_info.f_angle1 ) || ( f_NowDist < st_data.f_dist ) ){
			/* �t�F�C���Z�[�t */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;
			
			//break;
		}
	}
	else{
		while( ( f_NowAngle < st_info.f_angle1 ) ){
		//while( ( f_NowAngle < st_info.f_angle1 ) || ( f_NowDist < st_data.f_dist ) ){
			/* �t�F�C���Z�[�t */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;

			//break;
		}
	}
	
	
	/* -------- */
	/* �@�����@ */
	/* -------- */
	st_data.en_type			= CTRL_CONST_SLA;	
	st_data.f_acc			= 0;						// �����x�w��
	st_data.f_now			= st_info.f_now;			// ���ݑ��x
	st_data.f_trgt			= st_info.f_now;			// �ڕW���x
	st_data.f_nowDist		= f_entryLen + st_info.f_now * p_sla->us_accAngvelTime * 0.001;	
	st_data.f_dist			= f_entryLen + st_info.f_now * (p_sla->us_constAngvelTime + p_sla->us_accAngvelTime) * 0.001;		//��������
	st_data.f_accAngleS		= 0;						// �p�����x
	st_data.f_nowAngleS		= st_info.f_trgtAngleS;		// ���݊p���x
	st_data.f_trgtAngleS	= st_info.f_trgtAngleS;		// �ڕW�p���x
	st_data.f_nowAngle		= st_info.f_angle1;			// ���݊p�x
	st_data.f_angle			= st_info.f_angle1_2;		// �ڕW�p�x
	st_data.f_time			= p_sla->us_constAngvelTime * 0.001;
			
	CTRL_setData( &st_data );				// �f�[�^�Z�b�g
	
	if( IS_R_SLA( en_type ) == true ){		// -����
		while( ( f_NowAngle > st_info.f_angle1_2 ) ){
		//while( ( f_NowAngle > st_info.f_angle1_2 ) || ( f_NowDist < st_data.f_dist ) ){
			/* �t�F�C���Z�[�t */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;
			//break;
		}
	}
	else{
		while( ( f_NowAngle < st_info.f_angle1_2 ) ){
		//while( ( f_NowAngle < st_info.f_angle1_2 ) || ( f_NowDist < st_data.f_dist ) ){
			/* �t�F�C���Z�[�t */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;
			//break;
		}
	}
	
	
	/* -------- */
	/* �@�����@ */
	/* -------- */
	st_data.en_type			= CTRL_DEC_SLA;	
	st_data.f_acc			= 0;						// �����x�w��
	st_data.f_now			= st_info.f_now;			// ���ݑ��x
	st_data.f_trgt			= st_info.f_now;			// �ڕW���x
	st_data.f_nowDist		= f_entryLen + st_info.f_now * ( p_sla->us_constAngvelTime + p_sla->us_accAngvelTime)* 0.001;	
	st_data.f_dist			= f_entryLen + st_info.f_now * ( p_sla->us_constAngvelTime + p_sla->us_accAngvelTime * 2) * 0.001;		//��������
	st_data.f_accAngleS		= st_info.f_accAngleS3;		// �p�����x
	st_data.f_nowAngleS		= st_info.f_trgtAngleS;		// ���݊p���x
	st_data.f_trgtAngleS	= 0;						// �ڕW�p���x
	st_data.f_nowAngle		= st_info.f_angle1_2;		// ���݊p�x
	st_data.f_angle			= st_info.f_angle;			// �ڕW�p�x
	st_data.f_time			= p_sla->us_accAngvelTime * 0.001;

	CTRL_setData( &st_data );			// �f�[�^�Z�b�g
	
	if( IS_R_SLA( en_type ) == true ){	// -����
		while( ( f_NowAngle > st_info.f_angle ) ){
		//while( ( f_NowAngle > st_info.f_angle + 2.0f ) || ( f_NowDist < st_data.f_dist ) ){
			/* �t�F�C���Z�[�t */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;
		}
	
	}else{
		while( ( f_NowAngle < st_info.f_angle ) ){
		//while( ( f_NowAngle < st_info.f_angle -1.0f ) || ( f_NowDist < st_data.f_dist ) ){
			/* �t�F�C���Z�[�t */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;
		}
	}
	
	/* ---------------------- */
	/* �X�����[����̑O�i���� */
	/* ---------------------- */
	st_data.en_type			= CTRL_EXIT_SLA;
	st_data.f_acc			= 0;				//�����x�w��
	st_data.f_now			= st_info.f_now;	//���ݑ��x
	st_data.f_trgt			= st_info.f_now;	//�ڕW���x
	st_data.f_nowDist		= f_entryLen + st_info.f_now * (p_sla->us_constAngvelTime + p_sla->us_accAngvelTime * 2) * 0.001;			//�i��ł��Ȃ�
	st_data.f_dist			= f_escapeLen + f_entryLen + st_info.f_now * (p_sla->us_constAngvelTime + p_sla->us_accAngvelTime * 2)*0.001;		//�X�����[����̑O�i����
	st_data.f_accAngleS		= 0;				// �p�����x
	st_data.f_nowAngleS		= 0;				// ���݊p���x
	st_data.f_trgtAngleS	= 0;				// �ڕW�p�x
	st_data.f_nowAngle		= 0;				// ���݊p�x
	st_data.f_angle			= 0;				// �ڕW�p�x
	st_data.f_time 			= 0;				// �ڕW���� [sec] �� �w�肵�Ȃ�
	
	//CTRL_clrData();					// �}�E�X�̌��݈ʒu/�p�x���N���A
	CTRL_setData( &st_data );			// �f�[�^�Z�b�g

	while( f_NowDist < st_data.f_dist ){				// �w�苗�����B�҂�
			/* �t�F�C���Z�[�t */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;
	}
	LED_offAll();
	f_MotNowSpeed = st_info.f_now;			// ���ݑ��x�X�V

}

// *************************************************************************
//   �@�\		�F �X�����[���J�n���x��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �J�n���x
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			TKR			�V�K
// *************************************************************************/
PUBLIC	void MOT_setSlaStaSpeed( FLOAT f_speed ) {
	
	f_MotSlaStaSpeed = f_speed;

}

// *************************************************************************
//   �@�\		�F �X�����[���J�n���x���擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �J�n���x
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			TKR			�V�K
// *************************************************************************/
PUBLIC	FLOAT MOT_getSlaStaSpeed( void ) {
	return f_MotSlaStaSpeed;
}

// *************************************************************************
//   �@�\		�F �ǐ؂�␳�̃^�C�v���擾
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �ǐ؂�␳�̃^�C�v
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			TKR			�V�K
// *************************************************************************/
PUBLIC enMOT_WALL_EDGE_TYPE MOT_getWallEdgeType( void ){
	return en_WallEdge;
}

// *************************************************************************
//   �@�\		�F �ǐ؂�̌��m��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			TKR			�V�K
// *************************************************************************/
PUBLIC void MOT_setWallEdge( BOOL bl_val ){
	
	bl_IsWallEdge = bl_val;
	
}

// *************************************************************************
//   �@�\		�F �ǐ؂�̌��m��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			TKR			�V�K
// *************************************************************************/
PUBLIC void MOT_setWallEdgeType( enMOT_WALL_EDGE_TYPE en_type ){
	
	en_WallEdge = en_type;
	bl_IsWallEdge = false;			// �񌟒m
	
}

// *************************************************************************
//   �@�\		�F �ǂ̐؂�ڂŕ␳���鋗�����Z�o���Đݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F 
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			TKR			�V�K
// *************************************************************************/
PRIVATE BOOL MOT_setWallEdgeDIST( void ){
	
	FLOAT f_addDist;
	
	/* �ǂ̐؂�ڂ����m���Ă��Ȃ� */
	if( ( bl_IsWallEdge == false ) || ( en_WallEdge == MOT_WALL_EDGE_NONE ) ){	// �ǐ؂�ݒ肳��Ă��Ȃ����A���o���Ă��Ȃ��ꍇ�͏����𔲂���
		
		return false;
		
	}
	
	f_addDist = f_NowDist + MOT_WALL_EDGE_DIST;	//����J�n�ʒu
	
	/* ��������K�v������ */
	if( f_addDist > st_Info.f_dist ){
		f_WallEdgeAddDist = f_addDist - st_Info.f_dist;
	}
	
	/* �ǂ̐؂�ڕ␳�̕ϐ��������� */
	en_WallEdge	= MOT_WALL_EDGE_NONE;		//�ǂ̐؂�ڃ^�C�v
	bl_IsWallEdge	= false;			//�ǂ̐؂�ڌ��m
	
	return true;
}

// *************************************************************************
//   �@�\		�F �ǂ̐؂�ڂŕ␳���鋗�����Z�o���Đݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F 
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			TKR			�V�K
// *************************************************************************/
PRIVATE	BOOL MOT_setWallEdgeDIST_LoopWait( void ){
	
	/* �ǂ̐؂�ڂ����m���Ă��Ȃ� */
	if( bl_IsWallEdge == false ){		// �ǐ؂�ݒ肳��Ă��Ȃ����A���o���Ă��Ȃ��ꍇ�͏����𔲂���
		return	false;
	}
	
	f_WallEdgeAddDist = MOT_WALL_EDGE_DIST;		// ����J�n�ʒu
	
	return true;
}

// *************************************************************************
//   �@�\		�F �}�E�X�̖ڕW���x��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �ڕW���x
//   �Ԃ�l		�F �ڕW���x
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			TKR			�V�K
// *************************************************************************/
PUBLIC void MOT_setTrgtSpeed( FLOAT f_speed){
	
	f_MotTrgtSpeed = f_speed;

}

// *************************************************************************
//   �@�\		�F �}�E�X�̖ڕW���x���擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �ڕW���x
// **************************    ��    ��    *******************************
// 		v1.0		2019.11.28			TKR			�V�K
// *************************************************************************/
PUBLIC FLOAT MOT_getTrgtSpeed( void ){
	
	return f_MotTrgtSpeed;

}

// *************************************************************************
//   �@�\		�F �΂ߑ��s�̖ڕW���x��ݒ肷��
//   ����		�F f_MotTrgtSpeed�ɑ�����Ă��Ȃ��̂ɒ���
//   ����		�F �Ȃ�
//   ����		�F �ڕW���x
//   �Ԃ�l		�F �ڕW���x
// **************************    ��    ��    *******************************
// 		v1.0		2019.11.28			TKR			�V�K
// *************************************************************************/
PUBLIC void MOT_setTrgtSkewSpeed( FLOAT f_speed ){
	
	f_MotTrgtSkewSpeed = f_speed;

}

// *************************************************************************
//   �@�\		�F �΂ߑ��s�̖ڕW���x���擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �ڕW���x
// **************************    ��    ��    *******************************
// 		v1.0		2019.11.28			TKR			�V�K
// *************************************************************************/
PUBLIC FLOAT MOT_getTrgtSkewSpeed( void ){
	
	return f_MotTrgtSkewSpeed;

}

// *************************************************************************
//   �@�\		�F �}�E�X�̌��ݑ��x��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ���ݑ��x
//   �Ԃ�l		�F ���ݑ��x
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.1			TKR			�V�K
// *************************************************************************/
PUBLIC void MOT_setNowSpeed( FLOAT f_speed){
	
	f_MotNowSpeed = f_speed;

}

// *************************************************************************
//   �@�\		�F �t�F�C���Z�[�t
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F TRUE�F����	FALSE�F�����Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.9.22			TKR			�V�K
// *************************************************************************/
PRIVATE void MOT_Failsafe( BOOL* exists ){
	
	if( f_NowAccel < FAIL_THRESH_ACC ){
		CTRL_stop();
		*exists	= TRUE;

		SPK_on(F4,16.0f,120);
		SPK_on(E4,16.0f,120);
		SPK_on(Eb4,16.0f,120);

		while(1);
	}else{

		*exists	= FALSE;
	}
	
}

// *************************************************************************
//   �@�\		�F �T�[�L�b�g
//   ����		�F �E���FY	�����FX
//   ����		�F �Ȃ�
//   ����		�F ���̍L��X�C���̍L��Y�C�������C���񐔁C���x
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.10.18			TKR			�V�K
// *************************************************************************/
PUBLIC void MOT_circuit(FLOAT x,FLOAT y, enMOT_SULA_CMD en_type, int num, FLOAT f_speed){

	int i = 0;

	if( en_type == MOT_R90 ){
		MOT_goBlock_FinSpeed( y-1.5f+MOVE_BACK_DIST, f_speed );	
		for( i = 0; i < num; i++ ){
			MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );			// �X�����[��
			MOT_goBlock_FinSpeed( x-2.0f, f_speed );				
			MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );			// �X�����[��
			MOT_goBlock_FinSpeed( y-2.0f, f_speed);					
			MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );			// �X�����[��
			MOT_goBlock_FinSpeed( x-2.0f, f_speed);				
			MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );			// �X�����[��
			MOT_goBlock_FinSpeed( y-2.0f, f_speed);
		}
	}else{

		MOT_goBlock_FinSpeed( x-1.5f+MOVE_BACK_DIST, f_speed );
		for( i = 0; i < num; i++ ){
			MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );			// �X�����[��
			MOT_goBlock_FinSpeed( y-2.0f, f_speed );				
			MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );			// �X�����[��
			MOT_goBlock_FinSpeed( x-2.0f, f_speed);					
			MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );			// �X�����[��
			MOT_goBlock_FinSpeed( y-2.0f, f_speed);				
			MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );			// �X�����[��
			MOT_goBlock_FinSpeed( x-2.0f, f_speed);
		}

	}
	
	MOT_goBlock_FinSpeed(0.5f,0);					// ����摖�s
}