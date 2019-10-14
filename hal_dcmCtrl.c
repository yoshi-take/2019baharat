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

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>					// ��`
#include <common_define.h>				// ���ʒ�`
#include <iodefine.h>					// I/O
#include <stdio.h>						// �W�����o��
#include <math.h>						// ���l�v�Z

#include <parameter.h>                  // parameter

#include <hal_dcmCtrl.h>				// DCM_CTRL
#include <hal_dcm.h>                    // DCM
#include <hal_enc.h>					// Encoder
#include <hal_battery.h>				// �o�b�e���[
#include <hal_dist.h>					// DIST
#include <hal_gyro.h>					// GYRO
//**************************************************
// ��`�idefine�j
//**************************************************
#define     VCC_MAX             (8.4f)
#define     FF_BALANCE_R        (1.0f)
#define     FF_BALANCE_L        (1.0f)
#define     FF_HIT_BALANCE_R    (1.0f)
#define     FF_HIT_BALANCE_L    (1.0f)

//**************************************************
// �񋓑́ienum�j
//**************************************************

//**************************************************
// �\���́istruct�j
//**************************************************
/* ���O */
typedef struct 
{
	FLOAT	f_time;				// ����
	FLOAT	f_trgtSpeed;		// ���x�i���_�l�j
	FLOAT	f_nowSpeed;			// ���x�i�����l�j
	FLOAT	f_trgtPos;			// �ʒu�i���_�l�j
	FLOAT	f_nowPos;			// �ʒu�i�����l�j
	FLOAT	f_trgtAngleS;		// �p���x�i���_�l�j
	FLOAT	f_nowAngleS;		// �p���x�i�����l�j
	FLOAT	f_trgtAngle;		// �p�x�i���_�l�j
	FLOAT	f_nowAngle;			// �p�x�i�����l�j
	FLOAT	f_nowAccel;			// �����x�i�����l�j
}stDCMC_SET_LOG;



//**************************************************
// �ϐ�
//**************************************************
/* ���� */
PRIVATE enCTRL_TYPE     en_Type;                    // �������
PUBLIC  FLOAT           f_Time          = 0;        // ���쎞��[sec]
PUBLIC  FLOAT           f_TrgtTime      = 1000;     // ����ڕW����[msec]
PRIVATE UCHAR 			uc_CtrlFlag	    = false;	// �t�B�[�h�o�b�N or �t�B�[�h�t�H���[�h ����L���t���O�ifalse:�����A1�F�L���j �錾���͖����ɂ��邱��
PRIVATE LONG            l_CntR          = 0;        // �E���[�^�[�J�E���g��
PRIVATE LONG            l_CntL          = 0;        // �����[�^�[�J�E���g��

/* ���x���� */
PRIVATE FLOAT           f_Acc           = 3;        // [���x����]�@�����x
PRIVATE FLOAT           f_BaseSpeed     = 10;       // [���x����]�@�����x
PRIVATE FLOAT           f_LastSpeed     = 180;      // [���x����]�@�ŏI�ڕW���x
PRIVATE FLOAT           f_NowSpeed      = 0;        // [���x����]�@���݂̑��x[mm/s]     �i1[msec]���ɍX�V�����j
PUBLIC  FLOAT           f_TrgtSpeed     = 0;        // [���x����]�@�ڕW���x             �i1[msec]���ɍX�V�����j
PUBLIC	FLOAT			f_SpeedErrSum	= 0;		// [���x����]�@���x�ϕ��΍�			�i1[msec]���ɍX�V�����j

/* �������� */
PRIVATE FLOAT           f_BaseDist      = 0;        // [��������]�@�����ʒu
PRIVATE FLOAT           f_LastDist      = 0;        // [��������]�@�ŏI�ړ�����
PUBLIC  FLOAT           f_TrgtDist      = 0;        // [��������]�@�ڕW�ړ�����         �i1[msec]���ɍX�V�����j
PUBLIC  volatile FLOAT  f_NowDist       = 0;        // [��������]�@���݋���             �i1[msec]���ɍX�V�����j
PRIVATE FLOAT           f_NowDistR      = 0;        // [��������]�@���݋���(�E)         �i1[msec]���ɍX�V�����j
PRIVATE FLOAT           f_NowDistL      = 0;        // [��������]�@���݋���(��)         �i1[msec]���ɍX�V�����j
PUBLIC  FLOAT           f_DistErrSum    = 0;        // [��������]�@�����ϕ��΍�         �i1[msec]���ɍX�V�����j

/*�p���x����*/
PRIVATE FLOAT           f_AccAngleS     = 3;        // [�p���x����]�@�p�����x
PRIVATE FLOAT           f_BaseAngleS    = 10;       // [�p���x����]�@�����p���x
PRIVATE FLOAT           f_LastAngleS    = 180;      // [�p���x����]�@�ŏI�ڕW���x
PUBLIC  FLOAT           f_TrgtAngleS    = 0;        // [�p���x����]�@�ڕW�p���x         �i1[msec]���ɍX�V�����j
PUBLIC	FLOAT			f_AngleSpeedErrSum	= 0;	// [�p���x����]�@�p���x�ϕ�			�i1[msec]���ɍX�V�����j

/* �p�x���� */
PRIVATE FLOAT           f_BaseAngle     = 0;        // [�p�x����]�@�����p�x
PRIVATE FLOAT           f_LastAngle     = 0;        // [�p�x����]�@�ŏI�ڕW�p�x
PUBLIC  FLOAT           f_TrgtAngle     = 0;        // [�p�x����]�@�ڕW�p�x             �i1[msec]���ɍX�V�����j
PUBLIC  volatile FLOAT  f_NowAngle      = 0;        // [�p�x����]�@���݊p�x             �i1[msec]���ɍX�V�����j
PUBLIC  FLOAT           f_AngleErrSum   = 0;        // [�p�x����]�@�����ϕ�����         �i1[msec]���ɍX�V�����j

extern	PUBLIC volatile FLOAT  f_NowGyroAngle;		 					// �W���C���Z���T�̌��݊p�x
extern	PUBLIC volatile FLOAT  f_NowGyroAngleSpeed;						// �W���C���Z���T�̌��݊p���x
extern	PUBLIC volatile FLOAT  f_NowAccel;								// �i�s�����̌��݉����x

/* �ǐ��� */
PRIVATE LONG            l_WallErr       = 0;        // [�ǐ���]�@�ǂƂ̕΍�             �i1[msec]���ɍX�V�����j
PRIVATE FLOAT           f_ErrDistBuf    = 0;        // [�ǐ���]�@�����Z���T�̌덷�̐ϕ�  �i1[msec]���ɍX�V�����j

/* ���O */
PRIVATE	stDCMC_SET_LOG	st_Log[CTRL_LOG];			// ���O	
PRIVATE USHORT			us_LogPt		= 0;		// ���O�ʒu
PRIVATE BOOL			bl_log			= false;	// ���O�擾�����ifalse:�֎~�@true:���j


//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
// *************************************************************************
//   �@�\		�F �����������p�����[�^ID�ɕϊ�����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.4.4			�g�c			�V�K
// *************************************************************************/
PRIVATE enPARAM_MODE Chg_ParamID( enCTRL_TYPE en_type ){
	
	switch( en_type ){
		
		case CTRL_ACC:			return PARAM_ACC;			// ������(���i)
		case CTRL_CONST:		return PARAM_CONST;			// ������(���i)
		case CTRL_DEC:			return PARAM_DEC;			// ������(���i)
		
		case CTRL_HIT_WALL:		return PARAM_HIT_WALL;		// �ǂ��Đ���
		
		case CTRL_SKEW_ACC:		return PARAM_SKEW_ACC;		// ������(�΂ߒ��i)
		case CTRL_SKEW_CONST:	return PARAM_SKEW_CONST;	// ������(�΂ߒ��i)
		case CTRL_SKEW_DEC:		return PARAM_SKEW_DEC;		// ������(�΂ߒ��i)
		
		case CTRL_ACC_SMOOTH:	return PARAM_ACC_SMOOTH;	// ������(���i cos�ߎ�)
		case CTRL_CONST_SMOOTH:	return PARAM_CONST_SMOOTH;	// ������(���i cos�ߎ�)
		case CTRL_DEC_SMOOTH:	return PARAM_DEC_SMOOTH;	// ������(���i cos�ߎ�)
		
		case CTRL_ACC_TURN:		return PARAM_ACC_TURN;		// ������(���M�n����)
		case CTRL_CONST_TURN:	return PARAM_CONST_TURN;	// ������(���M�n����)
		case CTRL_DEC_TURN:		return PARAM_DEC_TURN;		// ������(���M�n����)
		
		case CTRL_ENTRY_SLA:	return PARAM_ENTRY_SURA;	// �X�����[���O�̑O�i����
		case CTRL_ACC_SLA:		return PARAM_ACC_SURA;		// ������(�X�����[��)
		case CTRL_CONST_SLA:	return PARAM_CONST_SURA;	// ������(�X�����[��)
		case CTRL_DEC_SLA:		return PARAM_DEC_SURA;		// ������(�X�����[��)
		case CTRL_EXIT_SLA:		return PARAM_EXIT_SURA;		// �X�����[����̑O�i����
		
		default:				return PARAM_NC;
	}
}


// *************************************************************************
//   �@�\		�F ������J�n����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.22			TKR			�V�K
// *************************************************************************/
PUBLIC  void    CTRL_sta(void){
    uc_CtrlFlag = true;
}

// *************************************************************************
//   �@�\		�F ������~����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.22			TKR			�V�K
// *************************************************************************/
PUBLIC  void    CTRL_stop(void){
    uc_CtrlFlag = false;
    DCM_brakeMot(DCM_R);        // �u���[�L
    DCM_brakeMot(DCM_L);        // �u���[�L
}

// *************************************************************************
//   �@�\		�F ����f�[�^���N���A����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.22			TKR			�V�K
// *************************************************************************/
PUBLIC void CTRL_clrData(void){

    ENC_clr();      // ENC���W���[��������
    l_CntR  = 0;     
    l_CntL  = 0;

    /* ���ݒl */
	f_NowDist 		= 0;						// �ړ��������Z�b�g
	f_NowDistR 		= 0;
	f_NowDistL 		= 0;
	f_NowSpeed		= 0;						// [���x����]   ���݂̑��x [mm/s]				�i1[msec]���ɍX�V�����j
	f_NowAngle		= 0;						// [�p�x����]   ���݊p�x						�i1[msec]���ɍX�V�����j
	GYRO_clrAngle();							// �p�x���Z�b�g
	
	/* �ڕW�l */
	f_TrgtSpeed		= 0;						// [���x����]   �ڕW�ړ����x [mm/s]				�i1[msec]���ɍX�V�����j
	f_TrgtDist 		= 0;						// [��������]   �ڕW�ړ�����					�i1[msec]���ɍX�V�����j
	f_TrgtAngleS	= 0;    					// [�p���x����] �ڕW�p���x [rad/s]				�i1[msec]���ɍX�V�����j
	f_TrgtAngle		= 0;						// [�p�x����]   �ڕW�p�x						�i1[msec]���ɍX�V�����j
	
	/* ����f�[�^ */
	f_DistErrSum 	= 0;						// [��������]   �����ϕ�����̃T���l			�i1[msec]���ɍX�V�����j
	f_AngleErrSum 	= 0;						// [�p�x����]   �p�x�ϕ�����̃T���l			�i1[msec]���ɍX�V�����j
	f_ErrDistBuf	= 0;						// [�ǐ���]     �����Z���T�[�G���[�l�̃o�b�t�@	�i1[msec]���ɍX�V�����j
}

// *************************************************************************
//   �@�\		�F ����f�[�^���Z�b�g����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ����f�[�^
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PUBLIC  void    CTRL_setData( stCTRL_DATA *p_data ){

    /* ������@ */
	en_Type = p_data->en_type;
    /* ���x���� */
    f_Acc 					= p_data->f_acc;
	f_BaseSpeed				= p_data->f_now;
	f_LastSpeed				= p_data->f_trgt;

    /* �������� */
    f_BaseDist 				= p_data->f_nowDist;
	f_LastDist 				= p_data->f_dist;

    /* �p���x���� */
    f_AccAngleS 			= p_data->f_accAngleS;
	f_BaseAngleS			= p_data->f_nowAngleS;
	f_LastAngleS			= p_data->f_trgtAngleS;

    /* �p�x���� */
    f_BaseAngle 			= p_data->f_nowAngle;
	f_LastAngle 			= p_data->f_angle;

    f_Time                  = 0;
    f_TrgtTime              = p_data->f_time;

    CTRL_sta();     // ����J�n

}

// *************************************************************************
//   �@�\		�F ����f�[�^�����݂̏�ԂɍX�V����
//   ����		�F CTRL_pol����̂ݎ��s�\
//   ����		�F 1msec���Ɏ��s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PRIVATE void    CTRL_refNow( void ){

    FLOAT f_speedR		= 0;							// �E���[�^���ݑ��x [mm/s]
	FLOAT f_speedL		= 0;							// �����[�^���ݑ��x [mm/s]
	FLOAT f_r 			= F_CNT2MM(l_CntR);				// �E���[�^�̐i�񂾋��� [mm]
	FLOAT f_l 			= F_CNT2MM(l_CntL);				// �����[�^�̐i�񂾋��� [mm]

	/* ���x�X�V */
	f_speedR = f_r * 1000;								// �E���[�^���x [mm/s] ( �ړ�����[�J�E���g] * 1�p���X�̈ړ���(0.0509[mm]) * 1000(msec��sec) 
	f_speedL = f_l * 1000;								// �����[�^���x [mm/s] ( �ړ�����[�J�E���g] * 1�p���X�̈ړ���(0.0509[mm]) * 1000(msec��sec) 
	f_NowSpeed  = ( f_speedR + f_speedL ) / 2;			// �}�E�X�i�i�s�������S���j [1mm/s] 
	
	/* �����X�V */
	f_NowDistR += f_r;									// �J�E���g�X�V
	f_NowDistL += f_l;									// �J�E���g�X�V
	f_NowDist  = ( f_NowDistR + f_NowDistL ) / 2;		// ���ϒl�X�V
	

}

// *************************************************************************
//   �@�\		�F ����f�[�^��ڕW�l�ɍX�V����
//   ����		�F CTRL_pol����̂ݎ��s�\
//   ����		�F 1msec���Ɏ��s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PUBLIC  void    CTRL_refTarget( void ){

    /*���샂�[�h�ɉ�����*/
    switch( en_Type ){
	
		/* ������(���i) */
		case CTRL_ACC:
		case CTRL_SKEW_ACC:
			if( f_TrgtSpeed < f_LastSpeed ){												// �����ڕW�X�V���
				f_TrgtSpeed = f_BaseSpeed + f_Acc * f_Time;									// �ڕW���x
				f_TrgtDist	= f_BaseSpeed * f_Time + 0.5 * f_Acc * f_Time * f_Time;			// �ڕW�ʒu
			}
			break;
		
		/* ������(���i) */
		case CTRL_CONST:
		case CTRL_SKEW_CONST:
			f_TrgtSpeed = f_BaseSpeed;														// �ڕW���x
			f_TrgtDist	= f_BaseDist + f_BaseSpeed * f_Time;								// �ڕW�ʒu
			break;
		
		/* ������(���i) */
		case CTRL_DEC:
		case CTRL_SKEW_DEC:
			/* ���x���� �{ �ʒu���� */
			if( f_TrgtSpeed > f_LastSpeed ){												// �����ڕW�X�V���
				f_TrgtSpeed = f_BaseSpeed - f_Acc * f_Time;									// �ڕW���x
				f_TrgtDist  = f_BaseDist + ( f_BaseSpeed + f_TrgtSpeed ) * f_Time / 2;		// �ڕW����
			}
			/* �ʒu���� */
			else{
				f_TrgtDist  = f_LastDist;													// �ڕW����
			}
			break;
			
		/* ������(���i cos�ߎ�) */
		case CTRL_ACC_SMOOTH:
			if( f_TrgtSpeed < f_LastSpeed ){												// �����ڕW�X�V���			
				f_TrgtSpeed = f_BaseSpeed + ((f_LastSpeed - f_BaseSpeed) / 2) * (1 - cos( (2 * f_Acc * f_Time) / (f_LastSpeed - f_BaseSpeed) ) );		// �ڕW���x
			}
			break;

		/* ������(���i cos�ߎ�) */
		case CTRL_CONST_SMOOTH:
			f_TrgtSpeed = f_BaseSpeed;														// �ڕW���x
			break;
		
		/* ������(���i cos�ߎ�) */
		case CTRL_DEC_SMOOTH:	
			/* ���x���� �{ �ʒu���� */
			if( f_TrgtSpeed > f_LastSpeed ){												// �����ڕW�X�V���
				f_TrgtSpeed = f_BaseSpeed + ( ( f_LastSpeed - f_BaseSpeed ) / 2) * (1 - cos( (2 * f_Acc * f_Time)/(f_LastSpeed - f_BaseSpeed ) ) );		// �ڕW���x
				f_TrgtDist  = f_BaseDist + ( (f_BaseSpeed - f_LastSpeed) / 2 ) * ( f_Time - ( (f_BaseSpeed - f_LastSpeed) / (2*f_Acc) ) * sin(((2*f_Acc)/(f_BaseSpeed - f_LastSpeed))*f_Time) ) ;		// �ڕW����
			}
			/* �ʒu���� */
			else{
				f_TrgtDist  = f_LastDist;													// �ڕW����
			}
			break;
			
		/* ������(���M�n����) */
		case CTRL_ACC_TURN:
			
			/*�����v���*/
			if((f_LastAngle > 0)&&(f_TrgtAngleS < f_LastAngleS)){
				f_TrgtAngleS = f_BaseAngleS + f_AccAngleS*f_Time;
			}
			/*���v���*/
			else if((f_LastAngle < 0)&&(f_TrgtAngleS > f_LastAngleS)){
				f_TrgtAngleS = f_BaseAngleS - f_AccAngleS*f_Time;
			}
			break;
		
		/* ������(���M�n����) */
		case CTRL_CONST_TURN:
			f_TrgtAngleS = f_BaseAngleS;
			break;
			
		/* ������(���M�n����) */
		case CTRL_DEC_TURN:
			
			/*�����v���*/
			if(f_LastAngle > 0){
				
				/* �p���x����{�p�x���� */
				if(f_TrgtAngleS > f_LastAngleS){								//�����ڕW�X�V���
					f_TrgtAngleS = f_BaseAngleS - f_AccAngleS * f_Time;					//�ڕW�p���x
					f_TrgtAngle  = f_BaseAngle  + (f_BaseAngleS + f_TrgtAngleS) * f_Time/2;			//�ڕW�p�x
				}
				/*�p�x����*/
				else{
					f_TrgtAngle = f_LastAngle;								//�ڕW�p�x
				}
			}
			
			/*���v���*/
			else{
				/*�p���x����{�p�x����*/
				if( f_TrgtAngleS < f_LastAngleS ){								//�����ڕW�X�V���
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;					//�ڕW�p���x
					f_TrgtAngle  = f_BaseAngle  + (f_BaseAngleS + f_TrgtAngleS) * f_Time / 2;		//�ڕW�p�x
				}
				/*�p�x����*/
				else{
					f_TrgtAngle = f_LastAngle;								//�ڕW�p�x
				}
			}

		/* �X�����[���O�̑O�i����(�X�����[��) */
		case CTRL_ENTRY_SLA:
			f_TrgtSpeed = f_BaseSpeed;
			if(f_TrgtDist <= f_LastDist){
				f_TrgtDist = f_BaseDist + f_TrgtSpeed * f_Time;		//�ڕW����
			}
			break;
		
		/* ������(�X�����[��) */
		case CTRL_ACC_SLA:
			f_TrgtSpeed = f_BaseSpeed;
			
			if( f_LastAngle > 0 ){
				/* �����v��� */
				if( f_TrgtAngleS < f_LastAngleS){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;				//�ڕW�p���x
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS +f_TrgtAngleS ) * f_Time / 2;	//�ڕW���x
				}else{
					f_TrgtAngle = f_LastAngle;
				}
			}else{
				/* ���v��� */
				if( f_TrgtAngleS > f_LastAngleS ){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;				//�ڕW�p���x
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS +f_TrgtAngleS ) * f_Time / 2;	//�ڕW���x
				}else{
					f_TrgtAngle = f_LastAngle;
				}
			}
			
			/* �ʒu���� */
			if( f_LastDist > f_TrgtDist ){
				f_TrgtDist = f_BaseDist + f_TrgtSpeed * f_Time;
			}else{
				f_TrgtDist = f_LastDist;
			}
			
			break;
		
		/* ������(�X�����[��) */
		case CTRL_CONST_SLA:
			f_TrgtSpeed = f_BaseSpeed;
			f_TrgtAngleS = f_BaseAngleS;		//�ڕW�p���x
			
			if( f_LastAngle > 0 ){
				/* �����v��� */
				if( f_TrgtAngle < f_LastAngle ){
					f_TrgtAngle = f_BaseAngle + f_TrgtAngleS * f_Time;		//�ڕW�p�x
				}else{
					f_TrgtAngle = f_LastAngle;					//�ڕW�p�x
				}
			}else{
				/* ���v��� */
				if( f_TrgtAngle > f_LastAngle ){
					f_TrgtAngle = f_BaseAngle + f_TrgtAngleS * f_Time;		//�ڕW�p�x
				}else{
					f_TrgtAngle = f_LastAngle;					//�ڕW�p�x
				}
			}
			
			/* �ʒu���� */
			if( f_LastDist > f_TrgtDist ){													// �ڕW�X�V���
				f_TrgtDist  = f_BaseDist + f_TrgtSpeed * f_Time;							// �ڕW�ʒu
			}
			else{
				f_TrgtDist  = f_LastDist;													// �ڕW����
			}
			break;
			
		/* ������(�X�����[��) */
		case CTRL_DEC_SLA:
			f_TrgtSpeed = f_BaseSpeed;
			
			if( f_LastAngle > 0 ){
				/* �����v��� */
				if( f_TrgtAngleS > f_LastAngleS){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;				//�ڕW�p���x
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS +f_TrgtAngleS ) * f_Time / 2;	//�ڕW���x
				}else{
					f_TrgtAngle = f_LastAngle;
				}
			}else{
				/* ���v��� */
				if( f_TrgtAngleS < f_LastAngleS ){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;				//�ڕW�p���x
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS +f_TrgtAngleS ) * f_Time / 2;	//�ڕW���x
				}else{
					f_TrgtAngle = f_LastAngle;
				}
			}
			
			/* ���x����{�ʒu���� */
			if( f_LastDist > f_TrgtDist){				//�ڕW�X�V���
				f_TrgtDist = f_BaseDist + f_TrgtSpeed * f_Time;	//�ڕW����
			}else{
				f_TrgtDist = f_LastDist;
			}
			break;
		
		/* �X�����[����̑O�i����(�X�����[��) */
		case CTRL_EXIT_SLA:
			f_TrgtSpeed 	= f_BaseSpeed;
			f_TrgtAngleS	= 0;
			if( f_TrgtDist <= f_LastDist ){
				f_TrgtDist = f_BaseDist + f_TrgtSpeed * f_Time;
			}else{
				f_TrgtDist = f_LastDist;
			}
			break;

		/* ��L�ȊO�̃R�}���h */
		default:
			break;
	}
	
}

// *************************************************************************
//   �@�\		�F �t�B�[�h�t�H���[�h�ʂ��擾����
//   ����		�F CTRL_pol����̂ݎ��s�\
//   ����		�F 1msec���Ɏ��s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �t�B�[�h�t�H���[�h��
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
//		v2.0		2019.9.15			TKR			�����C�����Ή�
// *************************************************************************/
PUBLIC void CTRL_getFF( FLOAT *p_err ){

    FLOAT   f_ff        = 0.0f;

    /* ���샂�[�h�ɉ����� */
    switch( en_Type ){
	
		/* ���� */
		case CTRL_ACC:
		case CTRL_HIT_WALL:
		case CTRL_ACC_SMOOTH:
		case CTRL_SKEW_ACC:
			f_ff = PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_speed_acc;	
			*p_err = f_Acc * f_ff;
			break;
			
		/* �����i���M�n����j*/
		case CTRL_ACC_TURN:
		case CTRL_ACC_SLA:
			f_ff = PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_angleS_acc;
			*p_err = f_AccAngleS * f_ff;
			break;

		/* ���� */
		case CTRL_CONST:
		case CTRL_CONST_SMOOTH:
		case CTRL_SKEW_CONST:
			f_ff = PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_speed;
			*p_err = f_TrgtSpeed * f_ff;
			break;
		
		/* �����i���M�n����j*/
		case CTRL_CONST_TURN:
		case CTRL_CONST_SLA:
			f_ff = PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_angleS;
			*p_err = f_TrgtAngleS * f_ff;
			break;

		/* ���� */
		case CTRL_DEC:
		case CTRL_DEC_SMOOTH:
		case CTRL_SKEW_DEC:
			f_ff = PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_speed_acc;
			*p_err = f_Acc * (-1) * f_ff;
			break;

		/* �����i���M�n����j*/
		case CTRL_DEC_TURN:
			f_ff = PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_angleS_acc;
			*p_err = f_AccAngleS* (-1) * f_ff;
			break;
	
		/* ���̑� */
		default:
			*p_err = 0;
			break;										// �������Ȃ�
	}

}

// *************************************************************************
//   �@�\		�F �t�B�[�h�t�H���[�h�ʂ��擾����i���i�����j
//   ����		�F CTRL_pol����̂ݎ��s�\
//   ����		�F 1msec���Ɏ��s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �t�B�[�h�t�H���[�h��
// **************************    ��    ��    *******************************
// 		v1.0		2019.10.11			TKR			�V�K
// *************************************************************************/
PUBLIC void CTRL_getFF_Speed( FLOAT *p_err ){

	FLOAT	f_ff_speed_acc		= 0.0f;
	FLOAT	f_ff_speed			= 0.0f;

	f_ff_speed_acc 	= PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_speed_acc;
	f_ff_speed		= PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_speed;

	/* ���샂�[�h�ɉ����� */
    switch( en_Type ){
	
		/* ���� */
		case CTRL_ACC:
		case CTRL_HIT_WALL:
		case CTRL_ACC_SMOOTH:
		case CTRL_SKEW_ACC:
		case CTRL_ACC_SLA:
			*p_err			= f_Acc * f_ff_speed_acc + f_TrgtSpeed * f_ff_speed;
			break;
		
		/* ���� */
		case CTRL_CONST:
		case CTRL_CONST_SMOOTH:
		case CTRL_SKEW_CONST:
		case CTRL_CONST_SLA:
		case CTRL_ENTRY_SLA:
		case CTRL_EXIT_SLA:
			*p_err			= f_TrgtSpeed * f_ff_speed;
			break;

		
		/* ���� */
		case CTRL_DEC:
		case CTRL_DEC_SMOOTH:
		case CTRL_SKEW_DEC:
		case CTRL_DEC_SLA:
			*p_err			= f_Acc * f_ff_speed_acc * (-1) + f_TrgtSpeed * f_ff_speed;
			break;

		default:
			*p_err			= 0;
			break;
	}

	
}

// *************************************************************************
//   �@�\		�F �t�B�[�h�t�H���[�h�ʂ��擾����i��]�����j
//   ����		�F CTRL_pol����̂ݎ��s�\
//   ����		�F 1msec���Ɏ��s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �t�B�[�h�t�H���[�h��
// **************************    ��    ��    *******************************
// 		v1.0		2019.10.11			TKR			�V�K
// *************************************************************************/
PUBLIC void CTRL_getFF_Angle( FLOAT *p_err ){

	FLOAT	f_ff_angleS_acc		= 0.0f;
	FLOAT	f_ff_angleS			= 0.0f;

	f_ff_angleS_acc 	= PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_angleS_acc;
	f_ff_angleS			= PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_angleS;

	/* ���샂�[�h�ɉ����� */
    switch( en_Type ){
	
		/* ���� */
		case CTRL_ACC_TURN:
		case CTRL_ACC_SLA:
			*p_err			= FABS(f_AccAngleS) * f_ff_angleS_acc + FABS(f_TrgtAngleS) * f_ff_angleS;
			break;
		
		/* ���� */
		case CTRL_CONST_TURN:
		case CTRL_CONST_SLA:
			*p_err			= FABS(f_TrgtAngleS) * f_ff_angleS;
			break;

		
		/* ���� */
		case CTRL_DEC_TURN:	
		case CTRL_DEC_SLA:
			*p_err			= FABS(f_AccAngleS) * f_ff_angleS_acc * (-1) + FABS(f_TrgtAngleS) * f_ff_angleS;
			break;

		default:
			*p_err			= 0;
			break;
	}

	
}


// *************************************************************************
//   �@�\		�F ���x�t�B�[�h�o�b�N�ʂ��擾����
//   ����		�F CTRL_pol����̂ݎ��s�\
//   ����		�F 1msec���Ɏ��s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F ���x�t�B�[�h�o�b�N�����
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
//		v2.0		2019.9.15			TKR			I����Ή�
// *************************************************************************/
PUBLIC void CTRL_getSpeedFB( FLOAT *p_err ){

    FLOAT	f_speedErr;		// [���x����] ���x�΍�
	FLOAT	f_kp = 0.0f;	// ���Q�C��
	FLOAT	f_ki = 0.0f;	// �ϕ��Q�C��

	f_kp = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_speed_kp;
	f_ki = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_speed_ki;

	/* ���x����(PI) */
	f_speedErr  	= f_TrgtSpeed - f_NowSpeed;			// ���x�΍�[mm/s]
	f_SpeedErrSum	+= f_speedErr * f_ki;			 
	
	if( f_SpeedErrSum > 10000 ){
		f_SpeedErrSum = 10000;
	}

	*p_err = f_SpeedErrSum + f_speedErr * f_kp;			// PI����ʎZ�o

}

// *************************************************************************
//   �@�\		�F �����t�B�[�h�o�b�N�ʂ��擾����
//   ����		�F CTRL_pol����̂ݎ��s�\
//   ����		�F 1msec���Ɏ��s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �����t�B�[�h�o�b�N�����
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PUBLIC void CTRL_getDistFB( FLOAT *p_err ){

    FLOAT       f_distErr;      // [��������]�����΍�
    FLOAT       f_kp    = 0;    // ���Q�C��
    FLOAT       f_ki    = 0;    // �ϕ��Q�C��

    *p_err = 0;
	
	f_kp = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_dist_kp;
	f_ki = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_dist_ki;
	
	/* �ʒu����(PI) */
	f_distErr = f_TrgtDist - f_NowDist;					// �����΍�[mm]
	f_DistErrSum += f_distErr * f_ki;					// I�����X�V
	
	if( f_DistErrSum > 10000 ){
		f_DistErrSum = 10000;
	}
	
	*p_err = f_distErr * f_kp + f_DistErrSum;		// PI����ʎZ�o
	
	// ��ł����Ƀt�F�[���Z�[�t��ǉ�
	
	/* �ݐϕ΍��N���A */
	if( FABS( f_TrgtDist - f_NowDist ) < 0.1 ){
		f_DistErrSum = 0;
	}
	
}

// *************************************************************************
//   �@�\		�F �p���x�t�B�[�h�o�b�N�ʂ��擾����
//   ����		�F CTRL_pol����̂ݎ��s�\
//   ����		�F 1msec���Ɏ��s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �p���x�t�B�[�h�o�b�N��
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
//		v2.0		2019.6.2			TKR			�p���x��GYRO_getNowAngleSpeed�֐�����擾
//		v3.0		2019.9.15			TKR			I����ǉ�
// *************************************************************************/
PUBLIC  void CTRL_getAngleSpeedFB( FLOAT *p_err ){

    FLOAT		f_err;			// [����] �W���C���Z���T�[�G���[ 
	FLOAT       f_kp    = 0;    // ���Q�C��
    FLOAT       f_ki    = 0;    // �ϕ��Q�C��
	
	f_kp = PARAM_getGain(Chg_ParamID(en_Type)) -> f_FB_angleS_kp;
	f_ki = PARAM_getGain(Chg_ParamID(en_Type)) -> f_FB_angleS_ki;
	
	/* �p���x����(PI) */
	f_err 				= f_TrgtAngleS - f_NowGyroAngleSpeed;		// �p���x�΍�[deg/s]
	f_AngleSpeedErrSum	+= f_err * f_ki;							// I�����X�V
	
	*p_err = f_err * f_kp + f_AngleSpeedErrSum;				// PI����ʎZ�o

}

// *************************************************************************
//   �@�\		�F �p�x�t�B�[�h�o�b�N�ʂ��擾����
//   ����		�F CTRL_pol����̂ݎ��s�\
//   ����		�F 1msec���Ɏ��s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �p�x�t�B�[�h�o�b�N�����
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PUBLIC  void    CTRL_getAngleFB( FLOAT *p_err ){

    FLOAT f_err;			// [����] �p�x�΍�[deg]
	FLOAT f_kp = 0.0f;		// ���Q�C��
	FLOAT f_ki = 0.0f;		// �ϕ��Q�C��
	
	*p_err = 0;
	
	f_NowAngle = f_NowGyroAngle;					// ���݊p�x[deg]

	f_err = f_TrgtAngle - f_NowAngle;
	
	/* ���i�� */
	if( ( en_Type == CTRL_ACC ) || ( en_Type == CTRL_CONST ) || ( en_Type == CTRL_DEC ) ||
		( en_Type == CTRL_ACC_SMOOTH ) || ( en_Type == CTRL_CONST_SMOOTH ) || ( en_Type == CTRL_DEC_SMOOTH ) ||
		( en_Type == CTRL_ENTRY_SLA ) || ( en_Type == CTRL_EXIT_SLA ) ||
		( en_Type == CTRL_DEC_TURN ) || ( en_Type == CTRL_ACC_SLA ) || ( en_Type == CTRL_CONST_SLA ) || ( en_Type == CTRL_DEC_SLA ) ||
		( en_Type == CTRL_SKEW_ACC ) || ( en_Type == CTRL_SKEW_CONST ) || ( en_Type == CTRL_SKEW_DEC )
	){
		
		f_kp = PARAM_getGain( Chg_ParamID(en_Type) ) -> f_FB_angle_kp;
		f_ki = PARAM_getGain( Chg_ParamID(en_Type) ) -> f_FB_angle_ki;
		
		f_AngleErrSum += f_err * f_ki;			// I�����X�V
								
		*p_err = f_err * f_kp + f_AngleErrSum;		// PI����ʎZ�o
		
		/* �ݐϕ΍��N���A */
		if( FABS( f_TrgtAngle - f_NowAngle ) < 0.1 ){
			f_AngleErrSum = 0;
		}
					
	}
}

// *************************************************************************
//   �@�\		�F �ǐ���̃t�B�[�h�o�b�N�ʂ��擾����
//   ����		�F CTRL_pol����̂ݎ��s�\
//   ����		�F 1msec���Ɏ��s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �ǐ���̃t�B�[�h�o�b�N��
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PUBLIC void CTRL_getSenFB( FLOAT *p_err ){

   	FLOAT f_err		= 0;
	FLOAT f_kp		= 0.0f;			// ���Q�C��
	FLOAT f_kd		= 0.0f;			// �����Q�C��
	
	/* ���i�� */
	if( ( en_Type == CTRL_ACC ) || ( en_Type == CTRL_CONST ) || ( en_Type == CTRL_DEC ) || 
		( en_Type == CTRL_ACC_SMOOTH ) || ( en_Type == CTRL_CONST_SMOOTH ) || ( en_Type == CTRL_DEC_SMOOTH ) || 
		( en_Type == CTRL_ENTRY_SLA ) || ( en_Type == CTRL_EXIT_SLA ) ){
		
		f_kp = PARAM_getGain( Chg_ParamID(en_Type) ) -> f_FB_wall_kp;
		f_kd = PARAM_getGain( Chg_ParamID(en_Type) ) -> f_FB_wall_kd;
						
		/* �΍��擾 */
		DIST_getErr( &l_WallErr );		
		f_err = (FLOAT)l_WallErr;
		
		*p_err = f_err * f_kp + ( f_err - f_ErrDistBuf ) * f_kd;		// PD����
		
		f_ErrDistBuf = f_err;			// �΍����o�b�t�@�����O
	}
	else if( ( en_Type == CTRL_SKEW_ACC ) || ( en_Type == CTRL_SKEW_CONST ) || ( en_Type == CTRL_SKEW_DEC ) ){
		
		// DIST_getErrSkew( &l_WallErr);
		f_err = (FLOAT)l_WallErr;
		
		*p_err = f_err * f_kp + ( f_err - f_ErrDistBuf ) * f_kd;		// PD����
		*p_err = f_err * f_kp;
	}
}

// *************************************************************************
//   �@�\		�F FF/FB����ʂ���DCM�ɏo�͂���
//   ����		�F CTRL_pol����̂ݎ��s�\
//   ����		�F 1msec���Ɏ��s�����
//   ����		�F �Eduty��C��duty��
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
// *************************************************************************/
PRIVATE void CTRL_outMot( FLOAT f_duty10_R, FLOAT f_duty10_L ){

    FLOAT   f_temp;     // �v�Z�p

    /* �d���ɉ�����PWM�o�͂�ύX���� */
    f_duty10_R  = f_duty10_R * VCC_MAX /( BAT_getLv() / 1000 );
    f_duty10_L  = f_duty10_L * VCC_MAX /( BAT_getLv() / 1000 );

    /* �E���[�^ */ 
    if( 20 < f_duty10_R ){									// �O�i
		DCM_setDirCcw( DCM_R );
		DCM_setPwmDuty( DCM_R, (USHORT)f_duty10_R );
	}
	else if( f_duty10_R < -20 ){							// ���
		f_temp = f_duty10_R * -1;
		DCM_setDirCw( DCM_R );
		DCM_setPwmDuty( DCM_R, (USHORT)f_temp );
	}
	else{
		DCM_brakeMot( DCM_R );								// �u���[�L
	}

	/* �����[�^ */
	if( 20 < f_duty10_L ){									// �O�i
		DCM_setDirCcw( DCM_L );
		DCM_setPwmDuty( DCM_L, (USHORT)f_duty10_L );
	}
	else if( f_duty10_L < -20 ){							// ���
		f_temp = f_duty10_L * -1;
		DCM_setDirCw( DCM_L );
		DCM_setPwmDuty( DCM_L, (USHORT)f_temp );
	}
	else{
		DCM_brakeMot( DCM_L );								// �u���[�L
	}
}

// *************************************************************************
//   �@�\		�F ����̃|�[�����O�֐�
//   ����		�F �Ȃ�
//   ����		�F ���荞�݂�����s�����B1msec���Ɋ��荞�ݏ������s���B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K
//		v2.0		2019.8.6			TKR			���O�@�\�ǉ�
// *************************************************************************/
PUBLIC void CTRL_pol( void ){

//  FLOAT f_feedFoard			= 0;		// [����] �t�B�[�h�t�H���[�h����
	FLOAT f_feedFoard_speed		= 0;		// [����] �t�B�[�h�t�H���[�h����i��������j
	FLOAT f_feedFoard_angle		= 0;		// [����] �t�B�[�h�t�H���[�h����i��]�����j
	FLOAT f_speedCtrl			= 0;		// [����] ���x�����
	FLOAT f_distCtrl			= 0;		// [����] ���������
	FLOAT f_angleSpeedCtrl		= 0;		// [����] �p���x�����
	FLOAT f_angleCtrl			= 0;		// [����] �p�x�����
	FLOAT f_distSenCtrl			= 0;		// [����] �����Z���T�[�����
	FLOAT f_duty10_R;						// [�o��] �E���[�^PWM-DUTY��[0.1%]
	FLOAT f_duty10_L;						// [�o��] �����[�^PWM-DUTY��[0.1%]
	static UCHAR uc_cycle		= 0;		// ���O�̋L�^�T�C�N��

	/* ������s�����̃`�F�b�N */
	if( uc_CtrlFlag != true ){
		return;		// ���䖳�����
	}
    
	/* �e��Z���T���� */
	ENC_GetDiv( &l_CntR, &l_CntL );					// �ړ���[�J�E���g�l]���擾
	CTRL_refNow();									// ����Ɏg�p����l�����݂̏�ԂɍX�V
	CTRL_refTarget();								// ����Ɏg�p����l��ڕW�l�ɍX�V

	/* ����l�擾 */
//	CTRL_getFF( &f_feedFoard );						// [����] �t�B�[�h�t�H���[�h
	CTRL_getFF_Speed( &f_feedFoard_speed );			// [����] �t�B�[�h�t�H���[�h����i��������j			
	CTRL_getFF_Angle( &f_feedFoard_angle );			// [����] �t�B�[�h�t�H���[�h����i��]�����j

	CTRL_getSpeedFB( &f_speedCtrl );				// [����] ���x
	CTRL_getDistFB( &f_distCtrl );					// [����] ����

	CTRL_getAngleSpeedFB( &f_angleSpeedCtrl );		// [����] �p���x
	CTRL_getAngleFB( &f_angleCtrl );				// [����] �p�x
	CTRL_getSenFB( &f_distSenCtrl );				// [����] ��

#if 1
	/* ���s���O */
	if(us_LogPt != CTRL_LOG){
		if( bl_log == true ){
			uc_cycle++;
		}

		/* ���O�L�^ */
		if( uc_cycle == CTRL_LOG_CYCLE ){		// ���̎����ŋL�^
			uc_cycle			= 0;
			st_Log[us_LogPt].f_time			= f_Time;					// ����
			st_Log[us_LogPt].f_trgtSpeed	= f_TrgtSpeed;				// ���x�i�ڕW�l�j
			st_Log[us_LogPt].f_nowSpeed		= f_NowSpeed;				// ���x�i�����l�j
			st_Log[us_LogPt].f_trgtPos		= f_TrgtDist;				// �ʒu�i�ڕW�l�j
			st_Log[us_LogPt].f_nowPos		= f_NowDist;				// �ʒu�i�����l�j
			st_Log[us_LogPt].f_trgtAngleS	= f_TrgtAngleS;				// �p���x�i�ڕW�l�j
			st_Log[us_LogPt].f_nowAngleS	= f_NowGyroAngleSpeed;		// �p���x�i�����l�j
			st_Log[us_LogPt].f_trgtAngle	= f_TrgtAngle;				// �p�x�i�ڕW�l�j
			st_Log[us_LogPt].f_nowAngle		= f_NowAngle;				// �p�x�i�����l�j
			st_Log[us_LogPt].f_nowAccel		= f_NowAccel;				// �����x�i�����l�j
			us_LogPt++;
			if(us_LogPt== CTRL_LOG) bl_log = false;
		}
	}
#endif

	/* ���i���� */
	if( ( en_Type == CTRL_ACC ) || ( en_Type == CTRL_CONST ) || ( en_Type == CTRL_DEC ) || 
		( en_Type == CTRL_ACC_SMOOTH ) || ( en_Type == CTRL_CONST_SMOOTH ) || ( en_Type == CTRL_DEC_SMOOTH ) || 
		( en_Type == CTRL_ENTRY_SLA ) || ( en_Type == CTRL_EXIT_SLA) ||
		( en_Type == CTRL_SKEW_ACC ) || ( en_Type == CTRL_SKEW_CONST ) || ( en_Type == CTRL_SKEW_DEC )
		){
//		f_duty10_R = f_feedFoard * FF_BALANCE_R +  f_distCtrl + f_speedCtrl + f_angleCtrl + f_angleSpeedCtrl + f_distSenCtrl;	// �E���[�^PWM-DUTY��[0.1%]
//		f_duty10_L = f_feedFoard * FF_BALANCE_L +  f_distCtrl + f_speedCtrl - f_angleCtrl - f_angleSpeedCtrl - f_distSenCtrl;	// �����[�^PWM-DUTY��[0.1%]
		f_duty10_R = f_feedFoard_speed * FF_BALANCE_R +  f_distCtrl + f_speedCtrl + f_angleCtrl + f_angleSpeedCtrl + f_distSenCtrl;	// �E���[�^PWM-DUTY��[0.1%]
		f_duty10_L = f_feedFoard_speed * FF_BALANCE_L +  f_distCtrl + f_speedCtrl - f_angleCtrl - f_angleSpeedCtrl - f_distSenCtrl;	// �����[�^PWM-DUTY��[0.1%]

	}
	
	/* �ǂ��Đ��� */
	else if( en_Type == CTRL_HIT_WALL){
		f_duty10_R = f_feedFoard_speed * FF_HIT_BALANCE_R * (-1);
		f_duty10_L = f_feedFoard_speed * FF_HIT_BALANCE_L * (-1);
	}
	
	/* �X�����[������ */
	else if( ( en_Type == CTRL_ACC_SLA ) || ( en_Type == CTRL_CONST_SLA ) || ( en_Type == CTRL_DEC_SLA ) ){
		if(f_LastAngle > 0){
//			f_duty10_R = f_feedFoard * FF_BALANCE_R		+ f_distCtrl + f_speedCtrl + f_angleCtrl + f_angleSpeedCtrl;		//�E���[�^PWM-DUTY��[0.1%]
//			f_duty10_L = f_feedFoard * FF_BALANCE_L*(-1)+ f_distCtrl + f_speedCtrl - f_angleCtrl - f_angleSpeedCtrl;		//�����[�^PWM-DUTY��[0.1%]
			f_duty10_R = f_feedFoard_speed * FF_BALANCE_R + f_feedFoard_angle * FF_BALANCE_R	+ f_distCtrl + f_speedCtrl + f_angleCtrl + f_angleSpeedCtrl;		//�E���[�^PWM-DUTY��[0.1%]
			f_duty10_L = f_feedFoard_speed * FF_BALANCE_L + f_feedFoard_angle * FF_BALANCE_L*(-1)		+ f_distCtrl + f_speedCtrl - f_angleCtrl - f_angleSpeedCtrl;		//�����[�^PWM-DUTY��[0.1%]
		}else{
//			f_duty10_R = f_feedFoard * FF_BALANCE_R*(-1)+ f_distCtrl + f_speedCtrl + f_angleCtrl + f_angleSpeedCtrl;		//�E���[�^PWM-DUTY��[0.1%]
//			f_duty10_L = f_feedFoard * FF_BALANCE_L		+ f_distCtrl + f_speedCtrl - f_angleCtrl - f_angleSpeedCtrl;		//�����[�^PWM-DUTY��[0.1%]
			f_duty10_R = f_feedFoard_speed * FF_BALANCE_R + f_feedFoard_angle * FF_BALANCE_R*(-1)		+ f_distCtrl + f_speedCtrl + f_angleCtrl + f_angleSpeedCtrl;		//�E���[�^PWM-DUTY��[0.1%]
			f_duty10_L = f_feedFoard_speed * FF_BALANCE_L + f_feedFoard_angle * FF_BALANCE_L	+ f_distCtrl + f_speedCtrl - f_angleCtrl - f_angleSpeedCtrl;		//�����[�^PWM-DUTY��[0.1%]
		}
	}
	
	/*���M�n����*/
	else{
	
		/*������*/
		if(f_LastAngle > 0){
//			f_duty10_R = f_feedFoard * FF_BALANCE_R	     + f_angleCtrl + f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
//			f_duty10_L = f_feedFoard * FF_BALANCE_L*(-1) - f_angleCtrl - f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
			f_duty10_R = f_feedFoard_angle * FF_BALANCE_R	   + f_angleCtrl + f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
			f_duty10_L = f_feedFoard_angle * FF_BALANCE_L*(-1) - f_angleCtrl - f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
		}
		else{
		/*�E����*/
//			f_duty10_R = f_feedFoard * FF_BALANCE_R*(-1) + f_angleCtrl + f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
//			f_duty10_L = f_feedFoard * FF_BALANCE_L	     - f_angleCtrl - f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
			f_duty10_R = f_feedFoard_angle * FF_BALANCE_R*(-1) 	+ f_angleCtrl + f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
			f_duty10_L = f_feedFoard_angle * FF_BALANCE_L		- f_angleCtrl - f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
		}
	}

	CTRL_outMot( f_duty10_R, f_duty10_L );				// ���[�^�֏o��
    		
	f_Time += 0.001;
	
#if 0	
	/* �ǐ؂�`�F�b�N */
	if( MOT_getWallEdgeType() == MOT_WALL_EDGE_RIGHT ){
		
		/* �ǔ��� */
		if( DIST_isWall_R_SIDE() == false){	
			MOT_setWallEdge( true );	//�ǂ̐؂�ڂ����m
		}
		
	}else if( MOT_getWallEdgeType() == MOT_WALL_EDGE_LEFT ){
		
		/* �ǔ��� */
		if( DIST_isWall_L_SIDE() == false){	
			MOT_setWallEdge( true );
		}
	}else{

		/* �������Ȃ� */
	
    }
#endif
}

// *************************************************************************
//   �@�\		�F ���O�̏o��
//   ����		�F �Ȃ�
//   ����		�F TeraTerm�ɏo�͂��Ccsv�ɂ��ĕۑ����CMATLAB�ɏo��
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.8.6			TKR			�V�K
// *************************************************************************/
PUBLIC void CTRL_showLog( void ){
	
	USHORT	i;
	
	printf("index,TrgtSpeed[mm/s],NowSpeed[mm/s],TrgtPos[mm],NowPos[mm],TrgtAngleS[deg/s],NowAngleS[deg/s],TrgtAngle[deg],NowAngle[deg],NowAccel[m/s^2]\n\r");
	for(i=0; i<CTRL_LOG; i++){
		printf("%4d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n\r",
				i,st_Log[i].f_trgtSpeed,st_Log[i].f_nowSpeed,st_Log[i].f_trgtPos,st_Log[i].f_nowPos,st_Log[i].f_trgtAngleS,st_Log[i].f_nowAngleS,st_Log[i].f_trgtAngle,st_Log[i].f_nowAngle,st_Log[i].f_nowAccel);
	}

}

// *************************************************************************
//   �@�\		�F ���O�̕ϐ��̏�����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.8.6			TKR			�V�K
// *************************************************************************/
PUBLIC void CTRL_Loginit( void ){

	memset( st_Log, 0, sizeof(st_Log) );
}

// *************************************************************************
//   �@�\		�F ���O�̋���
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.8.12			TKR			�V�K
// *************************************************************************/
PUBLIC void CTRL_LogSta( void ){
	
	bl_log	= true;

}