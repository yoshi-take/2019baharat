// *************************************************************************
//   ���{�b�g��	�F Baharat�i�o�n���b�g�j
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F GYRO
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.14			TKR			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/
//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <common_define.h>					// ���ʒ�`
#include <iodefine.h>						// I/O
#include <stdio.h>							// �W�����o��
#include <math.h>						// ���l�v�Z

#include <hal_gyro.h>                       // GYRO
#include <hal_spi.h>						// SPI
#include <parameter.h>						// parameter
#include <hal_led.h>						// LED

//**************************************************
// ��`�idefine�j
//**************************************************

//**************************************************
// �񋓑́ienum�j
//**************************************************

//**************************************************
// �\���́istruct�j
//**************************************************

//**************************************************
// �O���[�o���ϐ�
//**************************************************
/* �W���C���Z���T */
PUBLIC SHORT s_GyroVal_Lo;								// �W���C���Z���T�l(����)
PUBLIC SHORT s_GyroVal_Hi;								// �W���C���Z���T�l(���)
PUBLIC volatile FLOAT  f_NowGyroAngle;		 					// �W���C���Z���T�̌��݊p�x
PUBLIC volatile FLOAT  f_NowGyroAngleSpeed;						// �W���C���Z���T�̌��݊p���x	

PUBLIC SHORT 	s_AccelVal_Lo;							// �����x�Z���T�l(����)
PUBLIC SHORT 	s_AccelVal_Hi;							// �����x�Z���T�l(���)
PUBLIC FLOAT  	f_NowAccel;								// �i�s�����̌��݉����x	


PUBLIC	SHORT s_WhoamiVal;


extern PUBLIC	enSPI_STATE		en_SpiState;	// SPI�ʐM���
extern PUBLIC	SHORT*			p_SpiRcvData;	// SPI��M�f�[�^�i�[�A�h���X
extern PUBLIC	FUNC_PTR		p_SpiCallBackFunc;	// SPI��IDLE�J�ڊ��荞�ݎ��ɓo�^�����ƌĂяo�����֐�


//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
extern PUBLIC void TIME_wait(ULONG ul_time);

// *************************************************************************
//   �@�\		�F �W���C���̌��݂̊p�x���擾����
//   ����		�F �Ȃ�
//   ����		�F ��
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.14			TKR			�V�K
// *************************************************************************/
PUBLIC FLOAT GYRO_getNowAngle( void )
{
	return f_NowGyroAngle;
}

// *************************************************************************
//   �@�\		�F �W���C���̌��݂̊p���x���擾����
//   ����		�F �Ȃ�
//   ����		�F ��
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.14			TKR			�V�K
// *************************************************************************/
PUBLIC FLOAT GYRO_getNowAngleSpeed( void )
{
	return f_NowGyroAngleSpeed;
}

// *************************************************************************
//   �@�\		�F �W���C���̗ݐϊp�x�����Z�b�g����
//   ����		�F �Ȃ�
//   ����		�F ��
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.14			TKR			�V�K
//		v2.0		2019.6.2			TKR			�p�x�Ɗp���x�̕ϐ��ύX
// *************************************************************************/
PUBLIC void GYRO_clrAngle( void )
{
	f_NowGyroAngle 		= 0;
	f_NowGyroAngleSpeed	= 0;
}

// *************************************************************************
//   �@�\		�F �W���C���Z���T�p�|�[�����O�֐�
//   ����		�F �����n���h���������I�ɌĂяo��
//   ����		�F ���荞�݂�����s�����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.14			TKR			�V�K
//		v2.0		2019.6.2			TKR			�ړ����ς��폜(�O��)
// *************************************************************************/
PUBLIC void GYRO_Pol( void )
{
	f_NowGyroAngle += f_NowGyroAngleSpeed / 1000;		// �p�x�X�V   (0.001sec���ɉ��Z���邽��)
}

// *************************************************************************
//   �@�\		�F �N�̖��́H
//   ����		�F �R�[���o�b�N�֐��͎g��Ȃ�
//   ����		�F �W���C���̃f�o�b�O�p
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.6.2			TKR			�V�K
// *************************************************************************/
PUBLIC	void GYRO_get_WHOAMI( void ){
	
	p_SpiRcvData		= &s_WhoamiVal;		// ��ʃf�[�^
	SPI_staGetData(SPI_WHO_AM_I);			// ��M�������s

	printf("Who am I = 0x%x\n\r",s_WhoamiVal);
}

// *************************************************************************
//   �@�\		�F �W���C�������ݒ�
//   ����		�F SPI_init���s��Ɏ��s
//   ����		�F ������s
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.2			TKR			�V�K
//		v2.0		2019.8.24			TKR			�W���C���̃��W�X�^�`�F�b�N�ǉ�
// *************************************************************************/
PUBLIC void GYRO_init( void ){
	
	SHORT	us_dummy;
#if	0
	/* �p���[�}�l�W�����g1[No.107] */		 
	SPI_staSetData(SPI_PWR_MGMT_1,0x01);		 
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
	}

	/* �M�����Z�b�g[No.104] */
	/*
	SPI_staSetData(SPI_SIGNAL_RESET,0x03);		// �����x�E���x���Z�b�g
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
	}
	*/

	/* �R���t�B�O[No.26] */
	SPI_staSetData(SPI_CONFIG,0x00);
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
	}
	
	/* FSYNC�R���t�B�O[No.54] */
	p_SpiRcvData		= &us_dummy;		// �_�~�[
	SPI_staGetData(SPI_FSYNC_INT);			// �ǂݏo���p���W�X�^
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
	}
	
	/* INT_PIN�R���t�B�O[No.55] */
	SPI_staSetData(SPI_INT_PIN_COMFIG,0xe8);
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
	}
	

	/* SPI�L��[No.106] */
	SPI_staSetData(SPI_USER_CONTROL,0x01);		// �W���C���E�����x�E���x���Z�b�g
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
	}
	
	/* SPI�L��[No.112] */
	SPI_staSetData(SPI_I2C_IF,0x40);			// SPI�L��(I2C��؂�)
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
	}

	/* ���x(�W���C��)[No.27] */
	SPI_staSetData(SPI_GYRO_CFG,0x18);			// �}2000[dps]
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
	}

	/* �W���C���̃I�t�Z�b�g����[No.24] */
	/*
	SPI_staSetData(SPI_GYRO_OFFSET_L,0x18);			// �I�t�Z�b�g�ʂ���������
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
	}
	*/
#endif

	/* �p���[�}�l�W�����g1[No.107] */		 
	while(1){
		// �ݒ�̏�������
		SPI_staSetData(SPI_PWR_MGMT_1,0x01);		 
		TIME_wait(100);
		while(1){
			if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
		}

		// ���W�X�^�`�F�b�N
		p_SpiRcvData		= &us_dummy;		// �_�~�[
		SPI_staGetData(SPI_PWR_MGMT_1);			// �ǂݏo��
		TIME_wait(100);
		printf("SPI_PWR_MGMT_1:0x%x\n\r",us_dummy);
		if( us_dummy == 0x01 ){
			printf("SPI_PWR_MGMT_1:success\n\r");
			break;			// ���]�̐ݒ肪�������߂Ă�����OK
		}else{
			printf("SPI_PWR_MGMT_1:failure\n\r");
		}	
	}
	
	/* �R���t�B�O[No.26] */
	while(1){
		// �ݒ�̏�������
		SPI_staSetData(SPI_CONFIG,0x00);
		TIME_wait(100);
		while(1){
			if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
		}

		// ���W�X�^�`�F�b�N
		p_SpiRcvData		= &us_dummy;		// �_�~�[
		SPI_staGetData(SPI_CONFIG);				// �ǂݏo��
		TIME_wait(100);
		printf("SPI_CONFIG:0x%x\n\r",us_dummy);
		if( us_dummy == 0x00 ){
			printf("SPI_CONFIG:success\n\r");
			break;			// ���]�̐ݒ肪�������߂Ă�����OK			
		}else{
			printf("SPI_CONFIG:failure\n\r");
		}
	}

	/* FSYNC�R���t�B�O[No.54] */
	while(1){
		p_SpiRcvData		= &us_dummy;		// �_�~�[
		SPI_staGetData(SPI_FSYNC_INT);			// �ǂݏo���p���W�X�^
		TIME_wait(100);
		printf("SPI_FSYNC_INT:0x%x\n\r",us_dummy);
		while(1){
			if( en_SpiState == SPI_IDLE )break;	// SPI�ʐM����
		}
		if( (us_dummy>>7) == 0 ){
			printf("SPI_FSYNC_INT:success\n\r");
			break;			// �ŏ�ʃr�b�g��0�ɂȂ��Ă��邱�Ƃ��m�F
		}else{
			printf("SPI_FSYNC_INT:failure\n\r");
		}
	}

	/* INT_PIN�R���t�B�O[No.55] */
	while(1){
		// �ݒ�̏�������
		SPI_staSetData(SPI_INT_PIN_COMFIG,0xe8);
		TIME_wait(100);
		while(1){
			if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
		}

		// ���W�X�^�`�F�b�N
		p_SpiRcvData		= &us_dummy;		// �_�~�[
		SPI_staGetData(SPI_INT_PIN_COMFIG);		// �ǂݏo���p���W�X�^
		TIME_wait(100);
		printf("SPI_INT_PIN_COMFIG:0x%x\n\r",us_dummy);		
		if( us_dummy == 0xe8 ){
			printf("SPI_INT_PIN_COMFIG:success\n\r");
			break;			// ���]�̐ݒ肪�������߂Ă�����OK
		}else{
			printf("SPI_INT_PIN_COMFIG:failure\n\r");
		}			
	}

	/* SPI�L��[No.106] */	
	// �ݒ�̏�������
	SPI_staSetData(SPI_USER_CONTROL,0x01);		// �W���C���E�����x�E���x���Z�b�g
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
	}
	// ���W�X�^�`�F�b�N
	p_SpiRcvData		= &us_dummy;		// �_�~�[
	SPI_staGetData(SPI_USER_CONTROL);		// �ǂݏo���p���W�X�^
	TIME_wait(100);
	printf("SPI_USER_CONTROL:0x%x\n\r",us_dummy);
	
	/* SPI�L��[No.112] */
	while(1){
		// �ݒ�̏�������
		SPI_staSetData(SPI_I2C_IF,0x40);			// SPI�L��(I2C��؂�)
		TIME_wait(100);
		while(1){
			if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
		}

		// ���W�X�^�`�F�b�N
		p_SpiRcvData		= &us_dummy;		// �_�~�[
		SPI_staGetData(SPI_I2C_IF);				// �ǂݏo���p���W�X�^
		TIME_wait(100);
		printf("SPI_I2C_IF:0x%x\n\r",us_dummy);
		if( us_dummy == 0x40 ){
			printf("SPI_I2C_IF:success\n\r");
			break;			// ���]�̐ݒ肪�������߂Ă�����OK			
		}else{
			printf("SPI_I2C_IF:failure\n\r");
		}
	}

	/* ���x(�W���C��)[No.27] */
	while(1){	
		// �ݒ�̏�������
		SPI_staSetData(SPI_GYRO_CFG,0x18);			// �}2000[dps]
		TIME_wait(100);
		while(1){
			if( en_SpiState == SPI_IDLE )break;		// SPI�ʐM����
		}

		// ���W�X�^�`�F�b�N
		p_SpiRcvData		= &us_dummy;		// �_�~�[
		SPI_staGetData(SPI_GYRO_CFG);			// �ǂݏo���p���W�X�^
		TIME_wait(100);
		printf("SPI_GYRO_CFG:0x%x\n\r",us_dummy);
		if( us_dummy == 0x18 ){
			printf("SPI_GYRO_CFG:success\n\r");
			break;			// ���]�̐ݒ肪�������߂Ă�����OK
		}else{
			printf("SPI_GYRO_CFG:failure\n\r");
		}				
	}

	while( 0x12 != s_WhoamiVal ){
		printf("failure\n\r");
		GYRO_get_WHOAMI();
	}
	if( 0x12 == s_WhoamiVal )printf("success\n\r");

}

// *************************************************************************
//   �@�\		�F �W���C���Z���T�l�擾�p�֐�(2/2)
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.6.2			TKR			�V�K
// *************************************************************************/
PRIVATE void GYRO_getVal_2nd( void ){

	SHORT	s_count;			// ICM20602���瓾��ꂽ�p���x(�J�E���g�l)
	FLOAT	f_tempAngleSpeed;	// �p���x[dps]

	/* ������ */
	p_SpiRcvData		= NULL;
	p_SpiCallBackFunc	= NULL;

	/* �p���x�l */
	s_count				= (SHORT)(s_GyroVal_Lo | (s_GyroVal_Hi << 8) );		// �f�[�^����
	f_tempAngleSpeed	= (FLOAT)s_count / GYRO_SCALE_FACTOR;				// [�J�E���g]��[dps]�ɕϊ�

	/* SW�t�B���^��L���ɂ���(��ŏ���) */ 
	if((SW_FILTER_VAL_MIN<f_tempAngleSpeed)&&(f_tempAngleSpeed<SW_FILTER_VAL_MAX)){
		f_tempAngleSpeed	= 0;
	}
	
	/* �p���x�X�V */
	f_NowGyroAngleSpeed		= f_tempAngleSpeed;

}

// *************************************************************************
//   �@�\		�F �W���C���Z���T�l�擾�p�֐�(1/2)
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.6.2			TKR			�V�K
// *************************************************************************/
PRIVATE void GYRO_getVal_1st( void ){

	p_SpiRcvData		= &s_GyroVal_Hi;		// ��ʃf�[�^
	p_SpiCallBackFunc	= GYRO_getVal_2nd;
	SPI_staGetData(SPI_GYRO_Z_H);

}

// *************************************************************************
//   �@�\		�F �W���C���Z���T�̒l���擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.6.2			TKR			�V�K
// *************************************************************************/
PUBLIC void GYRO_getVal( void ){

	p_SpiRcvData		= &s_GyroVal_Lo;		// ��M�f�[�^�A�h���X�o�^(���ʃf�[�^)
	p_SpiCallBackFunc	= GYRO_getVal_1st;		// �R�[���o�b�N�֐��o�^
	SPI_staGetData(SPI_GYRO_Z_L);				// ��M�������s

}

// *************************************************************************
//   �@�\		�F �����x�Z���T�̒l���擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.7.30			TKR			�V�K
// *************************************************************************/
PUBLIC void GYRO_getAccVal( void ){

	p_SpiRcvData		= &s_AccelVal_Lo;		// ��M�f�[�^�A�h���X�o�^(���ʃf�[�^)
	p_SpiCallBackFunc	= GYRO_getAccVal_1st;		// �R�[���o�b�N�֐��o�^
	SPI_staGetData(SPI_ACC_L);				// ��M�������s

}

// *************************************************************************
//   �@�\		�F �����x�Z���T�l�擾�p�֐�(2/2)
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.7.30			TKR			�V�K
// *************************************************************************/
PRIVATE void GYRO_getAccVal_2nd( void ){

	SHORT	s_count;			// ICM20602���瓾��ꂽ�����x(�J�E���g�l)
	FLOAT	f_tempAcc;			// �����x[]

	/* ������ */
	p_SpiRcvData		= NULL;
	p_SpiCallBackFunc	= NULL;

	/* �p���x�l */
	s_count				= (SHORT)(s_AccelVal_Lo | (s_AccelVal_Hi << 8) );		// �f�[�^����
	f_tempAcc			= (FLOAT)s_count / ACC_SCALE_FACTOR;				// [�J�E���g]��[]�ɕϊ�

	/* SW�t�B���^��L���ɂ���(臒l���܂��W���C���d�l�ɂȂ��Ă���̂Ō�ŏC��) */ 
	if((SW_FILTER_VAL_MIN<f_tempAcc)&&(f_tempAcc<SW_FILTER_VAL_MAX)){
		f_tempAcc	= 0;
	}
	
	/* �����x�X�V */
	f_NowAccel		= f_tempAcc;

}

// *************************************************************************
//   �@�\		�F �����x�Z���T�l�擾�p�֐�(1/2)
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.7.30			TKR			�V�K
// *************************************************************************/
PRIVATE void GYRO_getAccVal_1st( void ){

	p_SpiRcvData		= &s_AccelVal_Hi;		// ��ʃf�[�^
	p_SpiCallBackFunc	= GYRO_getAccVal_2nd;
	SPI_staGetData(SPI_ACC_H);

}