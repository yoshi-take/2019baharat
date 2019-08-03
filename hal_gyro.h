// *************************************************************************
//   ���{�b�g��	�F Baharat�i�o�n���b�g�j
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F ENC
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.11			TKR			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/
// ���d�R���p�C���h�~
#ifndef	_HAL_GYRO_H
#define	_HAL_GYRO_H

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O
#include <stdio.h>							// �W�����o��

//**************************************************
// ��`�idefine�j
//**************************************************
#define     SPI_WHO_AM_I        ( 0x7500 )      // Who am I?

#define		SPI_PWR_MGMT_1		( 0x6b00 )		// [No.107]�d������1
#define		SPI_CONFIG			( 0x1a00 )		// [No.26]�R���t�B�O

#define		SPI_FSYNC_INT		( 0x3600 )		// [No.54]FSYNC�R���t�B�O
#define		SPI_INT_PIN_COMFIG	( 0x3700 )		// [No.55]INT_PIN�R���t�B�O

#define		SPI_SIGNAL_RESET	( 0x6800 )		// [No.104]�M�����Z�b�g
#define		SPI_USER_CONTROL	( 0x6a00 )		// [No.106]���[�U�[�R���g���[��
#define     SPI_I2C_IF          ( 0x7000 )      // [No.112]I2C�̖���
#define		SPI_GYRO_CFG		( 0x1b00 )		// [No.27]�W���C���Z���T�̃R���t�B�O
#define		SPI_ACC_CFG			( 0x1c00 )		// [No.28]�����x�Z���T�̃R���t�B�O
#define		SPI_GYRO_OFFSET_L	( 0x1800 )	    // [No.24]�W���C���I�t�Z�b�g�i���ʁj
#define		SPI_ACC_OFFSET_L	( 0x1600 )		// [No.20]�����x�I�t�Z�b�g�i���ʁj
#define		SPI_SEN_ENB			( 0x2300 )	    // �Z���T�L��

#define		SPI_GYRO_Z_L		( 0x4800 )		// �W���C���Z���T�̉��ʃf�[�^
#define		SPI_GYRO_Z_H		( 0x4700 )		// �W���C���Z���T�̏�ʃf�[�^
#define		SPI_TEMP_L			( 0x4200 )		// ���x�Z���T�̉��ʃf�[�^
#define		SPI_TEMP_H			( 0x4100 )		// ���x�Z���T�̏�ʃf�[�^
#define		SPI_ACC_L			( 0x3e00 )		// �����x�Z���T�̉��ʃf�[�^
#define		SPI_ACC_H			( 0x3d00 )		// �����x�Z���T�̏�ʃf�[�^

//**************************************************
// �񋓑́ienum�j
//**************************************************

//**************************************************
// �\���́istruct�j
//**************************************************

//**************************************************
// �O���[�o���ϐ�
//**************************************************

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
PUBLIC FLOAT GYRO_getNowAngle( void );
PUBLIC FLOAT GYRO_getNowAngleSpeed( void );
PUBLIC void GYRO_clrAngle( void );
PUBLIC void GYRO_Pol( void );
PUBLIC	void GYRO_get_WHOAMI( void );
PUBLIC void GYRO_init( void );
PRIVATE void GYRO_getVal_2nd( void );
PRIVATE void GYRO_getVal_1st( void );
PUBLIC void GYRO_getVal(void);
PRIVATE void GYRO_getAccVal_2nd( void );
PRIVATE void GYRO_getAccVal_1st( void );
PUBLIC void GYRO_getAccVal( void );

#endif
