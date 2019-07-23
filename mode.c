// *************************************************************************
//   ���{�b�g��	�F Baharat�i�o�n���b�g�j
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F MODE
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.2.5			TKR			�V�K
// *************************************************************************/

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <iodefine.h>		// I/O
#include <typedefine.h>		// ��`
#include <stdio.h>			// �W�����C�u����

#include <mode.h>			// MODE
#include <hal_led.h>		// LED
#include <hal_spk.h>		// SPK
#include <hal_battery.h>	// �o�b�e��
#include <hal_sci.h>		// SCI
#include <hal_spi.h>		// SPI
#include <hal_gyro.h>		// �W���C��
#include <hal_dcm.h>		// DCM
#include <hal_enc.h>		// ENC
#include <hal_dist.h>       // DIST
#include <motion.h>			// motion

#include <parameter.h>		// parameter

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
PRIVATE enMODE		en_Mode;		// ���݂̃��[�h	

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************

// *************************************************************************
//   �@�\		�F ���[�h�����s����B
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
//		v1.0		2019.2.5			TKR				�V�K
// *************************************************************************/
PUBLIC void	MODE_exe( void ){
	
	switch( en_Mode ){
		
		case MODE_0:
			LED_offAll();	
			BAT_Check();
			break;
			
		case MODE_1:
			LED_offAll();
			while(1){
				//printf("AngleSpeed:%f[deg]\r",GYRO_getNowAngleSpeed());
				printf("offset:[%x],GYRO_getNowAngleSpeed()*GYRO_SCALE_FACTOR");		// �I�t�Z�b�g��
				TIME_wait(100);
			}
			break;
			
		case MODE_2:
			LED_offAll();
			DCM_setDirCcw(DCM_L);
			DCM_setDirCw(DCM_R);
			DCM_setPwmDuty(DCM_L,100);
			//DCM_setPwmDuty(DCM_SUC,100);
			DCM_staMot(DCM_L);
			
			
			break;
			
		case MODE_3:
			LED_offAll();
			
			DIST_Check();		// �����Z���T�f�o�b�O
			
			break;
			
		case MODE_4:
			GYRO_get_WHOAMI();
			break;
			
		case MODE_5:
			LED_offAll();

			break;
			
		case MODE_6:
			LED_offAll();
			TIME_wait(2000);
			MOT_setTrgtSpeed( 400.0f );
			MOT_goBlock_FinSpeed(2,0);
			break;
			
		case MODE_7:
			LED_offAll();		
			break;
			
		case MODE_8:
			LED_offAll();
			break;
			
		case MODE_9:
			LED_offAll();
			break;
			
		default:
			break;
			
	}
}

// *************************************************************************
//   �@�\		�F �\�񂳂ꂽ���[�h�ɕύX����B
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
//		v1.0		2019.2.5			TKR				�V�K
// *************************************************************************/
PRIVATE void MODE_chg( enMODE en_mode ){
	
	LED_offAll();	
	
	switch( en_mode ){
		
		case MODE_0:
			//SPK_on(A4,16.0f,120);
			break;
			
		case MODE_1:
			//SPK_on(A4,16.0f,120);
			LED_on(LED0);
			break;
			
		case MODE_2:
			//SPK_on(A4,16.0f,120);
			LED_on(LED1);
			break;
			
		case MODE_3:
			//SPK_on(A4,16.0f,120);
			LED_on(LED2);
			break;
			
		case MODE_4:
			//SPK_on(A4,16.0f,120);
			LED_on(LED3);
			break;
			
		case MODE_5:
			//SPK_on(A4,16.0f,120);
			LED_on(LED4);
			break;
			
		case MODE_6:
			//SPK_on(A4,16.0f,120);
			LED_on(LED0);
			LED_on(LED1);
			break;
			
		case MODE_7:
			//SPK_on(A4,16.0f,120);
			LED_on(LED0);
			LED_on(LED2);
			break;
			
		case MODE_8:
			//SPK_on(A4,16.0f,120);
			LED_on(LED0);
			LED_on(LED3);
			break;
			
		case MODE_9:
			//SPK_on(A4,16.0f,120);
			LED_on(LED0);
			LED_on(LED4);
			break;
			
		default:
			break;
			
	}
	
	en_Mode = en_mode;	// ���݂̃��[�h			
}

// *************************************************************************
//   �@�\		�F ���[�h�����Z�ύX����B
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
//		v1.0		2019.2.5			TKR				�V�K
// *************************************************************************/
PUBLIC void MODE_inc( void ){
	
	en_Mode++;
	
	/* �ő�l�`�F�b�N */
	if( MODE_MAX == en_Mode ){
		en_Mode = MODE_0;
	}
	
	MODE_chg(en_Mode);		// ���[�h�ύX
	
}

