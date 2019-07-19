// *************************************************************************
//   ���{�b�g��	�F Baharat�i�o�n���b�g�j
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F LED
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

#include <hal_dcm.h>						// LED

//**************************************************
// ��`�idefine�j
//**************************************************
// ����
#define     DCM_R_IN1           (PORTB.PODR.BIT.B1)
#define     DCM_R_IN2           (PORTB.PODR.BIT.B7)
#define     DCM_L_IN1           (PORTE.PODR.BIT.B5)
#define     DCM_L_IN2           (PORTA.PODR.BIT.B3)

// �^�C�}�J�n
#define     DCM_R_TIMER         (MTU.TSTR.BIT.CST0)     // �E:MTU0
#define     DCM_L_TIMER         (MTU.TSTR.BIT.CST4)     // ��:MTU4
#define     DCM_SUC_TIMER       (TPUA.TSTR.BIT.CST5)    // �z��:TPU5

// �s���o�͐ݒ�
#define		DCM_R_TIORA			(MTU0.TIORH.BIT.IOA)
#define		DCM_R_TIORB			(MTU0.TIORH.BIT.IOB)
#define		DCM_L_TIORA			(MTU4.TIORH.BIT.IOA)
#define		DCM_L_TIORB			(MTU4.TIORH.BIT.IOB)
#define		DCM_SUC_TIORA		(TPU5.TIOR.BIT.IOA)
#define		DCM_SUC_TIORB		(TPU5.TIOR.BIT.IOB)

// �J�E���g�l
#define		DCM_R_TCNT			(MTU0.TCNT)
#define		DCM_L_TCNT			(MTU4.TCNT)
#define     DCM_SUC_TCNT        (TPU5.TCNT)

#define		DCM_R_GRA			(MTU0.TGRA)     // �E����
#define		DCM_R_GRB			(MTU0.TGRB)     // �EDuty��
#define		DCM_L_GRA			(MTU4.TGRA)     // ������
#define		DCM_L_GRB			(MTU4.TGRB)     // ��Duty��
#define		DCM_SUC_GRA			(TPU5.TGRA)     // �z������ 
#define		DCM_SUC_GRB			(TPU5.TGRB)     // �z��Duty��


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


// *************************************************************************
//   �@�\		�F DCM�̉�]������CW�i���v���j�ɂ���
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.12			TKR			�V�K
// *************************************************************************/
PUBLIC void DCM_setDirCw( enDCM_ID en_id )
{
	/* ��]�����ݒ� */
	if( en_id == DCM_R ){			// �E
		DCM_R_IN1 	= ON;			// BIN1
		DCM_R_IN2	= OFF;			// BIN2	
	}
	else if( en_id == DCM_L ){		// ��
		DCM_L_IN1 	= OFF;			// AIN1
		DCM_L_IN2	= ON;			// AIN2	
	}else{
	
	}
}


// *************************************************************************
//   �@�\		�F DCM�̉�]������CCW�i�����v���j�ɂ���
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.12			TKR			�V�K
// *************************************************************************/
PUBLIC void DCM_setDirCcw( enDCM_ID en_id )
{
	/* ��]�����ݒ� */
	if( en_id == DCM_R ){			// �E
		DCM_R_IN1 	= OFF;			// BIN1
		DCM_R_IN2	= ON;			// BIN2	
	}
	else if( en_id == DCM_L ){							// ��
		DCM_L_IN1 	= ON;			// AIN1
		DCM_L_IN2	= OFF;			// AIN2	
	}else{
		
	}
}

// *************************************************************************
//   �@�\		�F DCM���~����
//   ����		�F �Ȃ�
//   ����		�F PWM��HI�o�͒��ɖ{�֐������s����ƁA�s����100%�o�͏�ԂȂ邽�߁A�֐����Ńs�����N���A�iLo�j����B
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.14			TKR			�V�K
//		v2.0		2019.5.5			TKR			�z���ǉ�
// *************************************************************************/
PUBLIC void DCM_stopMot( enDCM_ID en_id )
{
	/* ��~�ݒ� */
	if( en_id == DCM_R ){			// �E
		DCM_R_IN1 = OFF;			// BIN1
		DCM_R_IN2 = OFF;			// BIN2
		DCM_R_TIMER = OFF;			// �^�C�}��~
		DCM_R_TIORA = 1;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
		DCM_R_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	}
	else if( en_id == DCM_L ){		// ��
		DCM_L_IN1 = OFF;			// AIN1
		DCM_L_IN2 = OFF;			// AIN2
		DCM_L_TIMER = OFF;			// �^�C�}��~
		DCM_L_TIORA = 1;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
		DCM_L_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	}
	else if(en_id == DCM_SUC ){
		DCM_SUC_TIMER	= OFF;		// �^�C�}��~
		DCM_SUC_TIORA	= 1;		// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
		DCM_SUC_TIORB	= 1;		// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	}
	else{
		
	}
}

// *************************************************************************
//   �@�\		�F DCM���u���[�L���O����
//   ����		�F �z���̏ꍇ�̓u���[�L�͂�������stopMot�Ŏ~�߂�
//   ����		�F �Ȃ�
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.14			TKR			�V�K
// *************************************************************************/
PUBLIC void DCM_brakeMot( enDCM_ID en_id )
{
	
	/* ��~�ݒ� */
	if( en_id == DCM_R ){			// �E
		DCM_R_IN1 = ON;				// BIN1
		DCM_R_IN2 = ON;				// BIN2
		DCM_R_TIMER = OFF;			// �^�C�}��~
		DCM_R_TIORA = 1;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
		DCM_R_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	}
	else if( en_id == DCM_L ){							// ��
		DCM_L_IN1 = ON;				// AIN1
		DCM_L_IN2 = ON;				// AIN2
		DCM_L_TIMER = OFF;			// �^�C�}��~
		DCM_L_TIORA = 1;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	    DCM_L_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
	
	}else{
	}
}

// *************************************************************************
//   �@�\		�F DCM�𓮍�J�n����
//   ����		�F �Ȃ�
//   ����		�F ����J�n�O��PWM�Ɖ�]�������w�肵�Ă�������
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.14			TKR			�V�K
//		v2.0		2019.5.12			TKR			�z���ǉ�
// *************************************************************************/
PUBLIC void DCM_staMot( enDCM_ID en_id )
{
	/* �^�C�}�X�^�[�g */
	if( en_id == DCM_R ){			// �E
		DCM_R_TIORA = 2;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 1 �o��
		DCM_R_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
		DCM_R_TIMER = ON;			// �^�C�}�J�n
	}
	else if( en_id == DCM_L ){							// ��
		DCM_L_TIORA = 2;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 1 �o��
	    DCM_L_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
		DCM_L_TIMER = ON;			// �^�C�}�J�n
	}
	else if( en_id == DCM_SUC ){	//�z��
		DCM_SUC_TIORA = 2;			// TIOCA �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 1 �o��
	    DCM_SUC_TIORB = 1;			// TIOCB �[�q�̋@�\ : �����o�͂� 0 �o�́B�R���y�A�}�b�`�� 0 �o��
		DCM_SUC_TIMER = ON;			// �^�C�}�J�n
	
	}else{
	}
}

// *************************************************************************
//   �@�\		�F �SDCM�𓮍�J�n����
//   ����		�F �Ȃ�
//   ����		�F ����J�n�O��PWM�Ɖ�]�������w�肵�Ă�������
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.14			�O��			�V�K
// *************************************************************************/
PUBLIC void DCM_staMotAll( void )
{
	DCM_staMot(DCM_R);									// �E���[�^ON
	DCM_staMot(DCM_L);									// �����[�^ON
}

// *************************************************************************
//   �@�\		�F DCM��PWM-Duty��ݒ肷��
//   ����		�F ���荞�݊O����ݒ肷��ƁA�_�u���o�b�t�@�łȂ��Ǝ������ɂȂ�ꍇ������B
//   ����		�F ���荞�݃n���h��������s���邱�ƁBDuty0%�̏ꍇ���[�^���~������iPWM�ɂЂ����o��j
//   ����		�F ���[�^ID
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.5			TKR			�V�K
// *************************************************************************/
PUBLIC void DCM_setPwmDuty( enDCM_ID en_id, USHORT us_duty10 )
{
	USHORT	us_cycle;							// ����
	USHORT	us_onReg;							// �ݒ肷��ON-duty
	
	/* PWM�ݒ� */
	//==== �E ====//
	if( en_id == DCM_R ){				
	
		if( 0 == us_duty10 ){			// Duty0%�ݒ�
			DCM_brakeMot( en_id );
		}
		else if( 1000 <= us_duty10 ){	// Duty100%

			DCM_R_TIMER 	= OFF;			// �^�C�}��~
			DCM_R_TCNT 		= 0;			// TCNT �J�E���^���N���A
			DCM_R_GRB 		= 5000;			// �^�C�}�l�ύX
			DCM_R_TIORA 	= 6;			// TIOCA �[�q�̋@�\ : �����o�͂� 1 �o�́B�R���y�A�}�b�`�� 1 �o��
		    DCM_R_TIORB 	= 6;			// TIOCB �[�q�̋@�\ : �����o�͂� 1 �o�́B�R���y�A�}�b�`�� 1 �o��
			DCM_R_TIMER 	= ON;			// �^�C�}�J�n
			us_duty10 		= 1000;
		}
		else{
			us_cycle 		= DCM_R_GRA;		// ����
			us_onReg 		= (USHORT)( (ULONG)us_cycle * (ULONG)us_duty10 / (ULONG)1000 );	// Duty2Reg �v�Z��
			DCM_R_TIMER 	= OFF;				// �^�C�}��~
			DCM_R_TCNT 		= 0;				// TCNT �J�E���^���N���A
			DCM_R_GRB 		= us_onReg;			// onDuty
			DCM_staMot( en_id );				// ��]�J�n
		}
	}
	
	//==== �� ====//
	else if( en_id == DCM_L ){							

		if( 0 == us_duty10 ){			// Duty0%
			DCM_brakeMot( en_id );
		}
		else if( 1000 <= us_duty10 ){	// Duty100%

			DCM_L_TIMER 	= OFF;			// �^�C�}��~
			DCM_L_TCNT 		= 0;			// TCNT �J�E���^���N���A
			DCM_L_GRB 		= 5000;			// �^�C�}�l�ύX
			DCM_L_TIORA 	= 6;			// TIOCA �[�q�̋@�\ : �����o�͂� 1 �o�́B�R���y�A�}�b�`�� 1 �o��
		    DCM_L_TIORB 	= 6;			// TIOCB �[�q�̋@�\ : �����o�͂� 1 �o�́B�R���y�A�}�b�`�� 1 �o��
			DCM_L_TIMER 	= ON;			// �^�C�}�J�n
			us_duty10 		= 1000;
		}
		else{
			us_cycle 		= DCM_L_GRA;		// ����
			us_onReg 		= (USHORT)( (ULONG)us_cycle * (ULONG)us_duty10 / (ULONG)1000 );	// Duty2Reg �v�Z��
			DCM_L_TIMER 	= OFF;			// �^�C�}��~
			DCM_L_TCNT 		= 0;			// TCNT �J�E���^���N���A
			DCM_L_GRB 		= us_onReg;		// �^�C�}�l�ύX
			DCM_staMot( en_id );			// ��]�J�n
			
			printf("DCM_L_GRB = %d\n\r",DCM_L_GRB);
			printf("us_onReg = %d\n\r",us_onReg);
			
			DCM_staMot( en_id );			// ��]�J�n
			
		}
		
	//==== �z�� ====//	
	}else{	
		
		if( 0 == us_duty10 ){			// Duty0%
			DCM_stopMot( en_id );		// �z���̏ꍇ�͎~�߂Ă��܂�
		}
		else if( 1000 <= us_duty10 ){	// Duty100%

			DCM_SUC_TIMER 	= OFF;			// �^�C�}��~
			DCM_SUC_TCNT 	= 0;			// TCNT �J�E���^���N���A
			DCM_SUC_GRB 	= 5000;			// �^�C�}�l�ύX
			DCM_SUC_TIORA 	= 6;			// TIOR(IOA)�[�q�̋@�\ : �����o�͂� 1 �o�́B�R���y�A�}�b�`�� 1 �o��
		    DCM_SUC_TIORB 	= 6;			// TIOR(IOB)�[�q�̋@�\ : �����o�͂� 1 �o�́B�R���y�A�}�b�`�� 1 �o��
			DCM_SUC_TIMER 	= ON;			// �^�C�}�J�n
			us_duty10 		= 1000;
		}
		else{
			us_cycle 		= DCM_SUC_GRA;		// ����
			us_onReg 		= (USHORT)( (ULONG)us_cycle * (ULONG)us_duty10 / (ULONG)1000 );	// Duty2Reg �v�Z��
			DCM_SUC_TIMER 	= OFF;			// �^�C�}��~
			DCM_SUC_TCNT 	= 0;			// TCNT �J�E���^���N���A
			DCM_SUC_GRB 	= us_onReg;		// �^�C�}�l�ύX
			DCM_staMot( en_id );			// ��]�J�n
		}
	}	
	
}