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
#include <hal_dcmCtrl.h>	// CTRL

#include <parameter.h>		// parameter
#include <map.h>			// map

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
PRIVATE	USHORT		us_DefaultCnt;	// �J�E���^�����l

extern PUBLIC UCHAR			g_sysMap[ MAP_Y_SIZE ][ MAP_X_SIZE ];			// ���H���

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************

// *************************************************************************
//   �@�\		�F ���[�h�̏������B
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
//		v1.0		2019.10.27			TKR				�V�K
// *************************************************************************/
PUBLIC void MODE_init( void ){
	us_DefaultCnt 	= ENC_getCnt(ENC_R);
}


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

	enMAP_HEAD_DIR		en_endDir;
	int i=0;
	
	/* ���s�p�����[�^�ݒ� */
	PARAM_setCntType( TRUE );
	MOT_setTrgtSpeed( MAP_SEARCH_SPEED );						// �ڕW���x�ݒ�
	PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );		// [���i]���x�ᑬ
	PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );	// [����]���x�ᑬ
	PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );	// [�X�����[��]���x�ᑬ
	
	
	/* �X�����[���f�[�^���� */
	PARAM_makeSla(500.0f, 150.0f, 5300.0f, SLA_90, PARAM_SLOW);		// 90�x
	PARAM_makeSla(500.0f, 100.0f, 2500.0f, SLA_45, PARAM_VERY_SLOW);	// 45�x
	// 135�x
	// �΂� �� 90���� �΂�

	switch( en_Mode ){
		
		case MODE_0:	// �o�b�e���[�`�F�b�Nor�ǃ`�F�b�N
			LED_offAll();	
			//BAT_Check();
			DIST_Check();
			break;
			
		case MODE_1:		// �T�����s
			LED_offAll();
			MODE_wait();			// �肩�����ҋ@
			TIME_wait(1000);
			GYRO_clrAngle();		// �p�x���Z�b�g

			/* ���s�p�����[�^ */
			PARAM_setCntType( FALSE );
			MOT_setTrgtSpeed( MAP_SEARCH_SPEED );			// �ڕW���x�ݒ�			
			PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );	// [���i]���x�ᑬ
			PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );	// [����]���x�ᑬ
			PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );	// [�X�����[��]���x�ᑬ

			/* ���H�T�� */
			MAP_setPos(0,0,NORTH);
			MAP_searchGoal(GOAL_MAP_X, GOAL_MAP_Y, SEARCH, SEARCH_SURA);
			//MAP_searchGoalKnown(GOAL_MAP_X, GOAL_MAP_Y, SEARCH_SURA);

			/* �A��T�� */
			TIME_wait(500);
			MAP_searchGoal(0, 0, SEARCH, SEARCH_SURA);

			/* �R�}���h�쐬 */
			PARAM_setCntType(TRUE);											// �ŒZ���s
			MAP_setPos(0,0,NORTH);											// �������W
			MAP_makeContourMap(GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY);			// �������}�b�v�쐬
			MAP_makeCmdList(0,0,NORTH,GOAL_MAP_X,GOAL_MAP_Y, &en_endDir);	// �h���C�u�R�}���h�쐬
			MAP_makeSuraCmdList();											// �X�����[���R�}���h�쐬

			break;
			
		case MODE_2:		// �}�b�v�f�[�^�̃��[�h
			LED_offAll();
			TIME_wait(500);
			LED_onAll();
			MAP_LoadMapData();
			LED_offAll();
			MAP_showLog();
			break;
			
		case MODE_3:		// �ŒZ���s
			LED_offAll();
			MODE_wait();			// �肩�����ҋ@
			TIME_wait(1500);
			GYRO_clrAngle();		// �p�x���Z�b�g
			
			/* �X�����[���f�[�^���� */
			PARAM_makeSla(500.0f, 150.0f, 5300.0f, SLA_90, PARAM_NORMAL);		// 90�x
			// 45�x
			// 135�x
			// �΂� �� 90���� �΂�

			/* ���s�p�����[�^ */
			PARAM_setCntType( TRUE );
			MOT_setTrgtSpeed(1500.0f);
			MOT_setSlaStaSpeed(500.0f);
			PARAM_setSpeedType( PARAM_ST, PARAM_NORMAL );		// [���i]
			PARAM_setSpeedType( PARAM_TURN, PARAM_NORMAL );		// [����]
			PARAM_setSpeedType( PARAM_SLA, PARAM_NORMAL );		// [�X�����[��]

			/* �R�}���h�쐬 */
			MAP_setPos(0,0,NORTH);
			MAP_makeContourMap(GOAL_MAP_X,GOAL_MAP_Y,BEST_WAY);
			MAP_makeCmdList(0,0,NORTH,GOAL_MAP_X,GOAL_MAP_Y,&en_endDir);
			MAP_makeSuraCmdList();
			MAP_makeSkewCmdList();
			MAP_showCmdLog();

			/* �R�}���h���s */
			MAP_drive(MAP_DRIVE_SURA);
			break;
			
		case MODE_4:		// �X�����[��
			LED_offAll();
			MODE_wait();			// �肩�����ҋ@
			TIME_wait(1500);
			GYRO_clrAngle();		// �p�x���Z�b�g
			
			CTRL_LogSta();			// ���O�J�n

			/* ���s�p�����[�^ */
			PARAM_setCntType( TRUE );
			MOT_setTrgtSpeed( 500.0f );						// �ڕW���x�ݒ�
			PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );		// [���i]���x�ᑬ
			PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );	// [����]���x�ᑬ
			PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );	// [�X�����[��]���x�ᑬ

			PARAM_makeSla(500.0f, 150.0f, 5300.0f, SLA_90, PARAM_SLOW);		// �X�����[���f�[�^����

			MOT_circuit( 3, 4, MOT_R90, 10, 500.0f);	

			SPK_on(Eb4,16.0f,120);
			SPK_on(E4,16.0f,120);
			SPK_on(F4,16.0f,120);

			break;
			
		case MODE_5:		// ���i&���M�n����			
			LED_offAll();
			MODE_wait();			// �肩�����ҋ@
			TIME_wait(1500);
			GYRO_clrAngle();		// �p�x���Z�b�g
			
			CTRL_LogSta();			// ���O�J�n

			/* ���s�p�����[�^ */
			PARAM_setCntType( TRUE );
			MOT_setTrgtSpeed( MAP_SEARCH_SPEED );						// �ڕW���x�ݒ�
			PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );	// [���i]
			PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );	// [����]
			PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );	// [�X�����[��]

			MOT_goBlock_FinSpeed(5,0);
			//MOT_turn2(MOT_L90,700.0f);
			TIME_wait(100);
			
			LED_onAll();
			
			break;
			
		case MODE_6:	// ���O�֌W
			LED_offAll();
			TIME_wait(100);
			//CTRL_showLog();		// ���O�̑|���o��
			MAP_showLog();

			break;
			
		case MODE_7:	// �΂�
			LED_offAll();
			MODE_wait();			// �肩�����ҋ@
			TIME_wait(1000);
			GYRO_clrAngle();		// �p�x���Z�b�g

			CTRL_LogSta();			// ���O�J�n

			/* �X�����[���f�[�^���� */
			PARAM_makeSla(500.0f, 150.0f, 5300.0f, SLA_90, PARAM_SLOW);					// 90�x
			PARAM_makeSla(500.0f, 100.0f, 2500.0f, SLA_45, PARAM_SLOW);					// 45�x
			// 135�x
			// �΂� �� 90���� �΂�
			
			/* ���s�p�����[�^ */
			PARAM_setCntType( FALSE );
			MOT_setTrgtSpeed( MAP_SEARCH_SPEED );				// �ڕW���x�ݒ�			
			PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );			// [���i]���x�ᑬ
			PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );		// [����]���x�ᑬ
			PARAM_setSpeedType( PARAM_SLA, PARAM_VERY_SLOW );	// [�X�����[��]���x�ᑬ

			/* ���s */
			MOT_goBlock_FinSpeed(1.0f+MOVE_BACK_DIST,MAP_SEARCH_SPEED);
			MOT_goSla(MOT_R45S_S2N,PARAM_getSra( SLA_45 ));
			MOT_goSkewBlock_FinSpeed(3,MAP_SEARCH_SPEED);
			MOT_goSla(MOT_L45S_N2S,PARAM_getSra( SLA_45 ));
			MOT_goBlock_FinSpeed(1,0);
			LED_onAll();

			break;
			
		case MODE_8:
			LED_offAll();
			printf("g_sysMap:%x\n\r",sizeof(g_sysMap));	
			break;

		case MODE_9:
			LED_offAll();
			LED_offAll();
			MODE_wait();			// �肩�����ҋ@
			TIME_wait(1000);
			GYRO_clrAngle();		// �p�x���Z�b�g

			/* ���s�p�����[�^ */
			PARAM_setCntType( FALSE );
			MOT_setTrgtSpeed( MAP_SEARCH_SPEED );			// �ڕW���x�ݒ�			
			PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );	// [���i]���x�ᑬ
			PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );	// [����]���x�ᑬ
			PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );	// [�X�����[��]���x�ᑬ

			/* ���H�T�� */
			MAP_setPos(0,0,NORTH);
			//MAP_searchGoal(GOAL_MAP_X, GOAL_MAP_Y, SEARCH, SEARCH_SURA);
			MAP_searchGoalKnown(GOAL_MAP_X, GOAL_MAP_Y, SEARCH_SURA);

			/* �A��T�� */
			TIME_wait(500);
			MAP_searchGoalKnown(0, 0, SEARCH_SURA);

			/* �R�}���h�쐬 */
			PARAM_setCntType(TRUE);											// �ŒZ���s
			MAP_setPos(0,0,NORTH);											// �������W
			MAP_makeContourMap(GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY);			// �������}�b�v�쐬
			MAP_makeCmdList(0,0,NORTH,GOAL_MAP_X,GOAL_MAP_Y, &en_endDir);	// �h���C�u�R�}���h�쐬
			MAP_makeSuraCmdList();											// �X�����[���R�}���h�쐬


			break;

		case MODE_10:
			break;

		case MODE_11:
			LED_offAll();
			MAP_SaveMapData();
			LED_onAll();
			break;

		case MODE_12:
			LED_offAll();
			MAP_LoadMapData();
			LED_onAll();
			break;

		case MODE_13:
			SPK_debug();
			break;

		case MODE_14:
			break;
	
		case MODE_15:			// �}�b�v�f�[�^�̏���
			LED_onAll();
			TIME_wait(1000);
			MAP_ClearMapData();
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
			SPK_on(D6,16.0f,120);
			break;
			
		case MODE_1:
			SPK_on(C4,16.0f,120);
			LED_on(LED0);
			break;
			
		case MODE_2:
			SPK_on(D4,16.0f,120);
			LED_on(LED1);
			break;
			
		case MODE_3:
			SPK_on(E4,16.0f,120);
			LED_on(LED2);
			break;
			
		case MODE_4:
			SPK_on(F4,16.0f,120);
			LED_on(LED3);
			break;
			
		case MODE_5:
			SPK_on(G4,16.0f,120);
			LED_on(LED4);
			break;
			
		case MODE_6:
			SPK_on(A4,16.0f,120);
			LED_on(LED0);
			LED_on(LED1);
			break;
			
		case MODE_7:
			SPK_on(B4,16.0f,120);
			LED_on(LED0);
			LED_on(LED2);
			break;
			
		case MODE_8:
			SPK_on(C5,16.0f,120);
			LED_on(LED0);
			LED_on(LED3);
			break;

		case MODE_9:
			SPK_on(D5,16.0f,120);
			LED_on(LED0);
			LED_on(LED4);
			break;

		case MODE_10:
			SPK_on(E5,16.0f,120);
			LED_on(LED1);
			LED_on(LED2);
			break;

		case MODE_11:
			SPK_on(F5,16.0f,120);
			LED_on(LED1);
			LED_on(LED3);
			break;

		case MODE_12:
			SPK_on(G5,16.0f,120);
			LED_on(LED1);
			LED_on(LED4);
			break;

		case MODE_13:
			SPK_on(A5,16.0f,120);
			LED_on(LED2);
			LED_on(LED3);
			break;

		case MODE_14:
			SPK_on(B_5,16.0f,120);
			LED_on(LED2);
			LED_on(LED4);
			break;
	
		case MODE_15:
			SPK_on(C6,16.0f,120);
			LED_on(LED3);
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

// *************************************************************************
//   �@�\		�F ���[�h�����Z�ύX����B
//   ����		�F �Ȃ�
//   ����		�F �G���R�[�_�Ń��[�h�ύX�ł���悤�ɒǉ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
//		v1.0		2019.10.27			TKR				�V�K
// *************************************************************************/
PUBLIC void MODE_dec( void ){
		
	/* �ő�l�`�F�b�N */
	if( MODE_0 == en_Mode ){
		en_Mode = MODE_MAX;
	}
	
	en_Mode--;

	MODE_chg(en_Mode);		// ���[�h�ύX
	
}


// *************************************************************************
//   �@�\		�F ���[�h�̐؂�ւ��`�F�b�N
//   ����		�F �Ȃ�
//   ����		�F 
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
//		v1.0		2019.10.27			TKR				�V�K
// *************************************************************************/
PUBLIC void MODE_chkMode( void ){

	if (( us_DefaultCnt - MODE_CHG_COUNT ) > ENC_getCnt(ENC_R) ){
		ENC_clr();
		MODE_inc();
	}else if( ( us_DefaultCnt + MODE_CHG_COUNT ) < ENC_getCnt(ENC_R) ){
		ENC_clr();
		MODE_dec();
	}

}


// *************************************************************************
//   �@�\		�F �O��(�E)��臒l�ȏゾ�ƃt���O������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F ���m�����Ftrue	���m�ł��Ȃ������Ffalse
// **************************    ��    ��    *******************************
// 		v1.0		2019.8.16			TKR			�V�K
// *************************************************************************/
PUBLIC BOOL MODE_DistRightCheck(){
	
	SHORT 	s_rightval;
	BOOL	bl_check;
	
	s_rightval 	= DIST_getNowVal(DIST_SEN_R_FRONT);
	
	if( s_rightval >= EXE_THRESH_R ){
		bl_check = true;
	
	}else{
		bl_check = false;
	
	}
	
	return bl_check;
}

// *************************************************************************
//   �@�\		�F �O��(��)��臒l�ȏゾ�ƃt���O������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F ���m�����Ftrue	���m�ł��Ȃ������Ffalse
// **************************    ��    ��    *******************************
// 		v1.0		2019.8.16			TKR			�V�K
// *************************************************************************/
PUBLIC BOOL MODE_DistLeftCheck(){
	
	SHORT 	s_leftval;
	BOOL	bl_check;
	
	s_leftval 	= DIST_getNowVal(DIST_SEN_L_FRONT);
	
	if( s_leftval >= EXE_THRESH_L ){
		bl_check = true;
	
	}else{
		bl_check = false;
	
	}
	
	return bl_check;
}

// *************************************************************************
//   �@�\		�F ����������Ƒҋ@��Ԃɓ���
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �������m�Ftrue	����ȊO�Ffalse
// **************************    ��    ��    *******************************
// 		v1.0		2019.8.16			TKR			�V�K
// *************************************************************************/
PUBLIC BOOL MODE_setWaitCheck(){
	
	BOOL bl_check;
	
	if( true == MODE_DistRightCheck() ){	// �E�������m
		LED_on_multi(0x18);

	}
	if( true == MODE_DistLeftCheck() ){		// ���������m
		LED_on_multi(0xc0);

	}
	
	if( ( true == MODE_DistRightCheck() ) && ( true == MODE_DistLeftCheck() ) ){
		LED_onAll();
		bl_check = true;
		
	}else if( ( false == MODE_DistRightCheck() ) && ( false == MODE_DistLeftCheck() )){
		LED_offAll();
		bl_check = false;
	
	}else{
		bl_check = false;
	}
	
	return bl_check;
}

// *************************************************************************
//   �@�\		�F ����������Ƒҋ@��Ԃɓ���A�������Ă��痣���Ǝ��s
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �ҋ@��Ԃ��甲���o���Ftrue	����ȊO�Ffalse
// **************************    ��    ��    *******************************
// 		v1.0		2018.8.16			�g�c			�V�K
// *************************************************************************/
PUBLIC BOOL MODE_CheckExe(){
	
	BOOL bl_check;
	
	if( true == MODE_setWaitCheck() ){
		TIME_wait(200);
		
		if( false == MODE_setWaitCheck() ){
			LED_offAll();
			TIME_wait(1000);
			bl_check = true;
			
		}else{
			bl_check = false;
		
		}
		
	}else{
		
		bl_check = false;
	}
	
	return bl_check;
}

// *************************************************************************
//   �@�\		�F ����������Ƒҋ@��Ԃɓ���A�������Ă��痣���Ǝ��s
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �ҋ@��Ԃ��甲���o���Ftrue	����ȊO�Ffalse
// **************************    ��    ��    *******************************
// 		v1.0		2018.8.16			�g�c			�V�K
// *************************************************************************/
PRIVATE void MODE_wait( void ){

	while(1){
		LED_onAll();

		if(( true == MODE_DistRightCheck() ) && ( true == MODE_DistLeftCheck() )){
			LED_offAll();
			TIME_wait(1000);
			break;
		}	
	}

}