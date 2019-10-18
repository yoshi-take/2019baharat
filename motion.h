// *************************************************************************
//   ���{�b�g��	�F Baharat�i�o�n���b�g�j
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F motion
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.26			TKR			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/
#ifndef _MOTION_H
#define	_MOTION_H

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <iodefine.h>						// I/O
#include <stdio.h>							// �W�����o��

#include <parameter.h>

//**************************************************
// ��`�idefine�j
//**************************************************

//**************************************************
// �񋓑́ienum�j
//**************************************************
/* ����R�}���h���X�g */
typedef enum{
	MOT_R90 =0,					// �E 90�x���n�M����
	MOT_L90,					// �� 90�x���n�M����
	MOT_R180,					// �E180�x���n�M����
	MOT_L180,					// ��180�x���n�M����
	MOT_R360,					// �E360�x���n�M����
	MOT_L360,					// ��360�x���n�M����
	MOT_TURN_CMD_MAX,
}enMOT_TURN_CMD;

/* �X�����[���R�}���h���X�g */
typedef enum{
	MOT_R90S =0,				// �E 90�x���X�����[��
	MOT_L90S,					// �� 90�x���X�����[��
	MOT_R45S_S2N,				// [�΂ߗp] �E 45�x���X�����[���A�X�g���[�g �� �΂�
	MOT_L45S_S2N,				// [�΂ߗp] �� 45�x���X�����[���A�X�g���[�g �� �΂�
	MOT_R45S_N2S,				// [�΂ߗp] �E 45�x���X�����[���A�΂� �� �X�g���[�g
	MOT_L45S_N2S,				// [�΂ߗp] �� 45�x���X�����[���A�΂� �� �X�g���[�g
	MOT_R90S_N,					// [�΂ߗp] �E 90�x���X�����[���A�΂� �� �΂�
	MOT_L90S_N,					// [�΂ߗp] �� 90�x���X�����[���A�΂� �� �΂�
	MOT_R135S_S2N,				// [�΂ߗp] �E135�x���X�����[���A�X�g���[�g �� �΂�
	MOT_L135S_S2N,				// [�΂ߗp] ��135�x���X�����[���A�X�g���[�g �� �΂�
	MOT_R135S_N2S,				// [�΂ߗp] �E135�x���X�����[���A�΂� �� �X�g���[�g
	MOT_L135S_N2S,				// [�΂ߗp] ��135�x���X�����[���A�΂� �� �X�g���[�g
	MOT_SURA_CMD_MAX,
}enMOT_SULA_CMD;

/* �}�E�X�i�s���� */
typedef enum{
	MOT_DIR_FRONT = 0,			// �O�i
	MOT_DIR_BACK,				// ���
	MOT_DIR_R,					// �E����
	MOT_DIR_L,					// ������
	MOT_DIR_MAX,
}enMOT_DIR;

/*�ǐ؂�␳*/
typedef enum{
	MOT_WALL_EDGE_NONE = 0,		//�ǂ̃G�b�W���o�ł̕␳����
	MOT_WALL_EDGE_RIGHT,		//�E�ǂ̃G�b�W���o�ł̕␳
	MOT_WALL_EDGE_LEFT,			//���ǂ̃G�b�W���o�ł̕␳
	MOT_WALL_EDGE_MAX,
}enMOT_WALL_EDGE_TYPE;

/* ����^�C�v */
typedef enum{
	MOT_ST_NC    =  0,
	MOT_ACC_CONST_DEC,				// [01] ��`����
	MOT_ACC_CONST_DEC_CUSTOM,	 	// [02] ��`�����i�����l�ύX�j
	MOT_ACC_CONST,					// [03] �����{����
	MOT_ACC_CONST_CUSTOM,		   	// [04] �����{�����i�����l�ύX�j
	MOT_CONST_DEC,					// [05] �����{����
	MOT_CONST_DEC_CUSTOM,			// [06] �����{�����i�����l�ύX�j
	
	/* cos�ߎ� */
	MOT_ACC_CONST_DEC_SMOOTH,			// [07] ��`����
	MOT_ACC_CONST_DEC_SMOOTH_CUSTOM,	// [08] ��`�����i�����l�ύX�j
	MOT_ACC_CONST_SMOOTH,				// [09] �����{����
	MOT_ACC_CONST_SMOOTH_CUSTOM,		// [10] �����{�����i�����l�ύX�j
	MOT_CONST_DEC_SMOOTH,				// [11] �����{����
	MOT_CONST_DEC_SMOOTH_CUSTOM,		// [12] �����{�����i�����l�ύX�j
	
	MOT_ST_MAX,
}enMOT_ST_TYPE;

/* ���i�^�C�v */
typedef enum{
	MOT_GO_ST_NORMAL    =  0,	// �ʏ�̒��i
	MOT_GO_ST_SKEW,				// �΂߂̒��i
	MOT_GO_ST_SMOOTH,			// cos�ߎ����i
	MOT_GO_ST_MAX,
}enMOT_GO_ST_TYPE;


//**************************************************
// �\���́istruct�j
//**************************************************

//**************************************************
/* �}�N��                                          */
//**************************************************
#define IS_R_SLA(a)			( ( (a) % 2 == 0 ) ? (TRUE) : (FALSE))

//**************************************************
// �O���[�o���ϐ�
//**************************************************

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
PUBLIC void MOT_setTrgtSpeed( FLOAT f_speed );
PUBLIC void MOT_setNowSpeed( FLOAT f_speed );
PUBLIC void MOT_setSlaStaSpeed( FLOAT f_speed );
PUBLIC FLOAT MOT_getSlaStaSpeed( void );

PUBLIC void MOT_goBlock_FinSpeed(FLOAT num, FLOAT f_fin);
PUBLIC void MOT_goSkewBlock_FinSpeed(FLOAT f_num, FLOAT f_fin);
PUBLIC void MOT_goBlock_Const( FLOAT f_num);
PUBLIC void MOT_goSla( enMOT_SULA_CMD en_type, stSLA *p_sla);
PUBLIC void MOT_goHitBackWall( void );
PUBLIC void MOT_Turn( enMOT_TURN_CMD en_type );
PUBLIC void MOT_goBack_Const( FLOAT f_dist );
PUBLIC void MOT_turn( enMOT_TURN_CMD en_type);
PUBLIC void MOT_turn2( enMOT_TURN_CMD en_type, FLOAT f_trgtAngleS );

PUBLIC void MOT_setWallEdgeType( enMOT_WALL_EDGE_TYPE en_type );
PUBLIC enMOT_WALL_EDGE_TYPE MOT_getWallEdgeType( void );
PUBLIC void MOT_setWallEdge( BOOL bl_val );
PRIVATE void MOT_Failsafe( BOOL* exists );
PUBLIC void MOT_circuit(FLOAT x,FLOAT y, enMOT_SULA_CMD en_type, int num, FLOAT f_speed);

#endif