// *************************************************************************
//   ���{�b�g��	�F Baharat�i�o�n���b�g�j
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F SCI
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.5.5			TKR			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/
#ifndef _HAL_DIST_H
#define	_HAL_DIST_H

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <iodefine.h>		// I/O
#include <typedefine.h>		// �E��E�`
#include <stdio.h>			// �E�W�E��E��E��E��E�C�E�u�E��E��E��E�

//**************************************************
// ��`�idefine�j
//**************************************************

//**************************************************
// �񋓑́ienum�j
//**************************************************
typedef enum{
	DIST_SEN_R_FRONT = 0,		// �E�E�E�O
	DIST_SEN_L_FRONT,			// �E��E��E�O
	//DIST_SEN_R_45,			// �E�E45�E��E�(�E��E��E��E�)
	//DIST_SEN_L_45,			// �E��E�45�E��E�(�E��E��E��E�)
	DIST_SEN_R_SIDE,			// �E�E�E��E�
	DIST_SEN_L_SIDE,			// �E��E��E��E�
	DIST_SEN_MAX
}enDIST_SEN_ID;

//**************************************************
// �\���́istruct�j
//**************************************************

//**************************************************
// �O���[�o���ϐ�
//**************************************************

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
PUBLIC SHORT DIST_getNowVal( enDIST_SEN_ID en_id );
PUBLIC void DIST_Pol_Front( void );
PUBLIC void DIST_Pol_Side( void );
PUBLIC void DIST_getErr( LONG* p_err );
PUBLIC void DIST_getErrSkew( LONG* p_err );
PUBLIC void DIST_getErrFront( LONG* p_err );
PUBLIC void DIST_Check( void );
PUBLIC void DIST_adj( void );
PUBLIC BOOL DIST_isWall_FRONT( void );
PUBLIC BOOL DIST_isWall_R_SIDE( void );
PUBLIC BOOL DIST_isWall_L_SIDE( void );
PUBLIC void DIST_LogSta( void );
PUBLIC void DIST_showLog( void );
#endif