// *************************************************************************
//   ���{�b�g��	�F Baharat�i�o�n���b�g�j
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F SCI
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.3.24			TKR			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/
#ifndef _HAL_SPK_H
#define	_HAL_SPK_H

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <iodefine.h>		// I/O
#include <typedefine.h>		// ��`
#include <stdio.h>			// �W�����o��

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
#define A3 		(220)			// ��
#define Bb3 	(233)			// 
#define B3 		(247)			// �V
#define C4 		(262)			// �h
#define Cb4		(277)			// 
#define D4 		(294)			// ��
#define Eb4 	(311)			// 
#define E4 		(330)			// �~
#define F4 		(349)			// �t�@
#define Gb4		(370)			// 
#define G4		(392)			// �\
#define Ab4 	(415)			//
#define	A4		(440)			// ��
#define B4      (494)           // �V
#define C5      (523)           // �h
#define REST 	(0)				// �x��

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
PUBLIC void SPK_on( int frq, float beat, int bpm );
PUBLIC void SPK_OnTest(void);
PUBLIC void SPK_Off(void);

#endif