// *************************************************************************
//   ���{�b�g��	�F Baharat�i�o�n���b�g�j
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F FLASH
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.3.27			TKR			�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************

//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <iodefine.h>		// I/O
#include <typedefine.h>		// ��`
#include <stdio.h>			// �W�����o��
#include <hal_flash.h>		// FLASH

//**************************************************
// ��`�idefine�j
//**************************************************
#define BASEADDR	0x00100000	// E2�f�[�^�t���b�V���̐擪�A�h���X
#define FLASHSIZE	0x7fff		//32kB

#define BLOCKSIZE	32		//bytes
#define BLOCKCOUNT	1024		//count

#define write_byte(A,D)	*(UCHAR *)(A)=(D)
#define write_word(A,D)	*(USHORT *)(A)=(D)


//**************************************************
// �񋓑́ienum�j
//**************************************************

//**************************************************
// �\���́istruct�j
//**************************************************

//**************************************************
// �O���[�o���ϐ�
//**************************************************
const UCHAR *BaseAddr = (const unsigned char *)BASEADDR;

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
PUBLIC void FLASH_init(){
	
	// �S�G���A
	FLASH.DFLRE0.WORD	= 0x2dff;		// �f�[�^�t���b�V���̓ǂݏo���̋���	
	FLASH.DFLRE1.WORD	= 0xd2ff;		// �f�[�^�t���b�V���̓ǂݏo���̋���		
	FLASH.DFLWE0.WORD	= 0x1eff;		// �f�[�^�t���b�V���̏����݁������̋���(P/E)
	FLASH.DFLWE1.WORD	= 0xe1ff;		// �f�[�^�t���b�V���̏����݁������̋���(P/E)
	
	if(FLASH.FENTRYR.WORD != 0x0000){
		FLASH.FENTRYR.WORD = 0xAA00;	// FCU���~
	}
	FLASH.FCURAME.WORD = 0xc401;		// FCU RAM�A�N�Z�X����

	for(int i=0;i<8192;i++){			// FCU RAM�Ƀt�@�[���E�F�A���R�s�[
		*(unsigned char *)(0x007F8000+i) = *(unsigned char *)(0xFEFFE000+i);
	}

	FLASH_PEMode();		// P/E���[�h�Ɉڍs

	FLASH.PCKAR.BIT.PCKA	= 48;	// �t���b�V���N���b�N�F48MHz

	//���ӃN���b�N�ʒm�R�}���h
	write_byte(BaseAddr, 0xE9);
	write_byte(BaseAddr, 0x03);
	write_word(BaseAddr, 0x0F0F);
	write_word(BaseAddr, 0x0F0F);
	write_word(BaseAddr, 0x0F0F);
	write_byte(BaseAddr, 0xD0);

	FLASH_waitFCU(2);		
	FLASH_ReadMode();

}


/* P/E���[�h�Ɉڍs */
PUBLIC void FLASH_PEMode(){
	FLASH.FENTRYR.WORD	= 0xAA00;			// FCU���~
	while(0x0000 != FLASH.FENTRYR.WORD);	
	FLASH.FENTRYR.WORD	= 0xAA80;

	FLASH_CheckError();

	FLASH.FWEPROR.BYTE	= 0x01;				// ���iP/E���b�N�r�b�g�̓ǂݏo���C�u�����N�`�F�b�N�j
}

/* �Ǎ��݃��[�h�ɑJ�� */
PUBLIC void FLASH_ReadMode(){
	FLASH.FENTRYR.WORD	= 0xAA00;
	while(0x0000 != FLASH.FENTRYR.WORD);
	FLASH.FWEPROR.BYTE	= 0x02;				// ���i�f�[�^�t���b�V�����[�h���[�h�j
}

/* FCU�ҋ@���� */
PUBLIC void FLASH_waitFCU( int timeout ){

	BOOL bl_Timeout	= FALSE;

	TIME_wait(timeout);

	if( FLASH.FSTATR0.BIT.FRDY == 0 ){		// �^�C���A�E�g�������Ă�����
		bl_Timeout = TRUE;
	}

	/* �^�C���A�E�g���Ă����烊�Z�b�g */
	if(bl_Timeout == TRUE){
		FLASH_FcuReset();
	}

}

/* FCU�̏����� */
PUBLIC void FLASH_FcuReset(){

	FLASH.FRESETR.BIT.FRESET	= 1;
	TIME_wait(2);
	FLASH.FRESETR.BIT.FRESET	= 0;

}

/* �C���[�X���s�� */
PUBLIC void FLASH_Erase(ULONG addr){

	volatile UCHAR	*a = (UCHAR *)addr;
	
	FLASH_PEMode();
		
		*a = 0x20;
		*a = 0xD0;
		FLASH_waitFCU(5);
		FLASH_CheckError();
	
	FLASH_ReadMode();

}

/* �w�肵���̈�ɏ������� */
PUBLIC void FLASH_WriteEE(ULONG addr, USHORT *data){

	volatile USHORT 	*a = (USHORT *)addr;
	volatile UCHAR 		*b = (UCHAR *)addr;

	FLASH_PEMode();
	{
		*b = 0xE8;
		*b = 0x01;
		*a = *data;
		*b = 0xd0;
		FLASH_waitFCU(3);
		FLASH_CheckError(); 
	}
	FLASH_ReadMode();

}

/* �w�肵���̈��ǂݏo�� */
PUBLIC void FLASH_Read(USHORT *add, USHORT *data){

	USHORT	*read;
	if(FLASH.FENTRYR.WORD&0x00ff){
		FLASH.FENTRYR.WORD = 0xAA00;
	}
	FLASH.DFLRE0.WORD = 0x2DFF;
	FLASH.DFLRE1.WORD = 0xD2FF;
			
	*read = *(USHORT *)add;
	*data = *read;
}


/* �G���[���m�F */
PUBLIC void FLASH_CheckError( void ){

	int	iserr	= 0;

	iserr |= FLASH.FSTATR0.BIT.ILGLERR;		//FCU�͕s���ȃR�}���h��s����E2�f�[�^�t���b�V���A�N�Z�X�����o
	iserr |= FLASH.FSTATR0.BIT.ERSERR;		//�C���[�X���ɃG���[����
	iserr |= FLASH.FSTATR0.BIT.PRGERR;		//�v���O�������ɃG���[����

	if(iserr == 0){
		return;
	}

	iserr = 1;
	//LED�f�o�b�O

	if(FLASH.FSTATR0.BIT.ILGLERR == 1){

		if(FLASH.FASTAT.BYTE != 0x10){
			FLASH.FASTAT.BYTE = 0x10;
		}
	}

	write_byte(BaseAddr,0x50);

}

