// *************************************************************************
//   ���{�b�g��	�F Baharat�i�o�n���b�g�j
//   �T�v		�F �T���V���C����HAL�i�n�[�h�E�G�A���ۑw�j�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F SPI�̐ݒ�
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

#include <hal_spi.h>		// SPI

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
PUBLIC	enSPI_STATE		en_SpiState;	// SPI�ʐM���
PUBLIC	SHORT*			p_SpiRcvData;	// SPI��M�f�[�^�i�[�A�h���X
PUBLIC	FUNC_PTR		p_SpiCallBackFunc;	// SPI��IDLE�J�ڊ��荞�ݎ��ɓo�^�����ƌĂяo�����֐�

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************

// *************************************************************************
//   �@�\		�F RSPI�̏�����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.2			TKR			�V�K
// *************************************************************************/
PUBLIC void SPI_init(void){
	
	UCHAR	uc_dummy;
	
    SYSTEM.PRCR.WORD	= 0xA502;
	MSTP(RSPI1)			= 0;
	SYSTEM.PRCR.WORD	= 0xA500;
	
	PORTE.PDR.BIT.B3	= 0;	// PE3�F���͂ɐݒ�(MISO)	
	
    MPC.PWPR.BIT.B0WI   	= 0;
    MPC.PWPR.BIT.PFSWE  	= 1;
    MPC.PE1PFS.BIT.PSEL     = 0x0e; // PE1:RSPCKB
    MPC.PE2PFS.BIT.PSEL     = 0x0e; // PE2:MOSIB
    MPC.PE3PFS.BIT.PSEL     = 0x0d; // PE3:MISOB
    MPC.PWPR.BIT.PFSWE  	= 0;
    MPC.PWPR.BIT.B0WI   	= 1;

	PORTE.PMR.BIT.B1	= 1;	// ���Ӌ@�\�̐ݒ�(CLK)
	PORTE.PMR.BIT.B2	= 1;	// ���Ӌ@�\�̐ݒ�(MOSI)		
	PORTE.PMR.BIT.B3	= 1;	// ���Ӌ@�\�̐ݒ�(MISO)
	PORTE.PDR.BIT.B4	= 1;	// PE4�F�o�͂ɐݒ�(CS)	
	
	RSPI1.SPDCR.BIT.SPLW	= 0;	// ���[�h�A�N�Z�X
	RSPI1.SPBR				= 5;	// 1MHz = 48MHz(PCLK) / ( 2 * ( SPBR + 1 ) * 2 ^ BRDV )
	RSPI1.SPCMD0.BIT.BRDV	= 2;	// ��SPBR:5,BRDV:2
	RSPI1.SPCMD0.BIT.CPHA	= 1;	// ��G�b�W�Ńf�[�^�ω��A�����G�b�W�Ńf�[�^�T���v��
	RSPI1.SPCMD0.BIT.CPOL	= 1;	// �A�C�h�����̃N���b�N��High���x��
	RSPI1.SPCMD0.BIT.SPB	= 0x0f;	// �f�[�^��16bit
	RSPI1.SPCMD0.BIT.LSBF	= 0;	// 0:MSB�t�@�[�X�g 1:LSB�t�@�[�X�g
	RSPI1.SPCMD0.BIT.SSLKP	= 0;	// �o�[�X�g�]�������iSSL��GPIO�Ȃ̂Őݒ肵�Ȃ��j
    RSPI1.SPCMD0.BIT.SCKDEN = 0;    // RSPCK��1RSPCK
    RSPI1.SPCMD0.BIT.SLNDEN = 0;    // �l�Q�[�g�x����1RSPCLK 
	RSPI1.SPCMD0.BIT.SPNDEN = 1;    // ���A�N�Z�X�x����1RSPCK+2PCLK
	RSPI1.SPCR.BIT.MSTR		= 1;	// �}�X�^�[���[�h
	uc_dummy 				= RSPI1.SPCR.BYTE;		// �_�~�[���[�h
	uc_dummy				= uc_dummy;				// warining�}�~
	
	ICU.IER[0x05].BIT.IEN2 		= 1;		// SPRI1�̊��荞�݂̋���
	ICU.IER[0x05].BIT.IEN3 		= 1;		// SPTI1�̊��荞�݂̋���
	ICU.IER[0x05].BIT.IEN4 		= 1;		// SPII1�̊��荞�݂̋���
	ICU.IPR[42].BIT.IPR 		= 7;		// SPI1�̊��荞�݃��x���̐ݒ�
	
}

// *************************************************************************
//   �@�\		�F RSPI�̎�M
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �A�h���X(�}�X�N�ς�)
//   �Ԃ�l		�F �ǂ݂������l
// **************************    ��    ��    *******************************
// 		v1.0		2019.4.3			TKR			�V�K
// *************************************************************************/
PUBLIC USHORT SPI_Recv(USHORT address){

    USHORT  data;
    RSPI1.SPDR.WORD.H   = address;
	
  	while(!RSPI1.SPSR.BIT.IDLNF);	//���M�J�n���m�F
	while(RSPI1.SPSR.BIT.IDLNF);	//RSPI1����ُ�Ԃ��m�F

    data = RSPI1.SPDR.WORD.H;
    return(data);
}

// *************************************************************************
//   �@�\		�F RSPI�̃f�[�^���M���荞�ݏ���
//   ����		�F �Ȃ�
//   ����		�F ���̊֐��̌��SPI_III()�̊��荞�݂����҂��Ă���i���M�������j
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �ǂ݂������l
// **************************    ��    ��    *******************************
// 		v1.0		2019.6.1			TKR			�V�K
// *************************************************************************/
PUBLIC void	SPI_TXI(void){
	
	RSPI1.SPCR.BIT.SPTIE	= 0;	// SPI���M���荞�ݗv���̔����̋֎~
	RSPI1.SPCR2.BIT.SPIIE	= 1;	// �A�C�h�����荞�ݗv���̔���������

}

// *************************************************************************
//   �@�\		�F RSPI�̃f�[�^�ݒ�J�n
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �A�h���X�C�������ޒl
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.6.1			TKR			�V�K
// *************************************************************************/
PUBLIC void SPI_staSetData( USHORT us_adr, USHORT us_val ){
	
	USHORT	us_dummy	= 0;
	UCHAR	uc_dummy	= 0;	

	/* IDLE���ȊO�ɂ͎��s���Ȃ� */
	if( en_SpiState != SPI_IDLE ){
		printf("SPI���M�J�nNG [���]%d \n\r", en_SpiState);
		return;
	}
	
	RSPI1.SPCR.BIT.SPE	= 0;		// SPI�@�\����
	en_SpiState			= SPI_SND;	// SPI��ԁF���M��
	
	/*--------------*/
	/*�@�]���O�����@*/
	/*--------------*/
	/*�@�t���O�N���A�����@*/
	RSPI1.SPSR.BIT.MODF	= 0;		// ���[�h�t�H���g�G���[�Ȃ�
	RSPI1.SPSR.BIT.OVRF	= 0;		// �I�[�o�[�����G���[�Ȃ�
	RSPI1.SPSR.BIT.PERF	= 0;		// �p���e�B�G���[�Ȃ�
	
	/*�@���荞�݋֎~�����@*/
	RSPI1.SPCR2.BIT.SPIIE	= 0;	// �A�C�h�����荞�݂̔������֎~
	
	/* �|�[�g���䏈�� */
	PORTE.PODR.BIT.B4		= 0;	// �|�[�gE-4��Lo�o��(SSL-ON)
	
	/* SPI�ʐM���� */
	RSPI1.SPCR.BIT.SPE		= 1;	// SPI�@�\�L��
	RSPI1.SPCR.BIT.SPTIE	= 1;	// SPI���M���荞�ݗv���̔���������
	RSPI1.SPCR.BIT.SPRIE	= 0;	// SPI��M���荞�ݗv���̔������֎~
	RSPI1.SPCR.BIT.SPEIE	= 0;	// SPI�G���[���荞�ݗv���̔������֎~
	
	RSPI1.SPDR.WORD.H		= (USHORT)( us_adr | us_val | SPI_W );	// �f�[�^�Z�b�g

	/* �_�~�[���� */
	us_dummy		= RSPI1.SPDR.WORD.H;	// dummy read
	uc_dummy		= RSPI1.SPCR.BYTE;		// dummy read
	NOUSE( us_dummy );
	NOUSE( uc_dummy );	
	
}

// *************************************************************************
//   �@�\		�F RSPI�f�[�^��M���荞�ݏ���
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.6.2			TKR			�V�K
// *************************************************************************/
PUBLIC void SPI_RXI( void ){
	
	USHORT	us_rcv;		// �f�[�^�擾
	
	RSPI1.SPCR.BIT.SPRIE	= 0;		// SPI��M���荞�ݗv���̔������֎~
	us_rcv	= RSPI1.SPDR.WORD.H;		// �f�[�^�擾

	/* �i�[����A�h���X�`�F�b�N */
	if( p_SpiRcvData != NULL ){
		*p_SpiRcvData	= (SHORT)( us_rcv & 0xff );	// 1byte�����̎��o��
	}
	
}	

// *************************************************************************
//   �@�\		�F RSPI�f�[�^��M�J�n
//   ����		�F �{�֐����s�O�ɁC�f�[�^�擾��(p_SpiRcvData)���w�肷�邱�ƁD
//				   �K�v�ɉ����ăR�[���o�b�N�֐�(p_SpiCallBackFunc)���w�肷�邱��
//   ����		�F �Ȃ�
//   ����		�F �A�h���X
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.6.2			TKR			�V�K
// *************************************************************************/
PUBLIC void SPI_staGetData( USHORT us_adr ){

	USHORT	us_dummy	= 0;
	UCHAR	uc_dummy	= 0;

	/* IDLE���ȊO�͎��s���Ȃ� */
	if( en_SpiState != SPI_IDLE ){
		printf("SPI�擾�J�nNG[���]%d \n\r",en_SpiState);
		return;
	}

	RSPI1.SPCR.BIT.SPE	= 0;		// SPI�@�\����
	en_SpiState			= SPI_RCV;	// SPI��ԁF��M��
	
	/*--------------*/
	/*�@�]���O�����@*/
	/*--------------*/
	/* �t���O�N���A���� */
	RSPI1.SPSR.BIT.MODF		= 0;	// ���[�h�t�H���g�G���[�t���O
	RSPI1.SPSR.BIT.OVRF		= 0;	// �I�[�o�[�����G���[�t���O
	RSPI1.SPSR.BIT.PERF		= 0;	// �p���e�B�G���[�t���O

	/* ���荞�݋֎~���� */
	RSPI1.SPCR2.BIT.SPIIE	= 0;	// �A�C�h�����荞�ݗv���̔������֎~

	/* �|�[�g���� */
	PORTE.PODR.BIT.B4		= 0;	// �|�[�gE-4��Lo�o��(SSL-ON)
	
	/* SPI�ʐM���� */
	RSPI1.SPCR.BIT.SPE		= 1;	// SPI�@�\�L��
	RSPI1.SPCR.BIT.SPTIE	= 1;	// SPI���M���荞�ݗv���̔���������
	RSPI1.SPCR.BIT.SPRIE	= 1;	// SPI��M���荞�ݗv���̔���������
	RSPI1.SPCR.BIT.SPEIE	= 0;	// SPI�G���[���荞�ݗv���̔���������
	
	RSPI1.SPDR.WORD.H		= (USHORT)( us_adr | SPI_R );	// �f�[�^�Z�b�g
	
	/* �_�~�[���� */
	us_dummy		= RSPI1.SPDR.WORD.H;	// dummy read
	uc_dummy		= RSPI1.SPCR.BYTE;		// dummy read
	NOUSE( us_dummy );
	NOUSE( uc_dummy );	
}


// *************************************************************************
//   �@�\		�F RSPI�A�C�h���J�ڊ��荞�ݏ����i�o�^�����R�[���o�b�N�֐������s�j
//   ����		�F �Ȃ�
//   ����		�F RSPI���A�C�h���ɂȂ����ۂ̊��荞��
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.6.2			TKR			�V�K
// *************************************************************************/
PUBLIC void	SPI_III(void){

	/* SPI�I������ */
	RSPI1.SPCR2.BIT.SPIIE	= 0;		// �A�C�h�����荞�ݗv���̔������֎~
	PORTE.PODR.BIT.B4		= 1;		// �|�[�gE-4��Hi�o�� (CS-OFF)
	RSPI1.SPCR.BIT.SPE		= 0;		// SPI�@�\����
	en_SpiState				= SPI_IDLE;	// SPI��ԁFIDLE

	/* �R�[���o�b�N�֐��Ή� */
	if( p_SpiCallBackFunc != NULL ){
		p_SpiCallBackFunc();
	}
}

// *************************************************************************
//   �@�\		�F RSPI���A�C�h����Ԃ��m�F����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.6.2			TKR			�V�K
// *************************************************************************/
PUBLIC BOOL SPI_isIdle(void){

	if( en_SpiState == SPI_IDLE ){
		return TRUE;
	}else{
		return FALSE;
	}
}