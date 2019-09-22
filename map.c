// *************************************************************************
//   ���{�b�g��	�F �e�B�E���e�B�E��2014
//   �T�v		�F ���H�T���A�񎟑��s
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
#ifdef __cplusplus
    extern "C"{
#endif


//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
#include <stdio.h>							// �W�����o��
#include <string.h>							// �����񑀍�imemset�p�j
#include <iodefine.h>						// I/O

#include <hal_dcm.h>						// DCM
#include <hal_dcmCtrl.h>					// DCM_CTRL
#include <hal_dist.h>						// DIST
#include <hal_led.h>						// LED
#include <hal_spk.h>						// SPK

#include <motion.h>							// ���[�V����
#include <parameter.h>						// [�@��ŗL] �p�����[�^�i���J�d�l/�Q�C��/�����l�j
#include <map.h>							// �}�b�v�{�񎟑��s
#include <hal_flash.h>						// FLASH


//**************************************************
// ��`�idefine�j
//**************************************************
#define MAP_SMAP_MAX_VAL		( MAP_X_SIZE * MAP_Y_SIZE ) 			// ������map�̍ő�l
#define MAP_SMAP_MAX_PRI_VAL	( MAP_SMAP_MAX_VAL * 4 )				// ������map�̗D��x�ő�l


//**************************************************
// �񋓑́ienum�j
//**************************************************

//**************************************************
// �\���́istruct�j
//**************************************************
/* �V�~�����[�V���� */
typedef struct{
	enMAP_CMD	en_cmd;			// �R�}���h
	FLOAT		f_x0_x1;		// [0]/[1]��X���W���Z�l
	FLOAT		f_y0_y1;		// [0]/[1]��y���W���Z�l
	FLOAT		f_x2_x3;		// [2]/[3]��X���W���Z�l
	FLOAT		f_y2_y3;		// [2]/[3]��y���W���Z�l
	FLOAT		f_x4_x5;		// [4]/[5]��X���W���Z�l
	FLOAT		f_y4_y5;		// [4]/[5]��y���W���Z�l
	FLOAT		f_x6_x7;		// [6]/[7]��X���W���Z�l
	FLOAT		f_y6_y7;		// [6]/[7]��y���W���Z�l
	SHORT		s_dir;			// �i�s�����i[0]�k [1]�k�� [2]�� [3]�쓌 [4]�� [5]�쐼 [6]�� [7]�k�� �j
}stMAP_SIM;


//**************************************************
// �O���[�o���ϐ�
//**************************************************
/* ---------- */
/*  ���H�T��  */
/* ---------- */
PRIVATE volatile enMAP_HEAD_DIR	en_Head;										// �}�E�X�̐i�s���� 0:N 1:E 2:S 3:W
PUBLIC UCHAR			my;												// �}�E�X�̂w���W
PUBLIC UCHAR			mx;												// �}�E�X�̂x���W
PUBLIC USHORT			us_cmap[MAP_Y_SIZE][MAP_X_SIZE];				// ������ �f�[�^
PUBLIC UCHAR			g_sysMap[ MAP_Y_SIZE ][ MAP_X_SIZE ];			// ���H���
PRIVATE FLOAT			f_MoveBackDist;									// �Ǔ��ē���Ō�ނ�������[���]
PRIVATE UCHAR			uc_TurnCnt = 0;									// ���M�n�A����
PRIVATE UCHAR			uc_SlaCnt = 0;									// �X�����[���A����
PRIVATE UCHAR			uc_back[ MAP_Y_SIZE ][ MAP_X_SIZE ];			// ���H�f�[�^
PRIVATE USHORT			us_DashCmdIndex = 0;							// ���m�R�}���h�C���f�b�N�X

/* -------------- */
/*  �R�}���h���s  */
/* -------------- */
/* �R�}���h���X�g */
PUBLIC	UCHAR		dcom[LIST_NUM];					// ���n�M����p
PUBLIC	UCHAR		scom[LIST_NUM];					// �X�����[���p
PUBLIC	UCHAR		tcom[LIST_NUM];					// �΂ߑ��s�p
PUBLIC	UCHAR		mcom[LIST_NUM];					// ���m�R�}���h���s�p
PRIVATE	USHORT		us_totalCmd;					// �g�[�^���R�}���h��
PRIVATE	FLOAT		f_PosX;							// X���W
PRIVATE	FLOAT		f_PosY;							// Y���W
PRIVATE	SHORT		s_PosDir;						// �i�s�����i[0]�k [1]�k�� [2]�� [3]�쓌 [4]�� [5]�쐼 [6]�� [7]�k�� �j

/* �R�}���h�ɉ��������W�X�V�f�[�^ */
PRIVATE CONST stMAP_SIM st_PosData[] = {
	
	//	�R�}���h	[0]/[1]��X	[0]/[1]��Y	[2]/[3]��X	[2]/[3]��Y	[4]/[5]��X	[4]/[5]��Y	[6]/[7]��X	[6]/[7]��Y	����
	{ R90,			0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-0.5,		0.5,		+2 },		// [0]
	{ L90,			-0.5,		0.5,		0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-2 },		// [1]
	{ R90S,			0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-0.5,		0.5,		+2 },		// [2]
	{ L90S,			-0.5,		0.5,		0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-2 },		// [3]
	{ RS45N,		0.25,		0.75,		0.75,		-0.25,		-0.25,		-0.75,		-0.75,		0.25,		+1 },		// [4]
	{ LS45N,		-0.25,		0.75,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-1 },		// [5]
	{ RS135N,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-0.25,		0.75,		+3 },		// [6]
	{ LS135N,		-0.75,		0.25,		0.25,		0.75,		0.75,		-0.25,		-0.25,		-0.75,		-3 },		// [7]
	{ RN45S,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-0.25,		0.75,		+1 },		// [8]
	{ LN45S,		0.25,		0.75,		0.75,		-0.25,		-0.25,		-0.75,		-0.75,		0.25,		-1 },		// [9]
	{ RN135S,		0.75,		-0.25,		-0.25,		-0.75,		-0.75,		0.25,		0.25,		0.75,		+3 },		// [10]
	{ LN135S,		-0.25,		0.75,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-3 },		// [11]
	{ RN90N,		0.5,		0,			0,			-0.5,		-0.5,		0,			0,			0.5,		+2 },		// [12]
	{ LN90N,		0,			0.5,		0.5,		0,			0,			-0.5,		-0.5,		0,			-2 },		// [13]
	{ GO1,			0,			0.5,		0.5,		0,			0,			-0.5,		-0.5,		0,			0  },		// [14]
	{ NGO1,			0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-0.5,		0.5,		0  },		// [15]
	{ MAP_CMD_MAX,	0,			0,			0,			0,			0,			0,			0,			0,			0  },
};

/* ���O */
PRIVATE FLOAT f_LogPosX[30];
PRIVATE FLOAT f_LogPosY[30];
PRIVATE USHORT us_LogIndex = 0;
PRIVATE USHORT us_LogWallCut[30];
PRIVATE USHORT us_LogIndexWallCut = 0;


//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
extern PUBLIC void TIME_wait( ULONG ul_time );
//extern PUBLIC BOOL SYS_isOutOfCtrl( void );
PRIVATE void MAP_setCmdPos( UCHAR uc_x, UCHAR uc_y, enMAP_HEAD_DIR en_dir );



// *************************************************************************
//   �@�\		�F ���H�f�[�^���N���A����
//   ����		�F RAM�̕����N���A
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_clearMap( void )
{
	USHORT	x, y;
	UCHAR	uc_data;

	/* ���ׂẴ}�b�v�f�[�^�𖢒T����Ԃɂ��� */
	for ( y = 0; y < MAP_Y_SIZE; y++){

		for( x = 0; x < MAP_X_SIZE; x++){

			uc_data = 0x00;
			if ( ( x == 0) && ( y == 0 ) ) uc_data = 0xfe;
			else if ( ( x == 1 ) && ( y == 0 ) ) uc_data = 0xcc;
			else if ( ( x == (MAP_X_SIZE-1) ) && ( y == 0 ) ) uc_data = 0x66;
			else if ( ( x == 0 ) && ( y == (MAP_Y_SIZE-1) ) ) uc_data = 0x99;
			else if ( ( x == (MAP_X_SIZE-1) ) && ( y == (MAP_Y_SIZE-1) ) ) uc_data = 0x33;
			else if ( x == 0 ) uc_data = 0x88;
			else if ( x == (MAP_X_SIZE-1) ) uc_data = 0x22;
			else if ( y == 0 ) uc_data = 0x44;
			else if ( y == (MAP_Y_SIZE-1) ) uc_data = 0x11;
			g_sysMap[y][x] = uc_data;

		}
	}
}


// *************************************************************************
//   �@�\		�F ���H���W���[��������������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_init( void )
{
	/* ���W�A�����A���H���������� */
	en_Head		= NORTH;
	mx			= 0;
	my			= 0;
	MAP_clearMap();
	
	/* ���s�p�̃p�����[�^ */
	f_MoveBackDist = 0;
	uc_SlaCnt = 0;
	
	/* �o�b�N�A�b�v���H�̕��A */
//	Storage_Load( (const void*)uc_back, sizeof(uc_back), ADR_MAP );				// �o�b�N�A�b�v�f�[�^�𕜋A����
	if( ( uc_back[0][0] & 0xf0 ) == 0xf0  ){									// �f�[�^����������
		
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�_�@���@�^");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�@�^�P�_�@�@�^�P�P�P�P�P�P�P�P�P");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@���i� �� ߁j���@���H�f�[�^��������");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�@�_�Q�^�@�@�_�Q�Q�Q�Q�Q�Q�Q�Q�Q");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�^�@���@�_");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@ �� �ȁ@�ȁ� �^�P�P�P�P�P�P�P�P�P�P");
		printf("\n\r�@�P�P�P�P�P�P�P�P�_ ���� �ȁ� �_�i ߁�߁j���@���A�����A�����A���I");
		printf("\n\r�@�@���A���`�`�`�I ���i߁�߁j/ �@�b�@�@�@/�@�_�Q�Q�Q�Q�Q�Q�Q�Q�Q�Q");
		printf("\n\r�@�Q�Q�Q�Q�Q�Q�Q�Q�^ �b�@�@�q�@�@�b�@�@ �b");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@ /�@�^�__�v�@/�@�^�_�v");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@ �P�@�@�@�@ / �^");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@  �P");

		//MAP_LoadMapData();		// ���H�f�[�^�𕜋A
	}
}


// *************************************************************************
//   �@�\		�F �o�b�N�A�b�v���H�������f�[�^�ɔ��f����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_LoadMapData( void )
{
	SHORT	i;
	USHORT	*map_add;
	map_add	= (USHORT*)&g_sysMap;

	// �}�b�v�f�[�^��RAM�ɃR�s�[
	for(i=0; i<128; i++){
		FLASH_Read( (USHORT*)(ADR_MAP+i*2), map_add );
		map_add++;
	}
}


// *************************************************************************
//   �@�\		�F �o�b�N�A�b�v���H�����o�b�N�A�b�v����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.09.8			TKR			�V�K
// *************************************************************************/
PUBLIC void MAP_SaveMapData( void )
{

	SHORT	i;
	USHORT	*map_add;
	map_add	= (USHORT*)g_sysMap;

	// �f�[�^�t���b�V���̃C���[�X
	for( i=0; i<8; i++ ){
		FLASH_Erase((ULONG)(ADR_MAP+i*32));
	}

	// MAP�f�[�^���f�[�^�t���b�V���ɏ�������
	for( i=0; i<128; i++){
		FLASH_WriteEE((ULONG)(ADR_MAP+i*2),map_add);
		map_add++;
	}

}


// *************************************************************************
//   �@�\		�F �o�b�N�A�b�v���H�����N���A����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_ClearMapData( void )
{
	SHORT	i;
	
	/* ���W�A�����A���H���������� */
	en_Head		= NORTH;
	mx			= 0;
	my			= 0;
	MAP_clearMap();		// RAM�̕����N���A
	
	/* ���s�p�̃p�����[�^ */
	f_MoveBackDist = 0;
	uc_SlaCnt = 0;
	
	/* �f�[�^�t���b�V���C���[�X */
	for( i=0; i<8; i++){
		FLASH_Erase((ULONG)(ADR_MAP+i*32));
	}
}


// *************************************************************************
//   �@�\		�F �o�b�N�A�b�v���H�f�[�^��\������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_showLog_BackUp( void )
{
	volatile SHORT	x,y;
	CHAR	c_data;
	
	memset( uc_back, 0, sizeof(uc_back) );				// ����
	
	/* ------------------ */
	/*  �o�b�N�A�b�v���H  */
	/* ------------------ */
	printf("\n\r  /* ------------------ */   ");
	printf("\n\r  /*  �o�b�N�A�b�v���H  */   ");
	printf("\n\r  /* ------------------ */   ");
//	Storage_Load( (const void*)uc_back, sizeof(uc_back), ADR_MAP );			// �f�[�^���[�h(dummy) ���ꂪ�Ȃ��Ǝ��̂P���ڂ�save�����܂������Ȃ�
	
	/* �G���[�`�F�b�N */
	if( ( uc_back[0][0] & 0xf0 ) != 0xf0  ){
		
		printf("\n\r  �\�������񂪂���܂���   ");
		printf("\n\r");
		
		return;		// ���s����
	}
	
	printf("\n\r     ");

	
	for( x=0; x<MAP_X_SIZE; x++){
		printf("._");
	}
	printf(".\n\r");
	
	for( y=MAP_Y_SIZE-1; y>-1; y-- ){
		
		printf("   %2d",y);
		for( x=0; x<MAP_X_SIZE; x++){
			c_data = (UCHAR)uc_back[y][x];
			if ( ( c_data & 0x08 ) == 0 ){
				printf(".");
			}
			else{
				printf("|");
			}
			if ( ( c_data & 0x04 ) == 0 ){
				printf(" ");
			}
			else{
				printf("_");
			}
		}
		printf("|   ");
		
		for( x=0; x<MAP_X_SIZE; x++ ){
			c_data = uc_back[y][x];
			c_data = (UCHAR)( (c_data >> 4) & 0x0f );
			printf("%x", c_data);
		}
		
		printf("\n\r");
	}
	
	printf("     ");
	for( x=0; x<MAP_X_SIZE; x++){
		printf("%2d",x%10);
	}
	printf("\n\r");
}


// *************************************************************************
//   �@�\		�F ���W�ʒu��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F x���W�Ay���W�A�����Ă������
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_setPos( UCHAR uc_x, UCHAR uc_y, enMAP_HEAD_DIR en_dir )
{
	mx			= uc_x;
	my			= uc_y;
	en_Head		= en_dir;
	
	MAP_setCmdPos( uc_x, uc_y, en_dir );
}


// *************************************************************************
//   �@�\		�F ���H�f�[�^��\������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_showLog( void )
{
	SHORT	x, y;
	CHAR	c_data;

	/* ---------- */
	/*  �ʏ���H  */
	/* ---------- */
	printf("\n\r  /* ---------- */   ");
	printf("\n\r  /*  �ʏ���H  */   ");
	printf("\n\r  /* ---------- */   ");

	printf("\n\r     ");
	for (x = 0; x < MAP_X_SIZE; x++) {
		printf("._");
	}
	printf(".\n\r");

	for (y = MAP_Y_SIZE - 1; y > -1; y--) {

		printf("   %2d", y);
		for (x = 0; x < MAP_X_SIZE; x++) {
			c_data = (UCHAR)g_sysMap[y][x];
			if ((c_data & 0x08) == 0) {
				printf(".");
}
			else {
				printf("|");
			}
			if ((c_data & 0x04) == 0) {
				printf(" ");
			}
			else {
				printf("_");
			}
		}
		printf("|   ");

		for (x = 0; x < MAP_X_SIZE; x++) {
			c_data = g_sysMap[y][x];
			c_data = (UCHAR)((c_data >> 4) & 0x0f);
			printf("%x", c_data);
		}

		printf("\n\r");
	}

	printf("     ");
	for (x = 0; x < MAP_X_SIZE; x++) {
		printf("%2d", x % 10);
	}
	printf("\n\r");
}


// *************************************************************************
//   �@�\		�F �Z���T��񂩂�Ǐ����擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ǐ��
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PRIVATE UCHAR MAP_getWallData( void )
{
	UCHAR	uc_wall;
	LED_offAll();	// debug

	/* �Z���T��񂩂�Ǐ��쐬 */
	uc_wall = 0;
	if( TRUE == DIST_isWall_FRONT() ){
		uc_wall = uc_wall | 0x11;
		LED_on(LED2);				// debug
	}
	if( TRUE == DIST_isWall_L_SIDE() ){
		uc_wall = uc_wall | 0x88;
		LED_on(LED0);			// debug
	}
	if( TRUE == DIST_isWall_R_SIDE() ){
		uc_wall = uc_wall | 0x22;
		LED_on(LED4);			// debug
	}

	/* �}�E�X�̐i�s�����ɂ��킹�ăZ���T�f�[�^���ړ����ǃf�[�^�Ƃ��� */
	if		( en_Head == EAST ){
		uc_wall = uc_wall >> 3;
	}
	else if ( en_Head == SOUTH ){
		uc_wall = uc_wall >> 2;
	}
	else if ( en_Head == WEST ){
		uc_wall = uc_wall >> 1;
	}

	/*	�T���ς݃t���O�𗧂Ă� */
	return ( uc_wall | 0xf0 );
}


// *************************************************************************
//   �@�\		�F �Ǐ����쐬����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PRIVATE void MAP_makeMapData( void )
{
	UCHAR uc_wall;

	/*	���s���̕Ǐ�����H���ɏ��� */
	if ( ( mx == 0 ) && ( my == 0 ) ){
		uc_wall = 0xfe;
	}
	else{
		uc_wall = MAP_getWallData();
	}
	g_sysMap[my][mx] = uc_wall;

	/*	�ׂ̋�Ԃ̂l�`�o�f�[�^���X�V���� */
	if ( mx != (MAP_X_SIZE-1) ){
		g_sysMap[my][mx+1] = ( g_sysMap[my][mx+1] & 0x77 ) | 0x80 | ( ( uc_wall << 2 ) & 0x08 );
	}
	if ( mx !=  0 ){
		g_sysMap[my][mx-1] = ( g_sysMap[my][mx-1] & 0xdd ) | 0x20 | ( ( uc_wall >> 2 ) & 0x02 );
	}
	if ( my != (MAP_Y_SIZE-1) ){
		g_sysMap[my+1][mx] = ( g_sysMap[my+1][mx] & 0xbb ) | 0x40 | ( ( uc_wall << 2 ) & 0x04 );
	}
	if ( my !=  0 ){
		g_sysMap[my-1][mx] = ( g_sysMap[my-1][mx] & 0xee ) | 0x10 | ( ( uc_wall >> 2 ) & 0x01 );
	}
}

// *************************************************************************
//   �@�\		�F �������}�b�v���쐬����i�T���j
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_makeContourMap_search( void )
{
	USHORT		x, y;			// ���[�v�ϐ�
	UCHAR		uc_dase;		// ��l
	UCHAR		uc_new;			// �V�l
	UCHAR		uc_level;		// ������
	UCHAR		uc_wallData;	// �Ǐ��

	/* �������}�b�v���쐬 */
	uc_dase = 0;
	do{
		uc_level = 0;
		uc_new = uc_dase + 1;
		for ( y = 0; y < MAP_Y_SIZE; y++ ){
			for ( x = 0; x < MAP_X_SIZE; x++ ){
				if ( us_cmap[y][x] == uc_dase ){
					uc_wallData = g_sysMap[y][x];
					
					if ( ( ( uc_wallData & 0x01 ) == 0x00 ) && ( y != (MAP_Y_SIZE-1) ) ){
						if ( us_cmap[y+1][x] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y+1][x] = uc_new;
							uc_level++;
						}
					}
					if ( ( ( uc_wallData & 0x02 ) == 0x00 ) && ( x != (MAP_X_SIZE-1) ) ){
						if ( us_cmap[y][x+1] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y][x+1] = uc_new;
							uc_level++;
						}
					}
					if ( ( ( uc_wallData & 0x04 ) == 0x00 ) && ( y != 0 ) ){
						if ( us_cmap[y-1][x] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y-1][x] = uc_new;
							uc_level++;
						}
					}
					if ( ( ( uc_wallData & 0x08 ) == 0x00 ) && ( x != 0 ) ){
						if ( us_cmap[y][x-1] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y][x-1] = uc_new;
							uc_level++;
						}
					}
				}
			}
		}
		uc_dase = uc_dase + 1;
	}
	while( uc_level != 0 );
	
#if 0
	/* debug */
	for( x=0; x<4; x++ ){
		
		for( y=0; y<4; y++ ){
			us_Log[y][x][us_LogPt] = us_cmap[y][x];
		}
	}
	us_LogPt++;
#endif
}


// *************************************************************************
//   �@�\		�F �������}�b�v���쐬����i�ŒZ�j
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_makeContourMap_bestWay( void )
{
	USHORT		x, y;			// ���[�v�ϐ�
	UCHAR		uc_dase;		// ��l
	UCHAR		uc_new;			// �V�l
	UCHAR		uc_level;		// ������
	UCHAR		uc_wallData;	// �Ǐ��

	/* �������}�b�v���쐬 */
	uc_dase = 0;
	do{
		uc_level = 0;
		uc_new = uc_dase + 1;
		for ( y = 0; y < MAP_Y_SIZE; y++ ){
			for ( x = 0; x < MAP_X_SIZE; x++ ){
				if ( us_cmap[y][x] == uc_dase ){
					uc_wallData = g_sysMap[y][x];
					
					if ( ( ( uc_wallData & 0x11 ) == 0x10 ) && ( y != (MAP_Y_SIZE-1) ) ){
						if ( us_cmap[y+1][x] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y+1][x] = uc_new;
							uc_level++;
						}
					}
					if ( ( ( uc_wallData & 0x22 ) == 0x20 ) && ( x != (MAP_X_SIZE-1) ) ){
						if ( us_cmap[y][x+1] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y][x+1] = uc_new;
							uc_level++;
						}
					}
					if ( ( ( uc_wallData & 0x44 ) == 0x40 ) && ( y != 0 ) ){
						if ( us_cmap[y-1][x] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y-1][x] = uc_new;
							uc_level++;
						}
					}
					if ( ( ( uc_wallData & 0x88 ) == 0x80 ) && ( x != 0 ) ){
						if ( us_cmap[y][x-1] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y][x-1] = uc_new;
							uc_level++;
						}
					}
				}
			}
		}
		uc_dase = uc_dase + 1;
	}
	while( uc_level != 0 );
	
#if 0
	/* debug */
	for( x=0; x<4; x++ ){
		
		for( y=0; y<4; y++ ){
			us_Log[y][x][us_LogPt] = us_cmap[y][x];
		}
	}
	us_LogPt++;
#endif
}


// *************************************************************************
//   �@�\		�F �������}�b�v���쐬����
//   ����		�F �Ȃ�
//   ����		�F �������}�b�v��������������@��4�ʂ肠��B
//   ����		�F �S�[��X���W�A�S�[��Y���W�A�v�Z���@�i�T��or�ŒZ�j
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_makeContourMap( UCHAR uc_goalX, UCHAR uc_goalY, enMAP_ACT_MODE en_type )
{
	USHORT		i;			// ���[�v�ϐ�
//	USHORT		j;			// ���[�v�ϐ�
//	USHORT*		p_adr;		// ���[�v�ϐ�

	/* ------------ */
	/*  ����������  */
	/* ------------ */
	/* �������}�b�v������������@�`����1�` */
	for ( i = 0; i < MAP_SMAP_MAX_VAL; i++ ){
		us_cmap[ i / MAP_Y_SIZE][ i & (MAP_X_SIZE-1) ] = MAP_SMAP_MAX_VAL - 1;
	}
	
#if 0
	/* �������}�b�v������������@�`����2�` */
	LED4(0x01);
	for ( i = 0; i < 32; i++ ){
		for ( j = 0; j < 32; j++ ){
			us_cmap[i][j] = MAP_SMAP_MAX_VAL - 1;
		}
	}
	LED4(0x00);

	/* �������}�b�v������������@�`����3�` */
	LED4(0x01);
	p_adr = &us_cmap[0][0];
	for ( i = 0; i < MAP_SMAP_MAX_VAL; i++ ){
		*p_adr = MAP_SMAP_MAX_VAL - 1;
		p_adr++;
	}
	LED4(0x00);
	
	/* �������}�b�v������������@�`����4�` */
	LED4(0x01);
	p_adr = &us_cmap[0][0];
	p_adrEnd = &us_cmap[31][31];
	while(1){
		
		if( p_adr == p_adrEnd ) break;
		*p_adr = MAP_SMAP_MAX_VAL - 1;
		p_adr++;
	}
	LED4(0x00);
#endif
	
	/* �ڕW�n�_�̓�������0�ɐݒ� */
	us_cmap[uc_goalY][uc_goalX] = 0;
	
	
	/* ------------ */
	/*  �������쐬  */
	/* ------------ */
	/* �T�����s */
	if( SEARCH == en_type ){
		MAP_makeContourMap_search();
	}
	/* �ŒZ���s */
	else{
		MAP_makeContourMap_bestWay();
	}
}


// *************************************************************************
//   �@�\		�F �}�E�X�̐i�s���������肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F [in] �v�Z���@�A[out] �i�s����
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PRIVATE void MAP_calcMouseDir( enMAP_SEARCH_TYPE en_calcType, enMAP_HEAD_DIR* p_head )
{
	UCHAR		uc_wall;				// �Ǐ��
	USHORT		us_base;				// �������D��x����l
	USHORT		us_new;
	enMAP_HEAD_DIR	en_tmpHead;

	/* ========== */
	/*  �����v�Z  */
	/* ========== */
	/* ������MAP�@ */
	if( CONTOUR_SYSTEM == en_calcType ){
		// ���ӂ�4���ň�ԖړI�n�ɋ߂��ړ��������Z�o����B
		// �������A�ړ��ł����ԋ߂���Ԃ���������ꍇ�ɂ́A���̏��őI������B
		// �@���T�����,���i �A���T�����,���� �B���T�����,���i �C���T�����,����
		uc_wall = g_sysMap[my][mx];
		us_base = MAP_SMAP_MAX_PRI_VAL;					// x[���]�~y[���]�~4[����]

		/* 4�������r */
		//	�k�����̋��̊m�F
		if ( ( uc_wall & 1 ) == 0 ){
			us_new = us_cmap[my+1][mx] * 4 + 4;
			if ( ( g_sysMap[my+1][mx] & 0xf0 ) != 0xf0 ) us_new = us_new - 2;
			if ( en_Head == NORTH ) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = NORTH;
			}
		}
		//	�������̋��̊m�F
		if ( ( uc_wall & 2 ) == 0 ){
			us_new = us_cmap[my][mx+1] * 4 + 4;
			if ( ( g_sysMap[my][mx+1] & 0xf0 ) != 0xf0 ) us_new = us_new - 2;
			if ( en_Head == EAST) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = EAST;
			}
		}
		//	������̋��̊m�F
		if ( ( uc_wall & 4 ) == 0 ){
			us_new = us_cmap[my-1][mx] * 4 + 4;
			if ( ( g_sysMap[my-1][mx] & 0xf0 ) != 0xf0) us_new = us_new - 2;
			if ( en_Head == SOUTH ) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = SOUTH;
			}
		}
		//	�������̋��̊m�F
		if ( ( uc_wall & 8 ) == 0 ){
			us_new = us_cmap[my][mx-1] * 4 + 4;
			if ( ( g_sysMap[my][mx-1] & 0xf0 ) != 0xf0 ) us_new = us_new - 2;
			if ( en_Head == WEST ) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = WEST;
			}
		}
		
		*p_head = (enMAP_HEAD_DIR)( (en_tmpHead - en_Head) & 3 );		// �ړ�����
	}
	// ������@�w��Ȃ�
	else{
		*p_head = (enMAP_HEAD_DIR)0;
	}
}


// *************************************************************************
//   �@�\		�F �}�E�X�̍��W�ʒu���X�V����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �i�s����
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PRIVATE void MAP_refMousePos( enMAP_HEAD_DIR en_head )
{
	switch( en_head ){

		case NORTH:
			my = my + 1;
			break;

		case EAST:
			mx = mx + 1;
			break;

		case SOUTH:
			my = my - 1;
			break;

		case WEST:
			mx = mx - 1;
			break;

		default:
			break;
	}
}


// *************************************************************************
//   �@�\		�F ���̋��Ɉړ�����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ���ΐi�s�����i�}�E�X�i�s������k�Ƃ��Ă���j�A�O�i��ԁiFALSE: �P��ԑO�i��ԁATURE:����ԑO�i��ԁj
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PRIVATE void MAP_moveNextBlock( enMAP_HEAD_DIR en_head, BOOL* p_type )
{
	*p_type = TRUE;
	f_MoveBackDist = 0;

	/* ���� */
	switch( en_head ){

		/* ���̂܂ܑO�i */
		case NORTH:
			*p_type = FALSE;
			MOT_goBlock_Const( 1 );				// 1���O�i
			break;

		/* �E�ɐ��񂷂� */
		case EAST:
			if( uc_TurnCnt < MAP_TURN_NUM_MAX ){
				MOT_goBlock_FinSpeed( 0.5, 0 );		// �����O�i
				TIME_wait( MAP_TURN_WAIT );
				MOT_turn(MOT_R90);					// �E90�x����
				TIME_wait( MAP_TURN_WAIT );
				uc_TurnCnt++;
			
			}else{
				MOT_goBlock_FinSpeed( 0.5, 0 );		// �����O�i
				TIME_wait( MAP_TURN_WAIT );
				MOT_turn(MOT_R90);					// �E90�x����
				TIME_wait( MAP_TURN_WAIT );
				uc_TurnCnt = 0;
				f_MoveBackDist = 0;
				
				/* �ǂ��Ďp������ �i���ɕǂ���������o�b�N�{�ړ����������Z����j*/
				if( ( (en_Head == NORTH)&&( (g_sysMap[my][mx] & WALL_WEST ) != 0 ) ) ||
				    ( (en_Head == EAST )&&( (g_sysMap[my][mx] & WALL_NORTH) != 0  ) ) ||
				    ( (en_Head == WEST )&&( (g_sysMap[my][mx] & WALL_SOUTH) != 0 ) ) ||
			      	( (en_Head == SOUTH)&&( (g_sysMap[my][mx] & WALL_EAST ) != 0 ) ) ){
						
					MOT_goHitBackWall();					// �o�b�N����
					f_MoveBackDist = MOVE_BACK_DIST;		// �o�b�N�������̈ړ�����[���]�����Z
					TIME_wait( 100 );						// ���ԑ҂�
				}
				*p_type = true;
			}		
			break;

		/* ���ɐ��񂷂� */
		case WEST:
			if( uc_TurnCnt < MAP_TURN_NUM_MAX ){
				MOT_goBlock_FinSpeed( 0.5, 0 );		// �����O�i
				TIME_wait( MAP_TURN_WAIT );
				MOT_turn(MOT_L90);									// �E90�x����
				TIME_wait( MAP_TURN_WAIT );
				uc_TurnCnt++;
				
			}else{
				MOT_goBlock_FinSpeed( 0.5, 0 );		// �����O�i
				TIME_wait( MAP_TURN_WAIT );
				MOT_turn(MOT_L90);									// �E90�x����
				TIME_wait( MAP_TURN_WAIT );
				uc_TurnCnt = 0;
				f_MoveBackDist = 0;
				
				/* �ǂ��Ďp������ �i�E�ɕǂ���������o�b�N�{�ړ����������Z����j*/
				if( ( (en_Head == NORTH)&&( (g_sysMap[my][mx] & WALL_EAST ) != 0 ) ) ||
				    ( (en_Head == EAST )&&( (g_sysMap[my][mx] & WALL_SOUTH) != 0  ) ) ||
				    ( (en_Head == WEST )&&( (g_sysMap[my][mx] & WALL_NORTH) != 0 ) ) ||
			      	( (en_Head == SOUTH)&&( (g_sysMap[my][mx] & WALL_WEST ) != 0 ) ) ){

					MOT_goHitBackWall();					// �o�b�N����
					f_MoveBackDist = MOVE_BACK_DIST;		// �o�b�N�������̈ړ�����[���]�����Z
					TIME_wait( 100 );						// ���ԑ҂�
				}
				*p_type = true;
			}
			break;

		/* ���]���Ė߂� */
		case SOUTH:
			MOT_goBlock_FinSpeed( 0.5, 0 );		// �����O�i
			TIME_wait( MAP_TURN_WAIT );
			MOT_turn(MOT_R180);									// �E180�x����
			TIME_wait( MAP_TURN_WAIT );
			uc_TurnCnt 			= 0;
			f_MoveBackDist 		= 0;

			/* �Ǔ��Ďp������i���ɕǂ���������o�b�N�{�ړ����������Z����j */
			if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & WALL_NORTH ) != 0 ) )  ||		// �k�������Ă��Ėk�ɕǂ�����
				( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & WALL_EAST  ) != 0 ) )  ||		// ���������Ă��ē��ɕǂ�����
				( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & WALL_SOUTH ) != 0 ) )  ||		// ��������Ă��ē�ɕǂ�����
				( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & WALL_WEST  ) != 0 ) ) 			// ���������Ă��Đ��ɕǂ�����
			){
				MOT_goHitBackWall();					// �o�b�N����
				f_MoveBackDist = MOVE_BACK_DIST;		// �o�b�N�������̈ړ�����[���]�����Z
				TIME_wait( MAP_TURN_WAIT );				// ���ԑ҂�
			}
			break;

		default:
			break;
	}
	
#ifndef POWER_RELESASE
	/* �i�s�����X�V */
//	en_Head = (enMAP_HEAD_DIR)( (en_Head + en_head) & (MAP_HEAD_DIR_MAX-1) );
	en_Head = (enMAP_HEAD_DIR)( ((UCHAR)en_Head + (UCHAR)en_head) & (MAP_HEAD_DIR_MAX-1) );
#else
	/* �O�i���Ƀp���[�����[�X�@�\�������ă��W���[�����Ȃ���΂Ȃ�Ȃ� */
	if( ( TRUE == DCMC_isPowerRelease() ) && ( en_head == NORTH ) ){
		
		MOT_goBack_Const( MOT_BACK_POLE );					// �P�O�̒��܂Ō��
		MAP_makeMapData();									// �ǃf�[�^������H�f�[�^���쐬			�� �����Ńf�[�^�쐬���~�X���Ă���
		MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			// ������MAP�@�Ői�s�������Z�o			�� �����MAP���쐬
		MAP_moveNextBlock(en_head, p_type);					// �����P�x�Ăяo���i���̋��ֈړ��j
	}
	else{
		/* �i�s�����X�V */
		en_Head = (enMAP_HEAD_DIR)( (en_Head + en_head) & (MAP_HEAD_DIR_MAX-1) );
	}
#endif
}


// *************************************************************************
//   �@�\		�F �X�����[���ɂĎ��̋��Ɉړ�����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ���ΐi�s�����i�}�E�X�i�s������k�Ƃ��Ă���j�A
//				   �O�i��ԁiFALSE: �P��ԑO�i��ԁATURE:����ԑO�i��ԁj
//				   ���W���[���w��iFALSE: ���W���[������ł͂Ȃ��ATURE:���W���[������j
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PRIVATE void MAP_moveNextBlock_Sura( enMAP_HEAD_DIR en_head, BOOL* p_type, BOOL bl_resume )
{
	*p_type = FALSE;
	
	/* ���� */
	switch( en_head ){

		/* ���̂܂ܑO�i */
		case NORTH:
			
			/* ���W���[������ł͂Ȃ� */
			if( bl_resume == FALSE ){
		
				MOT_goBlock_Const( 1 );					// 1���O�i
				uc_SlaCnt = 0;							// �X�����[�����Ă��Ȃ�
			}
			/* ���W���[������ */
			else{
				MOT_goBlock_FinSpeed( 1.0f, MAP_SEARCH_SPEED );		// �����O�i(�o�b�N�̈ړ��ʂ��܂�)
				uc_SlaCnt = 0;										// �X�����[�����Ă��Ȃ�
			}
			break;

		/* �E�ɃX�����[������ */
		case EAST:
			if( uc_SlaCnt < MAP_SLA_NUM_MAX ){
				MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// �E�X�����[��
				uc_SlaCnt++;
			}
			else{
				MOT_goBlock_FinSpeed( 0.5, 0 );			// �����O�i
				TIME_wait( MAP_TURN_WAIT );
				MOT_turn(MOT_R90);						// �E90�x����
				TIME_wait( MAP_TURN_WAIT );
				uc_SlaCnt = 0;
				/* �Ǔ��Ďp������i���ɕǂ���������o�b�N�{�ړ����������Z����j */
				if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) )  ||		// �k�������Ă��Đ��ɕǂ�����
					( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) )  ||		// ���������Ă��Ėk�ɕǂ�����
					( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// ��������Ă��ē��ɕǂ�����
					( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) ) 			// ���������Ă��ē�ɕǂ�����
				){
					MOT_goHitBackWall();					// �o�b�N����
					f_MoveBackDist = MOVE_BACK_DIST_SURA;	// �o�b�N�������̈ړ�����[���]�����Z
					TIME_wait( MAP_SLA_WAIT );				// ���ԑ҂�
				}
				*p_type = TRUE;							// ���͔���ԁi�{�o�b�N�j���i�߂�
			}
			break;

		/* ���ɃX�����[������ */
		case WEST:
			if( uc_SlaCnt < MAP_SLA_NUM_MAX ){
				MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );	// ���X�����[��
				uc_SlaCnt++;
			}
			else{
				
				MOT_goBlock_FinSpeed( 0.5, 0 );		// �����O�i
				TIME_wait( MAP_TURN_WAIT );
				MOT_turn(MOT_L90);					// ��90�x����
				TIME_wait( MAP_TURN_WAIT );
				uc_SlaCnt = 0;
				/* �Ǔ��Ďp������i���ɕǂ���������o�b�N�{�ړ����������Z����j */
				if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// �k�������Ă��ē��ɕǂ�����
					( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) )  ||		// ���������Ă��ē�ɕǂ�����
					( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) )  ||		// ��������Ă��Đ��ɕǂ�����
					( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) ) 			// ���������Ă��Ėk�ɕǂ�����
				){
					MOT_goHitBackWall();					// �o�b�N����
					f_MoveBackDist = MOVE_BACK_DIST_SURA;	// �o�b�N�������̈ړ�����[���]�����Z
					TIME_wait( MAP_SLA_WAIT );				// ���ԑ҂�
				}
				*p_type = TRUE;							// ���͔���ԁi�{�o�b�N�j���i�߂�
			}
			break;

		/* ���]���Ė߂� */
		case SOUTH:
			MOT_goBlock_FinSpeed( 0.5, 0 );			// �����O�i
			TIME_wait( MAP_SLA_WAIT );
			MOT_turn(MOT_R180);									// �E180�x����
			TIME_wait( MAP_SLA_WAIT );
			uc_SlaCnt = 0;

			/* �Ǔ��Ďp������i���ɕǂ���������o�b�N�{�ړ����������Z����j */
			if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) )  ||		// �k�������Ă��Ėk�ɕǂ�����
				( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// ���������Ă��ē��ɕǂ�����
				( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) )  ||		// ��������Ă��ē�ɕǂ�����
				( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) ) 			// ���������Ă��Đ��ɕǂ�����
			){
				MOT_goHitBackWall();					// �o�b�N����
				f_MoveBackDist = MOVE_BACK_DIST_SURA;	// �o�b�N�������̈ړ�����[���]�����Z
				TIME_wait( MAP_SLA_WAIT );				// ���ԑ҂�
			}
			*p_type = TRUE;								// ���͔���ԁ{�o�b�N���i�߂�
			break;
			
		default:
			break;
	}
	
#ifndef POWER_RELESASE
	/* �i�s�����X�V */
	en_Head = (enMAP_HEAD_DIR)( (en_Head + en_head) & (MAP_HEAD_DIR_MAX-1) );

#else
	/* �O�i���Ƀp���[�����[�X�@�\�������ă��W���[�����Ȃ���΂Ȃ�Ȃ� */
	if( ( TRUE == DCMC_isPowerRelease() ) && ( en_head == NORTH ) ){
		
		TIME_wait(1000);
		MOT_goBack_Const( MOT_BACK_POLE );					// �P�O�̒��܂Ō��
		MAP_makeMapData();									// �ǃf�[�^������H�f�[�^���쐬			�� �����Ńf�[�^�쐬���~�X���Ă���
		MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			// ������MAP�@�Ői�s�������Z�o			�� �����MAP���쐬
		MAP_moveNextBlock_Sura(en_head, p_type, TRUE );		// �����P�x�Ăяo���i���̋��ֈړ��j
	}
	else{
		/* �i�s�����X�V */
		en_Head = (enMAP_HEAD_DIR)( (en_Head + en_head) & (MAP_HEAD_DIR_MAX-1) );
	}
#endif
}


// *************************************************************************
//   �@�\		�F �S�[�����̓���
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PRIVATE void MAP_actGoal( void )
{
	MOT_goBlock_FinSpeed( 0.5, 0 );		// �����O�i
	TIME_wait(500);						// ��~����҂�����
	MOT_turn(MOT_R180);					// �E180�x����
	
	TIME_wait(500);						// ��~����҂�����
	LED_onAll();						

	//MAP_SaveMapData();					// ���H���̃o�b�N�A�b�v
	
	en_Head = (enMAP_HEAD_DIR)( (en_Head + 2) & (MAP_HEAD_DIR_MAX-1) );			//	�i�s�����X�V
}


// *************************************************************************
//   �@�\		�F �S�[�����̓���
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �ڕWx���W�A�ڕWy���W�A�T�����@�A�T������
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
//		v1.1		2019.09.22			TKR				�X�^�[�g���W�ɖ߂�Ȃ��΍�i�R�����g�A�E�g���j
// *************************************************************************/
PUBLIC void MAP_searchGoal( UCHAR uc_trgX, UCHAR uc_trgY, enMAP_ACT_MODE en_type, enSEARCH_MODE en_search )
{
	enMAP_HEAD_DIR	en_head = NORTH;
	BOOL			bl_type = TRUE;			// ���݈ʒu�AFALSE: �P��ԑO�i��ԁATURE:����ԑO�i���
	
	MOT_setTrgtSpeed(MAP_SEARCH_SPEED);		// �ڕW���x
	MOT_setNowSpeed( 0.0f );
	
	/* �S�[�����W�ڎw���Ƃ��͐K���čl�� */
	if( ( uc_trgX == GOAL_MAP_X ) && ( uc_trgY == GOAL_MAP_Y ) ){
		f_MoveBackDist = MOVE_BACK_DIST;		
	}else{
		f_MoveBackDist = 0;
	}

	uc_SlaCnt = 0;

	/* ���H�T�� */
	while(1){
		MAP_refMousePos( en_Head );							// ���W�X�V
		MAP_makeContourMap( uc_trgX, uc_trgY, en_type );	// �������}�b�v�����

		/* �_�~�[�Ǒ}�� */
	/*	
		if( (uc_trgX == GOAL_MAP_X) && (uc_trgY == GOAL_MAP_Y) ){
			g_sysMap[0][0]	= 0x01;
			g_sysMap[0][1]	= 0x04;
		}
	*/	
		if( TRUE == bl_type ){
			MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, MAP_SEARCH_SPEED );		// �����O�i(�o�b�N�̈ړ��ʂ��܂�)
			f_MoveBackDist = 0;	
			LED_onAll();
		}
		MAP_makeMapData();									// �ǃf�[�^������H�f�[�^���쐬
		MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			// ������MAP�@�Ői�s�������Z�o
		
		/* ���̋��ֈړ� */
		if(( mx == uc_trgX ) && ( my == uc_trgY )){

			/* �_�~�[�Ǎ폜 */
		/*	
			if( (uc_trgX == GOAL_MAP_X) && (uc_trgY == GOAL_MAP_Y) ){
				g_sysMap[0][0]	&= ~0x01;
				g_sysMap[0][1]	&= ~0x04;
			}
		*/
			MAP_actGoal();		// �S�[�����̓���
			return;				// �T���I��
		}
		else{
			/* ���M�n����T�� */
			if( SEARCH_TURN == en_search ){
				MAP_moveNextBlock(en_head, &bl_type);				// ���̋��ֈړ�	�� �����ŉ��߂ă����[�X�`�F�b�N�{�Ǎēx�쐬�{�������{���M�n���񓮍�
			}
			/* �X�����[���T�� */
			else if( SEARCH_SURA == en_search ){
				MAP_moveNextBlock_Sura(en_head, &bl_type, FALSE );	// ���̋��ֈړ�	�� �����ŉ��߂ă����[�X�`�F�b�N�{�Ǎēx�쐬�{�������{���M�n���񓮍�
			}
		}
#if 0		
		/* �r���Ő���s�\�ɂȂ��� */
		if( SYS_isOutOfCtrl() == TRUE ){
			
			/* ���H�֘A�������� */
			en_Head		= NORTH;
			mx			= 0;
			my			= 0;
			f_MoveBackDist = 0;
			
			// DCMC�͉��ʃ��W���[���Ŋ��ɃN���A�Ƌً}��~���s���Ă���B
			break;
		}
#endif		
	}

	TIME_wait(10000);
}


// *************************************************************************
//   �@�\		�F PCIF�A���݂̃}�b�v��\������
//   ����		�F �Ȃ�
//   ����		�F PCIF������s����
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_showMap_ForPCIF( void )
{
	MAP_showLog();
//	MAP_showLog_BackUp();			// ��
}


// *************************************************************************
//   �@�\		�F �ʒu���X�V����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ����R�}���h
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PRIVATE void MAP_refPos( UCHAR uc_cmd )
{
	UCHAR uc_index = 0;			// �e�[�u���̃C���f�b�N�X�ԍ�
	
	/* ------------------------------------------ */
	/*  �R�}���h����e�[�u���̃C���f�b�N�X���擾  */
	/* ------------------------------------------ */
	/* ���i */
	if( ( uc_cmd <=  GO71 ) && ( uc_cmd >=  GO1) ){
		
		uc_index = 14;		// st_PosData�e�[�u���̒��i�̃C���f�b�N�X�ԍ�
	}
	/* �΂ߒ��i */
	else if( ( uc_cmd <=  NGO71 ) && ( uc_cmd >=  NGO1) ){
		
		uc_index = 15;		// st_PosData�e�[�u���̎΂ߒ��i�̃C���f�b�N�X�ԍ�
	}
	/* ���̑��̃R�}���h */
	else{
		while(1){
			
			if( st_PosData[uc_index].en_cmd == uc_cmd )      break;			// �R�}���h����
			if( st_PosData[uc_index].en_cmd == MAP_CMD_MAX ) return;		// �R�}���h������
			uc_index++;
		}
	}
	
	/* �ʒu�X�V */
	switch( s_PosDir ){
		
		/* [0]�k [1]�k�� */
		case 0:
		case 1:
		
			/* ���i */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x0_x1 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y0_y1 * uc_cmd;
			}
			/* �΂ߒ��i */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x0_x1 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y0_y1 * ( uc_cmd - 81 );
			}
			/* ���̑��̃J�[�u */
			else{
				f_PosX += st_PosData[uc_index].f_x0_x1;
				f_PosY += st_PosData[uc_index].f_y0_y1;
			}
			break;
		
		/* [2]�� [3]�쓌 */
		case 2:
		case 3:

			/* ���i */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x2_x3 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y2_y3 * uc_cmd;
			}
			/* �΂ߒ��i */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x2_x3 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y2_y3 * ( uc_cmd - 81 );
			}
			/* ���̑��̃J�[�u */
			else{
				f_PosX += st_PosData[uc_index].f_x2_x3;
				f_PosY += st_PosData[uc_index].f_y2_y3;
			}
			break;

		/* [4]�� [5]�쐼 */
		case 4:
		case 5:

			/* ���i */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x4_x5 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y4_y5 * uc_cmd;
			}
			/* �΂ߒ��i */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x4_x5 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y4_y5 * ( uc_cmd - 81 );
			}
			/* ���̑��̃J�[�u */
			else{
				f_PosX += st_PosData[uc_index].f_x4_x5;
				f_PosY += st_PosData[uc_index].f_y4_y5;
			}
			break;

		/* [6]�� [7]�k�� */
		case 6:
		case 7:

			/* ���i */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x6_x7 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y6_y7 * uc_cmd;
			}
			/* �΂ߒ��i */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x6_x7 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y6_y7 * ( uc_cmd - 81 );
			}
			/* ���̑��̃J�[�u */
			else{
				f_PosX += st_PosData[uc_index].f_x6_x7;
				f_PosY += st_PosData[uc_index].f_y6_y7;
			}
			break;
	}
	
	/* �i�s�����X�V */
	s_PosDir += st_PosData[uc_index].s_dir;
	if( s_PosDir < 0 ) s_PosDir += 8;				// [0]�`[7]�ɂ�����
	else if( s_PosDir > 7 ) s_PosDir -= 8;
	
	f_LogPosX[us_LogIndex] = f_PosX;
	f_LogPosY[us_LogIndex] = f_PosY;
	
	us_LogIndex++;
	us_LogIndex %= 30;
}


// *************************************************************************
//   �@�\		�F �R�[�i�[�O�ɕǂ���������ǂ̐؂�ڕ␳���s���ݒ������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ����R�}���h
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PRIVATE BOOL MAP_setWallCut( UCHAR uc_cmd )
{
	UCHAR uc_val = 0;			// 1���O�̃R�[�i�[���̕ǂ����邩�i0�ȊO�Ȃ�ǂ���j
	UCHAR uc_valPrev = 0;		// 2���O�̃R�[�i�[���̕ǂ����邩�i0�ȊO�Ȃ�ǂ���j
	BOOL bl_wallCut = FALSE;
	
	/* �ʒu�X�V */
	switch( uc_cmd ){
		
		case R90S:
		case RS135N:
			
			/* 1���O�̃R�[�i�[���̕ǂ����邩�i0�ȊO�Ȃ炠��j */
			// s_PosDir�F�i�s�����i[0]�k [1]�k�� [2]�� [3]�쓌 [4]�� [5]�쐼 [6]�� [7]�k�� �j
			switch( s_PosDir ){
				
				/* ����Ő��񂷂�̂ŁA������O���ǂ̗L���𒲂ׂ������W�ƂȂ�i���ӁFg_sysMap��2�����z��ł��j */
				case 0: 
					if( 0 < f_PosY-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY-0.5)][(UCHAR)(f_PosX)] & 0x02;		// �k�������Ă���̂œ����̕ǂ����邩
					if( 0 < f_PosY-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY-1.5)][(UCHAR)(f_PosX)] & 0x02;		// �k�������Ă���̂œ����̕ǂ����邩
					break;	
				case 2: 
					if( 0 < f_PosX-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-0.5)] & 0x04;		// ���������Ă���̂œ쑤�̕ǂ����邩
					if( 0 < f_PosX-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-1.5)] & 0x04;		// ���������Ă���̂œ쑤�̕ǂ����邩
					break;
				case 4: 
					if( MAP_Y_SIZE_REAL > f_PosY+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY+0.5)][(UCHAR)(f_PosX)] & 0x08;		// ��������Ă���̂Ő����̕ǂ����邩
					if( MAP_Y_SIZE_REAL > f_PosY+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY+1.5)][(UCHAR)(f_PosX)] & 0x08;		// ��������Ă���̂Ő����̕ǂ����邩
					break;
				case 6:
					if( MAP_X_SIZE_REAL > f_PosX+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+0.5)] & 0x01;		// ���������Ă���̂Ŗk���̕ǂ����邩
					if( MAP_X_SIZE_REAL > f_PosX+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+1.5)] & 0x01;		// ���������Ă���̂Ŗk���̕ǂ����邩
					break;
			}
			/* �ǂ����邽�ߕǐ؂�␳���s�� */
			if( ( uc_val != 0 ) || ( ( uc_val != 0 ) && ( uc_valPrev != 0 ) ) ){
				
				MOT_setWallEdgeType( MOT_WALL_EDGE_RIGHT );		// �ǐ؂�␳�����{����
				bl_wallCut = TRUE;
			}
			break;
			
		case L90S:
		case LS135N:
			/* 1���O�̃R�[�i�[���̕ǂ����邩�i0�ȊO�Ȃ炠��j */
			// s_PosDir�F�i�s�����i[0]�k [1]�k�� [2]�� [3]�쓌 [4]�� [5]�쐼 [6]�� [7]�k�� �j
			switch( s_PosDir ){
				
				/* ����Ő��񂷂�̂ŁA������O���ǂ̗L���𒲂ׂ������W�ƂȂ�i���ӁFg_sysMap��2�����z��ł��j */
				case 0: 
					if( 0 < f_PosY-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY-0.5)][(UCHAR)(f_PosX)] & 0x08;			// �k�������Ă���̂Ő����̕ǂ����邩
					if( 0 < f_PosY-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY-1.5)][(UCHAR)(f_PosX)] & 0x08;			// �k�������Ă���̂Ő����̕ǂ����邩
					break;
				case 2: 
					if( 0 < f_PosX-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-0.5)] & 0x01;			// ���������Ă���̂Ŗk���̕ǂ����邩
					if( 0 < f_PosX-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-1.5)] & 0x01;			// ���������Ă���̂Ŗk���̕ǂ����邩
					break;
				case 4: 
					if( MAP_Y_SIZE_REAL > f_PosY+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY+0.5)][(UCHAR)(f_PosX)] & 0x02;			// ��������Ă���̂œ����̕ǂ����邩
					if( MAP_Y_SIZE_REAL > f_PosY+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY+1.5)][(UCHAR)(f_PosX)] & 0x02;			// ��������Ă���̂œ����̕ǂ����邩
					break;
				case 6: 
					if( MAP_X_SIZE_REAL > f_PosX+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+0.5)] & 0x04;			// ���������Ă���̂œ쑤�̕ǂ����邩
					if( MAP_X_SIZE_REAL > f_PosX+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+1.5)] & 0x04;			// ���������Ă���̂œ쑤�̕ǂ����邩
					break;
			}
			/* �ǂ����邽�ߕǐ؂�␳���s�� */
			if( ( uc_val != 0 ) || ( ( uc_val != 0 ) && ( uc_valPrev != 0 ) ) ){
				
				MOT_setWallEdgeType( MOT_WALL_EDGE_LEFT );		// �ǐ؂�␳�����{����
				bl_wallCut = TRUE;
			}
			break;
			
		default:
			break;
	}
	
	return bl_wallCut;
}


// *************************************************************************
//   �@�\		�F �R�}���h���s�p�̍��W�ʒu��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F x���W�Ay���W�A�i�s����
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PRIVATE void MAP_setCmdPos( UCHAR uc_x, UCHAR uc_y, enMAP_HEAD_DIR en_dir )
{
	f_PosX   = (FLOAT)uc_x;
	f_PosX   = (FLOAT)uc_y;
	s_PosDir = (SHORT)(en_dir * 2);	// �i�s�����i[0]�k [1]�k�� [2]�� [3]�쓌 [4]�� [5]�쐼 [6]�� [7]�k�� �j�A2�{����ƒ��x�l�����v����
}


// *************************************************************************
//   �@�\		�F ���H�R�}���h�f�[�^��\������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_showCmdLog( void )
{
	USHORT i=0;
	
	/* ���M�n����R�}���h */
	while(1){
		
		printf("dcom[%4d] = %02d  \n\r",i,dcom[i]);
		if( dcom[i] == CEND ) break;
		i++;
	}
	i=0;
	
	/* �X�����[���R�}���h */
	while(1){
		
		printf("scom[%4d] = %02d  \n\r",i,scom[i]);
		if( scom[i] == CEND ) break;
		i++;
	}
	i=0;

	/* �΂ߑ��s�R�}���h */
	while(1){
		
		printf("tcom[%4d] = %02d  \n\r",i,tcom[i]);
		if( tcom[i] == CEND ) break;
		i++;
	}
}


// *************************************************************************
//   �@�\		�F ���n�M����R�}���h�쐬
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �J�nX���W�A�J�nY���W�A�J�n���̕����A�I��X���W�A�I��Y���W�A[out]�I�����̕���
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_makeCmdList( UCHAR uc_staX, UCHAR uc_staY, enMAP_HEAD_DIR en_staDir, UCHAR uc_endX,	UCHAR uc_endY, enMAP_HEAD_DIR* en_endDir )
{
	UCHAR			uc_goStep;									// �O�i�̃X�e�b�v��
	USHORT			us_high;									// �������̍���
	USHORT			us_pt;										// �R�}���h�|�C���^
	enMAP_HEAD_DIR	en_nowDir;									// ���݃}�E�X�̌����Ă����Ε���
	enMAP_HEAD_DIR	en_tempDir;									// ���Ε���
//	USHORT			i;											// roop
	
	/* �O�i�X�e�b�v�������������� */
	uc_goStep = 0;
	us_pt = 0;

	/* ���H��񂩂�R�}���h�쐬 */
	while(1){	
		us_high = us_cmap[uc_staY][uc_staX]-1;
		if (en_staDir == NORTH){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else   while(1);
		}else if (en_staDir == EAST){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else   while(1);
		}else if (en_staDir == SOUTH){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else   while(1);
		}else if (en_staDir == WEST){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else   while(1);
		}
		
		en_tempDir = (enMAP_HEAD_DIR)( (en_nowDir - en_staDir) & (enMAP_HEAD_DIR)3 );		// �����X�V
		en_staDir = en_nowDir;

		if (en_tempDir == NORTH){
			uc_goStep = uc_goStep + 2;
		}
		else if (en_tempDir == EAST){
			dcom[us_pt] = uc_goStep;
			dcom[++us_pt] = R90;
			uc_goStep = 2;
			us_pt++;
		}
		else if (en_tempDir == WEST){
			dcom[us_pt] = uc_goStep;
			dcom[++us_pt] = L90;
			uc_goStep = 2;
			us_pt++;
		}
		else{
			dcom[us_pt] = uc_goStep;
			dcom[++us_pt] = R180;
			uc_goStep = 2;
			us_pt++;
		}

		if      (en_nowDir == NORTH) uc_staY = uc_staY + 1;
		else if (en_nowDir == EAST) uc_staX = uc_staX + 1;
		else if (en_nowDir == SOUTH) uc_staY = uc_staY - 1;
		else if (en_nowDir == WEST) uc_staX = uc_staX - 1;
		
		en_staDir = en_nowDir;
		
		if ((uc_staX == uc_endX) &&(uc_staY == uc_endY)) break;
	}
	
	/* ���n�M����p�̃R�}���h���X�g�쐬 */
	dcom[us_pt] = uc_goStep;
	dcom[++us_pt] = STOP;
	dcom[++us_pt] = CEND;
	us_totalCmd = us_pt+1;			// �R�}���h����


	/* �ŏI�I�Ɍ����Ă������ */
	*en_endDir = en_staDir;
	
#if 0
	/* debug */
	for( i = 0; i < us_totalCmd; i++){
		printf("dcom[%4d] = %02d  \n\r",i,dcom[i]);
	}
#endif

}


// *************************************************************************
//   �@�\		�F �X�����[������R�}���h�쐬
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_makeSuraCmdList( void )
{
	USHORT dcom_temp[1024];			// ����撴�M����R�}���h���X�g
	USHORT i=0,j=0;					// roop
	
	/* ���n�M����R�}���h���R�s�[ */
	for( i=0; i<us_totalCmd; i++ ){
		dcom_temp[i] = dcom[i];
	}

	i = 0;

	/* �z�񂪐���R�}���h�����`�F�b�N */
	while(1)
	{
		if( dcom_temp[i] == R90 ){		// �E90��
			dcom_temp[i-1] -= 1;		// 1��O������
			dcom_temp[i+1] -= 1;		// 1��O������
			dcom_temp[i] = R90S;		// �E�X�����[��90��
		}
		else if( dcom_temp[i] == L90 ){	// ��90��
			dcom_temp[i-1] -= 1;		// 1��O������
			dcom_temp[i+1] -= 1;		// 1��O������
			dcom_temp[i] = L90S;		// ���X�����[��90��
		}
		else{
			if( dcom_temp[i] == CEND ){
				break;
			}
		}
		i++;
	}

	i = j = 0;

	/* �X�����[���R�}���h�ϊ� */
	while(1)
	{
		if( dcom_temp[i+1] == CEND ){
			scom[j] = STOP;
			scom[j+1] = CEND;
			break;
		}
		else
		{
			/* �f�[�^���X�g�b�v�R�}���h�������� */
			if( dcom_temp[i] == 0 ){
				i++;
			}
			
			scom[j] = (UCHAR)dcom_temp[i];
			
			i++;
			j++;
		}
	}
	
#if 0
	for( i = 0; i < us_totalCmd; i++)
	{
		printf("scom[%4d] = %02d  \n\r",i,scom[i]);
	}
#endif
}


// *************************************************************************
//   �@�\		�F �΂ߓ���R�}���h�쐬
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v0.9		2013.09.17			�O��			135�x��L�^�[����R�^�[���ɂȂ��Ă����s��C��
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_makeSkewCmdList( void )
{
	USHORT	scom_temp[4096];			// ����撴�M����R�}���h���X�g
	USHORT	i;							// roop
	USHORT	c1, c2, c3, c4;				// �v�Z�p
	USHORT	x;
	USHORT	ct_n=0, ct_st=0;
	USHORT	flag = 0;					//	�΂ߑ��s�p�o�b�t�@  0:�����R�}���h�@1:�΂�  2:S135N �� N135S  3:���i
	
	/* ���n�M����R�}���h���R�s�[ */
	for( i=0; i<us_totalCmd; i++ )
	{
		scom_temp[i] = scom[i];
	}

	i=0;

	/* �z�񂪐���R�}���h�����`�F�b�N */
	while(1)
	{
		c1 = scom_temp[ct_st];
		c2 = scom_temp[ct_st+1];
		c3 = scom_temp[ct_st+2];
		c4 = scom_temp[ct_st+3];

		//	���i �� �E45�x �� �΂�
		if( (c1<=GO32) && (c2==R90S) && (c3==L90S) )
		{
			if( c1-1 != 0 ) tcom[ ct_n++ ] = c1 - 1;		//	�O�̕����R�}���h�ɂ���Ē�����Ԃ������Ȃ��ꍇ
			tcom[ ct_n++ ] = RS45N;
			ct_st ++;

			x = (USHORT)(NGO1 - 1);		//	�΂߃��[�h
			flag = 0;
		}
		//	���i �� ��45�x �� �΂�
		else if( (c1<=GO32) && (c2==L90S) && (c3==R90S) )
		{
			if( c1-1 != 0 ) tcom[ ct_n++ ] = c1 - 1;		//	�O�̕����R�}���h�ɂ���Ē�����Ԃ������Ȃ��ꍇ
			tcom[ ct_n++ ] = LS45N;
			ct_st ++;

			x = (USHORT)(NGO1 - 1);		//	�΂߃��[�h
			flag = 0;
		}

		//	���i �� �E90�x �� ���i
		else if( (c1<=GO32) && (c2==R90S) && (c3<=GO32) )
		{
			tcom[ ct_n++ ] = (UCHAR)c1;
			tcom[ ct_n++ ] = R90S;
			ct_st += 2;
			flag = 3;		//	���i
		}
		//	���i �� ��90�x �� ���i
		else if( (c1<=GO32) && (c2==L90S) && (c3<=GO32) )
		{
			tcom[ ct_n++ ] = (UCHAR)c1;
			tcom[ ct_n++ ] = L90S;
			ct_st += 2;
			flag = 3;		//	���i
		}
		//	���i �� �E135�x �� �΂�
		else if( (c1<=GO32) && (c2==R90S) && (c3==R90S) && (c4==L90S) )
		{
			tcom[ ct_n++ ] = (UCHAR)c1;
			tcom[ ct_n++ ] = RS135N;
			ct_st += 2;

			x = (USHORT)(NGO1 - 1);		//	�΂߃��[�h
			flag = 2;
		}
		//	���i �� ��135�x �� �΂�
		else if( (c1<=GO32) && (c2==L90S) && (c3==L90S) && (c4==R90S) )
		{
			tcom[ ct_n++ ] = (UCHAR)c1;
			tcom[ ct_n++ ] = LS135N;
			ct_st += 2;

			x = (USHORT)(NGO1 - 1);		//	�΂߃��[�h
			flag = 2;
		}

		//	���i �� �E180�x �� ���i
		else if( (c1<=GO32) && (c2==R90S) && (c3==R90S) && (c4<=GO32) )
		{
			tcom[ ct_n++ ] = (UCHAR)c1;
			tcom[ ct_n++ ] = R90S;
			tcom[ ct_n++ ] = R90S;
			ct_st += 3;
			flag = 3;		//	���i
		}
		//	���i �� ��180�x �� ���i
		else if( (c1<=GO32) && (c2==L90S) && (c3==L90S) && (c4<=GO32) )
		{
			tcom[ ct_n++ ] = (UCHAR)c1;
			tcom[ ct_n++ ] = L90S;
			tcom[ ct_n++ ] = L90S;
			ct_st += 3;
			flag = 3;		//	���i
		}

		//	�΂� �� �E45�x �� ���i
		else if( (c1==R90S) && (c2<=GO32) )
		{
			if( flag==1 ) tcom[ ct_n++ ] = (UCHAR)x;
			tcom[ ct_n++ ] = RN45S;
			scom_temp[ct_st+1] = c2 - 1;		//	������Ԃ�1���炷
			ct_st ++;
			flag = 3;		//	���i
		}
		//	�΂� �� ��45�x �� ���i
		else if( (c1==L90S) && (c2<=GO32) )
		{
			if( flag==1 ) tcom[ ct_n++ ] = (UCHAR)x;
			tcom[ ct_n++ ] = LN45S;
			scom_temp[ct_st+1] = c2 - 1;		//	������Ԃ�1���炷
			ct_st ++;
			flag = 3;		//	���i
		}
		//	�΂� �� �E90�x �� �΂�
		else if( (c1==L90S) && (c2==R90S) && (c3==R90S) && (c4==L90S) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45N����RN90N
			else if( flag==1 ) tcom[ ct_n++ ] = (UCHAR)(x+1);
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = RN90N;
			ct_st +=2;

			x = (USHORT)(NGO1 - 1);		//	�΂߃��[�h
			flag = 1;
		}
		//	�΂� �� ��90�x �� �΂�
		else if( (c1==R90S) && (c2==L90S) && (c3==L90S) && (c4==R90S) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45N����LN90N
			else if( flag==1 ) tcom[ ct_n++ ] = x+1;
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = LN90N;
			ct_st +=2;

			x = (USHORT)(NGO1 - 1);		//	�΂߃��[�h
			flag = 1;
		}
		//	�΂� �� �E135�x �� ���i
		else if( (c1==L90S) && (c2==R90S) && (c3==R90S) && (c4<=GO32) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45N����LN90N
			else if( flag==1 ) tcom[ ct_n++ ] = x+1;
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = RN135S;
			ct_st += 3;
			flag = 3;		//	���i
		}
		//	�΂� �� ��135�x �� ���i
		else if( (c1==R90S) && (c2==L90S) && (c3==L90S) && (c4<=GO32) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45N����LN90N
			else if( flag==1 ) tcom[ ct_n++ ] = x+1;
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = LN135S;
			ct_st += 3;
			flag = 3;		//	���i
		}
		//	�΂� �� �΂�
		else if( (c1==R90S) && (c2==L90S) && ( (c3==R90S) || (c3==L90S) || ( c3<=GO32 ) ) )
		{
			x++;
			ct_st ++;

			flag = 1;		//	�΂ߑ��s�o�b�t�@����
		}
		else if( (c1==L90S) && (c2==R90S) && ( (c3==L90S) || (c3==R90S) || ( c3<=GO32 ) ) )
		{
			//	�R�}���h�o��
			x++;
			ct_st ++;

			flag = 1;		//	�΂ߑ��s�o�b�t�@����
		}
		else
		{
			tcom[ ct_n ] = (UCHAR)scom_temp[ct_st];
			if( tcom[ ct_n ] == CEND ) break;
			ct_st ++;
			ct_n ++;
		}
	}
	
#if 0
	for( i = 0; i < us_totalCmd; i++)
	{
		printf("tcom[%4d] = %02d  \n\r",i,tcom[i]);
	}
#endif
}


// *************************************************************************
//   �@�\		�F �R�}���h���s���W���[��
//   ����		�F �Ȃ�
//   ����		�F ����J�n�̍��W�ʒu�́AMAP_setPos()�Őݒ肷��B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_drive( enMAP_DRIVE_TYPE en_driveType )
{
	USHORT				us_rp = 0;				// ���݂̓ǂݍ��݈ʒu
	enMOT_TURN_CMD 		en_type;
	BOOL				bl_isWallCut = FALSE;
	
	/* ���M���񃂁[�h*/
	if( en_driveType == MAP_DRIVE_TURN )
	{
		while(1)
		{
			if ( dcom[us_rp] == CEND  ) break;								//	�R�}���h�I��
			
			else if ( dcom[us_rp] == STOP  ){
				CTRL_stop();			// �����~
				DCM_stopMot( DCM_R );		// ��~
				DCM_stopMot( DCM_L );		// ��~
			}
			else if ( ( dcom[us_rp] <=  GO71 ) && ( dcom[us_rp] >=  GO1) )
			{
				MOT_goBlock_FinSpeed( (FLOAT)dcom[us_rp]*0.5f, 0 );		// �������s�R�}���h�A����ԑO�i��ɒ�~
			}
			else{
				
				if( dcom[us_rp] == R90 ) en_type = MOT_R90;
				else 					 en_type = MOT_L90;
				
				TIME_wait(500);
				MOT_turn( en_type );		//	����
				TIME_wait(500);
			}
			us_rp++;
#if 0			
			/* �r���Ő���s�\�ɂȂ��� */
			if( SYS_isOutOfCtrl() == TRUE ){
				break;
			}
#endif			
		}
		CTRL_stop();			// �����~
		DCM_stopMot( DCM_R );		// ��~
		DCM_stopMot( DCM_L );		// ��~
	}
	/* �X�����[�����[�h */
	else if( en_driveType == MAP_DRIVE_SURA )
	{
		while(1)
		{
			MAP_refPos( scom[us_rp] );									// ���s�����R�}���h���I�������ʒu�ɍX�V

			if ( scom[us_rp] == CEND  ) break;							//	�R�}���h�I��
			
			else if ( scom[us_rp] == STOP  )
			{
				CTRL_stop();			// �����~
				DCM_stopMot( DCM_R );		// ��~
				DCM_stopMot( DCM_L );		// ��~
			}
			else if ( ( scom[us_rp] <=  GO71 ) && ( scom[us_rp] >=  GO1) )
			{
				if( scom[us_rp+1] == STOP  ){
					MOT_goBlock_FinSpeed( (FLOAT)scom[us_rp]*0.5f, 0 );						// �������s�R�}���h�A����ԑO�i�i�ŏI���x�Ȃ��j
				}
				else{
					
					/* �ǂ̐؂�ڕ␳ */
					if( ( scom[us_rp+1] == R90S )   || ( scom[us_rp+1] == L90S ) ){
						bl_isWallCut = MAP_setWallCut( scom[us_rp+1] );		// �R�[�i�[�O�ɕǂ���������ǂ̐؂�ڕ␳���s���ݒ������
						
						if( bl_isWallCut == TRUE ){
							
							bl_isWallCut = FALSE;
							us_LogWallCut[us_LogIndexWallCut] = us_rp;
							us_LogIndexWallCut++;
							us_LogIndexWallCut %= 30;
						}
					}
					MOT_goBlock_FinSpeed( (FLOAT)scom[us_rp]*0.5f, MOT_getSlaStaSpeed() );		// �������s�R�}���h�A����ԑO�i�i�ŏI���x����j
				}
			}
			else if( scom[us_rp] == R90S )
			{
				MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// �E�X�����[��
			}
			else if( scom[us_rp] == L90S )
			{
				MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );	// ���X�����[��
			}
			us_rp++;
#if 0			
			/* �r���Ő���s�\�ɂȂ��� */
			if( SYS_isOutOfCtrl() == TRUE ){
				break;
			}
#endif			
		}
	}
	/* �΂߃��[�h */
	else if( en_driveType == MAP_DRIVE_SKEW )
	{
		while(1)
		{
			MAP_refPos( tcom[us_rp] );									// ���s�����R�}���h���I�������ʒu�ɍX�V
			
			if ( tcom[us_rp] == CEND  ) break;							//	�R�}���h�I��

			else if ( tcom[us_rp] == STOP  )
			{
			 	CTRL_stop();			// �����~
				DCM_stopMot( DCM_R );		// ��~
				DCM_stopMot( DCM_L );		// ��~
			}
			else if ( ( tcom[us_rp] <=  GO71 ) && ( tcom[us_rp] >=  GO1) )
			{
				if( tcom[us_rp+1] == STOP  ){
					MOT_goBlock_FinSpeed( (FLOAT)tcom[us_rp]*0.5f, 0 );						// �������s�R�}���h�A����ԑO�i�i�ŏI���x�Ȃ��j
				}
				else{
					
					/* �ǂ̐؂�ڕ␳ */
					if( ( tcom[us_rp+1] == R90S )   || ( tcom[us_rp+1] == L90S )   || 
					 	( tcom[us_rp+1] == RS135N ) || ( tcom[us_rp+1] == LS135N ) 
					 ){
						bl_isWallCut = MAP_setWallCut( tcom[us_rp+1] );		// �R�[�i�[�O�ɕǂ���������ǂ̐؂�ڕ␳���s���ݒ������
						
						if( bl_isWallCut == TRUE ){
							
							bl_isWallCut = FALSE;
							us_LogWallCut[us_LogIndexWallCut] = us_rp;
							us_LogIndexWallCut++;
							us_LogIndexWallCut %= 30;
						}
					}
					MOT_goBlock_FinSpeed( (FLOAT)tcom[us_rp]*0.5f, MOT_getSlaStaSpeed() );		// �������s�R�}���h�A����ԑO�i�i�ŏI���x����j
				}
			}
			else if ( ( tcom[us_rp] <=  NGO71 ) && ( tcom[us_rp] >=  NGO1) )
			{
				MOT_goSkewBlock_FinSpeed( (FLOAT)(tcom[us_rp]-81)*0.5f, MOT_getSlaStaSpeed());	// �΂ߒ������s�R�}���h�A����ԑO�i�i�ŏI���x����j
			}
			else
			{
				switch( tcom[us_rp] )
				{
					/* ���i �� ���i */
					case R90S:		MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );			break;
					case L90S:		MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );			break;
					
					/* ���i �� �΂� */
					case RS45N:		MOT_goSla( MOT_R45S_S2N, PARAM_getSra( SLA_45 ) ); 		break;
					case LS45N:		MOT_goSla( MOT_L45S_S2N, PARAM_getSra( SLA_45 ) ); 		break;
					case RS135N:	MOT_goSla( MOT_R135S_S2N, PARAM_getSra( SLA_135 ) ); 	break;
					case LS135N:	MOT_goSla( MOT_L135S_S2N, PARAM_getSra( SLA_135 ) ); 	break;

					/* �΂� �� ���i */
					case RN45S:		MOT_goSla( MOT_R45S_N2S, PARAM_getSra( SLA_45 ) ); 		break;
					case LN45S:		MOT_goSla( MOT_L45S_N2S, PARAM_getSra( SLA_45 ) ); 		break;
					case RN135S:	MOT_goSla( MOT_R135S_N2S, PARAM_getSra( SLA_135 ) ); 	break;
					case LN135S:	MOT_goSla( MOT_L135S_N2S, PARAM_getSra( SLA_135 ) ); 	break;

					/* �΂� �� �΂� */
					case RN90N:		MOT_goSla( MOT_R90S_N, PARAM_getSra( SLA_N90 ) ); 		break;
					case LN90N:		MOT_goSla( MOT_L90S_N, PARAM_getSra( SLA_N90 ) );		break;
				}
			}
			us_rp++;
#if 0			
			/* �r���Ő���s�\�ɂȂ��� */
			if( SYS_isOutOfCtrl() == TRUE ){
				break;
			}
#endif			
		}
	}

}


// *************************************************************************
//   �@�\		�F PCIF�A�R�}���h���X�g��\���i���삵�����O�j
//   ����		�F �Ȃ�
//   ����		�F PCIF������s����B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_showCmdList( void )
{
	USHORT i=0;

	/* ���M�n���� */
	printf("dcom \n\r");
	while(1){
		if( dcom[i] == 0 ) break;
		printf("%03d\n\r",dcom[i]);
		if( dcom[i] == CEND ) break;
		i++;

		if( i== LIST_NUM-1 ) break;
	}

	/* �X�����[�� */
	i = 0;
	printf("scom \n\r");
	while(1){
		if( scom[i] == 0 ) break;
		printf("%03d\n\r",scom[i]);
		if( scom[i] == CEND ) break;
		i++;

		if( i== LIST_NUM-1 ) break;
	}

	/* �΂� */
	i = 0;
	printf("tcom \n\r");
	while(1){
		if( tcom[i] == 0 ) break;
		printf("%03d\n\r",tcom[i]);
		if( tcom[i] == CEND ) break;
		i++;
		
		if( i== LIST_NUM-1 ) break;
	}

	/* �T���_�b�V�� */
	i = 0;
	printf("mcom \n\r");
	while(1){
		if( mcom[i] == 0 ) break;
		printf("%03d\n\r",mcom[i]);
		if( mcom[i] == CEND ) break;
		i++;
		
		if( i== LIST_NUM-1 ) break;
	}
	
	printf("[x]%d [y]%d [Head]%d \n\r", mx, my, en_Head);
	
	
	MAP_makeContourMap( 0, 0 , SEARCH);
}


// *************************************************************************
//   �@�\		�F PCIF�AMap����쐬�����R�}���h���X�g��\���i�V�~�����[�V�����j
//   ����		�F �Ȃ�
//   ����		�F PCIF������s����B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_showMakeCmdList( void )
{
	USHORT i=0;
	enMAP_HEAD_DIR		en_endDir;
	
	MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// �������}�b�v�����
	MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir );
	MAP_makeSuraCmdList();
	MAP_makeSkewCmdList();

	/* ���M�n���� */
	printf("dcom \n\r");
	while(1){
		printf("%03d\n\r",dcom[i]);
		if( dcom[i] == CEND ) break;
		i++;
		
		if( i== LIST_NUM-1 ) break;
	}

	/* �X�����[�� */
	i = 0;
	printf("scom \n\r");
	while(1){
		printf("%03d\n\r",scom[i]);
		if( scom[i] == CEND ) break;
		i++;

		if( i== LIST_NUM-1 ) break;
	}

	/* �΂� */
	i = 0;
	printf("tcom \n\r");
	while(1){
		printf("%03d\n\r",tcom[i]);
		if( tcom[i] == CEND ) break;
		i++;

		if( i== LIST_NUM-1 ) break;
	}
}


// *************************************************************************
//   �@�\		�FPCIF for debug
//   ����		�F �Ȃ�
//   ����		�F PCIF������s����B
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PUBLIC void MAP_debug( void )
{
//	USHORT i=0;
//	enMAP_HEAD_DIR		en_endDir;
	
//	UCHAR uc_dummy[ MAP_Y_SIZE ][ MAP_X_SIZE ];			// ���H�f�[�^


	MAP_showLog();
	MAP_showLog_BackUp();





	MAP_clearMap();

//	Storage_Load( (const void*)uc_dummy, sizeof(uc_dummy), ADR_MAP );			// �f�[�^���[�h(dummy) ���ꂪ�Ȃ��Ǝ��̂P���ڂ�save�����܂������Ȃ�

#if 0
	/* �o�b�N�A�b�v���H�̕��A */
	Storage_Load( (const void*)uc_back, sizeof(uc_back), ADR_MAP );				// �o�b�N�A�b�v�f�[�^�𕜋A����
	if( ( uc_back[0][0] & 0xf0 ) == 0xf0  ){									// �f�[�^����������
		
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�_�@���@�^");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�@�^�P�_�@�@�^�P�P�P�P�P�P�P�P�P");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@���i� �� ߁j���@���H�f�[�^��������");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�@�_�Q�^�@�@�_�Q�Q�Q�Q�Q�Q�Q�Q�Q");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�^�@���@�_");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@ �� �ȁ@�ȁ� �^�P�P�P�P�P�P�P�P�P�P");
		printf("\n\r�@�P�P�P�P�P�P�P�P�_ ���� �ȁ� �_�i ߁�߁j���@���A�����A�����A���I");
		printf("\n\r�@�@���A���`�`�`�I ���i߁�߁j/ �@�b�@�@�@/�@�_�Q�Q�Q�Q�Q�Q�Q�Q�Q�Q");
		printf("\n\r�@�Q�Q�Q�Q�Q�Q�Q�Q�^ �b�@�@�q�@�@�b�@�@ �b");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@ /�@�^�__�v�@/�@�^�_�v");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@ �P�@�@�@�@ / �^");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@  �P");
		Storage_Load( (const void*)g_sysMap, sizeof(g_sysMap), ADR_MAP );		// �o�b�N�A�b�v���H�f�[�^�Ō��݂̖��H�����㏑��
	}

#endif

	MAP_SaveMapData();














#if 0
	MAP_makeContourMap( 3, 3, BEST_WAY );					// �������}�b�v�����
	MAP_makeCmdList( 0, 0, NORTH, 3, 3, &en_endDir );
	MAP_makeSuraCmdList();
	MAP_makeSkewCmdList();

	while(1){
		
		printf("dcom[%4d] = %02d  \n\r",i,dcom[i]);

		if( ( dcom[i] == CEND ) || ( i ==500 ) ) break;
		
		i++;
	}

#endif	



#if 0
	printf( "0x%x \n\r", g_sysMap[0][0]);
	printf( "0x%x \n\r", g_sysMap[0][1]);
	printf( "0x%x \n\r", g_sysMap[0][2]);
	printf( "0x%x \n\r", g_sysMap[0][3]);
	printf( "0x%x \n\r", g_sysMap[1][0]);
	printf( "0x%x \n\r", g_sysMap[1][1]);
	printf( "0x%x \n\r", g_sysMap[1][2]);
	printf( "0x%x \n\r", g_sysMap[1][3]);
	printf( "0x%x \n\r", g_sysMap[2][0]);
	printf( "0x%x \n\r", g_sysMap[2][1]);
	printf( "0x%x \n\r", g_sysMap[2][2]);
	printf( "0x%x \n\r", g_sysMap[2][3]);
	printf( "0x%x \n\r", g_sysMap[3][0]);
	printf( "0x%x \n\r", g_sysMap[3][1]);
	printf( "0x%x \n\r", g_sysMap[3][2]);
	printf( "0x%x \n\r", g_sysMap[3][3]);
	MAP_clearMap();
	g_sysMap[0][0] = 0xfe;			us_cmap[0][0] = 10;
	g_sysMap[0][1] = 0xfc;			us_cmap[0][1] = 7;
	g_sysMap[0][2] = 0xf4;			us_cmap[0][2] = 6;
	g_sysMap[0][3] = 0xf6;			us_cmap[0][3] = 5;
	g_sysMap[0][4] = 0xfc;			us_cmap[0][4] = 255;
                                           
	g_sysMap[1][0] = 0xf8;			us_cmap[1][0] = 9;
	g_sysMap[1][1] = 0xf2;			us_cmap[1][1] = 8;
	g_sysMap[1][2] = 0xfb;			us_cmap[1][2] = 7;
	g_sysMap[1][3] = 0xfa;			us_cmap[1][3] = 4;
	g_sysMap[1][4] = 0xf8;			us_cmap[1][4] = 255;
                                           
	g_sysMap[2][0] = 0xfa;			us_cmap[2][0] = 10;
	g_sysMap[2][1] = 0xfa;			us_cmap[2][1] = 9;
	g_sysMap[2][2] = 0xfc;			us_cmap[2][2] = 2;
	g_sysMap[2][3] = 0xf3;			us_cmap[2][3] = 3;
	g_sysMap[2][4] = 0xf8;			us_cmap[2][4] = 255;
                                           
	g_sysMap[3][0] = 0xf9;			us_cmap[3][0] = 11;
	g_sysMap[3][1] = 0xf3;			us_cmap[3][1] = 10;
	g_sysMap[3][2] = 0xf9;			us_cmap[3][2] = 1;
	g_sysMap[3][3] = 0xf7;			us_cmap[3][3] = 0;
	g_sysMap[3][4] = 0xf8;			us_cmap[3][4] = 255;
                                           
	g_sysMap[4][0] = 0xfc;			us_cmap[4][0] = 255;
	g_sysMap[4][1] = 0xf4;			us_cmap[4][1] = 255;
	g_sysMap[4][2] = 0xf4;			us_cmap[4][2] = 255;
	g_sysMap[4][3] = 0xf4;			us_cmap[4][3] = 255;


	MAP_showLog();
	
	
	MAP_makeCmdList( 0, 0, NORTH, 3, 3, EAST );
	MAP_makeSuraCmdList();
	MAP_makeSkewCmdList();
	
#endif
	
	
#if 0
//	UCHAR x,z;

	for( z=0; z<20; z++ ){
		printf("\n\r");
		printf("\n\r");
		printf("[%d] \n\r", z);

		printf("%d ",3);
		for( x=0; x<4; x++ ){
			printf(" %4d ",us_Log[3][x][z]);
		}
		printf("\n\r");

		printf("%d ",2);
		for( x=0; x<4; x++ ){
			printf(" %4d ",us_Log[2][x][z]);
		}
		printf("\n\r");

		printf("%d ",1);
		for( x=0; x<4; x++ ){
			printf(" %4d ",us_Log[1][x][z]);
		}
		printf("\n\r");

		printf("%d ",0);
		for( x=0; x<4; x++ ){
			printf(" %4d ",us_Log[0][x][z]);
		}
		printf("\n\r");
	}
#endif
}




#ifdef __cplusplus
    }
#endif



