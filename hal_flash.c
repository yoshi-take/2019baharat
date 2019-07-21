// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： FLASH
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.3.27			TKR			新規（ファイルのインクルード）
// *************************************************************************

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <iodefine.h>		// I/O
#include <typedefine.h>		// 定義
#include <stdio.h>			// 標準入出力
#include <hal_flash.h>		// FLASH

//**************************************************
// 定義（define）
//**************************************************
#define BASEADDR	0x00100000	// E2データフラッシュの先頭アドレス
#define FLASHSIZE	0x7fff		//32kB

#define BLOCKSIZE	32		//bytes
#define BLOCKCOUNT	1024		//count

#define write_byte(A,D)	*(UCHAR *)(A)=(D)
#define write_word(A,D)	*(USHORT *)(A)=(D)


//**************************************************
// 列挙体（enum）
//**************************************************

//**************************************************
// 構造体（struct）
//**************************************************

//**************************************************
// グローバル変数
//**************************************************
const UCHAR *BaseAddr = (const unsigned char *)BASEADDR;

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
PUBLIC void FLASH_init(){
	
	// 全エリア
	FLASH.DFLRE0.WORD	= 0x2dff;		// データフラッシュの読み出しの許可	
	FLASH.DFLRE1.WORD	= 0xd2ff;		// データフラッシュの読み出しの許可		
	FLASH.DFLWE0.WORD	= 0x1eff;		// データフラッシュの書込み＆消去の許可(P/E)
	FLASH.DFLWE1.WORD	= 0xe1ff;		// データフラッシュの書込み＆消去の許可(P/E)
	
	if(FLASH.FENTRYR.WORD != 0x0000){
		FLASH.FENTRYR.WORD = 0xAA00;	// FCUを停止
	}
	FLASH.FCURAME.WORD = 0xc401;		// FCU RAMアクセス許可

	for(int i=0;i<8192;i++){			// FCU RAMにファームウェアをコピー
		*(unsigned char *)(0x007F8000+i) = *(unsigned char *)(0xFEFFE000+i);
	}

	FLASH_PEMode();		// P/Eモードに移行

	FLASH.PCKAR.BIT.PCKA	= 48;	// フラッシュクロック：48MHz

	//周辺クロック通知コマンド
	write_byte(BaseAddr, 0xE9);
	write_byte(BaseAddr, 0x03);
	write_word(BaseAddr, 0x0F0F);
	write_word(BaseAddr, 0x0F0F);
	write_word(BaseAddr, 0x0F0F);
	write_byte(BaseAddr, 0xD0);

	FLASH_waitFCU(2);		
	FLASH_ReadMode();

}


/* P/Eモードに移行 */
PUBLIC void FLASH_PEMode(){
	FLASH.FENTRYR.WORD	= 0xAA00;			// FCUを停止
	while(0x0000 != FLASH.FENTRYR.WORD);	
	FLASH.FENTRYR.WORD	= 0xAA80;

	FLASH_CheckError();

	FLASH.FWEPROR.BYTE	= 0x01;				// 許可（P/Eロックビットの読み出し，ブランクチェック）
}

/* 読込みモードに遷移 */
PUBLIC void FLASH_ReadMode(){
	FLASH.FENTRYR.WORD	= 0xAA00;
	while(0x0000 != FLASH.FENTRYR.WORD);
	FLASH.FWEPROR.BYTE	= 0x02;				// 許可（データフラッシュリードモード）
}

/* FCU待機時間 */
PUBLIC void FLASH_waitFCU( int timeout ){

	BOOL bl_Timeout	= FALSE;

	TIME_wait(timeout);

	if( FLASH.FSTATR0.BIT.FRDY == 0 ){		// タイムアウト発生していたら
		bl_Timeout = TRUE;
	}

	/* タイムアウトしていたらリセット */
	if(bl_Timeout == TRUE){
		FLASH_FcuReset();
	}

}

/* FCUの初期化 */
PUBLIC void FLASH_FcuReset(){

	FLASH.FRESETR.BIT.FRESET	= 1;
	TIME_wait(2);
	FLASH.FRESETR.BIT.FRESET	= 0;

}

/* イレースを行う */
PUBLIC void FLASH_Erase(ULONG addr){

	volatile UCHAR	*a = (UCHAR *)addr;
	
	FLASH_PEMode();
		
		*a = 0x20;
		*a = 0xD0;
		FLASH_waitFCU(5);
		FLASH_CheckError();
	
	FLASH_ReadMode();

}

/* 指定した領域に書き込み */
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

/* 指定した領域を読み出し */
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


/* エラーを確認 */
PUBLIC void FLASH_CheckError( void ){

	int	iserr	= 0;

	iserr |= FLASH.FSTATR0.BIT.ILGLERR;		//FCUは不正なコマンドや不正やE2データフラッシュアクセスを検出
	iserr |= FLASH.FSTATR0.BIT.ERSERR;		//イレース中にエラー発生
	iserr |= FLASH.FSTATR0.BIT.PRGERR;		//プログラム中にエラー発生

	if(iserr == 0){
		return;
	}

	iserr = 1;
	//LEDデバッグ

	if(FLASH.FSTATR0.BIT.ILGLERR == 1){

		if(FLASH.FASTAT.BYTE != 0x10){
			FLASH.FASTAT.BYTE = 0x10;
		}
	}

	write_byte(BaseAddr,0x50);

}

