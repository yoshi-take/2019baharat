/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :Tue, Oct 31, 2006                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/

//**************************************************
// プロジェクト作成時に生成されたコード （使わないけどとりあえず残しておく）
//**************************************************
#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <stdio.h>							// 標準入出力

#include <hal_led.h>						// LED
#include <mode.h>							// MODE
#include <hal_sci.h>						// SCI
#include <hal_flash.h>						// FLASH
#include <map.h>							// MAP
#include <hal_spi.h>						// SPI
#include <hal_gyro.h>						// ジャイロ
#include <hal_dist.h>						// DIST
#include <hal_dcmCtrl.h>					// CTRL
#include <hal_dist.h>						// DIST							

//**************************************************
// グローバル変数
//**************************************************
PRIVATE	VUSHORT		uc_Msec;	// 内部時計[msec]
PRIVATE	VUCHAR		uc_Sec;		// 内部時計[sec]
PRIVATE	VUCHAR		uc_Min;		// 内部時計[min]
PRIVATE VULONG		ul_Wait;	// 1msecのwaitに使用する カウンタ[msec]
PUBLIC	VUCHAR		uc_CntSec;	// 減速時にループから抜け出せなくなったとき用[sec]

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
PUBLIC void init_Clock( void );
PUBLIC void init_GPIO( void );
PUBLIC void init_AD(void);
PUBLIC void init_Timer(void);

// *************************************************************************
//   機能		： マイコンのタイマを開始する。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.2.04			TKR			新規
// *************************************************************************/
PRIVATE void RX631_staTimer(void)
{
	MTU.TSTR.BIT.CST1	= 1;		// MTU1カウント開始			システム
	MTU.TSTR.BIT.CST2	= 0;		// MTU2カウント開始			センサ
	MTU.TSTR.BIT.CST3	= 1;		// MTU3カウント開始			バッテリ監視
	TPUA.TSTR.BIT.CST1	= 1;		// TPU1カウント開始			位相計測
	TPUA.TSTR.BIT.CST2	= 1;		// TPU2カウント開始			位相計測

}

// *************************************************************************
//   機能		： 時間制御の初期化。
//   注意		： 起動後、1回だけ実行する関数。
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.2.4			TKR			新規
// *************************************************************************/
PUBLIC void TIME_init( void )
{
	/* 初期化 */
	uc_Msec = 0;		// 内部時計[msec]
	uc_Sec  = 0;		// 内部時計[sec]
	uc_Min  = 0;		// 内部時計[min]
	ul_Wait = 0;		// 1msecのwaitに使用する カウンタ[msec]
	uc_CntSec = TIME_THRE_WAIT;		// 減速時にループから抜け無くなったとき用[sec]
}

// *************************************************************************
//   機能		： 指定した ms 間、S/Wウェイトする。
//   注意		： なし
//   メモ		： ul_time: 待つ時間 ( msec 単位 )。 最大 600000(= 10 min) 未満
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.2.04			TKR			新規
// *************************************************************************/
PUBLIC void TIME_wait( ULONG ul_time )
{
	ul_Wait = 0;						// 0からカウントする、"ul_Wait"は1msec毎に1加算される。

	while( ul_Wait < ul_time );			// 指定時間経過するまでウェイト
	
	return;
}

// *************************************************************************
//   機能		： 指定したフリーカウント間、S/Wウェイトする。
//   注意		： なし
//   メモ		： ul_cnt: カウント値
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.2.09			TKR			新規
// *************************************************************************/
PUBLIC void TIME_waitFree( ULONG ul_cnt )
{
	while( ul_cnt-- );			// 0になるまでディクリメント
}

// *************************************************************************
//   機能		： マイコンのレジスタを初期化する。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.01.29			TKR				新規
// *************************************************************************/
PRIVATE void CPU_init(void){	// system.cに移す
	
	init_Clock();	// クロックの設定
	init_GPIO();	// GPIOの設定
	init_AD();		// AD変換の設定
	init_Timer();	// タイマの設定

}

// *************************************************************************
//   機能		： 初期化，クロックの設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.3.11			TKR			新規
// *************************************************************************/
PUBLIC void init_Clock( void ){
	
	/* ============== */
	/*  クロック設定  */
	/* ============== */
	SYSTEM.PRCR.WORD	= 0xa50b;		// クロックソース選択保護解除
	SYSTEM.PLLCR.WORD	= 0x0f00;		// PLL逓倍(12MHz×16 = 192MHz)
	SYSTEM.PLLCR2.BYTE	= 0x00;			// PLL動作をEnable
	SYSTEM.SCKCR.LONG	= 0x21C21211;	// クロックの分周
	SYSTEM.SCKCR2.WORD 	= 0x0032;		// クロックの分周
	SYSTEM.BCKCR.BYTE	= 0x01;			// BCLK = 1/2
	SYSTEM.SCKCR3.WORD	= 0x0400;		// PLL回路選択
	
	// ICK   : 192/2 = 96MHz 		// システムクロック CPU DMAC DTC ROM RAM
	// PCLKA : 192/2 = 96MHz	 	// 周辺モジュールクロックA ETHERC、EDMAC、DEU
	// PCLKB : 192/4 = 48MHz 		// 周辺モジュールクロックB 上記以外 PCLKB=PCLK
}

// *************************************************************************
//   機能		： 初期化，GPIOの設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.3.11			TKR			新規
// *************************************************************************/
PUBLIC void init_GPIO(void){
	
	/* ========== */
	/*  GPIO設定  */
	/* ========== */
	/* 出力値 */
	PORT5.PODR.BIT.B5	= 0;		// ポート5-5の初期出力0[V] （LED0）
	PORT5.PODR.BIT.B4	= 0;		// ポート5-4の初期出力0[V] （LED1）
	PORTC.PODR.BIT.B2	= 0;		// ポートC-2の初期出力0[V] （LED2）
	PORTC.PODR.BIT.B3	= 0;		// ポートC-3の初期出力0[V] （LED3）
	PORTC.PODR.BIT.B4	= 0;		// ポートC-4の初期出力0[V] （LED4）
	PORTC.PODR.BIT.B5	= 0;		// ポートC-5の初期出力0[V] （LED_SYS）
	PORTC.PODR.BIT.B6	= 0;		// ポートC-6の初期出力0[V] （LED_BATT）	
	
	PORTA.PODR.BIT.B4	= 0;		// ポートA-4の初期出力0[V] （前壁右）
	PORTA.PODR.BIT.B6	= 0;		// ポートA-6の初期出力0[V] （右壁）	
	PORTA.PODR.BIT.B1	= 0;		// ポートA-1の初期出力0[V]　(前壁左)
	PORT4.PODR.BIT.B6	= 0;		// ポート4-6の初期出力0[V] （左壁）	
	
	
	/* 入出力設定 */
	PORTB.PDR.BIT.B3	= 1;		// ポートB-3：出力（右モータPWM）
	PORTA.PDR.BIT.B0	= 1;		// ポートA-0：出力 (左モータPWM)
	PORTB.PDR.BIT.B6	= 1;		// ポートB-6：出力（吸引モーターPWM）
	
	PORTB.PDR.BIT.B5	= 1;		// ポートB-5：出力（スピーカーPWM）
	
	PORTB.PDR.BIT.B1	= 1;		// ポートB-1：出力（右IN1）
	PORTB.PDR.BIT.B7	= 1;		// ポートB-7：出力（右IN2）
	PORTE.PDR.BIT.B5	= 1;		// ポートE-5：出力（左IN1）
	PORTA.PDR.BIT.B3	= 1;		// ポートA-3：出力（左IN2）	
	
	PORT5.PDR.BIT.B5	= 1;		// ポート5-5：出力（LED0）
	PORT5.PDR.BIT.B4	= 1;		// ポート5-4：出力（LED1）	
	PORTC.PDR.BIT.B2	= 1;		// ポートC-2：出力（LED2)
	PORTC.PDR.BIT.B3	= 1;		// ポートC-3：出力（LED3）	
	PORTC.PDR.BIT.B4	= 1;		// ポートC-4：出力（LED4）
	PORTC.PDR.BIT.B5	= 1;		// ポートC-5：出力（LED_SYS）
	PORTC.PDR.BIT.B6	= 1;		// ポートC-6：出力（LED_BATT）
	
	PORTA.PDR.BIT.B4	= 1;		// ポートA-3：出力（前壁右）
	PORTA.PDR.BIT.B6	= 1;		// ポートA-6：出力（右壁）
	PORTA.PDR.BIT.B1	= 1;		// ポートA-1：出力（前壁左）
	PORT4.PDR.BIT.B6	= 1;		// ポート4-6：出力（左壁）
	
	
	/* 入力プルアップ設定 */
	PORT3.PCR.BIT.B1	= 1;		// ポート3-1はプルアップ：モード選択
	PORT2.PCR.BIT.B7	= 1;		// ポート2-7はプルアップ：モード実行
	
	
	/* ポート入力データレジスタ */
	PORT3.PIDR.BIT.B1	= 1;		// ポート3-1を入力ポート：モード選択
	PORT2.PIDR.BIT.B7	= 1;		// ポート2-7を入力ポート：モード実行
	
}

// *************************************************************************
//   機能		： 初期化，AD変換の設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.3.11			TKR			新規
// *************************************************************************/
PUBLIC void init_AD(void){
		
	/* ============= */
	/*  A/D変換設定  */
	/* ============= */
	SYSTEM.PRCR.WORD	= 0xA502;
	MSTP(S12AD)			= 0;
	SYSTEM.PRCR.WORD	= 0xA500;
	S12AD.ADCSR.BIT.CKS	= 3;		// PCLK (48MHz)
	S12AD.ADCSR.BIT.ADCS= 0;		// シングルスキャン
	
}

// *************************************************************************
//   機能		： 初期化，タイマの設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.3.11			TKR			新規
// *************************************************************************/
PUBLIC void init_Timer(void){
	
	/* ================= */
	/*  タイマ設定(MTU)  */
	/* ================= */
	SYSTEM.PRCR.WORD = 0xa502;		// PRC1書き込み許可
	MSTP(MTU1) = 0;					// 電力供給許可(システム用)
	MSTP(MTU2) = 0;					// 電力供給許可(センサ用)
	MSTP(MTU3) = 0;					// 電力供給許可(バッテリ用)			
	SYSTEM.PRCR.WORD = 0xa500;		// PRC1書き込み禁止
	
	MTU.TSTR.BYTE = 0;				// タイマカウント動作停止
	
	// -----------------------
	//  システム用(MTU1)
	// -----------------------
	/* タイマ割り込みの設定 */
	MTU1.TCR.BIT.CCLR	= 1;		// TGRAのコンペアマッチでTCNTクリア
	MTU1.TCR.BIT.TPSC	= 2;		// PCLK(48MHz)/16で1カウント
	MTU1.TIER.BIT.TGIEA	= 1;		// TGRAとのコンペアマッチでクリア
	MTU1.TGRA			= 750 * 4;	// 1msec毎に割込み
	MTU1.TCNT			= 0;		// タイマカウンタクリア
	
	IEN(MTU1,TGIA1)		= 1;		// IRレジスタのステータスの割込先に伝える
	IPR(MTU1,TGIA1)		= 7;		// 割込みの優先順位(一番優先)
	IR(MTU1,TGIA1)		= 0;
	
	MTU.TSTR.BIT.CST1	= 0;		// カウント動作停止
	
	
	// -----------------------
	//  センサ用(MTU2)
	// -----------------------
	/* タイマ割り込みの設定 */
	MTU2.TCR.BIT.CCLR	= 1;		// TGRAとのコンペアマッチでクリア
	MTU2.TCR.BIT.TPSC	= 2;		// PCLK(48MHz)/16で1カウント
	MTU2.TIER.BIT.TGIEA	= 1;		// TGRAとのコンペアマッチでクリア
	MTU2.TGRA			= 750;		// 250usec毎に割込み
	MTU2.TCNT			= 0;		// タイマカウンタクリア
	
	IEN(MTU2,TGIA2)		= 1;		// IRレジスタのステータスの割込先に伝える
	IPR(MTU2,TGIA2)		= 8;		// 割込みの優先順位(その他)
	IR(MTU2,TGIA2)		= 0;
	
	MTU.TSTR.BIT.CST2	= 0;		// カウント動作停止
	
	// ------------------------
	//  右モータPWM出力用(MTU0)
	// ------------------------
	MPC.PWPR.BIT.B0WI	= 0;		// PFSWEビットへの書き込み許可
	MPC.PWPR.BIT.PFSWE	= 1;		// PFSレジスタの書き込み許可
	MPC.PB3PFS.BIT.PSEL	= 1;		// PB3：MTIOC0A
	MPC.PWPR.BYTE		= 0x80;		
	PORTB.PMR.BIT.B3	= 1;		// 周辺機能に切替
	
	SYSTEM.PRCR.WORD = 0xa502;		// PRC1書き込み許可
	MSTP(MTU0) = 0;					// 電力供給許可(右PWM用)		
	SYSTEM.PRCR.WORD = 0xa500;		// PRC1書き込み禁止
	
	MTU0.TCR.BIT.CCLR	= 1;		// TGRAのコンペアマッチでクリア
	MTU0.TCR.BIT.TPSC	= 1;		// PCLK(48MHz)/4で1カウント
	MTU0.TMDR.BIT.MD	= 2;		// PWMモード1
	MTU0.TIORH.BIT.IOA	= 2;		// 初期：Low　CM：High
	MTU0.TIORH.BIT.IOB	= 1;		// 初期：Low　CM：Low
	MTU0.TGRA			= 120;		// 周期(10usec/100kHz)
	MTU0.TGRB			= 60;		// onDuty
	MTU0.TCNT			= 0;		// タイマクリア
		
	
	// ------------------------
	//  左モータPWM出力用(MTU4)
	// ------------------------		
	MPC.PWPR.BIT.B0WI	= 0;		// PFSWEビットへの書き込み許可
	MPC.PWPR.BIT.PFSWE	= 1;		// PFSレジスタの書き込み許可
	MPC.PA0PFS.BIT.PSEL	= 1;		// PA0：MTIOC4A
	MPC.PWPR.BYTE		= 0x80;		
	PORTA.PMR.BIT.B0	= 1;		// 周辺機能に切替
	
	SYSTEM.PRCR.WORD = 0xa502;		// PRC1書き込み許可
	MSTP(MTU4) = 0;					// 電力供給許可(左PWM用)		
	SYSTEM.PRCR.WORD = 0xa500;		// PRC1書き込み禁止
	
	MTU4.TCR.BIT.CCLR	= 1;		// TGRAのコンペアマッチでクリア
	MTU4.TCR.BIT.TPSC	= 1;		// PCLK(48MHz)/4で1カウント
	MTU4.TMDR.BIT.MD	= 2;		// PWMモード1
	
	MTU.TOER.BIT.OE4A	= 1;		// MTIOC4Aの出力許可
	
	MTU4.TIORH.BIT.IOA	= 2;		// 初期：Low　CM：High
	MTU4.TIORH.BIT.IOB	= 1;		// 初期：Low　CM：Low
	MTU4.TGRA			= 120;		// 周期(10usec/100kHz)
	MTU4.TGRB			= 60;		// onDuty
	MTU4.TCNT			= 0;		// タイマクリア



	// -----------------------
	//  バッテリ用(MTU3)
	// -----------------------	
	/* タイマ割り込みの設定 */
	MTU3.TCR.BIT.CCLR	= 1;		// TGRAとのコンペアマッチでクリア
	MTU3.TCR.BIT.TPSC	= 2;		// PCLK(48MHz)/16で1カウント
	MTU3.TIER.BIT.TGIEA	= 1;		// TGRAとのコンペアマッチでクリア
	MTU3.TGRA			= 7500*4;	// 250usec毎に割込み
	MTU3.TCNT			= 0;		// タイマカウンタクリア
	
	IEN(MTU3,TGIA3)		= 1;		// IRレジスタのステータスの割込先に伝える
	IPR(MTU3,TGIA3)		= 6;		// 割込みの優先順位(その他)
	IR(MTU3,TGIA3)		= 0;
	
	MTU.TSTR.BIT.CST3	= 0;		// カウント動作停止	
	
	
	/* ================== */
	/*  タイマ設定(TPU)   */
	/* ================== */
	SYSTEM.PRCR.WORD = 0xa502;		// PRC1書き込み許可
	MSTP(TPU1) = 0;					// 電力供給許可(左モーター位相計数用)
	MSTP(TPU2) = 0;					// 電力供給許可(右モーター位相計数用)
	MSTP(TPU4) = 0;					// 電力供給許可(ブザー用)		
	MSTP(TPU5) = 0;					// 電力供給許可(吸引PWM用)	
	SYSTEM.PRCR.WORD = 0xa500;		// PRC1書き込み禁止
	
	TPUA.TSTR.BYTE		= 0;		// カウント動作停止(TPU0～5)
	
	// ------------------------
	//  吸引モータPWM出力用(TPU5)
	// ------------------------		
	MPC.PWPR.BIT.B0WI	= 0;		// PFSWEビットへの書き込み許可
	MPC.PWPR.BIT.PFSWE	= 1;		// PFSレジスタの書き込み許可
	MPC.PB6PFS.BIT.PSEL	= 3;		// PB6：TIOCA5
	MPC.PWPR.BYTE		= 0x80;		
	PORTB.PMR.BIT.B6	= 1;		// 周辺機能に切替	
	
	TPU5.TCR.BIT.CCLR	= 1;		// TGRAのコンペアマッチでクリア
	TPU5.TCR.BIT.TPSC	= 1;		// PCLK(48MHz)/4で1カウント
	TPU5.TMDR.BIT.MD	= 2;		// PWMモード1
	TPU5.TIOR.BIT.IOA	= 2;		// 初期：Low　CM：High
	TPU5.TIOR.BIT.IOB	= 1;		// 初期：Low　CM：Low
	TPU5.TGRA			= 120;		// 周期(10usec/100kHz)
	TPU5.TGRB			= 60;		// onDuty
	TPU5.TCNT			= 0;		// タイマクリア
	
	// ------------------------
	//  スピーカー用(TPU4)
	// ------------------------		
	MPC.PWPR.BIT.B0WI	= 0;		// PFSWEビットへの書き込み許可
	MPC.PWPR.BIT.PFSWE	= 1;		// PFSレジスタの書き込み許可
	MPC.PB5PFS.BIT.PSEL	= 3;		// PB5：TIOCB4
	MPC.PWPR.BYTE		= 0x80;		// PFSレジスタをプロテクト
	PORTB.PMR.BIT.B5	= 1;		// PB5:周辺機器を使用	
		
	TPU4.TCR.BIT.TPSC	= 1;		// PCLK(48MHz)/4で1カウント
	TPU4.TCR.BIT.CCLR	= 1;		// TGRAのコンペアマッチでクリア
	TPU4.TCR.BIT.CKEG	= 1;		// 立ち上がりエッジでカウント
	TPU4.TMDR.BIT.MD	= 3;		// PWMモード2（TIOCB4で使うため）
	
	TPU4.TIOR.BIT.IOA	= 1;		// 初期：low　CM：low
	TPU4.TIOR.BIT.IOB	= 2;		// 初期：low　CM：high
	
	// 暫定で440Hzにする
	TPU4.TGRA			= (27272-1);	// TGRAの設定
	TPU4.TGRB			= (13636-1);	// TGRBの設定
	TPU4.TCNT			= 0;			// タイマクリア
	
	TPUA.TSTR.BIT.CST4	= 0;			// TPU4の停止

	// --------------------------
	//  左モータ位相計測用(TPU1)
	// --------------------------
	MPC.PWPR.BIT.B0WI	= 0;		// PFSWEビットへの書き込み許可
	MPC.PWPR.BIT.PFSWE	= 1;		// PFSレジスタの書き込み許可
	MPC.P14PFS.BIT.PSEL	= 4;		// P14：TCLKA
	MPC.P15PFS.BIT.PSEL	= 4;		// P15：TCKLB
	MPC.P16PFS.BIT.PSEL	= 4;		// P16：TCLKC
	MPC.P17PFS.BIT.PSEL	= 4;		// P17：TCKLD	
	MPC.PWPR.BYTE		= 0x80;		
	PORT1.PMR.BIT.B4	= 1;		// 周辺機能に切替
	PORT1.PMR.BIT.B5	= 1;		// 周辺機能に切替
	PORT1.PMR.BIT.B6	= 1;		// 周辺機能に切替
	PORT1.PMR.BIT.B7	= 1;		// 周辺機能に切替
		
	TPU1.TCR.BIT.CCLR	= 0;		// コンペアマッチでTCNTクリア禁止 
	TPU1.TIER.BIT.TGIEA	= 0;		// TGRA とのコンペアマッチで割り込み未発生
	TPU1.TMDR.BIT.MD	= 7;		// 位相計測モード4
	TPU1.TGRA			= 0;		// 初めは未設定
	TPU1.TCNT			= 32768;	// 初期化
	
	
	// --------------------------
	//  右モータ位相計測用(TPU2)
	// --------------------------
	TPU2.TCR.BIT.CCLR	= 0;		// コンペアマッチでTCNTクリア禁止 
	TPU2.TIER.BIT.TGIEA	= 0;		// TGRA とのコンペアマッチで割り込み未発生
	TPU2.TMDR.BIT.MD	= 7;		// 位相計測モード4
	TPU2.TGRA			= 0;		// 初めは未設定
	TPU2.TCNT			= 32768;	// 初期化



}



// *************************************************************************
//   機能		： メイン関数
//   注意		： なし
//   メモ		： MTU1のTGRA割り込み、1msec毎に関数が実行される。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.1.29			TKR			新規
// *************************************************************************/
PUBLIC void main(void){
	
	UCHAR	i=0;
	
	/* 初期化 */
	CPU_init();						// [CPU]レジスタ/GPIO/AD/TIMERなど
	MODE_init();					// [MODE]モード	
	TIME_init();					// [SYS]タイマ
	RX631_staTimer();				// [CPU]タイマスタート
	LED_init();						// [LED]LEDをリセット
	SCI1_init();					// [SCI]シリアルをリセット
	FLASH_init();					// [FLASH]データフラッシュをリセット
	MAP_init();						// [MAP] マップをリセット 
	SPI_init();						// [SPI]SPIをリセット
	GYRO_init();					// [ジャイロ]ジャイロをリセット
	MTU.TSTR.BIT.CST2	= 1;		// [センサ]MTU2カウント開始
	CTRL_Loginit();					// [CTRL]ログ変数のリセット

	/* 調整 */
	DIST_adj();
			
	for( i = 0; i < 10; i++ ){
		LED_onAll();
		TIME_wait(80);
		LED_offAll();
		TIME_wait(80);
	}
	
	LED_offAll();
	
	/* タイトル表示 */
	printf("-------------------------------------------------\r\n");
	printf("  Robo Name  : BAHARAT \r\n");
	printf("  Developer  : TKR \r\n");
	printf("  Version    : 1 \r\n");
	printf("  Project By : Hosei Univ. Denken Group \r\n");
	printf("-------------------------------------------------\r\n\r\n");
	
	while(1){

		MODE_chkMode();		// モードチェック

		if( SW_ON == SW_INC_PIN ){
			MODE_inc();							// モードを1つ進める
			TIME_wait( SW_CHATTERING_WAIT );	// SWが離されるまで待つ
			printf("mode selecting\r\n");
		}else if( (SW_ON == SW_EXE_PIN)){
		//}else if( (SW_ON == SW_EXE_PIN) || ( true == MODE_CheckExe() ) ){
			MODE_exe();							// モードを実行
			TIME_wait( SW_CHATTERING_WAIT );	// SWが離されるまで待つ
		}	
	}	
}

// *************************************************************************
//   機能		： 割り込み関数、システム用（msecタイマ）
//   注意		： なし
//   メモ		： MTU1のTGRA割り込み、1msec毎に関数が実行される。
//				: 正常にマイコンが動作している場合、1sec周期でLED_SYSが点滅する。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.1.29			TKR			新規
// *************************************************************************/
PUBLIC void INTC_sys(void){
	
	static USHORT i= 0;
	
	if(i == 500){
		// システム確認用LED
		i=0;
	}else{
		i++;
	}
	
	/* ---------- */
	/*  内部時計  */
	/* ---------- */
	uc_Msec++;					// msec
	if( uc_Msec > 999 ){		// msec → sec
		uc_Msec  = 0;
		uc_Sec++;

		if( uc_CntSec != TIME_THRE_WAIT )uc_CntSec++;	// 減速チェック時以外ではカウントしない

	}
	if( uc_Sec > 59 ){			// sec → min
		uc_Sec = 0;
		uc_Min++;
	}
	
	/* ----------------------- */
	/*  S/Wウェイト・カウンタ  */
	/* ----------------------- */
	ul_Wait++;
	ul_Wait %= 6000000;			// 10 min (= 6000000 カウント) で 0 クリア

	CTRL_pol();

	return;
}

// *************************************************************************
//   機能		： 割り込み関数、センサ用
//   注意		： なし
//   メモ		： MTU2のTGRA割り込み、0.25msec毎に関数が実行される。
//				： 正常にマイコンが動作している場合、250usec周期で
//				： 距離センサ0 → 距離センサ1 → ジャイロセンサ の順でスキャンする。
//				： 従って、1センサのスキャン周期は1msecとなる。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.6.8			TKR			新規
// *************************************************************************/
PUBLIC void INTC_sen( void )
{
	static UCHAR	i = 0;
	
	/* センサ処理  */
	switch( i ){
		case 0:		// ジャイロセンサ
			GYRO_Pol();		// 先に平均値からジャイロの値を算出する
			GYRO_getVal();				
			break;
		
		case 1:		// 前壁センサ
			DIST_Pol_Front();
			break;
		
		case 2:		// 横壁センサ
			DIST_Pol_Side();
			break;
		
		case 3:		// 加速度センサ
			GYRO_getAccVal();		// フェイルセーフ用
			break;
		
	}
	
	i = ( i + 1 ) % 4;			// roop
	
	return;
}

//**************************************************
// プロジェクト作成時に生成されたコード （使わないけどとりあえず残しておく）
//**************************************************
#ifdef __cplusplus
void abort(void)
{

}
#endif
