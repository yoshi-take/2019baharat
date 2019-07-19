// 多重コンパイル抑止
#ifndef _COMMON_DEFINE_H
#define _COMMON_DEFINE_H

/** RES型 */
// 関数の型に使用する、奇数がエラー
typedef enum {
	SUCCESS				= 0,		// 成功
	FAILURE 			= 1,		// 失敗
	NOT_FOUND 			= 3,		// 検索ミス
	INVALID_HANDLE	 	= 5,		// 無効な操作
	INVALID_PARAMETER 	= 7,		// 無効な値
}enRES;


typedef enum
{
	false = 0,	//偽
	true = 1,	//真
}BOOL;	//真偽値を取り扱う列挙型


#endif