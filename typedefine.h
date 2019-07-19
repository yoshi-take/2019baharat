/*********************************************************************
*
* Device     : RX
*
* File Name  : typedefine.h
*
* Abstract   : Aliases of Integer Type.
*
* History    : 1.00  (2009-08-07)
*
* NOTE       : THIS IS A TYPICAL EXAMPLE.
*
* Copyright (C) 2009 Renesas Electronics Corporation.
* and Renesas Solutions Corporation. All rights reserved.
*
*********************************************************************/
#ifndef	_TYPEDEF_DEFINE_H
#define	_TYPEDEF_DEFINE_H


/* -------------- */
/*  以下独自定義  */
/* -------------- */
/* ON/OFF */
#define ON					(1)
#define OFF					(0)

/* Hi/Lo */
#define HI					(1)
#define LO					(0)

/* スコープ設定 */
#define PUBLIC
#define PROTECTED
#define	PRIVATE				static

/* const、static、volatile */
#define CONST				const
#define STATIC				static
#define VOLATILE			volatile

/* 型定義 */
typedef unsigned char		UCHAR;
typedef char				CHAR;
typedef unsigned short		USHORT;
typedef short				SHORT;
typedef unsigned int        UINT;
typedef int                 INT;
typedef unsigned long		ULONG;
typedef long				LONG;
typedef float				FLOAT;
typedef double				DOUBLE;
typedef UCHAR*				PUCHAR;
typedef USHORT*				PUSHORT;
typedef ULONG*				PULONG;
typedef CHAR*				PCHAR;
typedef SHORT*				PSHORT;
typedef LONG*				PLONG;
typedef FLOAT*				PFLOAT;
typedef DOUBLE*				PDOUBLE;
typedef CHAR				STR;			// 文字列はこの定義を使用
typedef CONST CHAR			CSTR;			// 文字列はこの定義を使用
typedef CHAR*				PSTR;			// 文字列はこの定義を使用
typedef CONST CHAR* 		PCSTR;			// PSTRをROMに格納する場合はこちらを使用
typedef volatile CHAR		VCHAR;
typedef volatile UCHAR		VUCHAR;
typedef volatile SHORT		VSHORT;
typedef volatile USHORT		VUSHORT;
typedef volatile LONG		VLONG;
typedef volatile ULONG		VULONG;
typedef volatile FLOAT		VFLOAT;
typedef void *				PVOID;

/* BOOL型 */
#define	TRUE			(1)
#define	FALSE			(0)


//typedef	unsigned char	BOOL;
/* マクロ */
#define FABS(x)						( (x)>=0 ? (x) : -(x) )
/* warning対策 */
#define	NOUSE(a)			( a = a )

/* ------------- */
/*  Renesas定義  */
/* ------------- */
typedef signed char _SBYTE;
typedef unsigned char _UBYTE;
typedef signed short _SWORD;
typedef unsigned short _UWORD;
typedef signed int _SINT;
typedef unsigned int _UINT;
typedef signed long _SDWORD;
typedef unsigned long _UDWORD;
typedef signed long long _SQWORD;
typedef unsigned long long _UQWORD;

#endif
