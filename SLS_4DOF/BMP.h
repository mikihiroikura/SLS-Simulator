#ifndef _INC_BMP
#define _INC_BMP

#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <gl/gl.h>

//BitmapHeader構造体
typedef struct _BitmapHeaders{
	char	filetype1;
	char	filetype2;
	int		filesize;
	short	reserve1;
	short	reserve2;
	int		offset;

	int		header;
	int		width;
	int		height;
	short	planes;
	short	bit_count;
	int		compression;
	int		size_image;
	int		x_resolution;
	int		y_resolution;
	int		clr_used;
	int		clr_important;
}BitmapHeaders;

void initHeaders(BitmapHeaders *file);
void writeHeaders(BitmapHeaders *file,FILE *fp);
// 描画ピクセルデータをＢＭＰファイル出力（１コマンド＝1ファイル）
int writeBMP(const char*, int width, int height);

#endif
