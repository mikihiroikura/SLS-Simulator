#ifndef _INC_BMP
#define _INC_BMP

#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <gl/gl.h>

//BitmapHeader�\����
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
// �`��s�N�Z���f�[�^���a�l�o�t�@�C���o�́i�P�R�}���h��1�t�@�C���j
int writeBMP(const char*, int width, int height);

#endif
