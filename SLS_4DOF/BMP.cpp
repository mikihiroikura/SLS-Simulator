#include "BMP.h"

/////////////////////////////////////////////////////////////////////////
// BMP�w�b�_�̏����l
/////////////////////////////////////////////////////////////////////////
void initHeaders(BitmapHeaders *file){
	file->filetype1		='B';
	file->filetype2		='M';
	file->filesize		= 0;
	file->reserve1		= 0;
	file->reserve2		= 0;
	file->offset		= 54;

	file->header		=40;
	file->width			= 0;
	file->height		= 0;
	file->planes		= 1;
	file->bit_count		=24;
	file->compression	= 0;
	file->size_image	= 0;
	file->x_resolution	= 0;
	file->y_resolution	= 0;
	file->clr_used		= 0;
	file->clr_important	= 0;
};

/////////////////////////////////////////////////////////////////////////
// BMP�w�b�_�̏���
/////////////////////////////////////////////////////////////////////////
void writeHeaders(BitmapHeaders *file,FILE *fp){
	fwrite(&(file->filetype1), sizeof(char),1,fp);
	fwrite(&(file->filetype2), sizeof(char),1,fp);
	fwrite(&(file->filesize), sizeof(int),1,fp);
	fwrite(&(file->reserve1), sizeof(short),1,fp);
	fwrite(&(file->reserve2), sizeof(short),1,fp);
	fwrite(&(file->offset), sizeof(int),1,fp);

	fwrite(&(file->header), sizeof(int),1,fp);
	fwrite(&(file->width), sizeof(int),1,fp);
	fwrite(&(file->height), sizeof(int),1,fp);
	fwrite(&(file->planes), sizeof(short),1,fp);
	fwrite(&(file->bit_count), sizeof(short),1,fp);
	fwrite(&(file->compression), sizeof(int),1,fp);
	fwrite(&(file->size_image), sizeof(int),1,fp);
	fwrite(&(file->x_resolution), sizeof(int),1,fp);
	fwrite(&(file->y_resolution), sizeof(int),1,fp);
	fwrite(&(file->clr_used), sizeof(int),1,fp);
	fwrite(&(file->clr_important), sizeof(int),1,fp);
}

/////////////////////////////////////////////////////////////////////////
// �`��f�[�^��BMP�t�@�C���ւ̏���
// width��height��4�̔{���ł���K�v����
/////////////////////////////////////////////////////////////////////////
int writeBMP(const char* filename,int width, int height){

	int x, y;
	BitmapHeaders file;
	FILE *fp;
	GLubyte *pixel_data;

	// �������̈�m��
	pixel_data = (GLubyte*)malloc((width*3)*(height)*(sizeof(GLubyte)));
	glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixel_data);
	// �t�@�C���I�[�v��
	fp = fopen(filename, "wb");
	if(fp==NULL){
		printf("failure\n");
		return -1;
	}	
	// BMP�w�b�_�̏�����
	initHeaders(&file);
	// BMP�w�b�_�̏���
	file.width	= width;
	file.height = height;
	file.filesize =(width*3)*height+54;
	writeHeaders(&file,fp);
	// BMP�s�N�Z���f�[�^�̏���
	for(y=0;y<height;y++){
		for(x=0;x<width;x++){
			fwrite((pixel_data+x*3+(width*3)*y+2),sizeof(GLubyte),1,fp);
			fwrite((pixel_data+x*3+(width*3)*y+1),sizeof(GLubyte),1,fp);
			fwrite((pixel_data+x*3+(width*3)*y+0),sizeof(GLubyte),1,fp);
		}
	}
	// �������J��
	free(pixel_data);
	// �t�@�C���N���[�Y
	fclose(fp);
	return 0;
}
