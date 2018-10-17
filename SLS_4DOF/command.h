#ifndef _INC_COMMAND
#define _INC_COMMAND

#define BUFSIZE	512		// ���̓R�}���h�p�o�b�t�@�T�C�Y

int getChar(char *buf, const char *msg);
int showHelp();
int setTimer(double delay);
int copyData();
int saveData();
int saveInfo();
int saveImage(int width, int height);
int drawData();
int saveGraph();
int saveGraph2(int trial_num);

#endif
