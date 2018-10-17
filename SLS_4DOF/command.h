#ifndef _INC_COMMAND
#define _INC_COMMAND

#define BUFSIZE	512		// 入力コマンド用バッファサイズ

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
