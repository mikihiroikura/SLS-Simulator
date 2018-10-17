/////////////////////////////////////////////////////////////////////////
// OpenCVで動画保存
/////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <math.h>
#include "simMain.h"	// 動画表示サイズを読み込み
#include <GL/glut.h>	// stdlib.hより後に読み込む必要あり
#include "video.h"

#define VER_OPENCV2  1 //使用するOpenCVのバージョンが2.xなら1, 1.xなら0を設定して下さい．

#if VER_OPENCV2 
//OpenCV 2.xの場合
//「プロジェクト」→「simのプロパティ」→「C/C++」→「全般」から
// 追加のインクルードディレクトリにOpenCVの"include"を追加(C:\OpenCV2.4.0\build\include)
#include "opencv2\\opencv.hpp"
//ライブラリーファイルを自分の環境に合わせて設定
//例：(C:\\OpenCV2.3.1\\build\\x86\\vc9\\lib)
//Debugモードの場合　C:\\OpenCV2.3.1\\build\\x86\\vc9\\lib
//#pragma comment(lib,"C:\\OpenCV2.4.0\\build\\x86\\vc10\\lib\\opencv_core240d.lib")
//#pragma comment(lib,"C:\\OpenCV2.4.0\\build\\x86\\vc10\\lib\\opencv_imgproc240d.lib")
//#pragma comment(lib,"C:\\OpenCV2.4.0\\build\\x86\\vc10\\lib\\opencv_highgui240d.lib")
#define CVLIBVER CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
#ifdef NDEBUG
#define CVLIB(name) "opencv_" CVAUX_STR(name) CVLIBVER
#else
#define CVLIB(name) "opencv_" CVAUX_STR(name) CVLIBVER "d"
#endif
#pragma comment(lib, CVLIB(world))
#endif //VER_OPENCV2

////////////////////////////////////
// 動画保存変数
////////////////////////////////////
IplImage *frame;
CvVideoWriter *vw;

/////////////////////////////////////////////////////////////////////////
// 初期設定
/////////////////////////////////////////////////////////////////////////
void init_video()
{
	// ビデオライタ構造体を作成する
	// コーデックは以下を参照　http://www.buildinsider.net/small/opencv/05
//	vw = cvCreateVideoWriter("cap.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, cvSize(600, 600), 1);
//	vw = cvCreateVideoWriter("cap.avi", 0, 29.97, cvSize(600, 600), 1);		// 非圧縮AVIファイルを生成
//	vw = cvCreateVideoWriter("cap.avi", 0, 29.97, cvSize(DISPLAY_WIDTH, DISPLAY_HEIGHT), 1);		// 非圧縮AVIファイルを生成
	vw = cvCreateVideoWriter("cap.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30.0, cvSize(DISPLAY_WIDTH, DISPLAY_HEIGHT), 1);
//	cvNamedWindow("test",1);
//	frame = cvCreateImage(cvSize(600,600),IPL_DEPTH_8U,3);
	frame = cvCreateImage(cvSize(DISPLAY_WIDTH,DISPLAY_HEIGHT),IPL_DEPTH_8U,3);
}

/////////////////////////////////////////////////////////////////////////
// 終了設定
/////////////////////////////////////////////////////////////////////////
void final_video()
{
	cvReleaseVideoWriter(&vw);
}

/////////////////////////////////////////////////////////////////////////
// 動画保存
/////////////////////////////////////////////////////////////////////////
int save_video()
{
	GLint view[4];

	/* .$B2hLLI=<($N40N;$rBT$D.(B */
	glFinish();

	/* .$B8=:_$N%S%e!<%]!<%H$N%5%$%:$rF@$k.(B */
	glGetIntegerv(GL_VIEWPORT, view);
	glReadPixels(view[0], view[1], view[2], view[3], GL_BGR_EXT, GL_UNSIGNED_BYTE, frame->imageData);		// GL_BGR は未定義？
      
	// 動画書き出し
	cvFlip(frame,frame,0);	// 上下反転
//	cvCvtColor(frame,frame,CV_RGB2BGR);
//	cvShowImage("test",frame);		// 書き出しがうまくいっているか画像を表示して確認
	cvWriteFrame (vw, frame); 

	return 0;
}
