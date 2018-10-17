#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GL/glut.h>
#include "graphic.h"
#include "command.h"

#include "simMain.h"		// PIのため

////////////////////////////////////
// 動作制御変数
////////////////////////////////////
static FLAG	animation_disp_num;
static FLAG	animation_flag;
static FLAG	viewpoint_flag;
static FLAG	video_flag;
static int	push_point[2];		// ドラッグ開始の画像座標を保存
////////////////////////////////////
// アニメーション表示
////////////////////////////////////
static double	pov[3];		// 視点並進移動(point of view)
static double	pan_jnt_deg;	// 視点回転移動（パン）
static double	tilt_jnt_deg;	// 視点回転移動（チルト）
static int	disp_count;
static int	display_rate;		// 早送りだと+,遅いと-
//static GLfloat lightpos[] = {-2.0, 2.0, -2.0, 1.0};		// 光源位置
static GLfloat lightpos[] = {0.5, 0.5, 0.5, 1.0};		// 光源位置
////////////////////////////////////
// 動画保存変数
////////////////////////////////////
//IplImage *frame;
//CvVideoWriter *vw;


void idle()
{
	glutPostRedisplay();
}

/////////////////////////////////////////////////////////////////////////
// 文字表示設定
/////////////////////////////////////////////////////////////////////////
void  drawMessage()
{
	char  string[] = "x";
	char  str_play[] = "Play";
	char  str_reverse[] = "Reverse";
	char  str_pause[] = "Pause";
	char  str_stop[] = "Stop";
	char  str_rec[] = " REC";
	char  str_rate[10];
	char  message[20];
	int   i;
	int	win_width = 200, win_height = 200;

	// 射影行列を初期化（初期化の前に現在の行列を退避）
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D( 0.0, win_width, win_height, 0.0 );
	// モデルビュー行列を初期化（初期化の前に現在の行列を退避）
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();
	// Ｚバッファ・ライティングはオフにする
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );
	// メッセージの描画
	glColor3f( 1.0, 0.0, 0.0 );
	glRasterPos2i( 8, 8 + 18 );
	if(display_rate > 0)	sprintf(str_rate, "%d", display_rate);
	else	sprintf(str_rate, "1/%d", -display_rate);
	if(animation_flag == ANIMATION_FLAG_PLAY)	sprintf(message, "%s %s %s", str_rate, string, str_play);
	else if(animation_flag == ANIMATION_FLAG_PLAY_REVERSE)	sprintf(message, "%s %s %s", str_rate, string, str_reverse);
	else if(animation_flag == ANIMATION_FLAG_PAUSE)	sprintf(message, "%s %s %s", str_rate, string, str_pause);
	else if(animation_flag == ANIMATION_FLAG_STOP)	sprintf(message, "%s %s %s", str_rate, string, str_stop);
	if(video_flag == VIDEO_FLAG_REC)	strcat(message, str_rec);
	for ( i=0; message[i]!='\0'; i++ )	glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18, message[i] );
	// 設定を全て復元
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
	glMatrixMode( GL_PROJECTION );
	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );
	glPopMatrix();
}

/////////////////////////////////////////////////////////////////////////
// 描画設定
/////////////////////////////////////////////////////////////////////////
void display()
{
	static int	incount = 0;		// アニメーション終了で0に戻る
	static int	prev_disp_count = 0;		// アニメーションが終了したときの姿勢を描画するために導入

	// 画面クリア
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// モデルビュー変換行列の初期化
	glLoadIdentity();
	// 光源の位置
	glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
	// 視点の移動
	glTranslated(pov[0], pov[1], pov[2]);
	glRotated(pan_jnt_deg, 0.0, 1.0, 0.0);
	glRotated(tilt_jnt_deg, cos(pan_jnt_deg*PI/180), 0.0, sin(pan_jnt_deg*PI/180));

	//////////////////////////////
	// 描画
	//////////////////////////////
	// 地面
	myGround(0.0);
	// アニメーション描画
#if 0	//SYSTEM_HAND
	if(animation_disp_num == ANIMATION_DISP_HAND)
		if(animation_flag == ANIMATION_FLAG_STOP)	display_hand(prev_disp_count);
		else	display_hand(disp_count);
#endif
#if 1	//SYSTEM_WAM
	if(animation_disp_num == ANIMATION_DISP_WAM)
		if(animation_flag == ANIMATION_FLAG_STOP){
			display_arm(prev_disp_count);
			display_obj(prev_disp_count);
		}else{
			display_arm(disp_count);
			display_obj(disp_count);
		}
#endif

	// アニメーション描画速度設定
	if(animation_flag == ANIMATION_FLAG_PLAY){
		prev_disp_count = disp_count;		// 直前に描画されたカウント数を保存
		if(display_rate > 0)	disp_count += display_rate;
		else if(incount % -display_rate == 0)	disp_count++;
		incount++;
	}else if(animation_flag == ANIMATION_FLAG_PLAY_REVERSE){
		prev_disp_count = disp_count;		// 直前に描画されたカウント数を保存
		if(display_rate > 0)	disp_count -= display_rate;
		else if(incount % -display_rate == 0)	disp_count--;
		incount--;
	}
	// アニメーション終了設定
//	if(disp_count >= (int)(1000*MOTION_TIME) || disp_count < 0){		// 最後まで再生したとき or 最初まで逆再生したとき
	if(disp_count >= DATA_CNT_NUM || disp_count < 0){		// 最後まで再生したとき or 最初まで逆再生したとき
		disp_count = 0;	incount = 0;
		animation_flag = ANIMATION_FLAG_STOP;	glutIdleFunc(NULL);
	}
	// 動画保存
//	if(video_flag == VIDEO_FLAG_REC)	save_video();
//	if(disp_count % SAVE_IMG_RATE == 0)	saveImage(600, 600);	// SAVE_IMG_RATE毎に画像保存

// ここ以降の記述は動画には表示されない
	// 文字列表示
	drawMessage();
	// 画面のちらつきを抑制．glutInitDisplayMode 関数で GLUT_DOUBLE を指定すること．
	glutSwapBuffers();
}

/////////////////////////////////////////////////////////////////////////
// 描画設定
/////////////////////////////////////////////////////////////////////////
void resize(int w, int h)
{
	glViewport(0, 0, w, h);
	// 透視変換行列の設定
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
//	gluPerspective(30.0, (double)w/(double)h, 1.0, 100.0);
	gluPerspective(18.0, (double)w/(double)h, 1.0, 100.0);
	// モデルビュー変換行列の設定
	glMatrixMode(GL_MODELVIEW);
}

/////////////////////////////////////////////////////////////////////////
// マウス操作：視点変更
/////////////////////////////////////////////////////////////////////////
void mouse(int button, int state, int x, int y)
{
	switch (button){
		case GLUT_LEFT_BUTTON:
			if(state == GLUT_DOWN){
				push_point[0] = x;	push_point[1] = y;
			}else if(state == GLUT_UP){
				viewpoint_flag = VIEWPOINT_FIXED;
			}
			break;
	}
}

/////////////////////////////////////////////////////////////////////////
// マウス動作設定
/////////////////////////////////////////////////////////////////////////
void motion(int x, int y)
{
	static int save_point[2];
	int prev_point[2];
#define VIEWPOINT_UPDATE_RATE	0.5
	if(viewpoint_flag == VIEWPOINT_MOVE){
		prev_point[0] = save_point[0];	prev_point[1] = save_point[1];
	}else{		// ドラッグし始めた瞬間
		prev_point[0] = push_point[0];	prev_point[1] = push_point[1];
		viewpoint_flag = VIEWPOINT_MOVE;
	}
	// 視点角度の変更
	pan_jnt_deg += VIEWPOINT_UPDATE_RATE * (x-prev_point[0]);
	tilt_jnt_deg += VIEWPOINT_UPDATE_RATE * (y-prev_point[1]);
	// ドラッグ中の前フレーム画素を保存
	save_point[0] = x;	save_point[1] = y;
	// 描画更新
	glutPostRedisplay();
}

/////////////////////////////////////////////////////////////////////////
// コマンド設定
/////////////////////////////////////////////////////////////////////////
void keyboard(unsigned char key, int x, int y)
{
	switch(key){
	// 表示切替
#if SYSTEM_HAND
	case 'h': animation_disp_num = ANIMATION_DISP_HAND; break;		// Hand（ハンド表示）
#endif
#if SYSTEM_WAM
	case 'w': animation_disp_num = ANIMATION_DISP_WAM; break;		// WAM（アーム表示）
#endif
	case 'a': animation_disp_num = ANIMATION_DISP_HARM; break;		// HDSアームを表示
	// 視点移動
	case 'x': pov[0] += 0.01; break;		// x方向
	case 'X': pov[0] -= 0.01; break;		// x方向
	case 'y': pov[1] += 0.01; break;		// y方向
	case 'Y': pov[1] -= 0.01; break;		// y方向
	case 'z': pov[2] += 0.1; break;		// 	z方向 (Zoom In)
	case 'Z': pov[2] -= 0.1; break;		// z方向 (Zoom Out)
	// 再生速度（sで2倍，Sで1/2倍）
	case 's': if(display_rate > 0) display_rate *= 2; else if(display_rate < -2) display_rate /= 2; else display_rate = 1; break;		// Speed Up
	case 'S': if(display_rate > 1) display_rate /= 2; else if(display_rate < 0) display_rate *= 2; else display_rate = -2; break;		// Speed Down
	// リセット
	case 'R': init(); break;		// Reset
	// アニメーションフラグ
	case 'p':			// Play, Pause（開始と一時停止のトグル動作）
		if(animation_flag == ANIMATION_FLAG_PLAY){
			animation_flag = ANIMATION_FLAG_PAUSE; glutIdleFunc(NULL); break;
		}else{
			animation_flag = ANIMATION_FLAG_PLAY; glutIdleFunc(idle); return;
		}
	case 'b': animation_flag = ANIMATION_FLAG_PLAY_REVERSE; glutIdleFunc(idle); break;		// 逆再生 backward
	// 画像保存
//	case 'i': saveImage(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));	break;	//save_image(); break;		// image
	case 'i': saveImage(600, 600);	break;		// image
	// 動画保存
	case 'v':			// 保存と一時停止のトグル動作
		if(video_flag == VIDEO_FLAG_REC){ video_flag = VIDEO_FLAG_PAUSE; display_rate = 1; return; }
		else{ video_flag = VIDEO_FLAG_REC; display_rate = 33; return; }
	// 終了
	case 'q': case '\0x1B': exit(0);	// Quit or Esc
	default: return;
	}
	// 'p''q''v'以外のコマンドは一度再描画
	glutPostRedisplay();
}

/////////////////////////////////////////////////////////////////////////
// 初期設定
/////////////////////////////////////////////////////////////////////////
void init()
{
	glClearColor(1.0, 1.0, 1.0, 1.0);
	// デプス・バッファ
	glEnable(GL_DEPTH_TEST);
	// カリング処理
	glEnable(GL_CULL_FACE);
//	glFrontFace(GL_CW);
	glCullFace(GL_BACK);		// 裏面を非表示
	// 光源設定
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	// 変数初期化
	animation_disp_num = ANIMATION_DISP_WAM;
	animation_flag = ANIMATION_FLAG_STOP;
	video_flag = VIDEO_FLAG_STOP;
	viewpoint_flag = VIEWPOINT_FIXED;
	disp_count = 0;
	display_rate = 1;
	pov[0] = -0.4;	pov[1] = -0.0;	pov[2] = -10.0;		// 視点位置
//	pov[0] = -0.2;	pov[1] = 0.5;	pov[2] = -2.5;		// 視点位置
	pan_jnt_deg = 150.0;	tilt_jnt_deg = 20.0;		// 視点姿勢


	// ビデオライタ構造体を作成する
//	vw = cvCreateVideoWriter("cap.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, cvSize(600, 600), 1);
//	vw = cvCreateVideoWriter("cap.avi", 0, 29.97, cvSize(600, 600), 1);		// 非圧縮AVIファイルを生成
//	cvNamedWindow("test",1);
//	frame = cvCreateImage(cvSize(600,600),IPL_DEPTH_8U,3);
}
