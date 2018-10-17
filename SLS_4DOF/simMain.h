#ifndef _INC_SIMMAIN
#define _INC_SIMMAIN

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "matBase.h"

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#define dsDrawTriangle dsDrawTriangleD
#endif

////////////////////////////////////////////////////////
// バイナリフラグ
// ON(1)とOFF(0)のみ設定可能
////////////////////////////////////////////////////////
#define	GLAPHIC_OPENGL		0		// OpenGLで描画
#define	FLAG_DRAW_SIM		1		// ODEの標準描画
#define	FLAG_SAVE_IMAGE		1		// 画像保存
#define	FLAG_SAVE_VIDEO		0		// 動画保存(OpenCVが必要)

////////////////////////////////////////////////////////
// define定義
////////////////////////////////////////////////////////
#ifndef PI
#define PI (3.14159265358979323846)
#endif
// 画面表示定義
#define	DISPLAY_WIDTH	640
#define	DISPLAY_HEIGHT	480
// 次元・座標識別定義
#define	DIM2	2
#define	DIM3	3
#define	CRD_X	0
#define	CRD_Y	1
#define	CRD_Z	2
#define	AXIS_X	0
#define	AXIS_Y	1
#define	AXIS_Z	2
#define DIR_LONG_AXIS_Z	3	// 長軸方向(dMassSetCylinderTotalなどに利用)
// 定数定義
#define SYSTEM_CYCLE_TIME	(0.001)	// 実験用サイクルタイム
#define SIM_CYCLE_TIME	(0.001)	// シミュレーション用サイクルタイム
#define DATA_CNT_NUM	6000	// データ保存カウント数
#define SAVE_IMG_RATE	200		// 画像保存間隔カウント数
#define SAVE_VIDEO_RATE	33		// 動画保存間隔カウント数
// 文字列定義
#define GNUPLOT_PATH	"\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\""	// パスに空白があるため[\"]を前後に追加
//#define GNUPLOT_PATH	"\"C:\\Program Files (x86)\\gnuplot\\bin\\gnuplot.exe\""	// パスに空白があるため[\"]を前後に追加
#define FILENAME_DATA	"data_%.3d.txt"		// 連番3桁対応
#define FILENAME_INFO	"info_%.3d.txt"		// 連番3桁対応
#define FILENAME_GRAPH1	"img_jnt_pos.png"
#define FILENAME_GRAPH2	"img_jnt_vel.png"
#define FILENAME_GRAPH3	"img_jnt_force.png"
#define FILENAME_GRAPH4	"img_eff_force.png"
#define FILENAME_GRAPH5	"img_err.png"
#define	DATA_FILE_NAME_MAXLEN	15
// アーム定数
#define	ARM_JNT	4
#define	ARM_M1	0
#define	ARM_M2	1
#define	ARM_M3	2
#define	ARM_M4	3

////////////////////////////////////////////////////////
// 構造体定義
////////////////////////////////////////////////////////
// ODEパーツ用構造体
typedef struct{       // MyObject構造体
	dBodyID body;        // ボディ(剛体)のID番号（動力学計算用）
	dGeomID geom;        // ジオメトリのID番号(衝突検出計算用）
	dReal  l,r,m;       // 長さ[m], 半径[m]，質量[kg]
	dReal	sides[DIM3];	// 直方体x,y,zの辺長
} MyObject;

// 変数構造体
typedef struct{
	Matrix	q;	// 関節角
	Matrix	r;	// 手先位置
	Matrix	F;	// 手先外力
	Matrix	p;	// 接続点位置
	// 補足変数
	Matrix	q0;	// 初期関節角
	Matrix	r0;	// 初期手先位置
	Matrix	p0;	// 初期接続位置
	Matrix	dq;	// 関節速度
	Matrix	dr;	// 手先速度
	Matrix	dp;	// 接続点速度
} Variable;

// 運動学構造体
typedef struct{       //
	double	l[ARM_JNT];		// リンク長
	double	lg[ARM_JNT];		// リンク重心位置までの長さ
	double	r[ARM_JNT];		// リンク半径（太さ方向）
	Matrix	J;	// ヤコビアン
	// 補足変数
	Matrix	Jp;	// 接続点ヤコビアン
	Matrix	dJ;	// ヤコビアン微分
	Matrix	dJp;	// 接続点ヤコビアン微分
	Matrix	Jt, Jinv;	// 転置行列，逆行列
	Matrix	Jpt, Jpinv;	// 転置行列，逆行列
} Kinematics;

// 動力学構造体
// Mq*ddq + h + V*dq = tau + Jt*F
typedef struct{       //
	double	m[ARM_JNT];		// リンク質量
	Matrix	Mq;		// 慣性項
	Matrix	h;	// コリオリ・遠心力項
	double	V[ARM_JNT];	// 粘性摩擦係数
	// 補足変数
	Matrix	dMq;		// 慣性項微分
} Dynamics;

// インピーダンス構造体
typedef struct{
	Matrix	M, C, K;	// 目標インピーダンス（手先座標）
	Matrix	K0;	// SLS用
	double	T[DIM3];	// 周期
	// 補足変数
	Matrix	dM, dC, dK;	// 目標インピーダンス微分（手先座標）
	Matrix	Minv, Cinv, Kinv;	// 逆行列
} Impedance;

// シミュレーション構造体
typedef struct{
	// 制御用変数
	int	step;	//経過ステップ数
	int state_contact;		// 接触状態(0:OFF, 1:ON)
	double	dist;		// アームと対象の距離
	double	ref_jnt_pos[ARM_JNT];
	double	jnt_pos[ARM_JNT];		// 関節位置
	double	jnt_vel[ARM_JNT];		// 関節速度
	double	jnt_force[ARM_JNT];		// 関節力
	double	past_jnt_pos[ARM_JNT];
	double	eff_pos[DIM3];		// 手先位置
	double	eff_vel[DIM3];		// 手先速度
	double	eff_force[DIM3];		// 手先力
	double	node_pos[DIM3];		// 結節点位置
	double	node_vel[DIM3];		// 結節点速度
	double	obj_pos[DIM3];		// 対象位置
	double	obj_vel[DIM3];		// 対象速度
	// 初期変数
	double	init_jnt_pos[ARM_JNT];
	double	init_obj_pos[DIM3];
	double	init_obj_att[DIM3][DIM3];	// 絶対座標における対象座標軸の姿勢（軸は正規直交基底）
	// 変数構造体
	Variable	var;
	// 運動学変数
	Kinematics	kine;
	// 動力学変数
	Dynamics	dyn;
	// インピーダンス変数
	Impedance	imp;
	// 保存用データ変数
	int save_state_contact[DATA_CNT_NUM];
	double	save_dist[DATA_CNT_NUM];
	double	save_ref_jnt_pos[DATA_CNT_NUM][ARM_JNT];
	double	save_jnt_pos[DATA_CNT_NUM][ARM_JNT];
	double	save_jnt_vel[DATA_CNT_NUM][ARM_JNT];
	double	save_jnt_force[DATA_CNT_NUM][ARM_JNT];
	double	save_eff_pos[DATA_CNT_NUM][DIM3];
	double	save_eff_vel[DATA_CNT_NUM][DIM3];
	double	save_eff_force[DATA_CNT_NUM][DIM3];
	double	save_node_pos[DATA_CNT_NUM][DIM3];
	double	save_node_vel[DATA_CNT_NUM][DIM3];
	double	save_obj_pos[DATA_CNT_NUM][DIM3];
	double	save_obj_vel[DATA_CNT_NUM][DIM3];
	// 保存用ファイル名変数
	char	data_file_name[DATA_FILE_NAME_MAXLEN];
	char	filename_info[DATA_FILE_NAME_MAXLEN];
	char	filename_graph[DATA_FILE_NAME_MAXLEN];
} SIM;

////////////////////////////////////////////////////////
// プロトタイプ
////////////////////////////////////////////////////////
static void nearCallback(void *data, dGeomID o1, dGeomID o2);
static void simLoop(int pause);
static void start();
static void restart();
static void command(int cmd);
void setDrawStuff();
int exeCmd(int argc, char *argv[]);
int ctrlArm(dReal *jnt_force);
int armDynPara(Matrix *Mq, Matrix *h, Matrix *J, Matrix *dJ, Matrix *Jp, Matrix *dJp, const Matrix *q, const Matrix *dq);
int ctrlMaxwellParallel(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Fint, const Matrix *Md, const Matrix *Cd, const Matrix *Kd);
int ctrlMaxwellSeries(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *Jp, const Matrix *dJp, const Matrix *q, const Matrix *dq, const Matrix *rs, const Matrix *drs, const Matrix *F, const Matrix *Md, const Matrix *Cd, const Matrix *Kd);
int ctrlMaxwellSeries2(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *Jp, const Matrix *dJp, const Matrix *q, const Matrix *dq, const Matrix *rs, const Matrix *dre, const Matrix *F, const Matrix *Md, const Matrix *Cd, const Matrix *Kd);
int ctrlVoigt(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Md, const Matrix *Cd, const Matrix *Kd);
int ctrlHybrid(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Fint, const Matrix *Md, const Matrix *Cd, const Matrix *Kd);
int addExtForce();
int calcDist();
void rolling_function( dGeomID o, dReal coef, dContact *c );

#endif
