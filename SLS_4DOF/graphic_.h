#ifndef _INC_GRAPHIC
#define _INC_GRAPHIC

/////////////////////////////////////////////////
// 一般定義
/////////////////////////////////////////////////
#define SPHERE_APPROX	16		// 円の多角形近似数

/////////////////////////////////////////////////
// システム定義
/////////////////////////////////////////////////
#define	ARM_HAND_CONNECT

typedef enum {
	// アニメーション動作
	ANIMATION_FLAG_STOP,
	ANIMATION_FLAG_PLAY,
	ANIMATION_FLAG_PLAY_REVERSE,
	ANIMATION_FLAG_PAUSE,
	// ビデオ保存
	VIDEO_FLAG_STOP,
	VIDEO_FLAG_REC,
	VIDEO_FLAG_PAUSE,
	// アニメーション表示内容
	ANIMATION_DISP_HAND,
	ANIMATION_DISP_WAM,
	ANIMATION_DISP_HARM,
	// 視点
	VIEWPOINT_FIXED,
	VIEWPOINT_MOVE,
} FLAG;

/////////////////////////////////////////////////
// システムグラフィック表示用定数
/////////////////////////////////////////////////
#define ARM_PEDESTAL_HEIGHT		0.5
#define ARM_SIDECOVER_WIDTH		0.1
#define ARM_SIDECOVER_LENGTH	0.1
#define ARM_SIDECOVER_RADIUS	0.1
#define ARM_UPPER_ARM_LENGTH	0.5
#define ARM_UPPER_ARM_RADIUS	0.05
#define ARM_FRONT_ARM_LENGTH	0.5
#define ARM_FRONT_ARM_RADIUS	0.05
#define HAND_LINK_PART_LENGTH	0.0539/2
#define HAND_LINK1_LENGTH	0.062
#define HAND_LINK2_LENGTH	0.045

int myBox(double x, double y, double z);
int myCylinder(double radius, double height, double sides);
int myGround(double height);
int myPalm(double radius, double height, double length, double sides);
int myLinkPart(double radius, double height, double length, double sides);

int display_hand(int count);
int display_wam(int count);
int display_wamm(int count, double *wam_jnt_ang);
int display_arm(int count);
int display_obj(int count);

//////////////////////////////////
// 201105から追加
// z軸方向を統一
int myCuboid(double x, double y, double z, double *center);
int myCylinder2(double radius, double height, double sides, double *center);
int myCylinderHalf(double radius, double height, double sides, double *center);
int display_hand06(int count);
//////////////////////////////////

void display();
void resize(int w, int h);
void mouse(int button, int state, int x, int y);
void motion(int x, int y);
void keyboard(unsigned char key, int x, int y);
void init();
void final();
int save_video();

#endif
