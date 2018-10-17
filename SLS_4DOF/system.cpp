#include <stdio.h>
#include <math.h>
//#include <HAND.h>
//#include <WAM.h>
//#include <App.h>
//#include <DataDef.h>
#include "graphic.h"
#include "simMain.h"
#include <GL/glut.h>	// stdlib.hより後に読み込む必要あり

// 外部変数
extern SIM	sim;

////////////////////////////////////
// カラー表示
////////////////////////////////////
static GLfloat red[] = {0.8, 0.2, 0.2, 1.0};
static GLfloat blue[] = {0.2, 0.2, 0.8, 1.0};
static GLfloat gray[] = {0.8, 0.8, 0.8, 1.0};
static GLfloat yellow[] = {0.8, 0.8, 0.2, 1.0};
static GLfloat black[] = {0.2, 0.2, 0.2, 1.0};
static GLfloat green[] = {0.0, 0.5, 0.5, 1.0};
////////////////////////////////////
// データ参照
////////////////////////////////////
#if SYSTEM_HAND
extern double save_ref_hand_jnt_ang[(int)(1000*MOTION_TIME)][HAND_JNT];
#endif
#if SYSTEM_WAM1
extern double save_ref_wam_jnt_ang[(int)(1000*MOTION_TIME)][WAM_JNT];
#endif
#if SYSTEM_WAM2
extern double save_ref_wam2_jnt_ang[(int)(1000*MOTION_TIME)][WAM_JNT];
#endif

// 作り直す必要あり
int display_arm(int count)
{
#define	ARM_BASE_X_OFFSET	0.3
#define	ARM_BASE_X	0.5
#define	ARM_BASE_Y	0.4
#define	ARM_BASE_Z	0.3
#define	ARM_LINK1_RADIUS	0.15
#define	ARM_LINK1_LENGTH	0.75
#define	ARM_SENSOR_RADIUS	0.15
#define	ARM_SENSOR_LENGTH	0.0001
	static GLfloat shininess[] = {100.0};
	// 退避
	glPushMatrix();
	// 土台
	glTranslated(ARM_BASE_X_OFFSET, ARM_BASE_Y/2.0, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, yellow);
	glMaterialfv(GL_FRONT, GL_SHININESS, shininess); 
	myBox(ARM_BASE_X/2.0, ARM_BASE_Y/2.0, ARM_BASE_Z/2.0);
	glTranslated(-ARM_BASE_X_OFFSET, 0.0, 0.0);
	// 図形移動
	glTranslated(sim.save_jnt_pos[count][ARM_M1], 0.0, 0.0);	// 仮に1軸
	// アーム
	glRotated(90.0, 0.0, 0.0, 1.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, blue);
	glMaterialfv(GL_FRONT, GL_SHININESS, shininess); 
	myCylinder(ARM_LINK1_RADIUS, ARM_LINK1_LENGTH/2.0, SPHERE_APPROX);
	glTranslated(0.0, ARM_LINK1_LENGTH/2.0, 0.0);
	// センサ
	glTranslated(0.0, ARM_SENSOR_LENGTH/2.0, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, green);
	glMaterialfv(GL_FRONT, GL_SHININESS, shininess); 
	myCylinder(ARM_SENSOR_RADIUS, ARM_SENSOR_LENGTH/2.0, SPHERE_APPROX);
	glTranslated(0.0, ARM_SENSOR_LENGTH/2.0, 0.0);
	// 復帰
	glPopMatrix();

	return 0;
}

// 回転が反映されていないので，追加する必要あり
int display_obj(int count)
{
#define	OBJ_RADIUS	0.15
#define	OBJ_LENGTH	0.15
	static GLfloat shininess[] = {100.0};
	// 退避
	glPushMatrix();
	// 土台
//	glTranslated(sim.save_obj_pos[count], ARM_BASE_Y/2.0, 0.0);	// 1軸
	glTranslated(sim.save_obj_pos[count][CRD_X], sim.save_obj_pos[count][CRD_Z], sim.save_obj_pos[count][CRD_Y]);	// 1軸
	glRotated(90.0, 1.0, 0.0, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, red);
	glMaterialfv(GL_FRONT, GL_SHININESS, shininess); 
	myCylinder(OBJ_RADIUS, OBJ_LENGTH/2.0, SPHERE_APPROX);
	// 復帰
	glPopMatrix();

	return 0;
}

#if 0
int display_wam(int count)
{
	// 土台
	glTranslated(0.0, ARM_PEDESTAL_HEIGHT/2, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, yellow);
	myBox(0.1, ARM_PEDESTAL_HEIGHT/2, 0.1);
	glTranslated(0.0, ARM_PEDESTAL_HEIGHT/2, 0.0);
	// 図形回転
	glRotated(sim.save_jnt_pos[count]*180/PI, 0.0, 1.0, 0.0);	// 1軸
	// サイドカバー
	glPushMatrix();
	glTranslated(ARM_UPPER_ARM_RADIUS+ARM_SIDECOVER_WIDTH/2, ARM_SIDECOVER_LENGTH/2, 0.0);
	glRotated(90.0, 0.0, 0.0, 1.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
	myLinkPart(ARM_SIDECOVER_RADIUS, ARM_SIDECOVER_WIDTH/2, ARM_SIDECOVER_LENGTH/2, SPHERE_APPROX);
	glTranslated(0.0, ARM_SIDECOVER_WIDTH+2*ARM_UPPER_ARM_RADIUS, 0.0);
	myLinkPart(ARM_SIDECOVER_RADIUS, ARM_SIDECOVER_WIDTH/2, ARM_SIDECOVER_LENGTH/2, SPHERE_APPROX);
	glTranslated(-ARM_SIDECOVER_LENGTH/2+0.02, -ARM_SIDECOVER_WIDTH/2-ARM_UPPER_ARM_RADIUS, 0.0);
	glRotated(90.0, 0.0, 0.0, 1.0);
	myCylinder(ARM_SIDECOVER_WIDTH/2+0.08, 0.02, SPHERE_APPROX);
	glPopMatrix();
	glTranslated(0.0, ARM_SIDECOVER_LENGTH, 0.0);
	// 図形回転
	glRotated(sim.save_jnt_pos[count]*180/PI, 1.0, 0.0, 0.0);	// 2軸
	glRotated(sim.save_jnt_pos[count]*180/PI, 0.0, 1.0, 0.0);	// 3軸
	// 上腕
	glTranslated(0.0, ARM_UPPER_ARM_LENGTH/2, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
	myCylinder(ARM_UPPER_ARM_RADIUS, ARM_UPPER_ARM_LENGTH/2, SPHERE_APPROX);
	glTranslated(0.0, ARM_UPPER_ARM_LENGTH/2, 0.0);
	// 肘関節
	glPushMatrix();
	glRotated(90.0, 0.0, 0.0, 1.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
	myCylinder(0.055, 0.051, SPHERE_APPROX);
	glPopMatrix();

	// 前腕
	glRotated(sim.save_jnt_pos[count]*180/PI, 1.0, 0.0, 0.0);	// 4軸
	glTranslated(0.0, ARM_FRONT_ARM_LENGTH/2, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
	myCylinder(ARM_FRONT_ARM_RADIUS, ARM_FRONT_ARM_LENGTH/2, SPHERE_APPROX);
	// 手先
	glTranslated(0.0, ARM_FRONT_ARM_LENGTH/2, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
//	glutSolidCube(0.7);

	return 0;
}
#endif

#if 0
int hand_finger(double jnt_MP, double jnt_IP)
{
	glTranslated(0.0, 0.015, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
	myBox(0.01-0.0005, 0.015, 0.0065);
	glTranslated(0.0, 0.015, -0.01);
	glPushMatrix();
	glRotated(90.0, 1.0, 0.0, 0.0);
	glRotated(90.0, 0.0, 0.0, 1.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
	myLinkPart(0.01, 0.01, HAND_LINK_PART_LENGTH, SPHERE_APPROX);
	glPopMatrix();
	glTranslated(0.0, 0.0, HAND_LINK_PART_LENGTH);

	// 基節骨
	glRotated(jnt_MP, 1.0, 0.0, 0.0);
	glTranslated(0.0, 0.0, HAND_LINK1_LENGTH/2);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
	glPushMatrix();
	glRotated(90.0, 1.0, 0.0, 0.0);
	myCylinder(0.01, HAND_LINK1_LENGTH/2, SPHERE_APPROX);
	glPopMatrix();
	// 関節
	glTranslated(0.0, 0.0, HAND_LINK1_LENGTH/2);
	glPushMatrix();
	glRotated(90.0, 0.0, 0.0, 1.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
	myCylinder(0.01+0.0005, 0.01+0.0005, SPHERE_APPROX);
	glPopMatrix();

	// モデルビュー変換行列の保存
	glRotated(jnt_IP, 1.0, 0.0, 0.0);
	// 末節骨
	glTranslated(0.0, 0.0, HAND_LINK2_LENGTH/2.0);
	glPushMatrix();
	glRotated(90.0, 1.0, 0.0, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
	myCylinder(0.0085, HAND_LINK2_LENGTH/2.0, SPHERE_APPROX);
	glPopMatrix();
	// 指先
	glTranslated(0.0, 0.0, HAND_LINK2_LENGTH/2.0);
	glutSolidSphere(0.0085, SPHERE_APPROX, SPHERE_APPROX);

	return 0;
}

int display_hand(int count)
{
#define HAND_OFFSET1_Z_LENGTH	0.04
#define HAND_OFFSET2_Z_LENGTH	0.16
//#define HAND_WRIST_LENGTH	0.2*0.3
#define HAND_WRIST_OFFSET	0.025

	// 土台1
	glTranslated(0.0, HAND_OFFSET1_Z_LENGTH/2, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
	myCylinder(0.04, HAND_OFFSET1_Z_LENGTH/2, SPHERE_APPROX);
	glTranslated(0.0, HAND_OFFSET1_Z_LENGTH/2+HAND_OFFSET2_Z_LENGTH/2, 0.0);
	// 土台2
	myCylinder(0.03, HAND_OFFSET2_Z_LENGTH/2, SPHERE_APPROX);
	glTranslated(0.0, HAND_OFFSET2_Z_LENGTH/2+HAND_WRIST_OFFSET/2, 0.0);
	// 手首（9軸と10軸アクチュエータ部分のつもり）
	glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
	glRotated(save_ref_hand_jnt_ang[count][HAND_M11], 0.0, 1.0, 0.0);
	myCylinder(0.032, HAND_WRIST_OFFSET/2, SPHERE_APPROX);
	glTranslated(0.0, HAND_WRIST_OFFSET/2, 0.0);

	glPushMatrix();
	glRotated(90.0, 0.0, 0.0, 1.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
	myLinkPart(0.02, 0.025, 0.0, SPHERE_APPROX);
	glPopMatrix();

	glRotated(save_ref_hand_jnt_ang[count][HAND_M9], 1.0, 0.0, 0.0);
	glTranslated(0.0, 0.02+0.03, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
	myBox(0.015, 0.03, 0.01);
	glTranslated(0.0, 0.03, 0.01+0.0065+0.0025);
	// 掌
	glPushMatrix();
	glTranslated(0.0, 0.0, 0.0065+0.00125);
	glRotated(90.0, 1.0, 0.0, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, yellow);
	myPalm(0.018, 0.00125, 0.03, SPHERE_APPROX);
	glTranslated(0.0, -2*(0.0065+0.00125), 0.0);
	myPalm(0.018, 0.00125, 0.03, SPHERE_APPROX);
	glPopMatrix();
	// 中指
	glPushMatrix();
	hand_finger(save_ref_hand_jnt_ang[count][HAND_M3], save_ref_hand_jnt_ang[count][HAND_M4]);
	glPopMatrix();
	// 左指（正面から見て右）
	glPushMatrix();
	glTranslated(0.03, 0.0, 0.0);
	glRotated(save_ref_hand_jnt_ang[count][HAND_M7], 0.0, 0.0, 1.0);
	hand_finger(save_ref_hand_jnt_ang[count][HAND_M1], save_ref_hand_jnt_ang[count][HAND_M2]);
	glPopMatrix();
	// 右指（正面から見て左）
	glPushMatrix();
	glTranslated(-0.03, 0.0, 0.0);
	glRotated(save_ref_hand_jnt_ang[count][HAND_M8], 0.0, 0.0, 1.0);
	hand_finger(save_ref_hand_jnt_ang[count][HAND_M5], save_ref_hand_jnt_ang[count][HAND_M6]);
	glPopMatrix();

	return 0;
}

int display_wam(int count)
{
	// 土台
	glTranslated(0.0, ARM_PEDESTAL_HEIGHT/2, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, yellow);
	myBox(0.1, ARM_PEDESTAL_HEIGHT/2, 0.1);
	glTranslated(0.0, ARM_PEDESTAL_HEIGHT/2, 0.0);
	// 図形回転
	glRotated(save_ref_wam_jnt_ang[count][WAM_M1], 0.0, 1.0, 0.0);	// 1軸
	// サイドカバー
	glPushMatrix();
	glTranslated(ARM_UPPER_ARM_RADIUS+ARM_SIDECOVER_WIDTH/2, ARM_SIDECOVER_LENGTH/2, 0.0);
	glRotated(90.0, 0.0, 0.0, 1.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
	myLinkPart(ARM_SIDECOVER_RADIUS, ARM_SIDECOVER_WIDTH/2, ARM_SIDECOVER_LENGTH/2, SPHERE_APPROX);
	glTranslated(0.0, ARM_SIDECOVER_WIDTH+2*ARM_UPPER_ARM_RADIUS, 0.0);
	myLinkPart(ARM_SIDECOVER_RADIUS, ARM_SIDECOVER_WIDTH/2, ARM_SIDECOVER_LENGTH/2, SPHERE_APPROX);
	glTranslated(-ARM_SIDECOVER_LENGTH/2+0.02, -ARM_SIDECOVER_WIDTH/2-ARM_UPPER_ARM_RADIUS, 0.0);
	glRotated(90.0, 0.0, 0.0, 1.0);
	myCylinder(ARM_SIDECOVER_WIDTH/2+0.08, 0.02, SPHERE_APPROX);
	glPopMatrix();
	glTranslated(0.0, ARM_SIDECOVER_LENGTH, 0.0);
	// 図形回転
	glRotated(save_ref_wam_jnt_ang[count][WAM_M2], 1.0, 0.0, 0.0);	// 2軸
	glRotated(save_ref_wam_jnt_ang[count][WAM_M3], 0.0, 1.0, 0.0);	// 3軸
	// 上腕
	glTranslated(0.0, ARM_UPPER_ARM_LENGTH/2, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
	myCylinder(ARM_UPPER_ARM_RADIUS, ARM_UPPER_ARM_LENGTH/2, SPHERE_APPROX);
	glTranslated(0.0, ARM_UPPER_ARM_LENGTH/2, 0.0);
	// 肘関節
	glPushMatrix();
	glRotated(90.0, 0.0, 0.0, 1.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
	myCylinder(0.055, 0.051, SPHERE_APPROX);
	glPopMatrix();

	// 前腕
	glRotated(save_ref_wam_jnt_ang[count][WAM_M4], 1.0, 0.0, 0.0);	// 4軸
#ifndef	ARM_HAND_CONNECT
	glTranslated(0.0, ARM_FRONT_ARM_LENGTH/2, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
	myCylinder(ARM_FRONT_ARM_RADIUS, ARM_FRONT_ARM_LENGTH/2, SPHERE_APPROX);
	// 手先
	glTranslated(0.0, ARM_FRONT_ARM_LENGTH/2, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
	glutSolidCube(0.7);
#endif

	return 0;
}

//////////////////////////
// HDSアーム
//////////////////////////
int display_arm(int count)
{
#define	HARM_LINK0_01_W	(0.2)
#define	HARM_LINK0_01_D	(0.2)
#define	HARM_LINK0_01_H	(0.5)
#define	HARM_LINK0_02_R	(0.1)
#define	HARM_LINK0_02_H	(HARM_LINK0_02_R*2.0)
#define	HARM_LINK0_03_R	(0.1)
#define	HARM_LINK0_03_H	(0.1)
#define	HARM_LINK1_01_R	(0.1)
#define	HARM_LINK1_01_H	(0.1)
#define	HARM_LINK1_02_R	(0.1)
#define	HARM_LINK1_02_H	(HARM_LINK1_02_R*2.0)
#define	HARM_LINK1_03_R	(0.1)
#define	HARM_LINK1_03_H	(0.26)
#define	HARM_LINK2_01_W	(0.18)
#define	HARM_LINK2_01_D	(0.18)
#define	HARM_LINK2_01_H	(0.18)
#define	HARM_LINK2_02_R	(0.08)
#define	HARM_LINK2_02_H	(0.4)
#define	HARM_LINK3_01_R	(0.09)
#define	HARM_LINK3_01_H	(0.1)
#define	HARM_LINK3_02_R	(0.04)
#define	HARM_LINK3_02_H	(0.2)
#define	HARM_LINK4_01_R	(0.06)
#define	HARM_LINK4_01_H	(0.1)
#define	HARM_LINK4_02_R	(0.05)
#define	HARM_LINK4_02_H	(0.2)
#define	HARM_LINK5_01_R	(0.04)
#define	HARM_LINK5_01_H	(0.2)

	double	center[3];
	glRotated(-90.0, 1.0, 0.0, 0.0);	// 鉛直方向がz軸になるように回転
	// 土台
	glMaterialfv(GL_FRONT, GL_DIFFUSE, yellow);
	center[0] = 0.0; center[1] = 0.0; center[2] = HARM_LINK0_01_H/2.0;
	myCuboid(HARM_LINK0_01_W, HARM_LINK0_01_D, HARM_LINK0_01_H, center);
	glTranslated(0.0, 0.0, HARM_LINK0_01_H+HARM_LINK0_02_R);
	center[0] = 0.0; center[1] = 0.0; center[2] = 0.0;
	myCylinderHalf(HARM_LINK0_02_R, HARM_LINK0_02_H, SPHERE_APPROX, center);
	glRotated(-45.0, 0.0, 1.0, 0.0);
	center[0] = 0.0; center[1] = 0.0; center[2] = HARM_LINK0_03_H/2.0;
	myCylinder2(HARM_LINK0_03_R, HARM_LINK0_03_H, SPHERE_APPROX, center);
	glTranslated(0.0, 0.0, HARM_LINK0_03_H);
	// リンク1
	glRotated(save_ref_wam_jnt_ang[count][WAM_M1], 0.0, 0.0, 1.0);	// 1軸回転
	center[0] = 0.0; center[1] = 0.0; center[2] = HARM_LINK1_01_H/2.0;
	myCylinder2(HARM_LINK1_01_R, HARM_LINK1_01_H, SPHERE_APPROX, center);
	glTranslated(0.0, 0.0, HARM_LINK1_01_H);
	center[0] = 0.0; center[1] = 0.0; center[2] = 0.0;
	glRotated(225.0, 0.0, 1.0, 0.0);
	myCylinderHalf(HARM_LINK1_02_R, HARM_LINK1_02_H, SPHERE_APPROX, center);
	glRotated(-180.0, 0.0, 1.0, 0.0);
	center[0] = 0.0; center[1] = 0.0; center[2] = HARM_LINK1_02_R+HARM_LINK1_03_H/2.0;
	myCylinder2(HARM_LINK1_03_R, HARM_LINK1_03_H, SPHERE_APPROX, center);
	glTranslated(0.0, 0.0, HARM_LINK1_02_R+HARM_LINK1_03_H);
	// リンク2
	glRotated(save_ref_wam_jnt_ang[count][WAM_M2], 0.0, 0.0, 1.0);	// 2軸回転
	center[0] = 0.0; center[1] = 0.0; center[2] = HARM_LINK2_01_H/2.0;
	myCuboid(HARM_LINK2_01_W, HARM_LINK2_01_D, HARM_LINK2_01_H, center);
	glTranslated(HARM_LINK2_01_W/2.0, 0.0, HARM_LINK2_01_H/2.0);
	glRotated(90.0, 0.0, 1.0, 0.0);
	center[0] = 0.0; center[1] = 0.0; center[2] = HARM_LINK2_02_H/2.0;
	myCylinder2(HARM_LINK2_02_R, HARM_LINK2_02_H, SPHERE_APPROX, center);
	glTranslated(0.0, 0.0, HARM_LINK2_02_H);
	// リンク3
	glRotated(save_ref_wam_jnt_ang[count][WAM_M3], 0.0, 0.0, 1.0);	// 3軸回転
	center[0] = 0.0; center[1] = 0.0; center[2] = HARM_LINK3_01_H/2.0;
	myCylinder2(HARM_LINK3_01_R, HARM_LINK3_01_H, SPHERE_APPROX, center);
	glTranslated(0.0, 0.0, HARM_LINK3_01_H/2.0);
	glRotated(90.0, 0.0, 1.0, 0.0);
	center[0] = 0.0; center[1] = 0.0; center[2] = HARM_LINK3_02_H/2.0;
	myCylinder2(HARM_LINK3_02_R, HARM_LINK3_02_H, SPHERE_APPROX, center);
	glTranslated(0.0, 0.0, HARM_LINK3_02_H);
	// リンク4
	glRotated(save_ref_wam_jnt_ang[count][WAM_M4], 0.0, 0.0, 1.0);	// 4軸回転
	glRotated(-90.0, 0.0, 1.0, 0.0);
	center[0] = 0.0; center[1] = 0.0; center[2] = 0.0;
	myCylinder2(HARM_LINK4_01_R, HARM_LINK4_01_H, SPHERE_APPROX, center);
	// リンク5
	glRotated(save_ref_wam_jnt_ang[count][WAM_M5], 0.0, 0.0, 1.0);	// 5軸回転
	center[0] = 0.0; center[1] = 0.0; center[2] = HARM_LINK5_01_H/2.0;
	myCylinder2(HARM_LINK5_01_R, HARM_LINK5_01_H, SPHERE_APPROX, center);
	glTranslated(0.0, 0.0, HARM_LINK5_01_H);

	return 0;
}


//////////////////////////
// 06ハンド（HDSアーム用）
// 要確認!
//////////////////////////
int display_hand06(int count)
{
#define HAND_OFFSET1_Z_LENGTH	0.04
#define HAND_OFFSET2_Z_LENGTH	0.16
//#define HAND_WRIST_LENGTH	0.2*0.3
#define HAND_WRIST_OFFSET	0.025

	glRotated(90.0, 1.0, 0.0, 0.0);
	glRotated(90.0, 0.0, 1.0, 0.0);
	// 手首（9軸と10軸アクチュエータ部分のつもり）
	glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
	glRotated(save_ref_hand_jnt_ang[count][HAND_M11], 0.0, 1.0, 0.0);
	myCylinder(0.032, HAND_WRIST_OFFSET/2, SPHERE_APPROX);
	glTranslated(0.0, HAND_WRIST_OFFSET/2, 0.0);

	glPushMatrix();
	glRotated(90.0, 0.0, 0.0, 1.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
	myLinkPart(0.02, 0.025, 0.0, SPHERE_APPROX);
	glPopMatrix();

	glRotated(save_ref_hand_jnt_ang[count][HAND_M9], 1.0, 0.0, 0.0);
	glTranslated(0.0, 0.02+0.03, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
	myBox(0.015, 0.03, 0.01);
	glTranslated(0.0, 0.03, 0.01+0.0065+0.0025);
	// 掌
	glPushMatrix();
	glTranslated(0.0, 0.0, 0.0065+0.00125);
	glRotated(90.0, 1.0, 0.0, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, yellow);
	myPalm(0.018, 0.00125, 0.03, SPHERE_APPROX);
	glTranslated(0.0, -2*(0.0065+0.00125), 0.0);
	myPalm(0.018, 0.00125, 0.03, SPHERE_APPROX);
	glPopMatrix();
	// 中指
	glPushMatrix();
	hand_finger(save_ref_hand_jnt_ang[count][HAND_M3], save_ref_hand_jnt_ang[count][HAND_M4]);
	glPopMatrix();
	// 左指（正面から見て右）
	glPushMatrix();
	glTranslated(0.03, 0.0, 0.0);
	glRotated(save_ref_hand_jnt_ang[count][HAND_M7], 0.0, 0.0, 1.0);
	hand_finger(save_ref_hand_jnt_ang[count][HAND_M1], save_ref_hand_jnt_ang[count][HAND_M2]);
	glPopMatrix();
	// 右指（正面から見て左）
	glPushMatrix();
	glTranslated(-0.03, 0.0, 0.0);
	glRotated(save_ref_hand_jnt_ang[count][HAND_M8], 0.0, 0.0, 1.0);
	hand_finger(save_ref_hand_jnt_ang[count][HAND_M5], save_ref_hand_jnt_ang[count][HAND_M6]);
	glPopMatrix();

	return 0;
}


//////////////////////////
// 未完成
//////////////////////////
// 1つの関数で2台のWAMを表示し分ける
int display_wamm(int count, double *wam_jnt_ang)
{
	// 土台
	glTranslated(0.0, ARM_PEDESTAL_HEIGHT/2, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, yellow);
	myBox(0.1, ARM_PEDESTAL_HEIGHT/2, 0.1);
	glTranslated(0.0, ARM_PEDESTAL_HEIGHT/2, 0.0);
	// 図形回転
	glRotated(wam_jnt_ang[WAM_M1], 0.0, 1.0, 0.0);	// 1軸
	// サイドカバー
	glPushMatrix();
	glTranslated(ARM_UPPER_ARM_RADIUS+ARM_SIDECOVER_WIDTH/2, ARM_SIDECOVER_LENGTH/2, 0.0);
	glRotated(90.0, 0.0, 0.0, 1.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
	myLinkPart(ARM_SIDECOVER_RADIUS, ARM_SIDECOVER_WIDTH/2, ARM_SIDECOVER_LENGTH/2, SPHERE_APPROX);
	glTranslated(0.0, ARM_SIDECOVER_WIDTH+2*ARM_UPPER_ARM_RADIUS, 0.0);
	myLinkPart(ARM_SIDECOVER_RADIUS, ARM_SIDECOVER_WIDTH/2, ARM_SIDECOVER_LENGTH/2, SPHERE_APPROX);
	glTranslated(-ARM_SIDECOVER_LENGTH/2+0.02, -ARM_SIDECOVER_WIDTH/2-ARM_UPPER_ARM_RADIUS, 0.0);
	glRotated(90.0, 0.0, 0.0, 1.0);
	myCylinder(ARM_SIDECOVER_WIDTH/2+0.08, 0.02, SPHERE_APPROX);
	glPopMatrix();
	glTranslated(0.0, ARM_SIDECOVER_LENGTH, 0.0);
	// 図形回転
	glRotated(wam_jnt_ang[WAM_M2], 1.0, 0.0, 0.0);	// 2軸
	glRotated(wam_jnt_ang[WAM_M3], 0.0, 1.0, 0.0);	// 3軸
	// 上腕
	glTranslated(0.0, ARM_UPPER_ARM_LENGTH/2, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
	myCylinder(ARM_UPPER_ARM_RADIUS, ARM_UPPER_ARM_LENGTH/2, SPHERE_APPROX);
	glTranslated(0.0, ARM_UPPER_ARM_LENGTH/2, 0.0);
	// 肘関節
	glPushMatrix();
	glRotated(90.0, 0.0, 0.0, 1.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
	myCylinder(0.055, 0.051, SPHERE_APPROX);
	glPopMatrix();

	// 前腕
	glRotated(wam_jnt_ang[WAM_M4], 1.0, 0.0, 0.0);	// 4軸
#ifndef	ARM_HAND_CONNECT
	glTranslated(0.0, ARM_FRONT_ARM_LENGTH/2, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
	myCylinder(ARM_FRONT_ARM_RADIUS, ARM_FRONT_ARM_LENGTH/2, SPHERE_APPROX);
	// 手先
	glTranslated(0.0, ARM_FRONT_ARM_LENGTH/2, 0.0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, gray);
	glutSolidCube(0.7);
#endif

	return 0;
}
#endif
