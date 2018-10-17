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
// �o�C�i���t���O
// ON(1)��OFF(0)�̂ݐݒ�\
////////////////////////////////////////////////////////
#define	GLAPHIC_OPENGL		0		// OpenGL�ŕ`��
#define	FLAG_DRAW_SIM		1		// ODE�̕W���`��
#define	FLAG_SAVE_IMAGE		1		// �摜�ۑ�
#define	FLAG_SAVE_VIDEO		0		// ����ۑ�(OpenCV���K�v)

////////////////////////////////////////////////////////
// define��`
////////////////////////////////////////////////////////
#ifndef PI
#define PI (3.14159265358979323846)
#endif
// ��ʕ\����`
#define	DISPLAY_WIDTH	640
#define	DISPLAY_HEIGHT	480
// �����E���W���ʒ�`
#define	DIM2	2
#define	DIM3	3
#define	CRD_X	0
#define	CRD_Y	1
#define	CRD_Z	2
#define	AXIS_X	0
#define	AXIS_Y	1
#define	AXIS_Z	2
#define DIR_LONG_AXIS_Z	3	// ��������(dMassSetCylinderTotal�Ȃǂɗ��p)
// �萔��`
#define SYSTEM_CYCLE_TIME	(0.001)	// �����p�T�C�N���^�C��
#define SIM_CYCLE_TIME	(0.001)	// �V�~�����[�V�����p�T�C�N���^�C��
#define DATA_CNT_NUM	6000	// �f�[�^�ۑ��J�E���g��
#define SAVE_IMG_RATE	200		// �摜�ۑ��Ԋu�J�E���g��
#define SAVE_VIDEO_RATE	33		// ����ۑ��Ԋu�J�E���g��
// �������`
#define GNUPLOT_PATH	"\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\""	// �p�X�ɋ󔒂����邽��[\"]��O��ɒǉ�
//#define GNUPLOT_PATH	"\"C:\\Program Files (x86)\\gnuplot\\bin\\gnuplot.exe\""	// �p�X�ɋ󔒂����邽��[\"]��O��ɒǉ�
#define FILENAME_DATA	"data_%.3d.txt"		// �A��3���Ή�
#define FILENAME_INFO	"info_%.3d.txt"		// �A��3���Ή�
#define FILENAME_GRAPH1	"img_jnt_pos.png"
#define FILENAME_GRAPH2	"img_jnt_vel.png"
#define FILENAME_GRAPH3	"img_jnt_force.png"
#define FILENAME_GRAPH4	"img_eff_force.png"
#define FILENAME_GRAPH5	"img_err.png"
#define	DATA_FILE_NAME_MAXLEN	15
// �A�[���萔
#define	ARM_JNT	4
#define	ARM_M1	0
#define	ARM_M2	1
#define	ARM_M3	2
#define	ARM_M4	3

////////////////////////////////////////////////////////
// �\���̒�`
////////////////////////////////////////////////////////
// ODE�p�[�c�p�\����
typedef struct{       // MyObject�\����
	dBodyID body;        // �{�f�B(����)��ID�ԍ��i���͊w�v�Z�p�j
	dGeomID geom;        // �W�I���g����ID�ԍ�(�Փˌ��o�v�Z�p�j
	dReal  l,r,m;       // ����[m], ���a[m]�C����[kg]
	dReal	sides[DIM3];	// ������x,y,z�̕Ӓ�
} MyObject;

// �ϐ��\����
typedef struct{
	Matrix	q;	// �֐ߊp
	Matrix	r;	// ���ʒu
	Matrix	F;	// ���O��
	Matrix	p;	// �ڑ��_�ʒu
	// �⑫�ϐ�
	Matrix	q0;	// �����֐ߊp
	Matrix	r0;	// �������ʒu
	Matrix	p0;	// �����ڑ��ʒu
	Matrix	dq;	// �֐ߑ��x
	Matrix	dr;	// ��摬�x
	Matrix	dp;	// �ڑ��_���x
} Variable;

// �^���w�\����
typedef struct{       //
	double	l[ARM_JNT];		// �����N��
	double	lg[ARM_JNT];		// �����N�d�S�ʒu�܂ł̒���
	double	r[ARM_JNT];		// �����N���a�i���������j
	Matrix	J;	// ���R�r�A��
	// �⑫�ϐ�
	Matrix	Jp;	// �ڑ��_���R�r�A��
	Matrix	dJ;	// ���R�r�A������
	Matrix	dJp;	// �ڑ��_���R�r�A������
	Matrix	Jt, Jinv;	// �]�u�s��C�t�s��
	Matrix	Jpt, Jpinv;	// �]�u�s��C�t�s��
} Kinematics;

// ���͊w�\����
// Mq*ddq + h + V*dq = tau + Jt*F
typedef struct{       //
	double	m[ARM_JNT];		// �����N����
	Matrix	Mq;		// ������
	Matrix	h;	// �R���I���E���S�͍�
	double	V[ARM_JNT];	// �S�����C�W��
	// �⑫�ϐ�
	Matrix	dMq;		// ����������
} Dynamics;

// �C���s�[�_���X�\����
typedef struct{
	Matrix	M, C, K;	// �ڕW�C���s�[�_���X�i�����W�j
	Matrix	K0;	// SLS�p
	double	T[DIM3];	// ����
	// �⑫�ϐ�
	Matrix	dM, dC, dK;	// �ڕW�C���s�[�_���X�����i�����W�j
	Matrix	Minv, Cinv, Kinv;	// �t�s��
} Impedance;

// �V�~�����[�V�����\����
typedef struct{
	// ����p�ϐ�
	int	step;	//�o�߃X�e�b�v��
	int state_contact;		// �ڐG���(0:OFF, 1:ON)
	double	dist;		// �A�[���ƑΏۂ̋���
	double	ref_jnt_pos[ARM_JNT];
	double	jnt_pos[ARM_JNT];		// �֐߈ʒu
	double	jnt_vel[ARM_JNT];		// �֐ߑ��x
	double	jnt_force[ARM_JNT];		// �֐ߗ�
	double	past_jnt_pos[ARM_JNT];
	double	eff_pos[DIM3];		// ���ʒu
	double	eff_vel[DIM3];		// ��摬�x
	double	eff_force[DIM3];		// ����
	double	node_pos[DIM3];		// ���ߓ_�ʒu
	double	node_vel[DIM3];		// ���ߓ_���x
	double	obj_pos[DIM3];		// �Ώۈʒu
	double	obj_vel[DIM3];		// �Ώۑ��x
	// �����ϐ�
	double	init_jnt_pos[ARM_JNT];
	double	init_obj_pos[DIM3];
	double	init_obj_att[DIM3][DIM3];	// ��΍��W�ɂ�����Ώۍ��W���̎p���i���͐��K�������j
	// �ϐ��\����
	Variable	var;
	// �^���w�ϐ�
	Kinematics	kine;
	// ���͊w�ϐ�
	Dynamics	dyn;
	// �C���s�[�_���X�ϐ�
	Impedance	imp;
	// �ۑ��p�f�[�^�ϐ�
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
	// �ۑ��p�t�@�C�����ϐ�
	char	data_file_name[DATA_FILE_NAME_MAXLEN];
	char	filename_info[DATA_FILE_NAME_MAXLEN];
	char	filename_graph[DATA_FILE_NAME_MAXLEN];
} SIM;

////////////////////////////////////////////////////////
// �v���g�^�C�v
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
