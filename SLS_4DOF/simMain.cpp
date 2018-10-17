#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include "simMain.h"
#include "makeRobot.h"
#include "command.h"
#include "matBase.h"
#include "ctrlRobot.h"

#if	GLAPHIC_OPENGL
#include <GL/glut.h>	// stdlib.hより後に読み込む必要あり
#include "graphic.h"
#endif
#if	FLAG_SAVE_VIDEO
#include "video.h"
#endif

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// シミュレーション変数
SIM sim;
// ODE変数
dWorldID world;  // 動力学計算用ワールド
dSpaceID space;  // 衝突検出用スペース
dGeomID  ground; // 地面
dJointGroupID contactgroup; // コンタクトグループ
dJointFeedback force, *p_force;
MyObject base, arm[ARM_JNT], sensor, obj;    // 
dJointID f_joint, r_joint[ARM_JNT], f2_joint; // 固定関節と回転関節
// 描画変数
dsFunctions fn;
// 視点変数
static float xyz[3];
static float hpr[3];	// 単位はdeg

// シミュレーションモードの選択
#define	SIM_OBJ_IMPACT	0		// 対象衝突
#define	SIM_ADD_EXT_FORCE	1		// 定常外力
#define	SIM_DISPLACEMENT	0		// 強制変位

// 対象モードの選択
#define	SIM_OBJ_CASE1	0		//
#define	SIM_OBJ_CASE2	0		//
#define	SIM_OBJ_CASE3	1		//
#define	SIM_OBJ_INIT_ABS_VEL	(1.0)	// 対象の初期速度（移動方向をx軸に設定）

// 制御系の選択
#define	SIM_CTRL_MODE_HYBRID	0		// 弾塑性ハイブリッド制御
#define	SIM_CTRL_MODE_MAXWELL	1		// 塑性変形制御
#define	SIM_CTRL_MODE_VOIGT		0		// 弾性変形制御

// 初期設定変数
//#define	OBJ_RADIUS	(0.15)
#define	OBJ_RADIUS	(0.10)
//double	init_jnt_pos[ARM_JNT] = {3*PI/4.0, (PI-3*PI/4.0)*2.0, PI/4.0-(PI-3*PI/4.0), -PI/4.0*2.0};	// ロボット初期姿勢
//double	init_jnt_pos[ARM_JNT] = {7*PI/6.0, (PI-7*PI/6.0)*2.0, -PI/4.0-(PI-7*PI/6.0), PI/4.0*2.0};	// ロボット初期姿勢
double	init_jnt_pos[ARM_JNT] = {5*PI/6.0, (PI-5*PI/6.0)*2.0, -PI/4.0-(PI-5*PI/6.0), PI/4.0*2.0};	// ロボット初期姿勢
#if SIM_OBJ_CASE1
double	init_obj_pos[DIM3] = {-0.8-2*0.75*cos(PI/6.0)-2*0.6*cos(PI/4.0), -0.035, OBJ_RADIUS};	// 対象初期位置
double	init_obj_att[DIM3][DIM3] = { {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0} };	// 回転軸がy方向
#elif SIM_OBJ_CASE2
double	init_obj_pos[DIM3] = {-2*0.75*cos(PI/6.0)-2*0.6*cos(PI/4.0)-0.035, -0.8, OBJ_RADIUS};	// 対象初期位置
double	init_obj_att[DIM3][DIM3] = { {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}, {-1.0, 0.0, 0.0} };	// 回転軸がx方向
#elif SIM_OBJ_CASE3
double	init_obj_pos[DIM3] = {-0.8/sqrt(2.0)-2*0.75*cos(PI/6.0)-2*0.6*cos(PI/4.0), -0.8/sqrt(2.0), OBJ_RADIUS};	// 対象初期位置
double	init_obj_att[DIM3][DIM3] = { {sqrt(2.0)/2, sqrt(2.0)/2, 0.0}, {0.0, 0.0, -1.0}, {-sqrt(2.0)/2, sqrt(2.0)/2, 0.0} };	// 回転軸がx方向
#endif

////////////////////////////////////////////////////////
// 衝突検出計算コールバック関数
////////////////////////////////////////////////////////
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	static const int N = 10;     // 接触点数
	int	n, cnt;
	int	flag_ground, flag_sensor;	// 衝突検出用フラグ
	dContact contact[N];
	dBodyID b1, b2;
	dJointID c;

	// 地面との衝突検出
	flag_ground = ((o1 == ground) || (o2 == ground));
	// アームリンクとの衝突検出
//	flag_arm = ((o1 == arm.geom) || (o2 == arm.geom));
	// アーム手先との衝突検出
	flag_sensor = ((o1 == sensor.geom) || (o2 == sensor.geom));
	// 2つのボディがジョイントで結合されていたら衝突検出しない
	b1 = dGeomGetBody(o1);	b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;
	// 衝突設定
	n =  dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
	if(flag_ground){
		for(cnt=0; cnt<n; cnt++) {
			contact[cnt].surface.mode   = dContactBounce | dContactSoftERP | dContactSoftCFM;
			contact[cnt].surface.soft_erp   = 0.2;   // 接触点のERP
			contact[cnt].surface.soft_cfm   = 0.001; // 接触点のCFM
			contact[cnt].surface.mu     = 0.5; // 摩擦係数
			c = dJointCreateContact(world, contactgroup, &contact[cnt]);
			dJointAttach(c, dGeomGetBody(contact[cnt].geom.g1), dGeomGetBody(contact[cnt].geom.g2));
			// 転がり摩擦
//#define	COEF_FRIC_ROLL	0.006
#define	COEF_FRIC_ROLL	0.0006
//if(sim.step == 800) printf("%d \n", n);
			rolling_function( o1, COEF_FRIC_ROLL/n, contact+cnt );
			rolling_function( o2, COEF_FRIC_ROLL/n, contact+cnt );
		}
	}
	// 2DOF（並列ベースと直列ベースに対応）
	if(flag_sensor){
		for(cnt=0; cnt<n; cnt++) {
			contact[cnt].surface.mode   = dContactBounce | dContactSoftERP | dContactSoftCFM;
			contact[cnt].surface.soft_erp   = 0.2;   // 接触点のERP
			contact[cnt].surface.soft_cfm   = 0.005; // 接触点のCFM
			contact[cnt].surface.mu     = 0.5; // 摩擦係数
			contact[cnt].surface.bounce   = 0.5; // 反発係数
			c = dJointCreateContact(world, contactgroup, &contact[cnt]);
			dJointAttach(c, dGeomGetBody(contact[cnt].geom.g1), dGeomGetBody(contact[cnt].geom.g2));
		}
		if(n!=0)	sim.state_contact = 1;
	}
}

////////////////////////////////////////////////////////
// 転がり摩擦
////////////////////////////////////////////////////////
void rolling_function( dGeomID o, dReal coef, dContact *c )//床摩擦関数
{
//	if (o && dGeomGetClass(o) == dSphereClass) {
//	if (o && dGeomGetClass(o) == dCylinderClass) {
	if (o == obj.geom) {
		dBodyID b = dGeomGetBody(o);
		if (!b) return;
		dMass m;
		dBodyGetMass( b, &m );
		dReal* normal = c->geom.normal; // 垂直抗力ベクトル
		dReal w = m.mass*(normal[2]*9.8); // 質量, (memo:角度差cosΘ = (normal[0]*0.0 + normal[1]*0.0 + normal[2]*1.0))
//		dReal r = dGeomSphereGetRadius( o ); // 半径
		dReal r = OBJ_RADIUS; // 半径
		dReal F = coef * (w / r ); // 転がり摩擦(力)
		dReal T = F * r; // 転がり摩擦のトルク?
//		const dReal* av = dBodyGetAngularVel(b);
		const dReal* av = dBodyGetAngularVel(obj.body);
		dReal a_speed = sqrt(av[0]*av[0] + av[1]*av[1] + av[2]*av[2]); // 回転スピード
//if(sim.step == 800) printf("%f %f %f %f\n", c->geom.normal[0], c->geom.normal[1], c->geom.normal[2], a_speed);
		if (a_speed > 1.0e-5) {
			dReal n_av[3] = { av[0]/a_speed, av[1]/a_speed, av[2]/a_speed }; // 回転方向の正規化
			dBodyAddTorque( b, -n_av[0]*T, -n_av[1]*T, -n_av[2]*T ); // 転がり摩擦をトルクとしてあたえる
		}else {
			dBodySetAngularVel( b, 0.0f, 0.0f, 0.0f ); // 停止
		}
	}
}

////////////////////////////////////////////////////////
// 関節制御
// 戻り値：関節一般化力（直動関節では力，回転関節ではトルク）
////////////////////////////////////////////////////////
int ctrlArm(dReal *jnt_force)
{
	int	jnt, crd;
	dReal	fric_force[ARM_JNT];
	static Matrix	tau;		// 関節座標変数

	// 初期化
	if(sim.step == 0){
		matInit(&tau,ARM_JNT,1);
		// 初期化
		ctrlInit(&sim);
		// 初期値保存
		for(crd=0;crd<DIM2;crd++){
			sim.var.r0.el[crd][0] = sim.eff_pos[crd];
			sim.var.p0.el[crd][0] = sim.node_pos[crd];
		}
////////////////////////////////////////////////////////
// 初期偏差を設定
#if SIM_DISPLACEMENT
//		sim.var.r0.el[CRD_X][0] = sim.eff_pos[CRD_X]-0.1;
//		sim.var.r0.el[CRD_Y][0] = sim.eff_pos[CRD_Y];
		sim.var.r0.el[CRD_X][0] = -2.0;
		sim.var.r0.el[CRD_Y][0] = 0.2;
#endif
////////////////////////////////////////////////////////
		// 目標インピーダンス（手先）
#if SIM_CTRL_MODE_MAXWELL & SIM_ADD_EXT_FORCE
		sim.imp.M.el[CRD_X][CRD_X] = 2.0;	sim.imp.M.el[CRD_Y][CRD_Y] = 2.0;	// 慣性
		sim.imp.C.el[CRD_X][CRD_X] = 4.0;	sim.imp.C.el[CRD_Y][CRD_Y] = 4.0;	// 粘性
		sim.imp.K.el[CRD_X][CRD_X] = 100.0;	sim.imp.K.el[CRD_Y][CRD_Y] = 100.0;	// 弾性
//		sim.imp.K.el[CRD_X][CRD_X] = 40.0;	sim.imp.K.el[CRD_Y][CRD_Y] = 40.0;	// 弾性
//		sim.imp.K0.el[CRD_X][CRD_X] = 3.0;	sim.imp.K0.el[CRD_Y][CRD_Y] = 10.0;	// 弾性(SLS並列バネ)	// xyで非対称の場合
		sim.imp.K0.el[CRD_X][CRD_X] = 1.0;	sim.imp.K0.el[CRD_Y][CRD_Y] = 1.0;	// 弾性(SLS並列バネ)
#elif SIM_CTRL_MODE_MAXWELL & SIM_OBJ_IMPACT
//		sim.imp.m_d[CRD_X] = 0.5;	sim.imp.m_d[CRD_Y] = 0.5;	// 慣性
//		sim.imp.c_d[CRD_X] = 0.8;	sim.imp.c_d[CRD_Y] = 0.8;	// 粘性
//		sim.imp.k_d[CRD_X] = 5.2;	sim.imp.k_d[CRD_Y] = 5.2;	// 弾性
		sim.imp.M.el[CRD_X][CRD_X] = 0.5;	sim.imp.M.el[CRD_Y][CRD_Y] = 0.5;	// 慣性
		sim.imp.C.el[CRD_X][CRD_X] = 0.8;	sim.imp.C.el[CRD_Y][CRD_Y] = 0.8;	// 粘性
		sim.imp.K.el[CRD_X][CRD_X] = 5.2;	sim.imp.K.el[CRD_Y][CRD_Y] = 5.2;	// 弾性
//		sim.imp.m_d[CRD_X] = 1.0;	sim.imp.m_d[CRD_Y] = 1.0;	// 慣性
//		sim.imp.c_d[CRD_X] = 0.2;	sim.imp.c_d[CRD_Y] = 0.2;	// 粘性
//		sim.imp.k_d[CRD_X] = 15.2;	sim.imp.k_d[CRD_Y] = 15.2;	// 弾性
		sim.imp.K0.el[CRD_X][CRD_X] = 0.0;	sim.imp.K0.el[CRD_Y][CRD_Y] = 0.0;	// 弾性(SLS並列バネ)
#elif SIM_CTRL_MODE_HYBRID & SIM_ADD_EXT_FORCE
		sim.imp.m_d[CRD_X] = 2.0;	sim.imp.m_d[CRD_Y] = 1.0;	// 慣性
		sim.imp.c_d[CRD_X] = 4.0;	sim.imp.c_d[CRD_Y] = 0.5;	// 粘性
		sim.imp.k_d[CRD_X] = 100.0;	sim.imp.k_d[CRD_Y] = 50.0;	// 弾性
#elif SIM_DISPLACEMENT
//		sim.imp.m_d[CRD_X] = 10.0;	sim.imp.m_d[CRD_Y] = 10.0;	// 慣性
//		sim.imp.c_d[CRD_X] = 100.0;	sim.imp.c_d[CRD_Y] = 1.0;	// 粘性
//		sim.imp.k_d[CRD_X] = 10.0;	sim.imp.k_d[CRD_Y] = 10.0;	// 弾性
		sim.imp.M.el[CRD_X][CRD_X] = 2.0;	sim.imp.M.el[CRD_Y][CRD_Y] = 2.0;	// 慣性
		sim.imp.C.el[CRD_X][CRD_X] = 4.0;	sim.imp.C.el[CRD_Y][CRD_Y] = 4.0;	// 粘性
		sim.imp.K.el[CRD_X][CRD_X] = 50.0;	sim.imp.K.el[CRD_Y][CRD_Y] = 50.0;	// 弾性
		sim.imp.K0.el[CRD_X][CRD_X] = 10.0;	sim.imp.K0.el[CRD_Y][CRD_Y] = 10.0;	// 弾性(SLS並列バネ)
#endif
	}
#if SIM_CTRL_MODE_HYBRID
	// MaxwellとVoigtの方向切替
	if(sim.step == IMP_SWITCH_STEP){
		// 目標インピーダンス（手先）
		sim.imp.M.el[CRD_X][CRD_X] = 1.0;	sim.imp.M.el[CRD_Y][CRD_Y] = 2.0;	// 慣性
		sim.imp.C.el[CRD_X][CRD_X] = 0.5;	sim.imp.C.el[CRD_Y][CRD_Y] = 4.0;	// 粘性
		sim.imp.K.el[CRD_X][CRD_X] = 50.0;	sim.imp.K.el[CRD_Y][CRD_Y] = 100.0;	// 弾性
		for(crd=0;crd<2;crd++){
			sim.var.r0.el[crd][0] = sim.eff_pos[crd];
			sim.var.p0.el[crd][0] = sim.node_pos[crd];
		}
	}
#endif

	// ロボット機構設定(ODEでは関節摩擦を手動で設定)
	for(jnt=0;jnt<ARM_JNT;jnt++){
		fric_force[jnt] = -sim.dyn.V[jnt] * sim.jnt_vel[jnt];
		dJointAddHingeTorque(r_joint[jnt], fric_force[jnt]);
	}
	// 手先変数代入
	for(crd=0;crd<DIM2;crd++){
		sim.var.r.el[crd][0] = sim.eff_pos[crd];
		sim.var.dr.el[crd][0] = sim.eff_vel[crd];
		sim.var.p.el[crd][0] = sim.node_pos[crd];
		sim.var.dp.el[crd][0] = sim.node_vel[crd];
		sim.var.F.el[crd][0] = sim.eff_force[crd];
	}
	// 関節変数設定
	for(jnt=0;jnt<ARM_JNT;jnt++){
//		q.el[jnt][0] = sim.jnt_pos[jnt]; dq.el[jnt][0] = sim.jnt_vel[jnt];
		sim.var.q.el[jnt][0] = sim.jnt_pos[jnt];	sim.var.dq.el[jnt][0] = sim.jnt_vel[jnt];
	}
	// パラメータセット
	armDynPara(&sim);

	// 行列初期処理
	ctrlPreProcessing(&sim);

	// 制御指令計算
#if SIM_CTRL_MODE_HYBRID
	ctrlHybrid(&tau, &Mq, &h, &J, &dJ, &q, &dq, &re, &dre, &F, &Fint, &Md, &Cd, &Kd);
#elif SIM_CTRL_MODE_MAXWELL
//	ctrlMaxwellParallel(&tau, &Mq, &h, &J, &dJ, &q, &dq, &re, &dre, &F, &Fint, &Md, &Cd, &Kd);
//	ctrlMaxwellSeries(&tau, &Mq, &h, &J, &dJ, &Jp, &dJp, &q, &dq, &rs, &drs, &F, &Md, &Cd, &Kd);
//	ctrlMaxwellSeries(&sim, &tau);
//	ctrlSLS(&sim, &tau);
	ctrlSLS2(&sim, &tau);
//	ctrlMaxwellSeries2(&tau, &Mq, &h, &J, &dJ, &Jp, &dJp, &q, &dq, &rs, &dre, &F, &Md, &Cd, &Kd);
#elif SIM_CTRL_MODE_VOIGT
	ctrlVoigt(&tau, &Mq, &h, &J, &dJ, &q, &dq, &re, &dre, &F, &Md, &Cd, &Kd);
#endif
	for(jnt=0;jnt<ARM_JNT;jnt++)	jnt_force[jnt] = tau.el[jnt][0];
	// 駆動力制限
//	for(jnt=0;jnt<ARM_JNT;jnt++)	if(jnt_force[jnt] > 100 || jnt_force[jnt] < -100)	jnt_force[jnt] = 0.0;
	// 駆動力入力
	for(jnt=0;jnt<ARM_JNT;jnt++)	dJointAddHingeTorque(r_joint[jnt], jnt_force[jnt]);		// トルクは上書きではなくインクリメントされる
	return	0;
}

////////////////////////////////////////////////////////
// 距離関数
////////////////////////////////////////////////////////
int calcDist()
{
	double	tmp[DIM2];
	double	C1234, S1234;
	double	R[12];
	const dReal	*p_R = R;
	double	norX, norY;
	// 三角関数
	C1234 = cos(sim.jnt_pos[ARM_M1]+sim.jnt_pos[ARM_M2]+sim.jnt_pos[ARM_M3]+sim.jnt_pos[ARM_M4]);
	S1234 = sin(sim.jnt_pos[ARM_M1]+sim.jnt_pos[ARM_M2]+sim.jnt_pos[ARM_M3]+sim.jnt_pos[ARM_M4]);
	// 対象回転軸(p_R[2],p_R[6],p_R[10])のz成分を除去してxy方向で正規化
	p_R = dBodyGetRotation(obj.body);
	norX = p_R[2]/sqrt(p_R[2]*p_R[2]+p_R[6]*p_R[6]);	
	norY = p_R[6]/sqrt(p_R[2]*p_R[2]+p_R[6]*p_R[6]);
	// 手先に一番近い点の計算
	if(C1234*p_R[2]+S1234*p_R[6] > 0){
//		printf("%f %f \n", p_R[2], p_R[6]);
		tmp[CRD_X] = sim.obj_pos[CRD_X] -obj.l/2.0*norX +obj.r*norY;
		tmp[CRD_Y] = sim.obj_pos[CRD_Y] -obj.l/2.0*norY -obj.r*norX;
//		printf("%f %f \n", tmp[0], tmp[1]);
	}else{
		tmp[CRD_X] = sim.obj_pos[CRD_X] +obj.l/2.0*norX +obj.r*norY;
		tmp[CRD_Y] = sim.obj_pos[CRD_Y] +obj.l/2.0*norY -obj.r*norX;
	}
	// 距離計算
	sim.dist = fabs(C1234*(tmp[CRD_X]-sim.eff_pos[CRD_X])+S1234*(tmp[CRD_Y]-sim.eff_pos[CRD_Y]));
	return	0;
}

////////////////////////////////////////////////////////
// 外力設定
////////////////////////////////////////////////////////
int addExtForce()
{
	double ext_force[DIM3];
	// 外力を設定
#if 0
	ext_force[CRD_X] = 2.0;//0.2;
	ext_force[CRD_Y] = 2.0;//0.2;
	ext_force[CRD_Z] = 0.0;
#elif 0
	if(sim.step <= 2000){
		ext_force[CRD_X] = 2.0-time;
		ext_force[CRD_Y] = 2.0-time;
		ext_force[CRD_Z] = 2.0-time;
	}else{
		ext_force[CRD_X] = 0.0;
		ext_force[CRD_Y] = 0.0;
		ext_force[CRD_Z] = 0.0;
	}
#else
	if(sim.step <= 7000){
		ext_force[CRD_X] = 2.0;
		ext_force[CRD_Y] = 2.0;
		ext_force[CRD_Z] = 0.0;
	}else{
		ext_force[CRD_X] = 0.0;
		ext_force[CRD_Y] = 0.0;
		ext_force[CRD_Z] = 0.0;
	}
#endif
	// 手先リンク表面の中心に外力入力
	dBodyAddForceAtPos(sensor.body, ext_force[CRD_X], ext_force[CRD_Y], ext_force[CRD_Z], sim.eff_pos[CRD_X], sim.eff_pos[CRD_Y], sim.eff_pos[CRD_Z]);
	return	0;
}

////////////////////////////////////////////////////////
// シミュレーションループ
////////////////////////////////////////////////////////
static void simLoop(int pause)
{
	int	jnt, crd;

	if(!pause){
		// 初期設定
#if SIM_OBJ_IMPACT
//		if(sim.step == 0)	dBodySetLinearVel(obj.body, 1.0, 0.0, 0.0);		// 対象速度
		if(sim.step == 0)	dBodySetLinearVel(obj.body, sim.init_obj_att[AXIS_X][CRD_X]*SIM_OBJ_INIT_ABS_VEL, sim.init_obj_att[AXIS_X][CRD_Y]*SIM_OBJ_INIT_ABS_VEL, sim.init_obj_att[AXIS_X][CRD_Z]*SIM_OBJ_INIT_ABS_VEL);		// 対象速度
		if(sim.step == 0)	dBodySetAngularVel(obj.body, sim.init_obj_att[AXIS_Z][CRD_X]*SIM_OBJ_INIT_ABS_VEL/OBJ_RADIUS, sim.init_obj_att[AXIS_Z][CRD_Y]*SIM_OBJ_INIT_ABS_VEL/OBJ_RADIUS, sim.init_obj_att[AXIS_Z][CRD_Z]*SIM_OBJ_INIT_ABS_VEL/OBJ_RADIUS);		// 対象角速度
#elif SIM_ADD_EXT_FORCE
		if(sim.step == 0)	dBodyDisable(obj.body);		// 対象無効化
#endif
		// 外力設定
#if SIM_ADD_EXT_FORCE
		addExtForce();
#endif
		// 状態取得
		for(jnt=0;jnt<ARM_JNT;jnt++){
			sim.jnt_pos[jnt] = dJointGetHingeAngle(r_joint[jnt]) + sim.init_jnt_pos[jnt];	// 関節位置（x軸が基準角0）
			sim.jnt_vel[jnt] = dJointGetHingeAngleRate(r_joint[jnt]);	// 関節速度
		}
		dBodyGetRelPointPos(sensor.body, 0.0, 0.0, sensor.l/2.0, sim.eff_pos);			// 手先位置
		dBodyGetRelPointVel(sensor.body, 0.0, 0.0, sensor.l/2.0, sim.eff_vel);			// 手先速度
		dBodyGetRelPointPos(arm[ARM_M2].body, 0.0, 0.0, arm[ARM_M2].l/2.0, sim.node_pos);			// 結節点位置
		dBodyGetRelPointVel(arm[ARM_M2].body, 0.0, 0.0, arm[ARM_M2].l/2.0, sim.node_vel);			// 結節点速度
		p_force = dJointGetFeedback(f2_joint);
		for(crd=0;crd<DIM3;crd++){
			sim.eff_force[crd] = -p_force->f1[crd];	// 対象がセンサに及ぼしている力=センサが関節に及ぼしている力
			sim.obj_pos[crd] = (dBodyGetPosition(obj.body))[crd];		// 対象位置
			sim.obj_vel[crd] = (dBodyGetLinearVel(obj.body))[crd];		// 対象速度
		}
		// 距離計算
		calcDist();
		// 力計算
		ctrlArm(sim.jnt_force);
		// 過去データとして代入
		for(jnt=0;jnt<ARM_JNT;jnt++)	sim.past_jnt_pos[jnt] = sim.jnt_pos[jnt];
		// 現在値を保存領域へコピー
		copyData();
		// シミュレーションを１ステップ進行
		dSpaceCollide(space, 0, &nearCallback);
		dWorldStep(world, SIM_CYCLE_TIME);
		dJointGroupEmpty(contactgroup); // ジョイントグループを空にする
		sim.step++;
#if FLAG_DRAW_SIM
		// 終了設定
		if(sim.step == DATA_CNT_NUM)	dsStop();
#endif
	}
#if FLAG_DRAW_SIM
	drawRobot(); // ロボットの描画
#if SIM_OBJ_IMPACT
	drawObject(); // 衝突対象の描画
#elif SIM_ADD_EXT_FORCE
	drawExtForce(); // 外力の描画
#endif
#endif
#if FLAG_DRAW_SIM & FLAG_SAVE_IMAGE
	// 画像保存
	if(!pause){
		if(sim.step % SAVE_IMG_RATE == 0)	saveImage(640, 480);	// SAVE_IMG_RATE毎に画像保存
	}
#endif
#if	FLAG_SAVE_VIDEO
	if(sim.step % SAVE_VIDEO_RATE == 0)	save_video();
#endif
}

////////////////////////////////////////////////////////
// シミュレーションリスタート
////////////////////////////////////////////////////////
static void restart()
{
	int	jnt, crd;
	//変数初期化
	sim.step = 0;		//ステップ数初期化
	sim.state_contact = 0;				// 状態変数初期化
	sim.dist = 0.0;
	for(jnt=0;jnt<ARM_JNT;jnt++){
		sim.ref_jnt_pos[jnt] = 0.0;
		sim.jnt_pos[jnt] = 0.0;
		sim.jnt_vel[jnt] = 0.0;
		sim.jnt_force[jnt] = 0.0;
		sim.past_jnt_pos[jnt] = 0.0;
	}
	for(crd=0;crd<DIM3;crd++){
		sim.eff_pos[crd] = 0.0;
		sim.eff_vel[crd] = 0.0;
		sim.eff_force[crd] = 0.0;
		sim.node_pos[crd] = 0.0;
		sim.node_vel[crd] = 0.0;
	}
	// ODE
	destroyRobot();  // ロボットの破壊
	destroyObject();  // 衝突対象の破壊
	dJointGroupDestroy(contactgroup);     // ジョイントグループの破壊
	contactgroup = dJointGroupCreate(0);  // ジョイントグループの生成
	createRobot(&sim);                      // ロボットの生成
	createObject(&sim);                      // 衝突対象の生成
}

////////////////////////////////////////////////////////
// キーボードコマンドの処理関数
////////////////////////////////////////////////////////
static void command(int cmd)
{
	switch(cmd){
		case 'x': xyz[0] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// x方向
		case 'X': xyz[0] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -x方向
		case 'y': xyz[1] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// y方向
		case 'Y': xyz[1] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -y方向
		case 'z': xyz[2] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// z方向
		case 'Z': xyz[2] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -z方向
		case 'u':	dBodyAddForce(obj.body, 500.0, 0.0, 0.0);	break;
		case 'r':	restart();	break;
		case 'q':	dsStop();	break;
		default :printf("key missed \n")             ; break;
	}
}

////////////////////////////////////////////////////////
// ODE初期設定
////////////////////////////////////////////////////////
static void start()
{
#if 0
	xyz[0] = -0.2;	xyz[1] = 2.5;	xyz[2] = 0.5;
	hpr[0] = -90.0;	hpr[1] = 0.0;	hpr[2] = 0.0;	// +yからの視点(右が-x,上が+z)
#elif 0
	xyz[0] = 2.5;	xyz[1] = 0.2;	xyz[2] = 0.5;
	hpr[0] = -180.0;	hpr[1] = 0.0;	hpr[2] = 0.0;	// +xからの視点(右が+y,上が+z)
#elif 1
	xyz[0] = -1.2;	xyz[1] = 0.0;	xyz[2] = 2.5;
	hpr[0] = 0.0;	hpr[1] = -90.0;	hpr[2] = 90.0;	// +zからの視点(右が+x,上が+y)
#endif
	dsSetViewpoint(xyz, hpr);               // 視点，視線の設定
	dsSetSphereQuality(3);                 // 球の品質設定
#if	FLAG_SAVE_VIDEO
	init_video();
#endif
}

////////////////////////////////////////////////////////
// 描画関数の設定
////////////////////////////////////////////////////////
void setDrawStuff()
{
	fn.version = DS_VERSION;    // ドロースタッフのバージョン
	fn.start   = &start;        // 前処理 start関数のポインタ
	fn.step    = &simLoop;      // simLoop関数のポインタ
	fn.command = &command;      // キー入力関数へのポインタ
	fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH; // テクスチャ
}

////////////////////////////////////////////////////////
// コマンド実行
////////////////////////////////////////////////////////
int exeCmd(int argc, char *argv[])
{
	int   command;
	char  buf[BUFSIZE];
	static int hist = 0;   // コマンド保存
	static int incount = 0;
	int	flag_quit = 0;

	while(1){
#if 0
		// コマンド表示 & 取得
		getChar(buf,"command > ");
		command = atoi(buf);      // アスキーコードに変換
		sscanf(buf, "%c", &command);
		// コマンド処理
		switch(command){
//			case 'd': drawData(); break;
			case 'h': showHelp(); break;
			case 'q': flag_quit = 1;	// 終了判定
			default: ;
		}
		if(flag_quit)	break;
#endif

		// シミュレーションループ
#if FLAG_DRAW_SIM
		dsSimulationLoop(argc, argv, DISPLAY_WIDTH, DISPLAY_HEIGHT, &fn);
#else
		while(1){
			simLoop(0);
			if(sim.step == DATA_CNT_NUM)	break;				// 終了設定
		}
#endif
		// ファイル保存
		sprintf(sim.data_file_name, FILENAME_DATA, incount);		// ファイル名を連番に設定
		sprintf(sim.filename_info, FILENAME_INFO, incount);		// ファイル名を連番に設定
		saveData();
		saveInfo();
		saveGraph();
		// シミュレーションリスタート
		restart();
		// インクリメント
		incount++;

		// デバッグ用（1回で終了）
		break;
	}
	// ファイル保存
//	if(incount != 0)	saveGraph2(incount);
#if	FLAG_SAVE_VIDEO
	final_video();
#endif
	return 0;
}

////////////////////////////////////////////////////////
// main関数
////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
	int	jnt, crd, axis;
	// 初期設定
	dInitODE();
	world        = dWorldCreate();
	space        = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
#if FLAG_DRAW_SIM
	// 描画設定
	setDrawStuff();		// ドロースタッフ
#endif
	// 環境設定
	dWorldSetGravity(world, 0,0, -9.8);	// 重力設定
	dWorldSetERP(world, 0.9);          // ERPの設定
	dWorldSetCFM(world, 1e-4);         // CFMの設定
	// パラメータ設定
	for(jnt=0;jnt<ARM_JNT;jnt++)	sim.init_jnt_pos[jnt] = init_jnt_pos[jnt];
	for(crd=0;crd<DIM3;crd++){
		sim.init_obj_pos[crd] = init_obj_pos[crd];
		for(axis=0;axis<DIM3;axis++)	sim.init_obj_att[axis][crd] = init_obj_att[axis][crd];
	}
	// 物体生成
	ground = dCreatePlane(space, 0, 0, 1, 0);		// 地面の設定
	createRobot(&sim);
	createObject(&sim);
	// 乱数種設定
//	dRandSetSeed(0);
	// コマンド開始
	exeCmd(argc, argv);
	// シミュレーションループ
//	dsSimulationLoop (argc, argv, 640, 480, &fn);
	// 終了設定
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();
	// ファイル保存
//	saveData();
//	saveGraph();

#if	GLAPHIC_OPENGL
	// グラフィック表示
	glutInitWindowPosition(200, 100);
	glutInitWindowSize(600, 600);
	glutInit(&argc, (char**)argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow((char*)argv[0]);
	glutDisplayFunc(display);
	glutReshapeFunc(resize);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);
	init();
	glutMainLoop();
#endif
	return 0;
}
