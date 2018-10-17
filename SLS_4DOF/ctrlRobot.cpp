#include "simMain.h"
#include "ctrlRobot.h"

// シミュレーション変数
extern	SIM sim;

////////////////////////////////////////////////////////
// 初期化
////////////////////////////////////////////////////////
int ctrlInit(SIM *sim)
{
#if 0
	double	len[ARM_JNT] = {0.75, 0.75, 0.6, 0.6};
	double	len_g[ARM_JNT] = {0.75/2, 0.75/2, 0.6/2, 0.6/2};
	double	rad[ARM_JNT] = {0.125, 0.10, 0.10, 0.10};
	double	mass[ARM_JNT] = {1.0, 0.8, 0.6, 0.6};
	double	vis[ARM_JNT] = {1.0, 1.0, 1.0, 1.0};
#endif
	// 固有パラメータ設定
	sim->kine.l[ARM_M1] = 0.75;	sim->kine.l[ARM_M2] = 0.75;	sim->kine.l[ARM_M3] = 0.6;	sim->kine.l[ARM_M4] = 0.6;
	sim->kine.lg[ARM_M1] = sim->kine.l[ARM_M1]/2;	sim->kine.lg[ARM_M2] = sim->kine.l[ARM_M2]/2;	sim->kine.lg[ARM_M3] = sim->kine.l[ARM_M3]/2;	sim->kine.lg[ARM_M4] = sim->kine.l[ARM_M4]/2;
	sim->kine.r[ARM_M1] = 0.125;	sim->kine.r[ARM_M2] = 0.10;	sim->kine.r[ARM_M3] = 0.10;	sim->kine.r[ARM_M4] = 0.10;
	sim->dyn.m[ARM_M1] = 1.0;	sim->dyn.m[ARM_M2] = 0.8;	sim->dyn.m[ARM_M3] = 0.6;	sim->dyn.m[ARM_M4] = 0.6;
	sim->dyn.V[ARM_M1] = 1.0;	sim->dyn.V[ARM_M2] = 1.0;	sim->dyn.V[ARM_M3] = 1.0;	sim->dyn.V[ARM_M4] = 1.0;
	// 行列初期化
	matInit(&sim->var.q,ARM_JNT,1);	matInit(&sim->var.dq,ARM_JNT,1);	matInit(&sim->var.q0,ARM_JNT,1);
	matInit(&sim->var.F,DIM2,1);
	matInit(&sim->var.r,DIM2,1);	matInit(&sim->var.dr,DIM2,1);	matInit(&sim->var.r0,DIM2,1);
	matInit(&sim->var.p,DIM2,1);	matInit(&sim->var.dp,DIM2,1);	matInit(&sim->var.p0,DIM2,1);
	// 運動学
	matInit(&sim->kine.J,DIM2,ARM_JNT); matInit(&sim->kine.dJ,DIM2,ARM_JNT);
	matInit(&sim->kine.Jt,ARM_JNT,DIM2); matInit(&sim->kine.Jinv,ARM_JNT,DIM2);
	matInit(&sim->kine.Jp,DIM2,ARM_JNT); matInit(&sim->kine.dJp,DIM2,ARM_JNT);
	matInit(&sim->kine.Jpt,ARM_JNT,DIM2);	matInit(&sim->kine.Jpinv,ARM_JNT,DIM2);
	// 動力学
	matInit(&sim->dyn.Mq,ARM_JNT,ARM_JNT); matInit(&sim->dyn.h,ARM_JNT,1);
	matInit(&sim->dyn.dMq,ARM_JNT,ARM_JNT);
	// インピーダンス
	matInit(&sim->imp.M,DIM2,DIM2); matInit(&sim->imp.C,DIM2,DIM2); matInit(&sim->imp.K,DIM2,DIM2);
	matInit(&sim->imp.Minv,DIM2,DIM2); matInit(&sim->imp.Cinv,DIM2,DIM2); matInit(&sim->imp.Kinv,DIM2,DIM2);
	matInit(&sim->imp.dM,DIM2,DIM2); matInit(&sim->imp.dC,DIM2,DIM2); matInit(&sim->imp.dK,DIM2,DIM2);
	matInit(&sim->imp.K0,DIM2,DIM2);
	return	0;
}

////////////////////////////////////////////////////////
// 慣性インピーダンスに固有慣性を設定(Inertia Shaping なし)
// 出力：sim->imp.M, sim->imp.dM
////////////////////////////////////////////////////////
int setInherentInertia(SIM *sim)
{
	static Matrix	Jinvt, d_Jinv, d_Jinvt;		// J^{-T}, d[J^{-1}], d[J^{-1}]^{T}
	static Matrix	Tmp22, Tmp22_2, Tmp22_3, Tmp44;
	static Matrix	exJp, exJpt, exJpinv, I4;
	static Matrix	exJinv;
	if (sim->step == 0){
		matInit(&Jinvt, DIM2, ARM_JNT);	matInit(&d_Jinv, ARM_JNT, DIM2);		matInit(&d_Jinvt, DIM2, ARM_JNT);
		matInit(&Tmp22, DIM2, DIM2);	matInit(&Tmp22_2, DIM2, DIM2);	matInit(&Tmp22_3, DIM2, DIM2);	matInit(&Tmp44, 4, 4);
		matInit(&exJp,DIM2,ARM_JNT); matInit(&exJpt,ARM_JNT,DIM2); matInit(&exJpinv,ARM_JNT,DIM2);
		matUnit(matInit(&I4,ARM_JNT,ARM_JNT));
		matInit(&exJinv,ARM_JNT,DIM2);
	}
	// 前処理
	matTrans(&Jinvt, &sim->kine.Jinv);
//	matMulScl(&d_Jinv, -1, matMul3(&Tmp22, &sim->kine.Jinv, &sim->kine.dJ, &sim->kine.Jinv));
//	matTrans(&d_Jinvt, &d_Jinv);
	matMul(&exJp, &sim->kine.Jp, matSub(&Tmp44, &I4, matMul(&Tmp44, &sim->kine.Jinv, &sim->kine.J)));	// exJp = Jp*(I-J^{-1}*J)
	matTrans(&exJpt, &exJp);	// exJp^T
	matMul(&exJpinv, &exJpt, matInv(&Tmp22, NULL, matMul(&Tmp22, &exJp, &exJpt)));	// exJp^{-1} = exJp^T*(exJp*exJp^T)^{-1}
	matMul(&exJinv, matSub(&Tmp44, &I4, matMul(&Tmp44, &exJpinv, &sim->kine.Jp)), &sim->kine.Jinv);	// exJ^{-1} = (I-exJp^{-1}*Jp)*J^{-1}
	// 慣性設定
	matMul3(&sim->imp.M, &Jinvt, &sim->dyn.Mq, &exJinv);	// Md = J^{-T}*Mq*exJ^{-1}
//	matAdd3(&sim->imp.dM, matMul3(&Tmp22, &d_Jinvt, &sim->dyn.Mq, &sim->kine.Jinv), matMul3(&Tmp22_2, &Jinvt, &sim->dyn.dMq, &sim->kine.Jinv), matMul3(&Tmp22_3, &Jinvt, &sim->dyn.Mq, &d_Jinv));
	// デバッグ
	matPrint(&sim->imp.M);
	return	0;
}

////////////////////////////////////////////////////////
// 前処理(転置行列，逆行列等)
////////////////////////////////////////////////////////
int ctrlPreProcessing(SIM *sim)
{
	static	Matrix	Tmp22;
	if (sim->step == 0)	matInit(&Tmp22,DIM2,DIM2);
	// 前処理
	matTrans(&sim->kine.Jt, &sim->kine.J);
	matTrans(&sim->kine.Jpt, &sim->kine.Jp);
	matMul(&sim->kine.Jinv, &sim->kine.Jt, matInv(&Tmp22, NULL, matMul(&Tmp22, &sim->kine.J, &sim->kine.Jt)));	// J^{-1} = J^T*(J*J^T)^{-1}
	matMul(&sim->kine.Jpinv, &sim->kine.Jpt, matInv(&Tmp22, NULL, matMul(&Tmp22, &sim->kine.Jp, &sim->kine.Jpt)));	// Jp^{-1} = Jp^T*(Jp*Jp^T)^{-1}
	// インピーダンス設定
//	setInherentInertia(sim);
	// インピーダンス逆行列
	matInv(&sim->imp.Minv, NULL, &sim->imp.M);
	matInv(&sim->imp.Cinv, NULL, &sim->imp.C);
	matInv(&sim->imp.Kinv, NULL, &sim->imp.K);
	return	0;
}

////////////////////////////////////////////////////////
// ダイナミクス設定
// 入力：関節変数
// 出力：ヤコビアン，慣性項，遠心・コリオリ力項
////////////////////////////////////////////////////////
int armDynPara(SIM *sim)
{
	int	jnt;
	static double	m1, m2, m3, m4, l1, l2, l3, l4, lg1, lg2, lg3, lg4, I1, I2, I3, I4;		// 定数
	double	C1, C2, C3, C4, S1, S2, S3, S4, C12, S12, C23, S23, C34, S34, C123, S123, C234, S234, C1234, S1234;
	double	dq1, dq2, dq3, dq4, dq12, dq23, dq34, dq123, dq234, dq1234;
	// パラメータ設定
	if(sim->step == 0){
		m1 = sim->dyn.m[ARM_M1]; m2 = sim->dyn.m[ARM_M2]; m3 = sim->dyn.m[ARM_M3]; m4 = sim->dyn.m[ARM_M4];
		l1 = sim->kine.l[ARM_M1]; l2 = sim->kine.l[ARM_M2]; l3 = sim->kine.l[ARM_M3]; l4 = sim->kine.l[ARM_M4];
		lg1 = sim->kine.lg[ARM_M1]; lg2 = sim->kine.lg[ARM_M2]; lg3 = sim->kine.lg[ARM_M3]; lg4 = sim->kine.lg[ARM_M4];
		I1 = (sim->kine.r[ARM_M1]*sim->kine.r[ARM_M1]/4+l1*l1/12)*m1;
		I2 = (sim->kine.r[ARM_M2]*sim->kine.r[ARM_M2]/4+l2*l2/12)*m2;
		I3 = (sim->kine.r[ARM_M3]*sim->kine.r[ARM_M3]/4+l3*l3/12)*m3;
		I4 = (sim->kine.r[ARM_M4]*sim->kine.r[ARM_M4]/4+l4*l4/12)*m4;
	}
	// 三角関数
	C1 = cos(sim->var.q.el[0][0]); C2 = cos(sim->var.q.el[1][0]); C3 = cos(sim->var.q.el[2][0]); C4 = cos(sim->var.q.el[3][0]); 
	S1 = sin(sim->var.q.el[0][0]); S2 = sin(sim->var.q.el[1][0]); S3 = sin(sim->var.q.el[2][0]); S4 = sin(sim->var.q.el[3][0]);
	C12 = cos(sim->var.q.el[0][0]+sim->var.q.el[1][0]); S12 = sin(sim->var.q.el[0][0]+sim->var.q.el[1][0]);
	C23 = cos(sim->var.q.el[1][0]+sim->var.q.el[2][0]); S23 = sin(sim->var.q.el[1][0]+sim->var.q.el[2][0]);
	C34 = cos(sim->var.q.el[2][0]+sim->var.q.el[3][0]); S34 = sin(sim->var.q.el[2][0]+sim->var.q.el[3][0]);
	C123 = cos(sim->var.q.el[0][0]+sim->var.q.el[1][0]+sim->var.q.el[2][0]); S123 = sin(sim->var.q.el[0][0]+sim->var.q.el[1][0]+sim->var.q.el[2][0]);
	C234 = cos(sim->var.q.el[1][0]+sim->var.q.el[2][0]+sim->var.q.el[3][0]); S234 = sin(sim->var.q.el[1][0]+sim->var.q.el[2][0]+sim->var.q.el[3][0]);
	C1234 = cos(sim->var.q.el[0][0]+sim->var.q.el[1][0]+sim->var.q.el[2][0]+sim->var.q.el[3][0]); S1234 = sin(sim->var.q.el[0][0]+sim->var.q.el[1][0]+sim->var.q.el[2][0]+sim->var.q.el[3][0]);
	// 変数置換
	dq1 = sim->var.dq.el[0][0]; dq2 = sim->var.dq.el[1][0]; dq3 = sim->var.dq.el[2][0]; dq4 = sim->var.dq.el[3][0];
	dq12 = dq1+dq2; dq23 = dq2+dq3; dq34 = dq3+dq4; dq123 = dq12+dq3; dq234 = dq23+dq4; dq1234 = dq123+dq4;
	// 慣性行列
	sim->dyn.Mq.el[0][0] = I1+m1*lg1*lg1 + I2+m2*(l1*l1+lg2*lg2+2*l1*lg2*C2) + I3+m3*(l1*l1+l2*l2+lg3*lg3+2*l1*l2*C2+2*l1*lg3*C23+2*l2*lg3*C3) + I4+m4*(l1*l1+l2*l2+l3*l3+lg4*lg4+2*l1*l2*C2+2*l1*l3*C23+2*l1*lg4*C234+2*l2*l3*C3+2*l2*lg4*C34+2*l3*lg4*C4);
	sim->dyn.Mq.el[0][1] = sim->dyn.Mq.el[1][0] = I2+m2*(lg2*lg2+l1*lg2*C2) + I3+m3*(l2*l2+lg3*lg3+l1*l2*C2+l1*lg3*C23+2*l2*lg3*C3) + I4+m4*(l2*l2+l3*l3+lg4*lg4+l1*l2*C2+l1*l3*C23+l1*lg4*C234+2*l2*l3*C3+2*l2*lg4*C34+2*l3*lg4*C4);
	sim->dyn.Mq.el[0][2] = sim->dyn.Mq.el[2][0] = I3+m3*(lg3*lg3+l1*lg3*C23+l2*lg3*C3) + I4+m4*(l3*l3+lg4*lg4+l1*l3*C23+l1*lg4*C234+l2*l3*C3+l2*lg4*C34+2*l3*lg4*C4);
	sim->dyn.Mq.el[0][3] = sim->dyn.Mq.el[3][0] = I4+m4*(lg4*lg4+l1*lg4*C234+l2*lg4*C34+l3*lg4*C4);
	sim->dyn.Mq.el[1][1] = I2+m2*lg2*lg2 + I3+m3*(l2*l2+lg3*lg3+2*l2*lg3*C3) + I4+m4*(l2*l2+l3*l3+lg4*lg4+2*l2*l3*C3+2*l2*lg4*C34+2*l3*lg4*C4);
	sim->dyn.Mq.el[1][2] = sim->dyn.Mq.el[2][1] = I3+m3*(lg3*lg3+l2*lg3*C3) + I4+m4*(l3*l3+lg4*lg4+l2*l3*C3+l2*lg4*C34+2*l3*lg4*C4);
	sim->dyn.Mq.el[1][3] = sim->dyn.Mq.el[3][1] = I4+m4*(lg4*lg4+l2*lg4*C34+l3*lg4*C4);
	sim->dyn.Mq.el[2][2] = I3+m3*lg3*lg3 + I4+m4*(l3*l3+lg4*lg4+2*l3*lg4*C4);
	sim->dyn.Mq.el[2][3] = sim->dyn.Mq.el[3][2] = I4+m4*(lg4*lg4+l3*lg4*C4);
	sim->dyn.Mq.el[3][3] = I4+m4*lg4*lg4;
	// 遠心・コリオリ力
	sim->dyn.h.el[0][0] = -m2*l1*lg2*S2*dq2*(2*dq1+dq2) - m3*(l1*l2*S2*dq2*(2*dq1+dq2)+l1*lg3*S23*dq23*(2*dq1+dq23)+l2*lg3*S3*dq3*(2*dq12+dq3)) - m4*(l1*l2*S2*dq2*(2*dq1+dq2)+l1*l3*S23*dq23*(2*dq1+dq23)+l1*lg4*S234*dq234*(2*dq1+dq234)+l2*l3*S3*dq3*(2*dq12+dq3)+l2*lg4*S34*dq34*(2*dq12+dq34)+l3*lg4*S4*dq4*(2*dq123+dq4));
	sim->dyn.h.el[1][0] = m2*l1*lg2*S2*dq1*dq1 + m3*(l1*l2*S2*dq1*dq1+l1*lg3*S23*dq1*dq1-l2*lg3*S3*dq3*(2*dq12+dq3)) + m4*(l1*l2*S2*dq1*dq1+l1*l3*S23*dq1*dq1+l1*lg4*S234*dq1*dq1-l2*l3*S3*dq3*(2*dq12+dq3)-l2*lg4*S34*dq34*(2*dq12+dq34)-l3*lg4*S4*dq4*(2*dq123+dq4));
	sim->dyn.h.el[2][0] = m3*(l1*lg3*S23*dq1*dq1+l2*lg3*S3*dq12*dq12) + m4*(l1*l3*S23*dq1*dq1+l1*lg4*S234*dq1*dq1+l2*l3*S3*dq12*dq12+l2*lg4*S34*dq12*dq12-l3*lg4*S4*dq4*(2*dq123+dq4));
	sim->dyn.h.el[3][0] = m4*(l1*lg4*S234*dq1*dq1+l2*lg4*S34*dq12*dq12+l3*lg4*S4*dq123*dq123);
	// 関節粘性摩擦力をhに追加
	for(jnt=0;jnt<ARM_JNT;jnt++)	sim->dyn.h.el[jnt][0] += sim->dyn.V[jnt] * sim->jnt_vel[jnt];
	// 手先ヤコビアン
	sim->kine.J.el[0][0] = -(l1*S1+l2*S12+l3*S123+l4*S1234);	sim->kine.J.el[0][1] = -(l2*S12+l3*S123+l4*S1234); sim->kine.J.el[0][2] = -(l3*S123+l4*S1234); sim->kine.J.el[0][3] = -l4*S1234;
	sim->kine.J.el[1][0] = l1*C1+l2*C12+l3*C123+l4*C1234; sim->kine.J.el[1][1] = l2*C12+l3*C123+l4*C1234; sim->kine.J.el[1][2] = l3*C123+l4*C1234; sim->kine.J.el[1][3] = l4*C1234;
	// 手先ヤコビアン微分
	sim->kine.dJ.el[0][0] = -(sim->kine.J.el[1][0]*dq1+sim->kine.J.el[1][1]*dq2+sim->kine.J.el[1][2]*dq3+sim->kine.J.el[1][3]*dq4);
	sim->kine.dJ.el[0][1] = -(sim->kine.J.el[1][1]*dq12+sim->kine.J.el[1][2]*dq3+sim->kine.J.el[1][3]*dq4);
	sim->kine.dJ.el[0][2] = -(sim->kine.J.el[1][2]*dq123+sim->kine.J.el[1][3]*dq4);
	sim->kine.dJ.el[0][3] = -sim->kine.J.el[1][3]*dq1234;
	sim->kine.dJ.el[1][0] = sim->kine.J.el[0][0]*dq1+sim->kine.J.el[0][1]*dq2+sim->kine.J.el[0][2]*dq3+sim->kine.J.el[0][3]*dq4;
	sim->kine.dJ.el[1][1] = sim->kine.J.el[0][1]*dq12+sim->kine.J.el[0][2]*dq3+sim->kine.J.el[0][3]*dq4;
	sim->kine.dJ.el[1][2] = sim->kine.J.el[0][2]*dq123+sim->kine.J.el[0][3]*dq4;
	sim->kine.dJ.el[1][3] = sim->kine.J.el[0][3]*dq1234;
	// 接続点ヤコビアン
	sim->kine.Jp.el[0][0] = -(l1*S1+l2*S12);	sim->kine.Jp.el[0][1] = -l2*S12; sim->kine.Jp.el[0][2] = 0.0; sim->kine.Jp.el[0][3] = 0.0;
	sim->kine.Jp.el[1][0] = l1*C1+l2*C12; sim->kine.Jp.el[1][1] = l2*C12; sim->kine.Jp.el[1][2] = 0.0; sim->kine.Jp.el[1][3] = 0.0;
	// 接続点ヤコビアン微分
	sim->kine.dJp.el[0][0] = -(sim->kine.Jp.el[1][0]*dq1+sim->kine.Jp.el[1][1]*dq2); sim->kine.dJp.el[0][1] = -sim->kine.Jp.el[1][1]*dq12;
	sim->kine.dJp.el[0][2] = 0.0;	sim->kine.dJp.el[0][3] = 0.0;
	sim->kine.dJp.el[1][0] = sim->kine.Jp.el[0][0]*dq1+sim->kine.Jp.el[0][1]*dq2;	sim->kine.dJp.el[1][1] = sim->kine.Jp.el[0][1]*dq12;
	sim->kine.dJp.el[1][2] = 0.0;	sim->kine.dJp.el[1][3] = 0.0;
	return	0;
}

////////////////////////////////////////////////////////
// 弾塑性ハイブリッド制御則(Maxwell+Voigt)
////////////////////////////////////////////////////////
int ctrlHybrid(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Fint, const Matrix *Md, const Matrix *Cd, const Matrix *Kd)
{
	int	jnt, crd;
	static Matrix	S, I;	// Maxwell制御がS, Voigt制御がI-S
	static Matrix	Jinv, Jt, Tmp41, Tmp42, Tmp22, Tmp21, Tmp21_1, Tmp21_2;
	static Matrix	tauNC, tauVE, tauIN, tauPL, E;

	// 初期化
	if(sim.step == 0){
		matInit(&S,DIM2,DIM2); matUnit(matInit(&I,DIM2,DIM2));
		matInit(&Jinv,ARM_JNT,DIM2);	matInit(&Jt,ARM_JNT,DIM2);
		matInit(&Tmp41,ARM_JNT,1); matInit(&Tmp42,ARM_JNT,DIM2); matInit(&Tmp22,DIM2,DIM2); matInit(&Tmp21,DIM2,1); matInit(&Tmp21_1,DIM2,1); matInit(&Tmp21_2,DIM2,1);
		matInit(&tauNC,ARM_JNT,1); matInit(&tauVE,ARM_JNT,1); matInit(&tauIN,ARM_JNT,1); matInit(&tauPL,ARM_JNT,1); matInit(&E,ARM_JNT,DIM2);
		S.el[0][0] = 1.0;	S.el[1][1] = 0.0;	// x方向がMaxwell制御, y方向がVoigt制御
		for(crd=0;crd<DIM2;crd++){
			if(fabs(S.el[crd][crd]) < MAT_EPS && sim.imp.C.el[crd][crd]*sim.imp.C.el[crd][crd] < 4*sim.imp.M.el[crd][crd]*sim.imp.K.el[crd][crd] ){		// Voigt
				sim.imp.T[crd] = 2*PI/sqrt(sim.imp.K.el[crd][crd]/sim.imp.M.el[crd][crd]-sim.imp.C.el[crd][crd]*sim.imp.C.el[crd][crd]/(4*sim.imp.M.el[crd][crd]*sim.imp.M.el[crd][crd]));
				printf("T[%d]=%f\n", crd, sim.imp.T[crd]);
			}else if(sim.imp.C.el[crd][crd]*sim.imp.C.el[crd][crd] > sim.imp.M.el[crd][crd]*sim.imp.K.el[crd][crd]/4 ){		// Maxwell
				sim.imp.T[crd] = 2*PI/sqrt(sim.imp.K.el[crd][crd]/sim.imp.M.el[crd][crd]-sim.imp.K.el[crd][crd]*sim.imp.K.el[crd][crd]/(4*sim.imp.C.el[crd][crd]*sim.imp.C.el[crd][crd]));
				printf("T[%d]=%f\n", crd, sim.imp.T[crd]);
			}
		}
	}
	// MaxwellとVoigtの方向切替
	if(sim.step == IMP_SWITCH_STEP){ S.el[0][0] = 0.0;	S.el[1][1] = 1.0; }		// x方向がVoigt制御, y方向がMaxwell制御
	// 制御則
	matTrans(&Jt, J);	// J^T
	matMul(&Jinv, &Jt, matInv(&Tmp22, NULL, matMul(&Tmp22, J, &Jt)));	// J^{-1} = J^T*(J*J^T)^{-1}
	matMul3(&E, Mq, &Jinv, matInv(&Tmp22, NULL, Md));	// E = Mq*J^{-1}*Md^{-1}
	matSub(&tauNC, h, matMul4(&Tmp41, Mq, &Jinv, dJ, dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matAdd3(&Tmp21, matMul5(&Tmp21_1, Kd, matInv(&Tmp22, NULL, Cd), Md, &S, dre), matMul3(&Tmp21_2, Cd, matSub(&Tmp22, &I, &S), dre), matMul(&Tmp21, Kd, re));
	matMulScl(&tauVE, -1, matMul(&Tmp41, &E, &Tmp21));	// tauVE = -E{Kd*Cd^{-1}*Md*S*dr+Cd*(I-S)*dr+Kd*r}
	matMul(&tauIN, matSub(&Tmp42, &E, &Jt), F);		// tauIN = (E-J^T)F
	matMul5(&tauPL, &E, Kd, matInv(&Tmp22, NULL, Cd), &S, Fint);	// tauPL = Kd*Cd^{-1}*S*Fint
	matAdd4(tau, &tauNC, &tauVE, &tauIN, &tauPL);
	return	0;
}

////////////////////////////////////////////////////////
// Maxwell制御則（並列型）
////////////////////////////////////////////////////////
int ctrlMaxwellParallel(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Fint, const Matrix *Md, const Matrix *Cd, const Matrix *Kd)
{
	int	jnt, crd;
	static Matrix	Jinv, Jt, Tmp41, Tmp42, Tmp22, Tmp21, Tmp21_2;
	static Matrix	tauNC, tauVE, tauIN, tauPL, E;

	// 初期化
	if(sim.step == 0){
		matInit(&Jinv,ARM_JNT,DIM2);	matInit(&Jt,ARM_JNT,DIM2);
		matInit(&Tmp41,ARM_JNT,1); matInit(&Tmp42,ARM_JNT,DIM2); matInit(&Tmp22,DIM2,DIM2); matInit(&Tmp21,DIM2,1); matInit(&Tmp21_2,DIM2,1);
		matInit(&tauNC,ARM_JNT,1); matInit(&tauVE,ARM_JNT,1); matInit(&tauIN,ARM_JNT,1); matInit(&tauPL,ARM_JNT,1); matInit(&E,ARM_JNT,DIM2);
		for(crd=0;crd<DIM2;crd++){
			if(sim.imp.C.el[crd][crd]*sim.imp.C.el[crd][crd] > sim.imp.M.el[crd][crd]*sim.imp.K.el[crd][crd]/4 ){
				sim.imp.T[crd] = 2*PI/sqrt(sim.imp.K.el[crd][crd]/sim.imp.M.el[crd][crd]-sim.imp.K.el[crd][crd]*sim.imp.K.el[crd][crd]/(4*sim.imp.C.el[crd][crd]*sim.imp.C.el[crd][crd]));
				printf("T[%d]=%f\n", crd, sim.imp.T[crd]);
			}
		}
	}
	// 制御則
	matTrans(&Jt, J);	// J^T
	matMul(&Jinv, &Jt, matInv(&Tmp22, NULL, matMul(&Tmp22, J, &Jt)));	// J^{-1} = J^T*(J*J^T)^{-1}
	matMul3(&E, Mq, &Jinv, matInv(&Tmp22, NULL, Md));	// E = Mq*J^{-1}*Md^{-1}
	matSub(&tauNC, h, matMul4(&Tmp41, Mq, &Jinv, dJ, dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matAdd(&Tmp21, matMul4(&Tmp21, Kd, matInv(&Tmp22, NULL, Cd), Md, dre), matMul(&Tmp21_2, Kd, re));
	matMulScl(&tauVE, -1, matMul(&Tmp41, &E, &Tmp21));	// tauVE = -E{Kd*Cd^{-1}*Md*dr+Kd*r}
	matMul(&tauIN, matSub(&Tmp42, &E, &Jt), F);		// tauIN = (E-J^T)F
	matMul4(&tauPL, &E, Kd, matInv(&Tmp22, NULL, Cd), Fint);	// tauPL = Kd*Cd^{-1}*Fint
	matAdd4(tau, &tauNC, &tauVE, &tauIN, &tauPL);
	return	0;
}

////////////////////////////////////////////////////////
// SLS制御則(標準線形固体モデル)
////////////////////////////////////////////////////////
int ctrlSLS(SIM *sim, Matrix *tau)
{
#define	CORRECTION_GAIN	(15.0)
#define	GAIN_pP	(23.0)
#define	GAIN_xP	(23.0)
	int	jnt, crd;
	static Matrix	exJp, exJpt, exJpinv, I4, H;
	static Matrix	exJinv;
	static Matrix	Tmp41, Tmp41_2, Tmp42, Tmp44;
	static Matrix	Tmp22, Tmp22_2, Tmp21, Tmp21_2, Tmp24;
	static Matrix	E, tauNC, tauVE, tauIN, tauCR;
	static Matrix	tauP, tauI, tauD;		// tauVEのPID成分
	static Matrix	re;	// 手先位置変位
	static Matrix	rs, drs;	// バネ位置変位，バネ速度変位
	static Matrix	tauCR2, tauCR3;
	static Matrix	Gp_P, Gx_P;		// P:Pゲイン

	// 初期化
	if(sim->step == 0){
		matInit(&exJp,DIM2,ARM_JNT); matInit(&exJpt,ARM_JNT,DIM2); matInit(&exJpinv,ARM_JNT,DIM2);	matInit(&exJinv,ARM_JNT,DIM2);
		matUnit(matInit(&I4,ARM_JNT,ARM_JNT));
		matUnit(matInit(&H,DIM2,DIM2)); matMulScl(&H, CORRECTION_GAIN, &H);
		matInit(&Tmp41,ARM_JNT,1); matInit(&Tmp41_2,ARM_JNT,1); matInit(&Tmp42,ARM_JNT,DIM2); matInit(&Tmp44,ARM_JNT,ARM_JNT);
		matInit(&Tmp22,DIM2,DIM2); matInit(&Tmp22_2,DIM2,DIM2); matInit(&Tmp21,DIM2,1); matInit(&Tmp21_2,DIM2,1); matInit(&Tmp24,DIM2,ARM_JNT);
		matInit(&E,ARM_JNT,DIM2); matInit(&tauNC,ARM_JNT,1); matInit(&tauVE,ARM_JNT,1); matInit(&tauIN,ARM_JNT,1); matInit(&tauCR,ARM_JNT,1);
		matInit(&tauP, ARM_JNT, 1); matInit(&tauI, ARM_JNT, 1); matInit(&tauD, ARM_JNT, 1);
		matInit(&re,2,1);
		matInit(&rs,DIM2,1); matInit(&drs,DIM2,1);
		matInit(&tauCR2, ARM_JNT, 1);	matInit(&tauCR3, ARM_JNT, 1);
		matUnit(matInit(&Gp_P,DIM2,DIM2));	matMulScl(&Gp_P, GAIN_pP, &Gp_P);
		matUnit(matInit(&Gx_P,DIM2,DIM2));	matMulScl(&Gx_P, GAIN_xP, &Gx_P);
		for(crd=0;crd<DIM2;crd++){
			if(sim->imp.C.el[crd][crd]*sim->imp.C.el[crd][crd] > sim->imp.M.el[crd][crd]*sim->imp.K.el[crd][crd]/4 ){
				sim->imp.T[crd] = 2*PI/sqrt(sim->imp.K.el[crd][crd]/sim->imp.M.el[crd][crd]-sim->imp.K.el[crd][crd]*sim->imp.K.el[crd][crd]/(4*sim->imp.C.el[crd][crd]*sim->imp.C.el[crd][crd]));
				printf("T[%d]=%f\n", crd, sim->imp.T[crd]);
			}
		}
	}
	// 前処理
	matSub(&re, &sim->var.r, &sim->var.r0);		// 手先位置変位
	matSub(&rs, &re, matSub(&Tmp21_2, &sim->var.p, &sim->var.p0));		// バネ位置変位
	matSub(&drs, &sim->var.dr, &sim->var.dp);	// バネ速度変位（初期速度が0の場合のみ適用）
	matMul(&exJp, &sim->kine.Jp, matSub(&Tmp44, &I4, matMul(&Tmp44, &sim->kine.Jinv, &sim->kine.J)));	// exJp = Jp*(I-J^{-1}*J)
	matTrans(&exJpt, &exJp);	// exJp^T
	matMul(&exJpinv, &exJpt, matInv(&Tmp22, NULL, matMul(&Tmp22, &exJp, &exJpt)));	// exJp^{-1} = exJp^T*(exJp*exJp^T)^{-1}
	matMul(&exJinv, matSub(&Tmp44, &I4, matMul(&Tmp44, &exJpinv, &sim->kine.Jp)), &sim->kine.Jinv);	// exJ^{-1} = (I-exJp^{-1}*Jp)*J^{-1}
	// 制御則
	matMul3(&E, &sim->dyn.Mq, &exJinv, &sim->imp.Minv);	// E = Mq*exJ^{-1}*Md^{-1}
	matSub(&tauNC, &sim->dyn.h, matAdd(&Tmp41, matMul4(&Tmp41, &sim->dyn.Mq, &exJinv, &sim->kine.dJ, &sim->var.dq), matMul4(&Tmp41_2, &sim->dyn.Mq, &exJpinv, &sim->kine.dJp, &sim->var.dq)));	// tauNC = h-Mq*exJ^{-1}*dJ*dq-Mq*exJp^{-1}*dJp*dq
	matMul5(&tauD, &sim->dyn.Mq, &exJpinv, &sim->imp.Cinv, &sim->imp.K, &drs);	// tauD = Mq*exJp^{-1}*Cd^{-1}*Kd*drs
	matMul(&tauP, &E, matAdd(&tauP, matMul(&Tmp41, &sim->imp.K, &rs), matMul(&Tmp41_2, &sim->imp.K0, &re)));	// tauP = E*(Kd*rs+K0*r)
//	matMul(&tauP, &E, matAdd(&tauP, matMul3(&Tmp41, &sim->imp.C, &sim->kine.Jp, &sim->var.dq), matMul(&Tmp41_2, &sim->imp.K0, &re)));	// tauP = E*(Cd*Jp*dq+K0*r)
	matSub(&tauVE, &tauD, &tauP);	// tauVE = tauD-tauP
	matMul(&tauIN, matSub(&Tmp42, &E, &sim->kine.Jt), &sim->var.F);		// tauIN = (E-J^T)F
	matAdd3(tau, &tauNC, &tauVE, &tauIN);
#if 1
	matMul4(&tauCR, &sim->dyn.Mq, &exJpinv, &H, matSub(&Tmp21, matMul3(&Tmp21, matInv(&Tmp22, NULL, &sim->imp.C), &sim->imp.K, &rs), matMul(&Tmp21_2, &sim->kine.Jp, &sim->var.dq)));	// tauCR = Mq*exJp^{-1}*H*(Cd^{-1}*Kd*rs-Jp*dq)
	matAdd(tau, tau, &tauCR);	// 補正項追加
#elif 0
	matMul4(&tauCR, &sim->dyn.Mq, &exJpinv, &H, matSub(&Tmp21, matMul3(&Tmp21, &sim->imp.Cinv, &sim->imp.K, &rs), matMul(&Tmp21_2, &sim->kine.Jp, &sim->var.dq)));	// tauCR = Mq*exJp^{-1}*H*(Cd^{-1}*Kd*rs-Jp*dq)　pに関するD制御
	matMul4(&tauCR2, &sim->dyn.Mq, &exJpinv, &Gp_P, matSub(&Tmp21, &rs, matMul3(&Tmp21, &sim->imp.Kinv, &sim->imp.C, &sim->var.dp)));	// tauCR2 = Mq*exJp^{-1}*Gp_P*(rs-Kd^{-1}*Cd*dp)　pに関するP制御
	matMul4(&tauCR3, &sim->dyn.Mq, &exJinv, &Gx_P, matSub(&Tmp21, matMul3(&Tmp21, &sim->imp.Kinv, &sim->imp.C, &sim->var.dp), &rs));	// tauCR3 = Mq*exJ^{-1}*Gx_P*(-rs+Kd^{-1}*Cd*dp)　xに関するP制御
	matAdd4(tau, tau, &tauCR, &tauCR2, &tauCR3);	// 補正項追加
#endif
	// デバッグ
//	if(sim.step == 1000){ matPrint(&S); matPrint(&E); matPrint(&exJpinv);}
//	if(sim.step == 1000){ matPrint(matMul(&Tmp22, Jp, &exJpinv));}		// 単位行列になる
//	matPrint(matSub(&Tmp21, matMul(&Tmp21, &sim->kine.Jp, &sim->var.dq), matMul3(&Tmp21_2, matInv(&Tmp22, NULL, &sim->imp.C), &sim->imp.K, &rs)));		// 0になればOK
//	matPrint(&tauIN);		// Inertia Shapingしない場合，0になればOK
	return	0;
}

////////////////////////////////////////////////////////
// SLS2制御則(標準線形固体モデル)
////////////////////////////////////////////////////////
int ctrlSLS2(SIM *sim, Matrix *tau)
{
//#define	CORRECTION_GAIN	(15.0)
	int	jnt, crd;
	static Matrix	exJp, exJpt, exJpinv, I4, H;
	static Matrix	exJinv;
	static Matrix	Tmp41, Tmp41_2, Tmp42, Tmp44;
	static Matrix	Tmp22, Tmp22_2, Tmp21, Tmp21_2, Tmp24;
	static Matrix	E, tauNC, tauVE, tauIN, tauCR;
	static Matrix	tauP, tauI, tauD;		// tauVEのPID成分
	static Matrix	re;	// 手先位置変位
	static Matrix	rs, drs;	// バネ位置変位，バネ速度変位
	static Matrix	tauCR2, tauCR3;
	static Matrix	Ja, Jainv;	// [J; Jp]
	// 初期化
	if(sim->step == 0){
		matInit(&exJp,DIM2,ARM_JNT); matInit(&exJpt,ARM_JNT,DIM2); matInit(&exJpinv,ARM_JNT,DIM2);	matInit(&exJinv,ARM_JNT,DIM2);
		matUnit(matInit(&I4,ARM_JNT,ARM_JNT));
		matUnit(matInit(&H,DIM2,DIM2)); matMulScl(&H, CORRECTION_GAIN, &H);
		matInit(&Tmp41,ARM_JNT,1); matInit(&Tmp41_2,ARM_JNT,1); matInit(&Tmp42,ARM_JNT,DIM2); matInit(&Tmp44,ARM_JNT,ARM_JNT);
		matInit(&Tmp22,DIM2,DIM2); matInit(&Tmp22_2,DIM2,DIM2); matInit(&Tmp21,DIM2,1); matInit(&Tmp21_2,DIM2,1); matInit(&Tmp24,DIM2,ARM_JNT);
		matInit(&E,ARM_JNT,DIM2); matInit(&tauNC,ARM_JNT,1); matInit(&tauVE,ARM_JNT,1); matInit(&tauIN,ARM_JNT,1); matInit(&tauCR,ARM_JNT,1);
		matInit(&tauP, ARM_JNT, 1); matInit(&tauI, ARM_JNT, 1); matInit(&tauD, ARM_JNT, 1);
		matInit(&re,2,1);
		matInit(&rs,DIM2,1); matInit(&drs,DIM2,1);
		matInit(&tauCR2, ARM_JNT, 1);	matInit(&tauCR3, ARM_JNT, 1);
		matInit(&Ja,ARM_JNT,ARM_JNT);	matInit(&Jainv,ARM_JNT,ARM_JNT);
		for(crd=0;crd<DIM2;crd++){
			if(sim->imp.C.el[crd][crd]*sim->imp.C.el[crd][crd] > sim->imp.M.el[crd][crd]*sim->imp.K.el[crd][crd]/4 ){
				sim->imp.T[crd] = 2*PI/sqrt(sim->imp.K.el[crd][crd]/sim->imp.M.el[crd][crd]-sim->imp.K.el[crd][crd]*sim->imp.K.el[crd][crd]/(4*sim->imp.C.el[crd][crd]*sim->imp.C.el[crd][crd]));
				printf("T[%d]=%f\n", crd, sim->imp.T[crd]);
			}
		}
	}
	// 前処理
	matSub(&re, &sim->var.r, &sim->var.r0);		// 手先位置変位
	matSub(&rs, &re, matSub(&Tmp21_2, &sim->var.p, &sim->var.p0));		// バネ位置変位
	matSub(&drs, &sim->var.dr, &sim->var.dp);	// バネ速度変位（初期速度が0の場合のみ適用）
	matMul(&exJp, &sim->kine.Jp, matSub(&Tmp44, &I4, matMul(&Tmp44, &sim->kine.Jinv, &sim->kine.J)));	// exJp = Jp*(I-J^{-1}*J)
	matTrans(&exJpt, &exJp);	// exJp^T
	matMul(&exJpinv, &exJpt, matInv(&Tmp22, NULL, matMul(&Tmp22, &exJp, &exJpt)));	// exJp^{-1} = exJp^T*(exJp*exJp^T)^{-1}
	matMul(&exJinv, matSub(&Tmp44, &I4, matMul(&Tmp44, &exJpinv, &sim->kine.Jp)), &sim->kine.Jinv);	// exJ^{-1} = (I-exJp^{-1}*Jp)*J^{-1}
	matAssign(&Ja, &sim->kine.J, 0, 0);	matAssign(&Ja, &sim->kine.Jp, DIM2, 0);	// (0,0)にJを,(2,0)にJpをセット
	matInv(&Jainv, NULL, &Ja);
	// 制御則
	matMul3(&E, &sim->dyn.Mq, &exJinv, &sim->imp.Minv);	// E = Mq*exJ^{-1}*Md^{-1}
	matSub(&tauNC, &sim->dyn.h, matAdd(&Tmp41, matMul4(&Tmp41, &sim->dyn.Mq, &exJinv, &sim->kine.dJ, &sim->var.dq), matMul4(&Tmp41_2, &sim->dyn.Mq, &exJpinv, &sim->kine.dJp, &sim->var.dq)));	// tauNC = h-Mq*exJ^{-1}*dJ*dq-Mq*exJp^{-1}*dJp*dq
	matMul5(&tauD, &sim->dyn.Mq, &exJpinv, &sim->imp.Cinv, &sim->imp.K, &drs);	// tauD = Mq*exJp^{-1}*Cd^{-1}*Kd*drs
	matMul(&tauP, &E, matAdd(&tauP, matMul(&Tmp41, &sim->imp.K, &rs), matMul(&Tmp41_2, &sim->imp.K0, &re)));	// tauP = E*(Kd*rs+K0*r)
//	matMul(&tauP, &E, matAdd(&tauP, matMul3(&Tmp41, &sim->imp.C, &sim->kine.Jp, &sim->var.dq), matMul(&Tmp41_2, &sim->imp.K0, &re)));	// tauP = E*(Cd*Jp*dq+K0*r)
	matSub(&tauVE, &tauD, &tauP);	// tauVE = tauD-tauP
	matMul(&tauIN, matSub(&Tmp42, &E, &sim->kine.Jt), &sim->var.F);		// tauIN = (E-J^T)F
	matAdd3(tau, &tauNC, &tauVE, &tauIN);
#if 1
	matMul4(&tauCR, &sim->dyn.Mq, &exJpinv, &H, matSub(&Tmp21, matMul3(&Tmp21, matInv(&Tmp22, NULL, &sim->imp.C), &sim->imp.K, &rs), matMul(&Tmp21_2, &sim->kine.Jp, &sim->var.dq)));	// tauCR = Mq*exJp^{-1}*H*(Cd^{-1}*Kd*rs-Jp*dq)
	matAdd(tau, tau, &tauCR);	// 補正項追加
#endif
	// デバッグ
	if(sim->step == 1500){ matPrint(&Jainv); matPrint(&exJinv); matPrint(&exJpinv);}	// Jainvの左ブロックはexJinvと一緒．Jainvの右ブロックはexJpinvと一緒．
//	if(sim.step == 1000){ matPrint(&S); matPrint(&E); matPrint(&exJpinv);}
//	if(sim.step == 1000){ matPrint(matMul(&Tmp22, Jp, &exJpinv));}		// 単位行列になる
//	matPrint(matSub(&Tmp21, matMul(&Tmp21, &sim->kine.Jp, &sim->var.dq), matMul3(&Tmp21_2, matInv(&Tmp22, NULL, &sim->imp.C), &sim->imp.K, &rs)));		// 0になればOK
//	matPrint(&tauIN);		// Inertia Shapingしない場合，0になればOK
	return	0;
}

////////////////////////////////////////////////////////
// Maxwell制御則（直列型）
////////////////////////////////////////////////////////
int ctrlMaxwellSeries(SIM *sim, Matrix *tau)
{
#define	CORRECTION_GAIN	(10.0)
#define	GAIN_pP	(13.0)
#define	GAIN_xP	(13.0)
	int	jnt, crd;
	static Matrix	exJp, exJpt, exJpinv, I4, H;
	static Matrix	exJinv;
	static Matrix	Tmp41, Tmp41_2, Tmp42, Tmp44;
	static Matrix	Tmp22, Tmp22_2, Tmp21, Tmp21_2, Tmp24;
	static Matrix	E, tauNC, tauVE, tauIN, tauCR;
	static Matrix	rs, drs;	// バネ位置変位，バネ速度変位
	static Matrix	tauCR2, tauCR3;
	static Matrix	Gp_P, Gx_P;		// P:Pゲイン

	// 初期化
	if(sim->step == 0){
		matInit(&exJp,DIM2,ARM_JNT); matInit(&exJpt,ARM_JNT,DIM2); matInit(&exJpinv,ARM_JNT,DIM2);	matInit(&exJinv,ARM_JNT,DIM2);
		matUnit(matInit(&I4,ARM_JNT,ARM_JNT));
		matUnit(matInit(&H,DIM2,DIM2)); matMulScl(&H, CORRECTION_GAIN, &H);
		matInit(&Tmp41,ARM_JNT,1); matInit(&Tmp41_2,ARM_JNT,1); matInit(&Tmp42,ARM_JNT,DIM2); matInit(&Tmp44,ARM_JNT,ARM_JNT);
		matInit(&Tmp22,DIM2,DIM2); matInit(&Tmp22_2,DIM2,DIM2); matInit(&Tmp21,DIM2,1); matInit(&Tmp21_2,DIM2,1); matInit(&Tmp24,DIM2,ARM_JNT);
		matInit(&E,ARM_JNT,DIM2); matInit(&tauNC,ARM_JNT,1); matInit(&tauVE,ARM_JNT,1); matInit(&tauIN,ARM_JNT,1); matInit(&tauCR,ARM_JNT,1);
		matInit(&rs,DIM2,1); matInit(&drs,DIM2,1);
		matInit(&tauCR2, ARM_JNT, 1);	matInit(&tauCR3, ARM_JNT, 1);
		matUnit(matInit(&Gp_P,DIM2,DIM2));	matMulScl(&Gp_P, GAIN_pP, &Gp_P);
		matUnit(matInit(&Gx_P,DIM2,DIM2));	matMulScl(&Gx_P, GAIN_xP, &Gx_P);
		for(crd=0;crd<DIM2;crd++){
			if(sim->imp.C.el[crd][crd]*sim->imp.C.el[crd][crd] > sim->imp.M.el[crd][crd]*sim->imp.K.el[crd][crd]/4 ){
				sim->imp.T[crd] = 2*PI/sqrt(sim->imp.K.el[crd][crd]/sim->imp.M.el[crd][crd]-sim->imp.K.el[crd][crd]*sim->imp.K.el[crd][crd]/(4*sim->imp.C.el[crd][crd]*sim->imp.C.el[crd][crd]));
				printf("T[%d]=%f\n", crd, sim->imp.T[crd]);
			}
		}
	}
	// 前処理
	matSub(&rs, matSub(&Tmp21, &sim->var.r, &sim->var.r0), matSub(&Tmp21_2, &sim->var.p, &sim->var.p0));		// バネ位置変位
	matSub(&drs, &sim->var.dr, &sim->var.dp);	// バネ速度変位（初期速度が0の場合のみ適用）
	matMul(&exJp, &sim->kine.Jp, matSub(&Tmp44, &I4, matMul(&Tmp44, &sim->kine.Jinv, &sim->kine.J)));	// exJp = Jp*(I-J^{-1}*J)
	matTrans(&exJpt, &exJp);	// exJp^T
	matMul(&exJpinv, &exJpt, matInv(&Tmp22, NULL, matMul(&Tmp22, &exJp, &exJpt)));	// exJp^{-1} = exJp^T*(exJp*exJp^T)^{-1}
	matMul(&exJinv, matSub(&Tmp44, &I4, matMul(&Tmp44, &exJpinv, &sim->kine.Jp)), &sim->kine.Jinv);	// exJ^{-1} = (I-exJp^{-1}*Jp)*J^{-1}
	// 制御則
	matMul3(&E, &sim->dyn.Mq, &exJinv, &sim->imp.Minv);	// E = Mq*exJ^{-1}*Md^{-1}
	matSub(&tauNC, &sim->dyn.h, matAdd(&Tmp41, matMul4(&Tmp41, &sim->dyn.Mq, &exJinv, &sim->kine.dJ, &sim->var.dq), matMul4(&Tmp41_2, &sim->dyn.Mq, &exJpinv, &sim->kine.dJp, &sim->var.dq)));	// tauNC = h-Mq*exJ^{-1}*dJ*dq-Mq*exJp^{-1}*dJp*dq
	matSub(&tauVE, matMul5(&Tmp41, &sim->dyn.Mq, &exJpinv, &sim->imp.Cinv, &sim->imp.K, &drs), matMul3(&Tmp41_2, &E, &sim->imp.K, &rs));	// tauVE = Mq*exJp^{-1}*Cd^{-1}*Kd*drs-E*Kd*rs
	matMul(&tauIN, matSub(&Tmp42, &E, &sim->kine.Jt), &sim->var.F);		// tauIN = (E-J^T)F
	matAdd3(tau, &tauNC, &tauVE, &tauIN);
#if 0
	matMul4(&tauCR, &sim->dyn.Mq, &exJpinv, &H, matSub(&Tmp21, matMul3(&Tmp21, matInv(&Tmp22, NULL, &sim->imp.C), &sim->imp.K, &rs), matMul(&Tmp21_2, &sim->kine.Jp, &sim->var.dq)));	// tauCR = Mq*exJp^{-1}*H*(Cd^{-1}*Kd*rs-Jp*dq)
	matAdd(tau, tau, &tauCR);	// 補正項追加
#elif 1
	matMul4(&tauCR, &sim->dyn.Mq, &exJpinv, &H, matSub(&Tmp21, matMul3(&Tmp21, &sim->imp.Cinv, &sim->imp.K, &rs), matMul(&Tmp21_2, &sim->kine.Jp, &sim->var.dq)));	// tauCR = Mq*exJp^{-1}*H*(Cd^{-1}*Kd*rs-Jp*dq)　pに関するD制御
	matMul4(&tauCR2, &sim->dyn.Mq, &exJpinv, &Gp_P, matSub(&Tmp21, &rs, matMul3(&Tmp21, &sim->imp.Kinv, &sim->imp.C, &sim->var.dp)));	// tauCR2 = Mq*exJp^{-1}*Gp_P*(rs-Kd^{-1}*Cd*dp)　pに関するP制御
	matMul4(&tauCR3, &sim->dyn.Mq, &exJinv, &Gx_P, matSub(&Tmp21, matMul3(&Tmp21, &sim->imp.Kinv, &sim->imp.C, &sim->var.dp), &rs));	// tauCR3 = Mq*exJ^{-1}*Gx_P*(-rs+Kd^{-1}*Cd*dp)　xに関するP制御
	matAdd4(tau, tau, &tauCR, &tauCR2, &tauCR3);	// 補正項追加
#endif
	// デバッグ
//	if(sim.step == 1000){ matPrint(&S); matPrint(&E); matPrint(&exJpinv);}
//	if(sim.step == 1000){ matPrint(matMul(&Tmp22, Jp, &exJpinv));}		// 単位行列になる
//	matPrint(matSub(&Tmp21, matMul(&Tmp21, &sim->kine.Jp, &sim->var.dq), matMul3(&Tmp21_2, matInv(&Tmp22, NULL, &sim->imp.C), &sim->imp.K, &rs)));		// 0になればOK
//	matPrint(&tauIN);		// Inertia Shapingしない場合，0になればOK
	return	0;
}

////////////////////////////////////////////////////////
// Maxwell制御則（直列型）
////////////////////////////////////////////////////////
int ctrlMaxwellSeries2(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *Jp, const Matrix *dJp, const Matrix *q, const Matrix *dq, const Matrix *rs, const Matrix *dre, const Matrix *F, const Matrix *Md, const Matrix *Cd, const Matrix *Kd)
{
	int	jnt, crd;
	static Matrix	Jpt, exJp, exJpt, exJpinv, S, I4;
	static Matrix	Jinv, Jt, dJinv, dJt, Tmp41, Tmp41_2, Tmp42, Tmp44;
	static Matrix	Tmp22, Tmp22_2, Tmp21, Tmp21_2, Tmp24;
	static Matrix	tauNC, tauVE, tauVE1, tauVE2, tauIN, E;

	// 初期化
	if(sim.step == 0){
		matInit(&Jpt,ARM_JNT,DIM2); matInit(&exJp,DIM2,ARM_JNT); matInit(&exJpt,ARM_JNT,DIM2); matInit(&exJpinv,ARM_JNT,DIM2); matInit(&S,ARM_JNT,ARM_JNT); matUnit(matInit(&I4,ARM_JNT,ARM_JNT));
		matInit(&Jinv,ARM_JNT,DIM2); matInit(&Jt,ARM_JNT,DIM2); matInit(&dJinv,ARM_JNT,DIM2); matInit(&dJt,ARM_JNT,DIM2);
		matInit(&Tmp41,ARM_JNT,1); matInit(&Tmp41_2,ARM_JNT,1); matInit(&Tmp42,ARM_JNT,DIM2); matInit(&Tmp44,ARM_JNT,ARM_JNT);
		matInit(&Tmp22,DIM2,DIM2); matInit(&Tmp22_2,DIM2,DIM2); matInit(&Tmp21,DIM2,1); matInit(&Tmp21_2,DIM2,1); matInit(&Tmp24,DIM2,ARM_JNT);
		matInit(&tauNC,ARM_JNT,1); matInit(&tauVE,ARM_JNT,1); matInit(&tauVE1,ARM_JNT,1); matInit(&tauVE2,ARM_JNT,1); matInit(&tauIN,ARM_JNT,1); matInit(&E,ARM_JNT,DIM2);
		for(crd=0;crd<DIM2;crd++){
			if(sim.imp.C.el[crd][crd]*sim.imp.C.el[crd][crd] > sim.imp.M.el[crd][crd]*sim.imp.K.el[crd][crd]/4 ){
				sim.imp.T[crd] = 2*PI/sqrt(sim.imp.K.el[crd][crd]/sim.imp.M.el[crd][crd]-sim.imp.K.el[crd][crd]*sim.imp.K.el[crd][crd]/(4*sim.imp.C.el[crd][crd]*sim.imp.C.el[crd][crd]));
				printf("T[%d]=%f\n", crd, sim.imp.T[crd]);
			}
		}
	}
	// 制御則
	matTrans(&Jt, J); matTrans(&Jpt, Jp);	// J^T, Jp^T
	matMul(&Jinv, &Jt, matInv(&Tmp22, NULL, matMul(&Tmp22, J, &Jt)));	// J^{-1} = J^T*(J*J^T)^{-1}
	matTrans(&dJt, dJ);	// dJ^T
	matSub(&Tmp42, &dJt, matMul(&Tmp42, &Jinv, matAdd(&Tmp22, matMul(&Tmp22, dJ, &Jt), matMul(&Tmp22_2, J, &dJt))));
	matMul(&dJinv, &Tmp42, matInv(&Tmp22, NULL, matMul(&Tmp22, J, &Jt)));	// dJinvはd(Jinv)であり(dJ)invではないので注意！
	matMul(&exJp, Jp, matSub(&Tmp44, &I4, matMul(&Tmp44, &Jinv, J)));	// exJp = Jp*(I-J^{-1}*J)
	matTrans(&exJpt, &exJp);	// exJp^T
	matMul(&exJpinv, &exJpt, matInv(&Tmp22, NULL, matMul(&Tmp22, &exJp, &exJpt)));	// exJp^{-1} = exJp^T*(exJp*exJp^T)^{-1}
//	matSub(&S, &I4, matMul(&Tmp44, &exJpinv, Jp));	// S = I-exJp^{-1}*Jp
	matMul3(&E, Mq, &Jinv, matInv(&Tmp22, NULL, Md));	// E = Mq*J^{-1}*Md^{-1}
	matAdd(&tauNC, h, matMul3(&Tmp41, Mq, &dJinv, dre));	// tauNC = h+Mq*dJ^{-1}*dre
	matMul3(&tauVE1, &E, Kd, rs);
	matSub(&Tmp21, matMul3(&Tmp21, matInv(&Tmp22, NULL, Cd), Kd, rs), matMul3(&Tmp21_2, Jp, &Jinv, dre));
	matMul5(&tauVE2, &E, Md, dJ, &exJpinv, &Tmp21);
	matMulScl(&tauVE, -1, matAdd(&Tmp41, &tauVE1, &tauVE2));
	matMul(&tauIN, matSub(&Tmp42, &E, &Jt), F);		// tauIN = (E-J^T)F
	matAdd3(tau, &tauNC, &tauVE, &tauIN);
	// デバッグ
//	if(sim.step == 1000){ matPrint(&S); matPrint(&E); matPrint(&exJpinv);}
//	if(sim.step == 1000){ matPrint(matMul(&Tmp22, Jp, &exJpinv));}		// 単位行列になる
	matPrint(matSub(&Tmp21, matMul(&Tmp21, Jp, dq), matMul3(&Tmp21_2, matInv(&Tmp22, NULL, Cd), Kd, rs)));		// 0になればOK
	return	0;
}

////////////////////////////////////////////////////////
// Voigt制御則
////////////////////////////////////////////////////////
int ctrlVoigt(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Md, const Matrix *Cd, const Matrix *Kd)
{
	int	jnt, crd;
	static Matrix	Jinv, Jt, Tmp21, Tmp22;
	static Matrix	tauNC, tauVE, tauIN, E;

	// 初期化
	if(sim.step == 0){
		matInit(&Jinv,2,2);	matInit(&Jt,2,2);
		matInit(&Tmp21,2,1); matInit(&Tmp22,2,2);
		matInit(&tauNC,2,1); matInit(&tauVE,2,1); matInit(&tauIN,2,1); matInit(&E,2,2);
		for(crd=0;crd<2;crd++){
			if(sim.imp.C.el[crd][crd]*sim.imp.C.el[crd][crd] < 4*sim.imp.M.el[crd][crd]*sim.imp.K.el[crd][crd] ){
				sim.imp.T[crd] = 2*PI/sqrt(sim.imp.K.el[crd][crd]/sim.imp.M.el[crd][crd]-sim.imp.C.el[crd][crd]*sim.imp.C.el[crd][crd]/(4*sim.imp.M.el[crd][crd]*sim.imp.M.el[crd][crd]));
				printf("T[%d]=%f\n", crd, sim.imp.T[crd]);
			}
		}
	}
	// 制御則
	matTrans(&Jt, J);	matInv(&Jinv, NULL, J);	// J^T, J^{-1}
	matMul3(&E, Mq, &Jinv, matInv(&Tmp22, NULL, Md));	// E = Mq*J^{-1}*Md^{-1}
	matSub(&tauNC, h, matMul4(&Tmp21, Mq, &Jinv, dJ, dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matAdd(&Tmp21, matMul(&tauVE, Cd, dre), matMul(&Tmp21, Kd, re));
	matMulScl(&tauVE, -1, matMul(&Tmp21, &E, &Tmp21));	// tauVE = -E(Cd*dr+Kd*r)
	matMul(&tauIN, matSub(&Tmp22, &E, &Jt), F);		// tauIN = (E-J^T)F
	matAdd3(tau, &tauNC, &tauVE, &tauIN);
	return	0;
}
