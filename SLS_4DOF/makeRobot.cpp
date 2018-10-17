#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <stdio.h>
#include <math.h>
#include "texturepath.h"
#include "simMain.h"
#include "makeRobot.h"

// �O���ϐ�
extern dWorldID world;
extern dSpaceID space;
extern	dJointID f_joint, r_joint[ARM_JNT], f2_joint; // �Œ�֐߂Ɖ�]�֐�
extern	MyObject base, arm[ARM_JNT], sensor, obj;    // 
extern dJointFeedback force;

////////////////////////////////////////////////////////
// ���{�b�g�쐬
// ����2���R�x
////////////////////////////////////////////////////////
int createRobot(SIM *sim)
{
	int	jnt, crd;
	double	link_pos[ARM_JNT][DIM3];		// �����N�̏d�S�ʒu
	double	sensor_pos[DIM3];		// �Z���T�̏d�S�ʒu
	double	jntP_pos[ARM_JNT][DIM3];		// �֐߂�3�����ʒu
	double	abs_jnt_pos;		// ��΍��W�ɑ΂���֐ߊp
	dMass mass;
	dMatrix3	R;

	// �p�����[�^�ݒ�
	base.sides[CRD_X] = 0.3;	base.sides[CRD_Y] = 0.3;	base.sides[CRD_Z] = 0.4;	base.m = 14.0;	// �y�䒼����
	arm[ARM_M1].l = 0.75;	arm[ARM_M1].r = 0.125;	arm[ARM_M1].m = 1.0;	// �A�[�������N1�~��
	arm[ARM_M2].l = 0.75;	arm[ARM_M2].r = 0.10;	arm[ARM_M2].m = 0.8;	// �A�[�������N2�~��
	arm[ARM_M3].l = 0.6;	arm[ARM_M3].r = 0.10;	arm[ARM_M3].m = 0.6;	// �A�[�������N3�~��
	arm[ARM_M4].l = 0.6;	arm[ARM_M4].r = 0.10;	arm[ARM_M4].m = 0.6;	// �A�[�������N4�~��
	sensor.l = 0.0001;    // �����i�T�C�Y�̉e�����o�Ȃ��悤�ɏ����߂ɐݒ�j
	sensor.r = arm[ARM_M4].r;
	sensor.m   = sensor.l/arm[ARM_M4].l * arm[ARM_M4].m;		// �ŏI�����N�Ɩ��x�����낦��

	// �����ݒ�
#define	Z_OFFSET	0.08
	jntP_pos[ARM_M1][CRD_X] = 0.0;	jntP_pos[ARM_M1][CRD_Y] = 0.0;	jntP_pos[ARM_M1][CRD_Z] = base.sides[CRD_Z]/2.0-Z_OFFSET;	// ���_
	for(crd=0;crd<DIM3;crd++)	link_pos[ARM_M1][crd] = jntP_pos[ARM_M1][crd];
	abs_jnt_pos = 0.0;
	// �y�䐶��
	base.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, base.m, base.sides[CRD_X], base.sides[CRD_Y], base.sides[CRD_Z]);
	dBodySetMass(base.body, &mass);
	dBodySetPosition(base.body, jntP_pos[ARM_M1][CRD_X], jntP_pos[ARM_M1][CRD_Y], jntP_pos[ARM_M1][CRD_Z]+Z_OFFSET);
	base.geom = dCreateBox(space, base.sides[CRD_X], base.sides[CRD_Y], base.sides[CRD_Z]);
	dGeomSetBody(base.geom, base.body);
	// �A�[�������N����
	for(jnt=0;jnt<ARM_JNT;jnt++){
		// �ϐ��ݒ�
		abs_jnt_pos += sim->init_jnt_pos[jnt];
		link_pos[jnt][CRD_X] = jntP_pos[jnt][CRD_X] + arm[jnt].l/2.0*cos(abs_jnt_pos);		// �d�S�ʒu�ֈړ�
		link_pos[jnt][CRD_Y] = jntP_pos[jnt][CRD_Y] + arm[jnt].l/2.0*sin(abs_jnt_pos);		// �d�S�ʒu�ֈړ�
		link_pos[jnt][CRD_Z] = jntP_pos[jnt][CRD_Z];		// �d�S�ʒu�ֈړ�
		// �����N����
		arm[jnt].body   = dBodyCreate(world);
		dMassSetZero(&mass);
		dMassSetCylinderTotal(&mass, arm[jnt].m, DIR_LONG_AXIS_Z, arm[jnt].r, arm[jnt].l);
		dBodySetMass(arm[jnt].body, &mass);
		dBodySetPosition(arm[jnt].body, link_pos[jnt][CRD_X], link_pos[jnt][CRD_Y], link_pos[jnt][CRD_Z]);
		dRFromAxisAndAngle(R, -sin(abs_jnt_pos), cos(abs_jnt_pos), 0, PI/2);
		dBodySetRotation(arm[jnt].body, R);
		arm[jnt].geom = dCreateCylinder(space, arm[jnt].r, arm[jnt].l);
		dGeomSetBody(arm[jnt].geom, arm[jnt].body);
		// �㏈��
		if(jnt==ARM_JNT-1){		// �Z���T�ʒu�̎w��
			sensor_pos[CRD_X] = jntP_pos[jnt][CRD_X] + (arm[jnt].l+sensor.l/2.0)*cos(abs_jnt_pos);
			sensor_pos[CRD_Y] = jntP_pos[jnt][CRD_Y] + (arm[jnt].l+sensor.l/2.0)*sin(abs_jnt_pos);
			sensor_pos[CRD_Z] = jntP_pos[jnt][CRD_Z];
		}else{		// ���̊֐߈ʒu���w��
			jntP_pos[jnt+1][CRD_X] = jntP_pos[jnt][CRD_X] + arm[jnt].l*cos(abs_jnt_pos);
			jntP_pos[jnt+1][CRD_Y] = jntP_pos[jnt][CRD_Y] + arm[jnt].l*sin(abs_jnt_pos);
			jntP_pos[jnt+1][CRD_Z] = jntP_pos[jnt][CRD_Z];
		}
	}
	// �Z���T�i���ɕt�����C�T�C�Y�̉e�����o�Ȃ��悤�ɏ����߂ɐݒ�j
	sensor.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, sensor.m, DIR_LONG_AXIS_Z, sensor.r, sensor.l);
	dBodySetMass(sensor.body, &mass);
	dBodySetPosition(sensor.body, sensor_pos[CRD_X], sensor_pos[CRD_Y], sensor_pos[CRD_Z]);
	dRFromAxisAndAngle(R, -sin(abs_jnt_pos), cos(abs_jnt_pos), 0, PI/2);
	dBodySetRotation(sensor.body, R);
	sensor.geom = dCreateCylinder(space, sensor.r, sensor.l);
	dGeomSetBody(sensor.geom, sensor.body);
	// �Œ�W���C���g
	f_joint = dJointCreateFixed(world, 0);
	dJointAttach(f_joint, base.body, 0);
	dJointSetFixed(f_joint);
	// �q���W�W���C���g
	for(jnt=0;jnt<ARM_JNT;jnt++){
		r_joint[jnt] = dJointCreateHinge(world, 0);
		if(jnt==0)	dJointAttach(r_joint[jnt], arm[jnt].body, base.body);
		else	dJointAttach(r_joint[jnt], arm[jnt].body, arm[jnt-1].body);
		dJointSetHingeAnchor(r_joint[jnt], jntP_pos[jnt][CRD_X], jntP_pos[jnt][CRD_Y], jntP_pos[jnt][CRD_Z]);
		dJointSetHingeAxis(r_joint[jnt], 0, 0, 1);
		dJointSetHingeParam(r_joint[jnt], dParamLoStop, -M_PI);
		dJointSetHingeParam(r_joint[jnt], dParamHiStop, M_PI);
	}
	// �Œ�W���C���g
	f2_joint = dJointCreateFixed(world, 0);
	dJointAttach(f2_joint, sensor.body, arm[ARM_M4].body);
	dJointSetFixed(f2_joint);
	// �Z���T�ݒ�i�͂ƃg���N�̎擾�ɕK�v�j
	dJointSetFeedback(f2_joint, &force);

	return	0;
}

////////////////////////////////////////////////////////
// ���{�b�g�`��
////////////////////////////////////////////////////////
int drawRobot()
{
	int	jnt;
	double	link_color[ARM_JNT][DIM3] = {
		{0.0,0.0,1.0}, {0.0,0.5,0.5}, {0.0,0.0,1.0}, {0.0,0.5,0.5},	};
	const dReal *pos, *R;

	// �y��`��
	dsSetColor(1.0, 0.0, 0.0);                       // �ԐF
	pos = dBodyGetPosition(base.body);
	R   = dBodyGetRotation(base.body);
	dsDrawBox(pos, R, base.sides);
	// �A�[���`��
	for(jnt=0;jnt<ARM_JNT;jnt++){
		pos = dBodyGetPosition(arm[jnt].body);
		R   = dBodyGetRotation(arm[jnt].body);
		dsSetColor(link_color[jnt][0],link_color[jnt][1],link_color[jnt][2]);
		dsDrawCylinder(pos, R, arm[jnt].l, arm[jnt].r);
	}
	// �Z���T�`��
	pos = dBodyGetPosition(sensor.body);
	R   = dBodyGetRotation(sensor.body);
	dsSetColor(0.0,0.5,0.5);
	dsDrawCylinder(pos, R, sensor.l, sensor.r);
	return	0;
}

////////////////////////////////////////////////////////
// ���ʒ��S�\��
////////////////////////////////////////////////////////
/*
int drawCoM()
{
	int i,j,k;
	double pos_CoM[3]={0.0};
	double m_all=0.0;
	dMatrix3 dammy_R;

	dRSetIdentity(dammy_R);
	for (i=0;i<2;i++){
		for (j=0;j<5;j++){
			for (k=0;k<DIM_THREE;k++) pos_CoM[k] += *(dBodyGetPosition(leg[i][j].body)+k) *  leg[i][j].m;
			m_all += leg[i][j].m;
		}
	}

	for (i=0;i<2;i++){
		for (k=0;k<DIM_THREE;k++) pos_CoM[k] +=*(dBodyGetPosition(hip[i].body)+k) *  hip[i].m;
		m_all += hip[i].m;
	}

	for (i=0;i<3;i++)  pos_CoM[i] /= m_all;

	dsSetColor(0.5, 0.0, 0.0);
	dsDrawSphere(pos_CoM, dammy_R , 0.01);

	pos_CoM[2]=0.001;
	dsSetColor(0.2, 0.0, 0.1);
	dsDrawSphere(pos_CoM, dammy_R , 0.01);

	// �f�o�b�O�\��
	printf("mass=%f ", m_all);
	return 0;
}
*/

#if 0
////////////////////////////////////////////////////////
// �O�͕`��
// �̓Z���T�̒l�ɔ�Ⴕ��������\������
////////////////////////////////////////////////////////
int drawExtForce()
{
	int crd;
	dJointFeedback *fb;
	dVector3	p_s, p_e;    // ���̎n�_�ƏI�_
	double k1 = 0.5;  // �����̔��萔
	dVector3	arrow_center, arrow_r, arrow_l;    // ���̒��_
	dMatrix3	R;
	
	dBodyGetRelPointPos(sensor.body, 0.0, 0.0, sensor.l/2.0, p_s);			// ���ʒu
//		pos = dBodyGetPosition(leg[0][1].body);
//	dJointGetHingeAnchor(f2_joint, pos);
//		endP[0] = pos[0] + k1*sensor[jnt].f1[0];
//		endP[1] = pos[1] + k1*sensor[jnt].f1[1];
//		endP[2] = pos[2] + k1*sensor[jnt].f1[2];
	fb = dJointGetFeedback(f2_joint);
	for(crd=0;crd<DIM3;crd++)	p_e[crd] = p_s[crd] + k1*fb->f1[crd];
	p_e[CRD_Z] = p_s[CRD_Z];	// z�����𖳎�
	dsSetColor(0.5, 0.5, 1.0);                    // 
#if 0
	dsDrawLine(p_s, p_e); // p_s����p_e�܂ł̒�����`��
#else
	arrow_l[CRD_X] = p_s[CRD_X]+k1/2*fb->f1[CRD_X]-k1/2*fb->f1[CRD_Y];
	arrow_l[CRD_Y] = p_s[CRD_Y]+k1/2*fb->f1[CRD_Y]+k1/2*fb->f1[CRD_X];
	arrow_l[CRD_Z] = p_s[CRD_Z];
	arrow_r[CRD_X] = p_s[CRD_X]+k1/2*fb->f1[CRD_X]+k1/2*fb->f1[CRD_Y];
	arrow_r[CRD_Y] = p_s[CRD_Y]+k1/2*fb->f1[CRD_Y]-k1/2*fb->f1[CRD_X];
	arrow_r[CRD_Z] = p_s[CRD_Z];
	for(crd=0;crd<DIM3;crd++)	arrow_center[crd] = (p_s[crd]+arrow_r[crd]+arrow_l[crd])/3.0;
//	for(crd=0;crd<DIM3;crd++){p_s[crd] -= arrow_center[crd];arrow_r[crd] -= arrow_center[crd];arrow_l[crd] -= arrow_center[crd];}
	dRFromAxisAndAngle(R, 0, 0, 1, 0);
//	printf("%f %f %f\n%f %f %f\n%f %f %f\n\n", R[0],R[1],R[2],R[4],R[5],R[6],R[8],R[9],R[10]);
//	printf("%f %f %f\n\n", arrow_center[0],arrow_center[1],arrow_center[2]);
	dsDrawTriangle(arrow_center, R, p_s, arrow_l, arrow_r, 1); // 
	dsDrawLine(p_s, p_e); // p_s����p_e�܂ł̒�����`��
#endif
	return	0;
}
#endif

////////////////////////////////////////////////////////
// �O�͕`��
// �̓Z���T�̒l�ɔ�Ⴕ��������\������
////////////////////////////////////////////////////////
int drawExtForce()
{
	int crd, width;
	dJointFeedback *fb;
	dVector3	p_s, p_e;    // ���̎n�_�ƏI�_
	double k1 = 0.3;  // �����̔��萔
	double line_w = 0.05;
	dVector3	arrow_center, arrow_r, arrow_l;    // ���̒��_
	dVector3	rect_ul, rect_ll, rect_ur, rect_lr;    // ���̒��_
	dVector3	line, line_e;    // 
	dVector3	ext_f;	// �O��
	dMatrix3	R;
	double angArrow = PI/6;  //���̊p�x rad
	dJointFeedback *p_force;
	
	dBodyGetRelPointPos(sensor.body, 0.0, 0.0, sensor.l/2.0, p_s);			// ���ʒu
//		endP[0] = pos[0] + k1*sensor[jnt].f1[0];
//		endP[1] = pos[1] + k1*sensor[jnt].f1[1];
//		endP[2] = pos[2] + k1*sensor[jnt].f1[2];
	p_force = dJointGetFeedback(f2_joint);
	for(crd=0;crd<DIM3;crd++)	ext_f[crd] = -p_force->f1[crd];	// �Ώۂ��Z���T�ɋy�ڂ��Ă����=�Z���T���֐߂ɋy�ڂ��Ă����
	p_s[CRD_Z] += sensor.r; //�r�̏�ɕ\��
	for(crd=0;crd<DIM3;crd++)	p_e[crd] = p_s[crd] - k1*ext_f[crd];
//	p_e[CRD_Z] = p_s[CRD_Z] + sensor.r;	// �r�̏�ɕ\��
	p_e[CRD_Z] = p_s[CRD_Z];	// z�����̗͖͂���
	dsSetColor(1.0, 1.0, 1.0);                    // 
#if 1
//	arrow_l[CRD_X] = p_s[CRD_X]+k1/2*fb->f1[CRD_X]-k1/2*fb->f1[CRD_Y];
//	arrow_l[CRD_Y] = p_s[CRD_Y]+k1/2*fb->f1[CRD_Y]+k1/2*fb->f1[CRD_X];
//	arrow_l[CRD_Z] = p_s[CRD_Z];
//	arrow_r[CRD_X] = p_s[CRD_X]+k1/2*fb->f1[CRD_X]+k1/2*fb->f1[CRD_Y];
//	arrow_r[CRD_Y] = p_s[CRD_Y]+k1/2*fb->f1[CRD_Y]-k1/2*fb->f1[CRD_X];
//	arrow_r[CRD_Z] = p_s[CRD_Z];
	arrow_center[CRD_X] = 0;	arrow_center[CRD_Y] = 0.0;	arrow_center[CRD_Z] = 0;
	arrow_l[CRD_X] = sin(angArrow)* ext_f[0] * k1 / 3;
	arrow_l[CRD_Y] = cos(angArrow)* ext_f[0] * k1 / 3;
	arrow_l[CRD_Z] = 0;
	arrow_r[CRD_X] = -sin(angArrow)* ext_f[0] * k1 / 3;
	arrow_r[CRD_Y] = cos(angArrow)* ext_f[0] * k1 / 3;
	arrow_r[CRD_Z] = 0;
	rect_ul[CRD_X] = rect_ll[CRD_X] = sin(angArrow)* ext_f[0] * k1 / 8;
	rect_ur[CRD_X] = rect_lr[CRD_X] = -sin(angArrow)* ext_f[0] * k1 / 8;
	rect_ul[CRD_Y] = rect_ur[CRD_Y] = cos(angArrow)* ext_f[0] * k1 / 3;
	rect_ll[CRD_Y] = rect_lr[CRD_Y] = k1 * ext_f[0];
	rect_ul[CRD_Z] = rect_ll[CRD_Z] = rect_ur[CRD_Z] = rect_lr[CRD_Z] = 0.0;

	dRFromAxisAndAngle(R, 0, 0, 1, PI - atan2(p_s[CRD_Y] - p_e[CRD_Y], p_s[CRD_X] - p_e[CRD_X]));
//	printf("%f %f %f\n\n", rect_ll[0],rect_ll[1],rect_ll[2]);

	// ���̓�
	dsDrawTriangle(p_s, R, arrow_center, arrow_l, arrow_r, 1); // 
	// ���̐��i�O�p�`��2���킹�ĕ��̎������l�p�`�Ƃ��ĕ\���j
	dsDrawTriangle(p_s, R, rect_ul, rect_ll, rect_ur, 1); // 
	dsDrawTriangle(p_s, R, rect_ur, rect_ll, rect_lr, 1); // 

//	dsDrawLine(p_s, p_e); // p_s����p_e�܂ł̒�����`��
//	dsDrawLine(line_center, p_e); // p_s����p_e�܂ł̒�����`��
#endif
	return	0;
}

////////////////////////////////////////////////////////
// ���{�b�g�j��
////////////////////////////////////////////////////////
int destroyRobot()
{
	int	jnt;
	// �W���C���g�j��
	dJointDestroy(f_joint);   // �y��Œ�
	for(jnt=0;jnt<ARM_JNT;jnt++)	dJointDestroy(r_joint[jnt]);   // �A�[��
	dJointDestroy(f2_joint);   // �Z���T�Œ�
	// �{�f�B�j��
	dBodyDestroy(base.body); // �y��
	for(jnt=0;jnt<ARM_JNT;jnt++)	dBodyDestroy(arm[jnt].body);  // �A�[��
	dBodyDestroy(sensor.body);  // �Z���T
	// �W�I���g���j��
	dGeomDestroy(base.geom); // �y��
	for(jnt=0;jnt<ARM_JNT;jnt++)	dGeomDestroy(arm[jnt].geom);  // �A�[��
	dGeomDestroy(sensor.geom);  // �Z���T
	return	0;
}

////////////////////////////////////////////////////////
// �Ώۍ쐬
////////////////////////////////////////////////////////
int createObject(SIM *sim)
{
	dMass mass;
	dMatrix3	R;
	// �p�����[�^�ݒ�
//	obj.l = 0.15;	obj.r = 0.15;	obj.m = 0.5;	// �~��
	obj.l = 0.15;	obj.r = 0.10;	obj.m = 0.2;	// �~��
#if 1
	// �~��
	obj.body   = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, obj.m, DIR_LONG_AXIS_Z, obj.r, obj.l);
	dBodySetMass(obj.body, &mass);
	dBodySetPosition(obj.body, sim->init_obj_pos[CRD_X], sim->init_obj_pos[CRD_Y], sim->init_obj_pos[CRD_Z]);
//	dRFromAxisAndAngle(R, 1, 0, 0, PI/2);	// ������y�����ɉ�]
	dRFrom2Axes(R, sim->init_obj_att[AXIS_X][CRD_X], sim->init_obj_att[AXIS_X][CRD_Y], sim->init_obj_att[AXIS_X][CRD_Z], sim->init_obj_att[AXIS_Y][CRD_X], sim->init_obj_att[AXIS_Y][CRD_Y], sim->init_obj_att[AXIS_Y][CRD_Z]);
	dBodySetRotation(obj.body, R);
	obj.geom = dCreateCylinder(space, obj.r, obj.l);
	dGeomSetBody(obj.geom, obj.body);
#else
	// ��
	obj.body   = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetSphereTotal(&mass, obj.m, obj.r);
	dBodySetMass(obj.body, &mass);
	dBodySetPosition(obj.body, sim->init_obj_pos[CRD_X], sim->init_obj_pos[CRD_Y], sim->init_obj_pos[CRD_Z]);
	obj.geom = dCreateSphere(space, obj.r);
	dGeomSetBody(obj.geom, obj.body);
#endif
	return	0;
}

////////////////////////////////////////////////////////
// �Ώە`��
////////////////////////////////////////////////////////
int drawObject()
{
	int	i;
	const dReal *pos,*R;
	// �ՓˑΏە`��
	pos = dBodyGetPosition(obj.body);
	R   = dBodyGetRotation(obj.body);
	dsSetColor(0.5, 0.5, 1.0);                    // 
#if 1
	dsDrawCylinder(pos, R, obj.l, obj.r);
#else
	dsDrawSphere(pos, R, obj.r);
#endif
	return	0;
}

////////////////////////////////////////////////////////
// �Ώ۔j��
////////////////////////////////////////////////////////
int destroyObject()
{
	// �{�f�B�j��
	dBodyDestroy(obj.body);
	// �W�I���g���j��
	dGeomDestroy(obj.geom);
	return	0;
}
