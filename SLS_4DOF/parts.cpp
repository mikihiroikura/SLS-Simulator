#include <stdio.h>
#include <math.h>
#include <GL/glut.h>
#include "graphic.h"

////////////////////////////////////////////////////////////////////
// 現在位置を中心にして 2x 2y 2z の大きさの直方体を表示
////////////////////////////////////////////////////////////////////
int myBox(double x, double y, double z)
{
  int i, j;
  static int face[][4] = {
    { 0, 1, 2, 3 }, { 1, 5, 6, 2 }, { 5, 4, 7, 6 }, { 4, 0, 3, 7 }, { 4, 5, 1, 0 }, { 3, 2, 6, 7 }
  };
  static GLdouble normal[][3] = {
    { 0.0, 0.0,-1.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 0.0, 1.0 }, {-1.0, 0.0, 0.0 }, { 0.0,-1.0, 0.0 }, { 0.0, 1.0, 0.0 }
  };
  GLdouble vertex[][3] = {
    { -x, -y, -z },	{  x, -y, -z },	{  x,  y, -z },	{ -x,  y, -z },
    { -x, -y,  z },	{  x, -y,  z },	{  x,  y,  z },	{ -x,  y,  z },
  };
  glBegin(GL_QUADS);
  for (j = 0; j < 6; ++j) {
    glNormal3dv(normal[j]);
    for (i = 4; --i >= 0;)	glVertex3dv(vertex[face[j][i]]);
  }
  glEnd();
  return 0;
}

int myCylinder(double radius, double height, double sides)
{
	double step = 6.28318 / (double)sides;
	int i = 0;
	// 上面
	glNormal3d(0.0, 1.0, 0.0);
	glBegin(GL_TRIANGLE_FAN);
	while(i<sides){
		double t = step * i++;
		glVertex3d(radius*sin(t), height, radius*cos(t));
	}
	glEnd();
	// 底面
	glNormal3d(0.0, -1.0, 0.0);
	glBegin(GL_TRIANGLE_FAN);
	while(--i>=0){
		double t = step * i;
		glVertex3d(radius*sin(t), -height, radius*cos(t));
	}
	glEnd();
	// 側面
	glBegin(GL_QUAD_STRIP);
	while(i<=sides){
		double t = step * i++;
		double x = sin(t);
		double z = cos(t);
		glNormal3d(x, 0.0, z);
		glVertex3f(radius*x, height, radius*z);
		glVertex3f(radius*x, -height, radius*z);
	}
	glEnd();
	return 0;
}

////////////////////////////////////////////////////////////////////
// 格子模様
////////////////////////////////////////////////////////////////////
int myGround(double height)
{
#define GROUND_WIDTH	0.3
#define GROUND_BLOCK_NUM_X	8
#define GROUND_BLOCK_NUM_Z	5
	static GLfloat ground[][4] = {
//		{0.6, 0.6, 0.6, 1.0},
//		{0.3, 0.3, 0.3, 1.0},
		{0.9, 0.9, 0.9, 1.0},
		{0.5, 0.5, 0.5, 1.0},
	};
	static GLfloat shininess[] = {100.0};
	int i, j;
	glNormal3d(0.0, 1.0, 0.0);
	for(j=-GROUND_BLOCK_NUM_Z; j<GROUND_BLOCK_NUM_Z; j++){
		for(i=-GROUND_BLOCK_NUM_X; i<GROUND_BLOCK_NUM_X; i++){
			glMaterialfv(GL_FRONT, GL_DIFFUSE, ground[(i+j)&1]);
			glMaterialfv(GL_FRONT, GL_SHININESS, shininess); 
			glBegin(GL_QUADS);
			glVertex3d((GLdouble)i*GROUND_WIDTH, height, (GLdouble)j*GROUND_WIDTH);
			glVertex3d((GLdouble)i*GROUND_WIDTH, height, (GLdouble)(j+1)*GROUND_WIDTH);
			glVertex3d((GLdouble)(i+1)*GROUND_WIDTH, height, (GLdouble)(j+1)*GROUND_WIDTH);
			glVertex3d((GLdouble)(i+1)*GROUND_WIDTH, height, (GLdouble)j*GROUND_WIDTH);
			glEnd();
		}
	}
	return 0;
}

int myPalm(double radius, double height, double length, double sides)
{
  GLdouble vertex[][3] = {
    { -length,  height, -radius },
    {  length,  height, -radius },
    {  length,  height,  radius },
    { -length,  height,  radius },
  };
  GLdouble vertex2[][3] = {
    { -length,  -height, -radius },
    {  length,  -height, -radius },
    {  length,  -height,  radius },
    { -length,  -height,  radius },
  };
  GLdouble vertex3[][3] = {
    {  length, -height,  radius },
    { -length, -height,  radius },
    { -length,  height,  radius },
    {  length,  height,  radius },
  };
	GLdouble vertex4[][3] = {
	{ -length, -height, -radius },
	{  length, -height, -radius },
	{  length,  height, -radius },
	{ -length,  height, -radius },
	};
	double step = 6.28318 / (double)sides;
	int i = 0;
	// 上面
	glNormal3d(0.0, 1.0, 0.0);
	glBegin(GL_TRIANGLE_FAN);
	while(i<sides/2+1){
		double t = step * i++;
		glVertex3d(radius*sin(t)+length, height, radius*cos(t));
	}
	while(i<sides){
		double t = step * i++;
		glVertex3d(radius*sin(t)-length, height, radius*cos(t));
	}
	glEnd();
	glBegin(GL_QUADS);
    for (i = 4; --i >= 0;) {
      glVertex3dv(vertex[i]);
    }
	glEnd();
	// 底面
	glNormal3d(0.0, -1.0, 0.0);
	glBegin(GL_TRIANGLE_FAN);
	i=sides;
	while(--i>=sides/2+1){
		double t = step * i;
		glVertex3d(radius*sin(t)-length, -height, radius*cos(t));
	}
	while(--i>=0){
		double t = step * i;
		glVertex3d(radius*sin(t)+length, -height, radius*cos(t));
	}
	glEnd();
	glBegin(GL_QUADS);
    for (i=0;i<4;i++) {
      glVertex3dv(vertex2[i]);
    }
	glEnd();
	// 側面
	glBegin(GL_QUAD_STRIP);
	i=0;
	while(i<sides/2+1){
		double t = step * i++;
		double x = sin(t);
		double z = cos(t);
		glNormal3d(x, 0.0, z);
		glVertex3f(radius*x+length, height, radius*z);
		glVertex3f(radius*x+length, -height, radius*z);
	}
	glEnd();
	glBegin(GL_QUAD_STRIP);
	i=sides/2;
	while(i<=sides){
		double t = step * i++;
		double x = sin(t);
		double z = cos(t);
		glNormal3d(x, 0.0, z);
		glVertex3f(radius*x-length, height, radius*z);
		glVertex3f(radius*x-length, -height, radius*z);
	}
	glEnd();
	glBegin(GL_QUADS);
	glNormal3d(0.0, 0.0, 1.0);
    for (i=3;i>=0;i--)	glVertex3dv(vertex3[i]);
	glNormal3d(0.0, 0.0, -1.0);
    for (i=3;i>=0;i--)	glVertex3dv(vertex4[i]);
	glEnd();
	return 0;
}

int myLinkPart(double radius, double height, double length, double sides)
{
  GLdouble vertex[][3] = {
    { -length,  height, -radius },
    {  length,  height, -radius },
    {  length,  height,  radius },
    { -length,  height,  radius },
  };
  GLdouble vertex2[][3] = {
    { -length,  -height, -radius },
    {  length,  -height, -radius },
    {  length,  -height,  radius },
    { -length,  -height,  radius },
  };
  GLdouble vertex3[][3] = {
    {  length, -height,  radius },
    { -length, -height,  radius },
    { -length,  height,  radius },
    {  length,  height,  radius },
  };
	GLdouble vertex4[][3] = {
	{ -length, -height, -radius },
	{  length, -height, -radius },
	{  length,  height, -radius },
	{ -length,  height, -radius },
	};
  GLdouble vertex5[][3] = {
    { -length, -height,  radius },
    { -length, -height, -radius },
    { -length,  height, -radius },
    { -length,  height,  radius }
  };
	double step = 6.28318 / (double)sides;
	int i = 0;
	// 上面
	glNormal3d(0.0, 1.0, 0.0);
	glBegin(GL_TRIANGLE_FAN);
	while(i<sides/2+1){
		double t = step * i++;
		glVertex3d(radius*sin(t)+length, height, radius*cos(t));
	}
	glEnd();
	glBegin(GL_QUADS);
    for (i = 4; --i >= 0;) {
      glVertex3dv(vertex[i]);
    }
	glEnd();
	// 底面
	glNormal3d(0.0, -1.0, 0.0);
	glBegin(GL_TRIANGLE_FAN);
	i=sides/2+1;
	while(--i>=0){
		double t = step * i;
		glVertex3d(radius*sin(t)+length, -height, radius*cos(t));
	}
	glEnd();
	glBegin(GL_QUADS);
    for (i=0;i<4;i++) {
      glVertex3dv(vertex2[i]);
    }
	glEnd();
	// 側面
	glBegin(GL_QUAD_STRIP);
	i=0;
	while(i<sides/2+1){
		double t = step * i++;
		double x = sin(t);
		double z = cos(t);
		glNormal3d(x, 0.0, z);
		glVertex3f(radius*x+length, height, radius*z);
		glVertex3f(radius*x+length, -height, radius*z);
	}
	glEnd();
	glBegin(GL_QUADS);
	glNormal3d(0.0, 0.0, 1.0);
    for (i=3;i>=0;i--) {
      glVertex3dv(vertex3[i]);
    }
	glNormal3d(0.0, 0.0, -1.0);
    for (i=3;i>=0;i--) {
      glVertex3dv(vertex4[i]);
    }
	glNormal3d(-1.0, 0.0, 0.0);
    for (i=3;i>=0;i--) {
      glVertex3dv(vertex5[i]);
    }
	glEnd();
	return 0;
}


////////////////////////////////////////////////////////////////////
// 中心座標が(x_center, y_center, z_center)で x y z の大きさの直方体を表示
////////////////////////////////////////////////////////////////////
int myCuboid(double x, double y, double z, double *center)
{
  int i, j;
  static int face[][4] = {
    { 0, 1, 2, 3 }, { 1, 5, 6, 2 }, { 5, 4, 7, 6 }, { 4, 0, 3, 7 }, { 4, 5, 1, 0 }, { 3, 2, 6, 7 }
  };
  static GLdouble normal[][3] = {
    { 0.0, 0.0,-1.0 }, { 1.0, 0.0, 0.0 }, { 0.0, 0.0, 1.0 }, {-1.0, 0.0, 0.0 }, { 0.0,-1.0, 0.0 }, { 0.0, 1.0, 0.0 }
  };
  GLdouble vertex[][3] = {
    { -x/2.0, -y/2.0, -z/2.0 },	{  x/2.0, -y/2.0, -z/2.0 },	{  x/2.0,  y/2.0, -z/2.0 },	{ -x/2.0,  y/2.0, -z/2.0 },
	{ -x/2.0, -y/2.0,  z/2.0 },	{  x/2.0, -y/2.0,  z/2.0 },	{  x/2.0,  y/2.0,  z/2.0 },	{ -x/2.0,  y/2.0,  z/2.0 },
  };
  for (i=0; i<8; i++)	  for (j=0; j<3; j++)	vertex[i][j] += center[j];
  glBegin(GL_QUADS);
  for (j = 0; j < 6; ++j) {
    glNormal3dv(normal[j]);
    for (i = 4; --i >= 0;)	glVertex3dv(vertex[face[j][i]]);
  }
  glEnd();
  return 0;
}

////////////////////////////////////////////////////////////////////
// 円柱中心座標を*centerで渡す
////////////////////////////////////////////////////////////////////
int myCylinder2(double radius, double height, double sides, double *center)
{
	double step = 6.28318 / (double)sides;
	double	x, y, t, tmp;
	int i = 0;
	// 上面
	glNormal3d(0.0, 0.0, 1.0);
	glBegin(GL_TRIANGLE_FAN);
	for(i=0; i<sides; i++){
		t = step * i;
		x = radius*cos(t);	y = radius*sin(t);
		glVertex3d(x+center[0], y+center[1], height/2.0+center[2]);
	}
	glEnd();
	// 底面
	glNormal3d(0.0, 0.0, -1.0);
	glBegin(GL_TRIANGLE_FAN);
	for(i=0; i<sides; i++){
		t = -step * i;		// 時計回りにする
		x = radius*cos(t);	y = radius*sin(t);
		glVertex3d(x+center[0], y+center[1], -height/2.0+center[2]);
	}
	glEnd();
	// 側面
	glBegin(GL_QUAD_STRIP);
	for(i=0; i<=sides; i++){
		double t = step * i;
		x = radius*cos(t);	y = radius*sin(t);
		glNormal3d(cos(t), sin(t), 0.0);
		glVertex3f(x+center[0], y+center[1], height/2.0+center[2]);
		glVertex3f(x+center[0], y+center[1], -height/2.0+center[2]);
	}
	glEnd();
	return 0;
}

////////////////////////////////////////////////////////////////////
// 円柱を点(-radius,0,-height/2)と点(radius,0,height/2)を通る平面で切断したもの
// 切断前の円柱中心座標を*centerで渡す
////////////////////////////////////////////////////////////////////
int myCylinderHalf(double radius, double height, double sides, double *center)
{
	double step = 6.28318 / (double)sides;
	double	x, y, t, tmp;
	int i = 0;
	// 斜面
	tmp = sqrt(4*radius*radius+height*height);
	glNormal3d(-height/tmp, 0.0, 2*radius/tmp);
	glBegin(GL_TRIANGLE_FAN);
	for(i=0; i<sides; i++){
		t = step * i;
		x = radius*cos(t);	y = radius*sin(t);
		glVertex3d(x+center[0], y+center[1], height*x/(2.0*radius)+center[2]);
	}
	glEnd();
	// 底面
	glNormal3d(0.0, 0.0, -1.0);
	glBegin(GL_TRIANGLE_FAN);
	for(i=0; i<sides; i++){
		t = -step * i;		// 時計回りにする
		x = radius*cos(t);	y = radius*sin(t);
		glVertex3d(x+center[0], y+center[1], -height/2.0+center[2]);
	}
	glEnd();
	// 側面
	glBegin(GL_QUAD_STRIP);
	for(i=0; i<=sides; i++){
		t = step * i;
		x = radius*cos(t);	y = radius*sin(t);
		glNormal3d(cos(t), sin(t), 0.0);
		glVertex3f(x+center[0], y+center[1], height*x/(2.0*radius)+center[2]);
		glVertex3f(x+center[0], y+center[1], -height/2.0+center[2]);
	}
	glEnd();
	return 0;
}
