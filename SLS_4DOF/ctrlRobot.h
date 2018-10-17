#ifndef _INC_CTRLROBOT
#define _INC_CTRLROBOT

#include "simMain.h"

#define IMP_SWITCH_STEP		5000

////////////////////////////////////////////////////////
// プロトタイプ
////////////////////////////////////////////////////////
int ctrlInit(SIM *sim);
int	setInherentInertia(SIM *sim);
int ctrlPreProcessing(SIM *sim);
int armDynPara(SIM *sim);
int ctrlMaxwellVar(SIM *sim, Matrix *tau);
int ctrlMaxwell(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Fint, const Matrix *Md, const Matrix *Cd, const Matrix *Kd);
int ctrlVoigt(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Md, const Matrix *Cd, const Matrix *Kd);
int ctrlHybrid(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Fint, const Matrix *Md, const Matrix *Cd, const Matrix *Kd);
int ctrlMaxwellSeries(SIM *sim, Matrix *tau);
int ctrlSLS(SIM *sim, Matrix *tau);
int ctrlSLS2(SIM *sim, Matrix *tau);

#endif
