#ifndef _INC_MATBASE
#define _INC_MATBASE

#include <stdio.h>

typedef	double	SCALAR;
#define MAT_EPS		(1e-6)		// ���e�덷
///////////////////////////
// �s��\����
///////////////////////////
typedef struct{
	int	row, col;   // �s���C��
	int	state;		// ���
	SCALAR	**el;	// �v�f�ւ̃|�C���^
}Matrix;

///////////////////////////
// state�萔
///////////////////////////
#define MAT_STATE_UNINIT	0		// ����������Ă��Ȃ�
#define MAT_STATE_NORMAL	1
#define MAT_STATE_EROOR		2		// 

///////////////////////////
// �G���[�萔
///////////////////////////
#define MAT_ERROR_MEMORY	0			// �������m�ۂɎ��s
#define MAT_ERROR_UNMATCH_SIZE	1		// �s��T�C�Y���Ⴄ���ߌv�Z�s�\
#define MAT_ERROR_NONEXISTENT	2		// �v�Z���ʂ����݂��Ȃ�
#define MAT_ERROR_ARGUMENT	3		// �������Ԉ���Ă���

///////////////////////////
// �f�o�b�O�萔
///////////////////////////
#define MAT_CHECK_SIZE	1

///////////////////////////
// �s���{����
///////////////////////////
Matrix *matInit(Matrix *A, int row, int col);
int matFree(Matrix *mat);
Matrix *matSet(Matrix *mat, double *element);
Matrix *matRand(Matrix *A);
int matPrint(const Matrix *A);
Matrix *matCopy(Matrix *B, const Matrix *A);
Matrix *matAssign(Matrix *B, const Matrix *A, int start_row, int start_col);
Matrix *matCut(Matrix *B, const Matrix *A, int start_row, int start_col);
Matrix *matReplacRow(Matrix *B, const Matrix *A, int row1, int row2);
//Matrix *matCutRow(Matrix *B, const Matrix *A, int row);
int matErr(Matrix *A, int err_no);
Matrix *matReset(Matrix *A, int row, int col);

Matrix *matShareInit(Matrix *A, int row, int col);
Matrix *matShareCut(Matrix *B, const Matrix *A, int start_row, int start_col);
Matrix *matShareResize(Matrix *A, int row, int col);
int matShareFree(Matrix *A);

///////////////////////////
// �s���{���Z
///////////////////////////
Matrix *matZero(Matrix *A);
Matrix *matUnit(Matrix *A);
Matrix *matAdd(Matrix *C, const Matrix *A, const Matrix *B);
Matrix *matAdd3(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C);
Matrix *matAdd4(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C, const Matrix *D);
Matrix *matSub(Matrix *C, const Matrix *A, const Matrix *B);
Matrix *matMul(Matrix *C, const Matrix *A, const Matrix *B);
Matrix *matMul3(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C);
Matrix *matMul4(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C, const Matrix *D);
Matrix *matMul5(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C, const Matrix *D, const Matrix *E);
Matrix *matTrans(Matrix *B, const Matrix *A);
double matTrace(const Matrix *A);
Matrix *matMulScl(Matrix *B, double k, const Matrix *A);
double matInnerProd(const Matrix *A, const Matrix *B);
double matQuadForm(Matrix *vec1, Matrix *mat, Matrix *vec2);

///////////////////////////////////////////////////
// �啶���ɂ͍s��C�������ɂ̓x�N�g������
///////////////////////////////////////////////////
int matRank(const Matrix *A);
double matDet(const Matrix *A);
Matrix *matInv(Matrix *B, double *det, const Matrix *A);
int matLUD(Matrix *L, Matrix *U, const Matrix *A);
int matQRD(Matrix *Q, Matrix *R, const Matrix *A);
Matrix *matFwdSubsti(Matrix *c, const Matrix *L, const Matrix *b);
Matrix *matBackSubsti(Matrix *x, const Matrix *U, const Matrix *c);

Matrix *matLinEqLUD(Matrix *x, const Matrix *A, const Matrix *b);
Matrix *matDiv(Matrix *X, const Matrix *A, const Matrix *B);

// �쐬��
Matrix *matEgVal(Matrix *x, const Matrix *A);
Matrix *matHouseholder(Matrix *H, const Matrix *x);
Matrix *matHessenberg(Matrix *H, const Matrix *A);
int matQRDHessen(Matrix *Q, Matrix *R, const Matrix *A);
Matrix *matEigen(Matrix *S, Matrix *x, const Matrix *A);
Matrix *matEigenInvPower(Matrix *x, const Matrix *A);

#endif
