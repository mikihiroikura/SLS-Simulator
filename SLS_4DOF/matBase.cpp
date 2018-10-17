#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "matBase.h"

//////////////////////////////////////////////////
// ��{���j
// �E�Ȃ�ׂ�Matrix*��Ԓl�Ƃ���
// �E�G���[�̏ꍇ�̕Ԓl�� NULL
// �E���͈�����NULL���������ꍇ�̑Ώ��͂��Ȃ�
// �E�o�͈����ɓ��͈����Ɠ����|�C���^�������Ă������ŏ���
// �E�o�͈����͍s��̃T�C�Y�������ĂȂ��Ă������ŏ���
//////////////////////////////////////////////////

//////////////////////////////////////////////////
// �s��ݒ�
//////////////////////////////////////////////////

// �s�񏉊���
// 0�s��ɏ�����
Matrix *matInit(Matrix *A, int row, int col)
{
	int  _row, _col;
	// �����`�F�b�N
	if(row > 0 && col > 0){ A->row = row; A->col = col; }
	else{ matErr(A, MAT_ERROR_MEMORY); return NULL; }
	// �s�������m��
	A->el = (SCALAR **)malloc(sizeof(SCALAR *) * A->row);
	if(A->el == NULL){ matErr(A, MAT_ERROR_MEMORY); return NULL; }
	// �񃁃����m��
	for(_row=0; _row<A->row; _row++){
		A->el[_row] = (SCALAR *)malloc(sizeof(SCALAR) * A->col);
		if(A->el[_row] == NULL){
			while(_row > 0)	free(A->el[--_row]);
			free(A->el); matErr(A, MAT_ERROR_MEMORY); return NULL;
		}
	}
	A->state = MAT_STATE_NORMAL;
	// �e�v�f��0�ŏ�����
	for(_row=0; _row<A->row; _row++) for(_col=0; _col<A->col; _col++)  A->el[_row][_col] = 0.0;
	return	A;
}

// �s����
int matFree(Matrix *mat)
{
	int  row;
	for(row=0; row<mat->row; row++)  free(mat->el[row]);
	free(mat->el);
	return	0;
}

// �s��v�f�ݒ�
Matrix *matSet(Matrix *A, double *element)
{
	int  row, col;
	for(row=0; row<A->row; row++) for(col=0; col<A->col; col++)  A->el[row][col] = element[A->col*row+col];
	return	A;
}

// �����_���s�񐶐�
Matrix *matRand(Matrix *A)
{
	int  row, col;
	srand((unsigned)time(NULL));
	for(row=0; row<A->row; row++) for(col=0; col<A->col; col++)  A->el[row][col] = (SCALAR)rand()/RAND_MAX;
	return	A;
}

// �s��\��
int matPrint(const Matrix *A)
{
	int  row, col;
	for(row=0; row<A->row; row++){ for(col=0; col<A->col; col++) printf("%9f ",A->el[row][col]); printf("\n"); }
	printf("\n");
	return 0;
}

// �s��R�s�[
Matrix *matCopy(Matrix *B, const Matrix *A)
{
	int  row, col;
	if(B->row != A->row || B->col != A->col)	matReset(B, A->row, A->col);
	for(row=0; row<A->row; row++)	for(col=0; col<A->col; col++)	B->el[row][col] = A->el[row][col];
	return	B;
}

// �s��B�̈ʒu(start_row,start_col)�ɍs��A����
Matrix *matAssign(Matrix *B, const Matrix *A, int start_row, int start_col)
{
	int  row, col;
	if(B->row < A->row+start_row || B->col < A->col+start_col){ matErr(NULL, MAT_ERROR_UNMATCH_SIZE); return NULL; }
	for(row=0; row<A->row; row++)	for(col=0; col<A->col; col++)	B->el[row+start_row][col+start_col] = A->el[row][col];
	return	B;
}

// �s��A�̈ʒu(start_row,start_col)����s��B�̃T�C�Y�̃u���b�N�𒊏o
Matrix *matCut(Matrix *B, const Matrix *A, int start_row, int start_col)
{
	int  row, col;
	if(B->row > A->row-start_row || B->col > A->col-start_col){ matErr(NULL, MAT_ERROR_UNMATCH_SIZE); return NULL; }
	for(row=0; row<B->row; row++)	for(col=0; col<B->col; col++)	B->el[row][col] = A->el[row+start_row][col+start_col];
	return	B;
}

// �s��A��row1�Ԗڂ̍s��row2�Ԗڂ̍s�����ւ�
Matrix *matReplacRow(Matrix *B, const Matrix *A, int row1, int row2)
{
	SCALAR	*tmp;
	if(B != A)	matCopy(B, A);
	tmp = B->el[row1]; B->el[row1] = B->el[row2]; B->el[row2] = tmp;		// �s�̃|�C���^�����ւ�
	return	B;
}

// �s��A��B���r
// ����Ȃ�1�C�Ⴄ�Ȃ�0�C�s��T�C�Y���Ⴄ�ꍇ��-1��Ԃ�
int matCompare(const Matrix *A, const Matrix *B)
{
	int  row, col;
	if(B->row != A->row || B->col != A->col){ matErr(NULL, MAT_ERROR_UNMATCH_SIZE); return -1; }
	for(row=0; row<A->row; row++)	for(col=0 ;col<A->col; col++)	if(fabs(A->el[row][col]-B->el[row][col]) > MAT_EPS)		return 0;
	return	1;
}

// �G���[����
int matErr(Matrix *A, int err_no)
{
	if(err_no == MAT_ERROR_MEMORY){ A->row = 0; A->col = 0; A->state = MAT_STATE_UNINIT; A->el = NULL; }
	if(err_no == MAT_ERROR_ARGUMENT){ A->row = 0; A->col = 0; A->state = MAT_STATE_UNINIT; A->el = NULL; }
	return	0;
}

// ���Z�b�g����
// �s���C�񐔂�ύX
Matrix *matReset(Matrix *A, int row, int col)
{
	if(A->state == MAT_STATE_NORMAL)	matFree(A);
	return	matInit(A, row, col);
}

//////////////////////////////////////////////////
// �s�񋤗L���Z
// �l�̃R�s�[���Ȃ����č�����
//////////////////////////////////////////////////
// �s��v�f���L
// matInit ����� matFree �͂��Ȃ�����
Matrix *matShareInit(Matrix *A, int row, int col)
{
	// �����`�F�b�N
	if(row > 0 && col > 0){ A->row = row; A->col = col; }
	else{ matErr(A, MAT_ERROR_MEMORY); return NULL; }
	// �s�������m��
	A->el = (SCALAR **)malloc(sizeof(SCALAR *) * A->row);
	if(A->el == NULL){ matErr(A, MAT_ERROR_MEMORY); return NULL; }
	A->state = MAT_STATE_NORMAL;
	return	A;
}
/*
Matrix *matShareInit(Matrix *B, const Matrix *A)
{
	int		row;
	B->row = A->row; B->col = B->col;
	// �s�������m��
	B->el = (SCALAR **)malloc(sizeof(SCALAR *) * A->row);
	if(B->el == NULL){ matErr(B, MAT_ERROR_MEMORY); return NULL; }
	for(row=0; row<A->row; row++)	*(B->el+row) = *(A->el+row);
	B->state = MAT_STATE_NORMAL;
	return	B;
}
*/

// �������ύX�𔺂�Ȃ��s��T�C�Y�ύX
Matrix *matShareResize(Matrix *A, int row, int col)
{
	// �����`�F�b�N
	if(row > 0 && col > 0){ A->row = row; A->col = col; }
	else{ matErr(A, MAT_ERROR_ARGUMENT); return NULL; }
	return	A;
}

// �u���b�N�s��̒��o
Matrix *matShareCut(Matrix *B, const Matrix *A, int start_row, int start_col)
{
	int  row;
	if(B->row > A->row-start_row || B->col > A->col-start_col){ matErr(NULL, MAT_ERROR_UNMATCH_SIZE); return NULL; }
	for(row=0; row<B->row; row++)	*(B->el+row) = *(A->el+start_row+row) + start_col;
	return	B;
}

// �s����
int matShareFree(Matrix *A)
{
	free(A->el);
	return	0;
}

//////////////////////////////////////////////////
// �s���{�v�Z
//////////////////////////////////////////////////

// �[���s��
// �����s��łȂ��Ă�OK
// �S�Ă̐������O�ɂ���
Matrix *matZero(Matrix *A)
{
	int  row, col;
	for(row=0; row<A->row; row++)	for(col=0; col<A->col; col++)	A->el[row][col] = 0.0;
	return	A;
}

// �P�ʍs��
// �����s��łȂ��Ă�OK
// �Ίp�������P�C�����O�ɂ���
Matrix *matUnit(Matrix *A)
{
	int  row, col;
	for(row=0; row<A->row; row++)	for(col=0; col<A->col; col++)	A->el[row][col] = (row == col) ? 1.0 : 0.0;
	return	A;
}

// �s����Z
Matrix *matAdd(Matrix *C, const Matrix *A, const Matrix *B)
{
	int  row, col;
	if(A->row != B->row || A->col != B->col){ matErr(NULL, MAT_ERROR_UNMATCH_SIZE); return NULL; }
	if(C->row != A->row || C->col != A->col)	matReset(C, A->row, A->col);
	for(row=0; row<A->row; row++)	for(col=0; col<A->col; col++)  C->el[row][col] = A->el[row][col] + B->el[row][col];
	return C;
}

// �s����Z�i3�̍s��j
Matrix *matAdd3(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C)
{
	Matrix  D, E;
	matInit(&D, A->row, A->col);		matInit(&E, A->row, A->col);
	matAdd(&D, A, B);	matAdd(&E, &D, C);
	matCopy(S, &E);	matFree(&D); matFree(&E);
	return	S;
}

// �s����Z�i4�̍s��j
Matrix *matAdd4(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C, const Matrix *D)
{
	Matrix  E, F;
	matInit(&E, A->row, A->col);		matInit(&F, A->row, A->col);
	matAdd3(&E, A, B, C);	matAdd(&F, &E, D);
	matCopy(S, &F);	matFree(&E); matFree(&F);
	return	S;
}

// �s�񌸎Z
Matrix *matSub(Matrix *C, const Matrix *A, const Matrix *B)
{
	int  row, col;
	if(A->row != B->row || A->col != B->col){ matErr(NULL, MAT_ERROR_UNMATCH_SIZE); return NULL; }
	if(C->row != A->row || C->col != A->col)	matReset(C, A->row, A->col);
	for(row=0; row<A->row; row++)	for(col=0; col<A->col; col++)  C->el[row][col] = A->el[row][col] - B->el[row][col];
	return C;
}

// �s���Z
Matrix *matMul(Matrix *C, const Matrix *A, const Matrix *B)
{
	int  row, col, tmp;
	Matrix  D;
	if(A->col != B->row){ matErr(NULL, MAT_ERROR_UNMATCH_SIZE); return NULL; }
	matInit(&D, A->row, B->col);
	for(row=0; row<A->row; row++){
		for(col=0; col<B->col; col++){
			D.el[row][col] = 0.0;
			for(tmp=0; tmp<A->col; tmp++) D.el[row][col] += A->el[row][tmp] * B->el[tmp][col];
		}
	}
	matCopy(C, &D);	matFree(&D);
	return	C;
}

// �s���Z�i3�̍s��j
Matrix *matMul3(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C)
{
	Matrix  D, E;
	matInit(&D, A->row, B->col);		matInit(&E, A->row, C->col);
	matMul(&D, A, B);		matMul(&E, &D, C);
	matCopy(S, &E);	matFree(&D); matFree(&E);
	return	S;
}

// �s���Z�i4�̍s��j
Matrix *matMul4(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C, const Matrix *D)
{
	Matrix  E, F;
	matInit(&E, A->row, C->col);		matInit(&F, A->row, D->col);
	matMul3(&E, A, B, C);		matMul(&F, &E, D);
	matCopy(S, &F);	matFree(&E); matFree(&F);
	return	S;
}

// �s���Z�i5�̍s��j
Matrix *matMul5(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C, const Matrix *D, const Matrix *E)
{
	Matrix  F, G;
	matInit(&F, A->row, D->col);		matInit(&G, A->row, E->col);
	matMul4(&F, A, B, C, D);		matMul(&G, &F, E);
	matCopy(S, &G);	matFree(&F); matFree(&G);
	return	S;
}

// �s��]�u
Matrix *matTrans(Matrix *B, const Matrix *A)
{
	int  row, col;
	Matrix  C;
	matInit(&C, A->col, A->row);
	for(row=0; row<A->row; row++)	for(col=0; col<A->col; col++)  C.el[col][row] = A->el[row][col];
	matCopy(B, &C);	matFree(&C);
	return	B;
}

// �g���[�X
// �Ίp�s��łȂ��Ă�OK
// �Ίp�����̘a���o��
double matTrace(const Matrix *A)
{
	int	sqr;
	double	trace=0.0;
//	if(A->row != A->col)	return	-1;
	for(sqr=0; sqr<A->row; sqr++)	trace += A->el[sqr][sqr];
	return	trace;
}

// �搔�{
Matrix *matMulScl(Matrix *B, double k, const Matrix *A)
{
	int  row, col;
	if(B->row != A->row || B->col != A->col)	matReset(B, A->row, A->col);
	for(row=0; row<A->row; row++)  for(col=0; col<A->col; col++)  B->el[row][col] = k * A->el[row][col];
	return	B;
}

// ����
// 1�s���邢��1��̍s�����
double matInnerProd(const Matrix *A, const Matrix *B)
{
	int  el;
	double  inner_product = 0.0;
	if(A->row == 1 && B->row == 1 && A->col == B->col)
		for(el=0; el<A->col; el++)  inner_product += A->el[0][el] * B->el[0][el];
	else if(A->row == 1 && B->col == 1 && A->col == B->row)
		for(el=0; el<A->col; el++)  inner_product += A->el[0][el] * B->el[el][0];
	else if(A->col == 1 && B->row == 1 && A->row == B->col)
		for(el=0; el<A->row; el++)  inner_product += A->el[el][0] * B->el[0][el];
	else if(A->col == 1 && B->col == 1 && A->row == B->row)
		for(el=0; el<A->row; el++)  inner_product += A->el[el][0] * B->el[el][0];
	else	{ matErr(NULL, MAT_ERROR_UNMATCH_SIZE); return 0; }
	return  inner_product;
}

// 2���`��
double matQuadForm(Matrix *vec1, Matrix *mat, Matrix *vec2)
{
  int  row, col;
  double  val;
  Matrix  vec;
  if(vec1->col != mat->row || vec2->row != mat->col)  return -1;
  matInit(&vec, mat->row,1);
  for(row=0; row<mat->row; row++){
    vec.el[row][0] = 0.0;
    for(col=0; col<mat->col; col++)  vec.el[row][0] += mat->el[row][col] * vec2->el[col][0];
  }
  val = matInnerProd(vec1, &vec);
  matFree(&vec);
  return  val;
}

//////////////////////////////////////////////////////
// Gauss-Jordan�@�ɂ��t�s���@����эs�񎮌v�Z
// �s�񎮂��s�v�̂Ƃ���det��NULL����
// A:nxn	B:nxn
//////////////////////////////////////////////////////
Matrix *matInv(Matrix *B, double *det, const Matrix *A)
{
	int	row, col, _row;
	double	comp;
	double	pivot, _det=1.0;
	Matrix  C, D;

	if(A->row != A->col){ matErr(NULL, MAT_ERROR_UNMATCH_SIZE); if(det != NULL) *det = 0; return NULL; }
	matCopy( matInit(&C, A->row, A->col), A );
	matUnit( matInit(&D, A->row, A->col) );

	// C��P�ʍs��ɕό`
	for(row=0; row<A->row; row++){
		// pivot�̑I��
		for(_row=row; _row<A->row; _row++){
			pivot = C.el[_row][row];
			if(fabs(pivot) < MAT_EPS)
				if(_row==A->row-1){ matErr(NULL, MAT_ERROR_NONEXISTENT); if(det != NULL) *det = 0; return NULL; }		// �t�s�񂪑��݂��Ȃ��Ƃ�
				else	continue;
			// 2�̍s����ւ�
			if(_row != row){
				matReplacRow(&C, &C, row, _row);
				matReplacRow(&D, &D, row, _row);
				_det = -_det;		// �s����ւ��ɂ��s�񎮕����ω�
			}
			break;
		}
		// �s�񎮂̍X�V
		_det *= pivot;
		// C��row�Ԗڂ̍s�ɂ���Ίp������1�ɂ���
		for(col=row; col<A->col; col++)	C.el[row][col] /= pivot;
		for(col=0; col<A->col; col++)	D.el[row][col] /= pivot;
		// C��row�Ԗڂ̗�ɂ��鐬����0�ɂ���
		for(_row=0; _row<A->row; _row++){
			if(_row != row){
				comp = C.el[_row][row];
				for(col=row; col<A->col; col++)	C.el[_row][col] -= comp * C.el[row][col];
				for(col=0; col<A->col; col++)	D.el[_row][col] -= comp * D.el[row][col];
			}
		}
	}
	if(det != NULL)	*det = _det;
	matCopy(B, &D);
	matFree(&C);  matFree(&D);
	return	B;
}
