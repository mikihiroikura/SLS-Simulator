#include <stdio.h>
#include <string.h>
#include <time.h>
#include <direct.h>
#include "command.h"
#include "simMain.h"
#include "BMP.h"

// �O���ϐ�
extern SIM	sim;

////////////////////////////////////////////////////////
// �����擾
////////////////////////////////////////////////////////
int getChar(char *buf, const char *msg)
{
  fprintf(stdout,"%s",msg);
  fgets(buf, BUFSIZE, stdin);
  buf[strlen(buf)-1] = '\0';      // �������폜
  return 0;
}

////////////////////////////////////////////////////////
// �w���v�\��
////////////////////////////////////////////////////////
int showHelp()
{
  char *help = " a: arm operation\n g: animation\n s: data save\n h: help\n q: quit";
  printf("%s\n", help);
  return 0;
}

////////////////////////////////////////////////////////
// �^�C�}�[
////////////////////////////////////////////////////////
int setTimer(double delay)
{
  clock_t st;
  double time;
  st = clock();		// �J�n�����̎擾
  do{time = (double)(clock()-st)/CLOCKS_PER_SEC;}while(time < delay);
  return 0;
}

////////////////////////////////////////////////////////
// �A�ԕ����񐶐�
////////////////////////////////////////////////////////
int makeSeqNum(int reset)
{
	// �摜�ۑ��p�ϐ�
	static char filename[14]="img/000.bmp";	// �u000�`999�v�̘A�ԂƂȂ�t�@�C��
	// �t�@�C������A�Ԃɂ���
	filename[6]++;
	if(filename[6]=='9'+1){
		filename[6]='0';	filename[5]++;
		if(filename[5]=='9'+1){filename[5]='0'; filename[4]++;}
	}
	return	0;
}

////////////////////////////////////////////////////////
// �f�[�^�R�s�[
////////////////////////////////////////////////////////
int copyData()
{
	int	jnt, crd;
	if(sim.step < DATA_CNT_NUM){
		sim.save_state_contact[sim.step] = sim.state_contact;
		sim.save_dist[sim.step] = sim.dist;
		for(jnt=0;jnt<ARM_JNT;jnt++){
			sim.save_ref_jnt_pos[sim.step][jnt] = sim.ref_jnt_pos[jnt];
			sim.save_jnt_pos[sim.step][jnt] = sim.jnt_pos[jnt];
			sim.save_jnt_vel[sim.step][jnt] = sim.jnt_vel[jnt];
			sim.save_jnt_force[sim.step][jnt] = sim.jnt_force[jnt];
		}
		for(crd=0;crd<DIM3;crd++){
			sim.save_eff_pos[sim.step][crd] = sim.eff_pos[crd];
			sim.save_eff_vel[sim.step][crd] = sim.eff_vel[crd];
			sim.save_eff_force[sim.step][crd] = sim.eff_force[crd];
			sim.save_node_pos[sim.step][crd] = sim.node_pos[crd];
			sim.save_node_vel[sim.step][crd] = sim.node_vel[crd];
			sim.save_obj_pos[sim.step][crd] = sim.obj_pos[crd];
			sim.save_obj_vel[sim.step][crd] = sim.obj_vel[crd];
		}
	}
	return 0;
}

////////////////////////////////////////////////////////
// �t�@�C���ۑ�
// �V�~�����[�V�������n��f�[�^
////////////////////////////////////////////////////////
int saveData()
{
	int	count, jnt, crd, min;
	FILE	*fp;

	min = (sim.step<=DATA_CNT_NUM) ? sim.step : DATA_CNT_NUM;	// sim.step��DATA_CNT_NUM�̏������l��min�ɑ��
	fp = fopen(sim.data_file_name, "w");
	for (count = 0; count<min; count++){
		fprintf(fp, "%f ", count*SIM_CYCLE_TIME);		// ����
		for(jnt=0;jnt<ARM_JNT;jnt++)	fprintf(fp, "%f ", sim.save_jnt_pos[count][jnt]);		// �֐߈ʒu
		for(jnt=0;jnt<ARM_JNT;jnt++)	fprintf(fp, "%f ", sim.save_jnt_vel[count][jnt]);		// �֐ߑ��x
		for(jnt=0;jnt<ARM_JNT;jnt++)	fprintf(fp, "%f ", sim.save_jnt_force[count][jnt]);		// �֐ߗ�
		for(crd=0;crd<DIM3;crd++)	fprintf(fp, "%f ", sim.save_eff_pos[count][crd]);		// ���ʒu
		for(crd=0;crd<DIM3;crd++)	fprintf(fp, "%f ", sim.save_eff_vel[count][crd]);		// ���ʒu
		for(crd=0;crd<DIM3;crd++)	fprintf(fp, "%f ", sim.save_eff_force[count][crd]);		// ���O��
		for(crd=0;crd<DIM3;crd++)	fprintf(fp, "%f ", sim.save_obj_pos[count][crd]);		// �Ώۈʒu
		for(crd=0;crd<DIM3;crd++)	fprintf(fp, "%f ", sim.save_obj_vel[count][crd]);		// �Ώۑ��x
		fprintf(fp, "%d ", sim.save_state_contact[count]);		// �ڐG���
		fprintf(fp, "%f ", sim.save_dist[count]);		// ����
		fprintf(fp, "\n");
	}
	fclose(fp);
	fp = fopen("arm_node.txt", "w");
	for (count = 0; count<min; count++){
		fprintf(fp, "%f ", count*SIM_CYCLE_TIME);		// ����
		for(crd=0;crd<DIM3;crd++)	fprintf(fp, "%f ", sim.save_node_pos[count][crd]);		// ���ߓ_�ʒu
		for(crd=0;crd<DIM3;crd++)	fprintf(fp, "%f ", sim.save_node_vel[count][crd]);		// ���ߓ_���x
		fprintf(fp, "\n");
	}
	fclose(fp);
	fp = fopen("displacement.txt", "w");
	for (count = 0; count<min; count++){
		fprintf(fp, "%f ", count*SIM_CYCLE_TIME);		// ����
		for(crd=0;crd<DIM2;crd++)	fprintf(fp, "%f ", (sim.save_eff_pos[count][crd]-sim.save_eff_pos[0][crd])-(sim.save_node_pos[count][crd]-sim.save_node_pos[0][crd]));		// �o�l�ψ�
		for(crd=0;crd<DIM2;crd++)	fprintf(fp, "%f ", (sim.save_node_pos[count][crd]-sim.save_node_pos[0][crd]));		// �_���p�ψ�
		for(crd=0;crd<DIM2;crd++)	fprintf(fp, "%f ", sim.save_eff_vel[count][crd]-sim.save_node_vel[count][crd]);		// �o�l���x
		for(crd=0;crd<DIM2;crd++)	fprintf(fp, "%f ", sim.save_node_vel[count][crd]);		// �_���p���x
		fprintf(fp, "\n");
	}
	fclose(fp);
	return 0;
}

////////////////////////////////////////////////////////
// �t�@�C���ۑ�
// �V�~�����[�V�������
////////////////////////////////////////////////////////
int saveInfo()
{
	int		crd;
	FILE	*fp;
	fp = fopen(sim.filename_info, "w");
//	fprintf(fp, "m_a = %f\n", sim.imp.m_a);		// ����
//	fprintf(fp, "c_a = %f\n", sim.imp.c_a);		// �֐߈ʒu
//	fprintf(fp, "k_a = %f\n", sim.imp.k_a);		// �֐ߑ��x
	for(crd=0;crd<DIM2;crd++)	fprintf(fp, "V[%d] = %f ", crd, sim.dyn.V[crd]);	fprintf(fp, "\n");		// �S��
	for(crd=0;crd<DIM2;crd++)	fprintf(fp, "M[%d] = %f ", crd, sim.imp.M.el[crd][crd]);	fprintf(fp, "\n");		// ����
	for(crd=0;crd<DIM2;crd++)	fprintf(fp, "C[%d] = %f ", crd, sim.imp.C.el[crd][crd]);	fprintf(fp, "\n");		// �S��
	for(crd=0;crd<DIM2;crd++)	fprintf(fp, "K[%d] = %f ", crd, sim.imp.K.el[crd][crd]);	fprintf(fp, "\n");		// �e��
	for(crd=0;crd<DIM2;crd++)	fprintf(fp, "K0[%d] = %f ", crd, sim.imp.K0.el[crd][crd]);	fprintf(fp, "\n");		// �e��
	for(crd=0;crd<DIM3;crd++)	fprintf(fp, "T[%d] = %f ", crd, sim.imp.T[crd]);	fprintf(fp, "\n");		// �U������
	fclose(fp);
	return 0;
}

////////////////////////////////////////////////////////
// �摜�ۑ�
// �t�@�C������A�Ԃɂ���
////////////////////////////////////////////////////////
int saveImage(int width, int height)
{
	// �摜�ۑ��p�ϐ�
	static char filename[14]="img/000.bmp";	// �u000�`999�v�̘A�ԂƂȂ�t�@�C��
//	int bmp_flag = 1;			// BMP�t�@�C���o�̓t���O
//	double bmp_time=0.1;		// BMP�t�@�C���o�͂̎��ԊԊu
	_mkdir("img");		// img�t�H���_�쐬
	writeBMP(filename, width, height);	// BMP�t�@�C�����o��
	// �t�@�C������A�Ԃɂ���
	filename[6]++;
	if(filename[6]=='9'+1){
		filename[6]='0';	filename[5]++;
		if(filename[5]=='9'+1){filename[5]='0'; filename[4]++;}
	}
	return	0;
}

////////////////////////////////////////////////////////
// �O���t�\���ignuplot�j
////////////////////////////////////////////////////////
#if 0
int drawData()
{
	FILE *gp;
	if((gp=_popen(GNUPLOT_PATH,"w"))==NULL){fprintf(stderr,"Can not find %s!",GNUPLOT_PATH);return -1;}
	// gnuplot�ɃR�}���h�𑗂�
	fprintf(gp, "pl \"%s\" us 1:2 w l, \"%s\" us 1:3 w l\n", DATA_FILE_NAME, DATA_FILE_NAME);
	// �O���t�ۑ�
	fprintf(gp, "set terminal png\n set out \"img_pos_vel.png\"\n rep\n");		// png�o��
	// gnuplot�ɃR�}���h�𑗂�
	fprintf(gp, "pl \"%s\" us 1:4 w l\n", DATA_FILE_NAME);
	// �O���t�ۑ�
	fprintf(gp, "set terminal png\n set out \"img_force.png\"\n rep\n");		// png�o��
//	fprintf(gp, "rep \"%s\" us 1:12 w l\n", DATA_FILE_NAME);
//	fprintf(gp, "rep \"%s\" us 1:21 w l\n", DATA_FILE_NAME);
//	fprintf(gp, "rep \"%s\" us 1:29 w l\n", DATA_FILE_NAME);
//	fprintf(gp, "rep \"%s\" us 1:22 w l, \"%s\" us 1:32 w l\n", DATA_FILE_NAME, DATA_FILE_NAME);
//	fprintf(gp, "cd \"%s\" \n", "data");	// �f�[�^�̂���f�B���N�g���ֈړ�
//	fprintf(gp, "load \"%s\" \n", "jnt_pos.gp");	// gnuplot�X�N���v�g�Ăяo��
//	fprintf(gp, "load \"%s\" \n", "jnt_trq.gp");	// gnuplot�X�N���v�g�Ăяo��
//	fprintf(gp, "rep \"%s\" us 1:22 w l, \"%s\" us 1:32 w l\n", "biped_trq.dat", "biped_trq2.dat");
	// �O���t�ۑ�
//	fprintf(gp, "set terminal png\n set out \"img.png\"\n rep\n");		// png�o��
//	fprintf(gp, "set terminal postscript eps\n set out \"img.eps\"\n rep\n");		// eps�o��
	fflush(gp); // �o�b�t�@�Ɋi�[����Ă���f�[�^��f���o���i�K�{�j
	getchar(); // ���͑҂�
	_pclose(gp);
	return 0;
}
#endif

////////////////////////////////////////////////////////
// �O���t�ۑ��ignuplot�j
////////////////////////////////////////////////////////
int saveGraph()
{
	FILE *gp;
	if((gp=_popen(GNUPLOT_PATH,"w"))==NULL){fprintf(stderr,"Can not find %s!",GNUPLOT_PATH);return -1;}
	fprintf(gp, "pl \"%s\" us 1:2 w l, \"%s\" us 1:3 w l, \"%s\" us 1:4 w l, \"%s\" us 1:5 w l\n", sim.data_file_name, sim.data_file_name, sim.data_file_name, sim.data_file_name);	// �֐߈ʒu�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH1);		// png�o��
	fprintf(gp, "pl \"%s\" us 1:6 w l, \"%s\" us 1:7 w l, \"%s\" us 1:8 w l, \"%s\" us 1:9 w l\n", sim.data_file_name, sim.data_file_name, sim.data_file_name, sim.data_file_name);	// �֐ߑ��x�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH2);		// png�o��
	fprintf(gp, "pl \"%s\" us 1:10 w l, \"%s\" us 1:11 w l, \"%s\" us 1:12 w l, \"%s\" us 1:13 w l\n", sim.data_file_name, sim.data_file_name, sim.data_file_name, sim.data_file_name);		// �֐ߗ͂̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH3);		// png�o��
	fprintf(gp, "pl \"%s\" us 1:14 w l, \"%s\" us 1:15 w l\n", sim.data_file_name, sim.data_file_name);		// ���ʒu�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_eff_pos.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:17 w l, \"%s\" us 1:18 w l\n", sim.data_file_name, sim.data_file_name);		// ��摬�x�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_eff_vel.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:20 w l, \"%s\" us 1:21 w l\n", sim.data_file_name, sim.data_file_name);		// ���O�͂̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH4);		// png�o��
	fprintf(gp, "pl \"%s\" us 1:23 w l, \"%s\" us 1:24 w l\n", sim.data_file_name, sim.data_file_name);		// �Ώۈʒu�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_obj_pos.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:26 w l, \"%s\" us 1:27 w l\n", sim.data_file_name, sim.data_file_name);		// �Ώۑ��x�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_obj_vel.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:29 w l\n", sim.data_file_name);		// �ڐG��Ԃ̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_state_contact.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:30 w l\n", sim.data_file_name);		// �����̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_dist.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:2 w l, \"%s\" us 1:3 w l\n", "arm_node.txt", "arm_node.txt");		// ���ߓ_�ʒu�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_node_pos.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:5 w l, \"%s\" us 1:6 w l\n", "arm_node.txt", "arm_node.txt");		// ���ߓ_���x�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_node_vel.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:2 w l, \"%s\" us 1:3 w l\n", "displacement.txt", "displacement.txt");		// �o�l�ψʂ̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_spring_pos.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:4 w l, \"%s\" us 1:5 w l\n", "displacement.txt", "displacement.txt");		// �_���p�ψʂ̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_damper_pos.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:6 w l, \"%s\" us 1:7 w l\n", "displacement.txt", "displacement.txt");		// �o�l�ψʑ��x�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_spring_vel.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:8 w l, \"%s\" us 1:9 w l\n", "displacement.txt", "displacement.txt");		// �_���p�ψʑ��x�̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_damper_vel.png");		// png�o��
	fprintf(gp, "pl \"%s\" us 1:($2+$4) w l, \"%s\" us 1:($3+$5) w l\n", "displacement.txt", "displacement.txt");		// ���ψʂ̃O���t
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_dev_eff.png");		// png�o��
	fflush(gp); // �o�b�t�@�Ɋi�[����Ă���f�[�^��f���o���i�K�{�j
	_pclose(gp);
	return 0;
}

int saveGraph2(int trial_num)
{
	FILE *gp;
	int	incount;
	char filename[256];

	if((gp=_popen(GNUPLOT_PATH,"w"))==NULL){fprintf(stderr,"Can not find %s!",GNUPLOT_PATH);return -1;}
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// �t�@�C������A�Ԃɐݒ�
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:2 w l\n", filename);	// �ʒu�̃O���t
		else	fprintf(gp, "rep \"%s\" us 1:2 w l\n", filename);	// �ʒu�̃O���t
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH1);		// png�o��
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// �t�@�C������A�Ԃɐݒ�
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:3 w l\n", filename);	// ���x�̃O���t
		else	fprintf(gp, "rep \"%s\" us 1:3 w l\n", filename);	// ���x�̃O���t
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH2);		// png�o��
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// �t�@�C������A�Ԃɐݒ�
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:4 w l\n", filename);		// �֐ߗ͂̃O���t
		else	fprintf(gp, "rep \"%s\" us 1:4 w l\n", filename);		// �֐ߗ͂̃O���t
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH3);		// png�o��
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// �t�@�C������A�Ԃɐݒ�
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:5 w l\n", filename);		// �O�͂̃O���t
		else	fprintf(gp, "rep \"%s\" us 1:5 w l\n", filename);		// �O�͂̃O���t
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH4);		// png�o��
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// �t�@�C������A�Ԃɐݒ�
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:($2-$6-0.75/2-0.0001-0.15) w l\n", filename);		// ���ƕ��̂̋����̃O���t
		else	fprintf(gp, "rep \"%s\" us 1:($2-$6-0.75/2-0.0001-0.15) w l\n", filename);		// ���ƕ��̂̋����̃O���t
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH5);		// png�o��
	fflush(gp); // �o�b�t�@�Ɋi�[����Ă���f�[�^��f���o���i�K�{�j
	_pclose(gp);
	return 0;
}
