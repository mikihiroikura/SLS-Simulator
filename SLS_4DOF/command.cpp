#include <stdio.h>
#include <string.h>
#include <time.h>
#include <direct.h>
#include "command.h"
#include "simMain.h"
#include "BMP.h"

// 外部変数
extern SIM	sim;

////////////////////////////////////////////////////////
// 文字取得
////////////////////////////////////////////////////////
int getChar(char *buf, const char *msg)
{
  fprintf(stdout,"%s",msg);
  fgets(buf, BUFSIZE, stdin);
  buf[strlen(buf)-1] = '\0';      // 復改を削除
  return 0;
}

////////////////////////////////////////////////////////
// ヘルプ表示
////////////////////////////////////////////////////////
int showHelp()
{
  char *help = " a: arm operation\n g: animation\n s: data save\n h: help\n q: quit";
  printf("%s\n", help);
  return 0;
}

////////////////////////////////////////////////////////
// タイマー
////////////////////////////////////////////////////////
int setTimer(double delay)
{
  clock_t st;
  double time;
  st = clock();		// 開始時刻の取得
  do{time = (double)(clock()-st)/CLOCKS_PER_SEC;}while(time < delay);
  return 0;
}

////////////////////////////////////////////////////////
// 連番文字列生成
////////////////////////////////////////////////////////
int makeSeqNum(int reset)
{
	// 画像保存用変数
	static char filename[14]="img/000.bmp";	// 「000～999」の連番となるファイル
	// ファイル名を連番にする
	filename[6]++;
	if(filename[6]=='9'+1){
		filename[6]='0';	filename[5]++;
		if(filename[5]=='9'+1){filename[5]='0'; filename[4]++;}
	}
	return	0;
}

////////////////////////////////////////////////////////
// データコピー
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
// ファイル保存
// シミュレーション時系列データ
////////////////////////////////////////////////////////
int saveData()
{
	int	count, jnt, crd, min;
	FILE	*fp;

	min = (sim.step<=DATA_CNT_NUM) ? sim.step : DATA_CNT_NUM;	// sim.stepかDATA_CNT_NUMの小さい値をminに代入
	fp = fopen(sim.data_file_name, "w");
	for (count = 0; count<min; count++){
		fprintf(fp, "%f ", count*SIM_CYCLE_TIME);		// 時間
		for(jnt=0;jnt<ARM_JNT;jnt++)	fprintf(fp, "%f ", sim.save_jnt_pos[count][jnt]);		// 関節位置
		for(jnt=0;jnt<ARM_JNT;jnt++)	fprintf(fp, "%f ", sim.save_jnt_vel[count][jnt]);		// 関節速度
		for(jnt=0;jnt<ARM_JNT;jnt++)	fprintf(fp, "%f ", sim.save_jnt_force[count][jnt]);		// 関節力
		for(crd=0;crd<DIM3;crd++)	fprintf(fp, "%f ", sim.save_eff_pos[count][crd]);		// 手先位置
		for(crd=0;crd<DIM3;crd++)	fprintf(fp, "%f ", sim.save_eff_vel[count][crd]);		// 手先位置
		for(crd=0;crd<DIM3;crd++)	fprintf(fp, "%f ", sim.save_eff_force[count][crd]);		// 手先外力
		for(crd=0;crd<DIM3;crd++)	fprintf(fp, "%f ", sim.save_obj_pos[count][crd]);		// 対象位置
		for(crd=0;crd<DIM3;crd++)	fprintf(fp, "%f ", sim.save_obj_vel[count][crd]);		// 対象速度
		fprintf(fp, "%d ", sim.save_state_contact[count]);		// 接触状態
		fprintf(fp, "%f ", sim.save_dist[count]);		// 距離
		fprintf(fp, "\n");
	}
	fclose(fp);
	fp = fopen("arm_node.txt", "w");
	for (count = 0; count<min; count++){
		fprintf(fp, "%f ", count*SIM_CYCLE_TIME);		// 時間
		for(crd=0;crd<DIM3;crd++)	fprintf(fp, "%f ", sim.save_node_pos[count][crd]);		// 結節点位置
		for(crd=0;crd<DIM3;crd++)	fprintf(fp, "%f ", sim.save_node_vel[count][crd]);		// 結節点速度
		fprintf(fp, "\n");
	}
	fclose(fp);
	fp = fopen("displacement.txt", "w");
	for (count = 0; count<min; count++){
		fprintf(fp, "%f ", count*SIM_CYCLE_TIME);		// 時間
		for(crd=0;crd<DIM2;crd++)	fprintf(fp, "%f ", (sim.save_eff_pos[count][crd]-sim.save_eff_pos[0][crd])-(sim.save_node_pos[count][crd]-sim.save_node_pos[0][crd]));		// バネ変位
		for(crd=0;crd<DIM2;crd++)	fprintf(fp, "%f ", (sim.save_node_pos[count][crd]-sim.save_node_pos[0][crd]));		// ダンパ変位
		for(crd=0;crd<DIM2;crd++)	fprintf(fp, "%f ", sim.save_eff_vel[count][crd]-sim.save_node_vel[count][crd]);		// バネ速度
		for(crd=0;crd<DIM2;crd++)	fprintf(fp, "%f ", sim.save_node_vel[count][crd]);		// ダンパ速度
		fprintf(fp, "\n");
	}
	fclose(fp);
	return 0;
}

////////////////////////////////////////////////////////
// ファイル保存
// シミュレーション情報
////////////////////////////////////////////////////////
int saveInfo()
{
	int		crd;
	FILE	*fp;
	fp = fopen(sim.filename_info, "w");
//	fprintf(fp, "m_a = %f\n", sim.imp.m_a);		// 時間
//	fprintf(fp, "c_a = %f\n", sim.imp.c_a);		// 関節位置
//	fprintf(fp, "k_a = %f\n", sim.imp.k_a);		// 関節速度
	for(crd=0;crd<DIM2;crd++)	fprintf(fp, "V[%d] = %f ", crd, sim.dyn.V[crd]);	fprintf(fp, "\n");		// 粘性
	for(crd=0;crd<DIM2;crd++)	fprintf(fp, "M[%d] = %f ", crd, sim.imp.M.el[crd][crd]);	fprintf(fp, "\n");		// 慣性
	for(crd=0;crd<DIM2;crd++)	fprintf(fp, "C[%d] = %f ", crd, sim.imp.C.el[crd][crd]);	fprintf(fp, "\n");		// 粘性
	for(crd=0;crd<DIM2;crd++)	fprintf(fp, "K[%d] = %f ", crd, sim.imp.K.el[crd][crd]);	fprintf(fp, "\n");		// 弾性
	for(crd=0;crd<DIM2;crd++)	fprintf(fp, "K0[%d] = %f ", crd, sim.imp.K0.el[crd][crd]);	fprintf(fp, "\n");		// 弾性
	for(crd=0;crd<DIM3;crd++)	fprintf(fp, "T[%d] = %f ", crd, sim.imp.T[crd]);	fprintf(fp, "\n");		// 振動周期
	fclose(fp);
	return 0;
}

////////////////////////////////////////////////////////
// 画像保存
// ファイル名を連番にする
////////////////////////////////////////////////////////
int saveImage(int width, int height)
{
	// 画像保存用変数
	static char filename[14]="img/000.bmp";	// 「000～999」の連番となるファイル
//	int bmp_flag = 1;			// BMPファイル出力フラグ
//	double bmp_time=0.1;		// BMPファイル出力の時間間隔
	_mkdir("img");		// imgフォルダ作成
	writeBMP(filename, width, height);	// BMPファイルを出力
	// ファイル名を連番にする
	filename[6]++;
	if(filename[6]=='9'+1){
		filename[6]='0';	filename[5]++;
		if(filename[5]=='9'+1){filename[5]='0'; filename[4]++;}
	}
	return	0;
}

////////////////////////////////////////////////////////
// グラフ表示（gnuplot）
////////////////////////////////////////////////////////
#if 0
int drawData()
{
	FILE *gp;
	if((gp=_popen(GNUPLOT_PATH,"w"))==NULL){fprintf(stderr,"Can not find %s!",GNUPLOT_PATH);return -1;}
	// gnuplotにコマンドを送る
	fprintf(gp, "pl \"%s\" us 1:2 w l, \"%s\" us 1:3 w l\n", DATA_FILE_NAME, DATA_FILE_NAME);
	// グラフ保存
	fprintf(gp, "set terminal png\n set out \"img_pos_vel.png\"\n rep\n");		// png出力
	// gnuplotにコマンドを送る
	fprintf(gp, "pl \"%s\" us 1:4 w l\n", DATA_FILE_NAME);
	// グラフ保存
	fprintf(gp, "set terminal png\n set out \"img_force.png\"\n rep\n");		// png出力
//	fprintf(gp, "rep \"%s\" us 1:12 w l\n", DATA_FILE_NAME);
//	fprintf(gp, "rep \"%s\" us 1:21 w l\n", DATA_FILE_NAME);
//	fprintf(gp, "rep \"%s\" us 1:29 w l\n", DATA_FILE_NAME);
//	fprintf(gp, "rep \"%s\" us 1:22 w l, \"%s\" us 1:32 w l\n", DATA_FILE_NAME, DATA_FILE_NAME);
//	fprintf(gp, "cd \"%s\" \n", "data");	// データのあるディレクトリへ移動
//	fprintf(gp, "load \"%s\" \n", "jnt_pos.gp");	// gnuplotスクリプト呼び出し
//	fprintf(gp, "load \"%s\" \n", "jnt_trq.gp");	// gnuplotスクリプト呼び出し
//	fprintf(gp, "rep \"%s\" us 1:22 w l, \"%s\" us 1:32 w l\n", "biped_trq.dat", "biped_trq2.dat");
	// グラフ保存
//	fprintf(gp, "set terminal png\n set out \"img.png\"\n rep\n");		// png出力
//	fprintf(gp, "set terminal postscript eps\n set out \"img.eps\"\n rep\n");		// eps出力
	fflush(gp); // バッファに格納されているデータを吐き出す（必須）
	getchar(); // 入力待ち
	_pclose(gp);
	return 0;
}
#endif

////////////////////////////////////////////////////////
// グラフ保存（gnuplot）
////////////////////////////////////////////////////////
int saveGraph()
{
	FILE *gp;
	if((gp=_popen(GNUPLOT_PATH,"w"))==NULL){fprintf(stderr,"Can not find %s!",GNUPLOT_PATH);return -1;}
	fprintf(gp, "pl \"%s\" us 1:2 w l, \"%s\" us 1:3 w l, \"%s\" us 1:4 w l, \"%s\" us 1:5 w l\n", sim.data_file_name, sim.data_file_name, sim.data_file_name, sim.data_file_name);	// 関節位置のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH1);		// png出力
	fprintf(gp, "pl \"%s\" us 1:6 w l, \"%s\" us 1:7 w l, \"%s\" us 1:8 w l, \"%s\" us 1:9 w l\n", sim.data_file_name, sim.data_file_name, sim.data_file_name, sim.data_file_name);	// 関節速度のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH2);		// png出力
	fprintf(gp, "pl \"%s\" us 1:10 w l, \"%s\" us 1:11 w l, \"%s\" us 1:12 w l, \"%s\" us 1:13 w l\n", sim.data_file_name, sim.data_file_name, sim.data_file_name, sim.data_file_name);		// 関節力のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH3);		// png出力
	fprintf(gp, "pl \"%s\" us 1:14 w l, \"%s\" us 1:15 w l\n", sim.data_file_name, sim.data_file_name);		// 手先位置のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_eff_pos.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:17 w l, \"%s\" us 1:18 w l\n", sim.data_file_name, sim.data_file_name);		// 手先速度のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_eff_vel.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:20 w l, \"%s\" us 1:21 w l\n", sim.data_file_name, sim.data_file_name);		// 手先外力のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH4);		// png出力
	fprintf(gp, "pl \"%s\" us 1:23 w l, \"%s\" us 1:24 w l\n", sim.data_file_name, sim.data_file_name);		// 対象位置のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_obj_pos.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:26 w l, \"%s\" us 1:27 w l\n", sim.data_file_name, sim.data_file_name);		// 対象速度のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_obj_vel.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:29 w l\n", sim.data_file_name);		// 接触状態のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_state_contact.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:30 w l\n", sim.data_file_name);		// 距離のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_dist.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:2 w l, \"%s\" us 1:3 w l\n", "arm_node.txt", "arm_node.txt");		// 結節点位置のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_node_pos.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:5 w l, \"%s\" us 1:6 w l\n", "arm_node.txt", "arm_node.txt");		// 結節点速度のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_node_vel.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:2 w l, \"%s\" us 1:3 w l\n", "displacement.txt", "displacement.txt");		// バネ変位のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_spring_pos.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:4 w l, \"%s\" us 1:5 w l\n", "displacement.txt", "displacement.txt");		// ダンパ変位のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_damper_pos.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:6 w l, \"%s\" us 1:7 w l\n", "displacement.txt", "displacement.txt");		// バネ変位速度のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_spring_vel.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:8 w l, \"%s\" us 1:9 w l\n", "displacement.txt", "displacement.txt");		// ダンパ変位速度のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_damper_vel.png");		// png出力
	fprintf(gp, "pl \"%s\" us 1:($2+$4) w l, \"%s\" us 1:($3+$5) w l\n", "displacement.txt", "displacement.txt");		// 手先変位のグラフ
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", "img_dev_eff.png");		// png出力
	fflush(gp); // バッファに格納されているデータを吐き出す（必須）
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
		sprintf(filename, FILENAME_DATA, incount);		// ファイル名を連番に設定
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:2 w l\n", filename);	// 位置のグラフ
		else	fprintf(gp, "rep \"%s\" us 1:2 w l\n", filename);	// 位置のグラフ
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH1);		// png出力
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// ファイル名を連番に設定
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:3 w l\n", filename);	// 速度のグラフ
		else	fprintf(gp, "rep \"%s\" us 1:3 w l\n", filename);	// 速度のグラフ
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH2);		// png出力
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// ファイル名を連番に設定
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:4 w l\n", filename);		// 関節力のグラフ
		else	fprintf(gp, "rep \"%s\" us 1:4 w l\n", filename);		// 関節力のグラフ
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH3);		// png出力
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// ファイル名を連番に設定
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:5 w l\n", filename);		// 外力のグラフ
		else	fprintf(gp, "rep \"%s\" us 1:5 w l\n", filename);		// 外力のグラフ
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH4);		// png出力
	for(incount=0;incount<trial_num;incount++){
		sprintf(filename, FILENAME_DATA, incount);		// ファイル名を連番に設定
		if(incount == 0)	fprintf(gp, "pl \"%s\" us 1:($2-$6-0.75/2-0.0001-0.15) w l\n", filename);		// 手先と物体の距離のグラフ
		else	fprintf(gp, "rep \"%s\" us 1:($2-$6-0.75/2-0.0001-0.15) w l\n", filename);		// 手先と物体の距離のグラフ
	}
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH5);		// png出力
	fflush(gp); // バッファに格納されているデータを吐き出す（必須）
	_pclose(gp);
	return 0;
}
