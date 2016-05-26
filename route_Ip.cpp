#define _CRT_SECURE_NO_WARNINGS
#include "router.h"
#include "lib_record.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stack>
#include <vector>
#include "glpk.h"
int isRound();
void saveResult();
using namespace std;
#define MAXVSIZE 610            /* 最大顶点个数 */
#define MAXSIDES 3000
#define MAXPSIZE 85 			/* 必经节点个数 */
#define WEIGHT(k) m[u][k].length-20*isvia[k]  /* 获得权重 */
#define INT_MAX 2000
//struct timeval start, end;
int total_edge = 0;
int avg_weight = 1;
int getted[MAXVSIZE][MAXVSIZE];   /* 标记对应边是否被搜索到 */
int stacked[MAXVSIZE];  		  /* 标记是否入栈 */
int isvia[MAXVSIZE] = { 0 }; 		  /* 标记必经节点 */
int path[MAXVSIZE] = { 0 };		  /* 保存路径 */
int historylen[MAXVSIZE][MAXPSIZE] = { 0 };      /* 保存记忆中的最优路径情况 */
												 //int historyvia[MAXVSIZE] = {0};      /* 保存记忆中的必经节点个数 */
int largest = 0;                  /* 保存的最大点数 */
int edge_num = 0;				  /* 路径边个数 */
int path_len = 0;				  /* 路径权值长度 */
int via_num = 0;    				  /* 此路径包含的必经节点个数 */
int v_num = 0;					  /* 此路径包含的点的总个数 */
int min_len = INT_MAX;            /* 最小权值 */
int finalflag = 0;
bool simple = false;
float keyvalue = 0.24;
stack<int> mystack, printstack;
int designatedP[MAXPSIZE];      /* 记录必经的节点*/
int designatedN;                /* 必经节点的个数*/
int source = 0, dest = 0;            /* 源节点和目的节点 */
									 //int child_num[MAXVSIZE] = {0};        /* 节点的后续节点个数 */
									 //struct node {
									 //	int node_l;                 /* 具有相同起点的上一个节点，0意味无 */
									 //	int node_r;                 /* 具有相同起点的下一个节点，0意味无 */
									 //	int length;                 /* 边的长度*/
									 //	int side;                   /* 边的编号*/
									 //}m[MAXVSIZE][MAXVSIZE];         /* m数组模拟邻接多重链表，利用第一行和第一列作为表头 */

struct node
{
	vector<int> preSides;
	vector<int> postSides;
} nodes[MAXVSIZE];

struct side
{
	int from;
	int to;
	//int id;
	int weight;
} sides[MAXSIDES];

struct matrixNode
{
	int id;
	int weight;
} matrixNode[MAXVSIZE][MAXVSIZE];

//线性规划求解变量
#define MAXROWNUM 10000
//#define SIDENUM 3000
bool cointainSides[MAXSIDES] = { false };    //线性规划求出来的边的集合
int row_count = 0; //行数的计数
int row_variable_count = 0; //变量系数的计数
int ia[MAXROWNUM], ja[MAXROWNUM];
double ar[MAXROWNUM];
int eageNum;
vector<int> result;   //存放线性规划求出的边的情况
vector< vector<int> > tours; //存放环
vector<int> side_result; //存放结果
void add_edge_into_list(int u, int v, int length, int side) /* 把边(u,v)添加到多重链表中 */
{
	//child_num[u]++;
	/*if (m[u][v].length>0)
	{
	if (length<m[u][v].length)  m[u][v].length = length;
	return;
	}
	m[u][v].length = length;
	m[u][v].side = side;
	m[u][v].node_l = m[u][0].node_l;
	m[u][m[u][0].node_l].node_r = v;
	m[u][v].node_r = 0;
	m[u][0].node_l = v;*/
	/*
	int k;
	int weight = length-20*isvia[v];
	k = (m[u][0]).node_r;
	while(k!=0&&WEIGHT(k)<weight){
	k = (m[u][k]).node_r;
	}
	m[u][v].node_l=m[u][k].node_l;
	m[u][m[u][k].node_l].node_r=v;
	m[u][k].node_l=v;
	m[u][v].node_r=k;
	*/

	nodes[u].postSides.push_back(side);
	nodes[v].preSides.push_back(side);
	sides[side].from = u;
	sides[side].to = v;
	sides[side].weight = length;
	matrixNode[u][v].id = side;
	matrixNode[u][v].weight = length;
}
void print_result()
{
	int i;
	//printf("最优路径是:"); 
	for (i = 0; i<side_result.size(); i++)
	{
		//printf("%d|",path[i]);
		record_result(side_result[i]);
	}
	//printf("权值为:%d\n",min_len);
}

//////////////////////////////////////
/////////线性规划求解/////////////////
/////////////////////////////////////
void lpSolver()
{
	int i;
	bool isLoop = true;
	int add_constrants = 0;
	int row = 2 * (largest + 1);
	while (isLoop)
	{
		row_count = 0;
		row_variable_count = 0;
		row += add_constrants;    //总的约束条件
		glp_prob *lp = glp_create_prob();
		glp_add_rows(lp, row); //添加的条件个数
		glp_add_cols(lp, eageNum);  //添加的变量个数
		glp_set_obj_dir(lp, GLP_MIN);      //取最小值


		for (i = 0; i < eageNum; i++)      //给线性规划添加条件变量范围
		{
			int tempSide = i + 1;
			glp_set_col_bnds(lp, tempSide, GLP_DB, 0, 1); //设置变量为0到1之间
			glp_set_obj_coef(lp, tempSide, sides[i].weight);
			glp_set_col_kind(lp, tempSide, GLP_IV);           //设置变量为正整数


		}

		///添加的不让成环的约束
		for (i = 0; i < tours.size(); i++)
		{
			vector<int> tempVec = tours[i];
			//for()
			double size = tempVec.size();
			glp_set_row_bnds(lp, ++row_count, GLP_DB, 0, size - 1);
			for (int j = 0; j < tempVec.size(); j++)
			{

				ia[++row_variable_count] = row_count;
				ja[row_variable_count] = tempVec[j] + 1;
				ar[row_variable_count] = 1;
			}
		}

		for (i = 0; i <= largest; i++)
		{
			if (i == source)    //当i是起始点时
			{
				glp_set_row_bnds(lp, ++row_count, GLP_FX, 1, 1);
				//glp_set_col_bnds(lp, i, GLP_DB, 0.0, 1.0);
				for (int j = 0; j < nodes[i].postSides.size(); j++)       //当为起点时，总的出度为1
				{
					//int current_side = nodes[i].postSides[j];
					//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //设置该变量的范围
					//glp_set_col_kind(lp, current_side, GLP_IV);       //设置该变量为正整数
					ia[++row_variable_count] = row_count;
					ja[row_variable_count] = nodes[i].postSides[j] + 1;
					ar[row_variable_count] = 1;
				}

				if (nodes[i].preSides.size() > 0)
				{
					glp_set_row_bnds(lp, ++row_count, GLP_FX, 0, 0);
					for (int j = 0; j < nodes[i].preSides.size(); j++)       //当为起点时，总的入度为0
					{
						//int current_side = nodes[i].preSides[j];
						//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //设置该变量的范围
						//glp_set_col_kind(lp, current_side, GLP_IV);       //设置该变量为正整数
						ia[++row_variable_count] = row_count;
						ja[row_variable_count] = nodes[i].preSides[j] + 1;
						ar[row_variable_count] = 1;
					}
				}

				continue;
			}
			if (i == dest)      //当i是目标点时，总的入度为1
			{
				glp_set_row_bnds(lp, ++row_count, GLP_FX, 1, 1);
				for (int j = 0; j < nodes[i].preSides.size(); j++)       //当为终点时，总的入度为1
				{
					//int current_side = nodes[i].preSides[j];
					//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //设置该变量的范围
					//glp_set_col_kind(lp, current_side, GLP_IV);       //设置该变量为正整数
					ia[++row_variable_count] = row_count;
					ja[row_variable_count] = nodes[i].preSides[j] + 1;
					ar[row_variable_count] = 1.0;
				}

				if (nodes[i].postSides.size() > 0)
				{
					//glp_set_row_bnds(lp, ++row_count, GLP_FX, 0.0, 0.0);
					//glp_set_col_bnds(lp, i, GLP_DB, 0.0, 1.0);
					for (int j = 0; j < nodes[i].postSides.size(); j++)       //当为终点时，总的出度为0
					{
						//int current_side = nodes[i].postSides[j];
						//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //设置该变量的范围
						//glp_set_col_kind(lp, current_side, GLP_IV);       //设置该变量为正整数
						ia[++row_variable_count] = row_count;
						ja[row_variable_count] = nodes[i].postSides[j] + 1;
						ar[row_variable_count] = 1;
					}
				}
				continue;
			}
			if (isvia[i])       //当i是必经节点时,入度和出度都为1
			{
				glp_set_row_bnds(lp, ++row_count, GLP_FX, 1, 1);
				for (int j = 0; j < nodes[i].postSides.size(); j++)       //当为必经节点时，总的出度为1
				{
					//int current_side = nodes[i].postSides[j];
					//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //设置该变量的范围
					//glp_set_col_kind(lp, current_side, GLP_IV);       //设置该变量为正整数
					ia[++row_variable_count] = row_count;
					ja[row_variable_count] = nodes[i].postSides[j] + 1;
					ar[row_variable_count] = 1;
				}
				glp_set_row_bnds(lp, ++row_count, GLP_FX, 1, 1);
				for (int j = 0; j < nodes[i].preSides.size(); j++)       //当为必经节点时，总的出度为1
				{
					//int current_side = nodes[i].preSides[j];
					//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //设置该变量的范围
					//glp_set_col_kind(lp, current_side, GLP_IV);       //设置该变量为正整数
					ia[++row_variable_count] = row_count;
					ja[row_variable_count] = nodes[i].preSides[j] + 1;
					ar[row_variable_count] = 1;
				}
				continue;
			}
			//剩下情况为非必经节点，限定条件为入度等于出度，并且入度小于等于1
			{
				glp_set_row_bnds(lp, ++row_count, GLP_FX, 0, 0);       //保证入度和出度相等
				for (int j = 0; j < nodes[i].postSides.size(); j++)
				{
					//int current_side = nodes[i].postSides[j];
					//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //设置该变量的范围
					//glp_set_col_kind(lp, current_side, GLP_IV);       //设置该变量为正整数
					ia[++row_variable_count] = row_count;
					ja[row_variable_count] = nodes[i].postSides[j] + 1;
					ar[row_variable_count] = 1;
				}
				//int tempCount = nodes[i].postSides.size() + 1;
				for (int j = 0; j < nodes[i].preSides.size(); j++)
				{
					//int current_side = nodes[i].preSides[j];
					//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //设置该变量的范围
					//glp_set_col_kind(lp, current_side, GLP_IV);       //设置该变量为正整数
					ia[++row_variable_count] = row_count;
					ja[row_variable_count] = nodes[i].preSides[j] + 1;
					ar[row_variable_count] = -1.0;
				}

				glp_set_row_bnds(lp, ++row_count, GLP_DB, 0, 1); //保证入度在0到1之间
				for (int j = 0; j < nodes[i].postSides.size(); j++)
				{
					//int current_side = nodes[i].postSides[j];
					//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //设置该变量的范围
					//glp_set_col_kind(lp, current_side, GLP_IV);       //设置该变量为正整数
					ia[++row_variable_count] = row_count;
					ja[row_variable_count] = nodes[i].postSides[j] + 1;
					ar[row_variable_count] = 1;
				}
			}
		}



		glp_load_matrix(lp, row_variable_count, ia, ja, ar);
		//glp_simplex(lp, NULL);
		glp_iocp parm;
		glp_init_iocp(&parm);
		parm.presolve = GLP_ON;
		int err = glp_intopt(lp, &parm);
		for (i = 0; i < eageNum; i++)
		{
			int re = glp_mip_col_val(lp, i + 1);
			if (re)
			{
				result.push_back(i);
				cointainSides[i] = true;
			}

		}

		add_constrants = isRound();
		if (add_constrants == -1)
		{
			isLoop = false;
		}
		else
		{
			result.clear();
			//cointainSides = { false };
			for (i = 0; i < eageNum; i++)
				cointainSides[i] = false;
		}
		//glp_delete_prob(lp);
	}

	saveResult();

}

//存放结果
void saveResult()
{
	int currentNode = source;
	int i;

	while (currentNode != dest)       //找到一条从起点到终点的路
	{
		int currentPostSide;
		for (i = 0; i < nodes[currentNode].postSides.size(); i++)
		{
			currentPostSide = nodes[currentNode].postSides[i];
			if (cointainSides[currentPostSide])
			{
				//currentContain[currentPostSide] = true;
				side_result.push_back(currentPostSide);
				currentNode = sides[currentPostSide].to;
				break;
			}

		}
	}
}

//判断是否成环，及返回成环的个数，如果没有成环则返回-1
int isRound()
{
	int countRound = 0; //环路的个数
	int i;
	bool currentContain[MAXSIDES] = { false };
	int currentNode = source;
	while (currentNode != dest)       //找到一条从起点到终点的路
	{
		int currentPostSide;
		for (i = 0; i < nodes[currentNode].postSides.size(); i++)
		{
			currentPostSide = nodes[currentNode].postSides[i];
			if (cointainSides[currentPostSide])
			{
				currentContain[currentPostSide] = true;
				//currentPostSide = result.erase(currentPostSide);
				currentNode = sides[currentPostSide].to;
				break;
			}

		}
	}

	//其余没在起点到终点的路上的路径肯定在一个环路上
	for (i = 0; i < result.size(); i++)
	{
		int currentSide = result[i];
		if (cointainSides[currentSide] && !currentContain[currentSide])  //找到了一个环路上的点
		{
			vector<int> temp_tour;
			countRound++;
			temp_tour.push_back(currentSide);
			int currentNode;
			int currentSides;
			int s = sides[currentSide].from;
			int t = sides[currentSide].to;
			bool hasChild = true;
			currentContain[currentSide] = true;
			currentNode = t;
			while (hasChild)
			{
				for (int j = 0; j < nodes[currentNode].postSides.size(); j++)
				{
					currentSides = nodes[currentNode].postSides[j];
					if (cointainSides[currentSides] && !currentContain[currentSides])
					{
						currentNode = sides[currentSides].to;
						temp_tour.push_back(currentSides);
						currentContain[currentSides] = true;
						break;
					}
					if (j == nodes[currentNode].postSides.size() - 1)
						hasChild = false;
				}
			}
			//if(temp_tour.size() > 1)
			tours.push_back(temp_tour);
		}
	}
	if (countRound)
		return countRound;
	return -1;
}
/*******************暴力求解*******************/
//你要完成的功能总入口
void search_route(char *topo[5000], int edge_num, char *demand)
{
	//gettimeofday(&start, NULL);
	eageNum = edge_num;
	int i = 0, j = 0, index = 0, from = 0, to = 0, weight = 0, via = 0, total_weight = 0;
	for (i = 0; i < MAXVSIZE; i++)
	{
		for (j = 0; j < MAXPSIZE; j++)
			historylen[i][j] = INT_MAX;
	}
	total_edge = edge_num;
	char via_str[800];//储存必经节点
					  //printf("\n-------------------------------------------------------\n");
					  //printf("edge_num:%d;demand:%s;\n",edge_num,demand);
	sscanf(demand, "%d,%d,%s", &source, &dest, via_str);
	source;
	dest;
	char *ptr = strtok(via_str, "|");
	while (ptr != NULL) {
		sscanf(ptr, "%d", &via);
		designatedP[designatedN++] = via;
		isvia[via] = 1;
		ptr = strtok(NULL, "|");
	}
	for (i = 0; i < edge_num; i++) {
		sscanf(topo[i], "%d,%d,%d,%d", &index, &from, &to, &weight);
		//from++;
		//to++;
		total_weight += weight;
		if (largest < from)                           // largest是到目前为止最大节点 
			largest = from;
		if (largest < to)
			largest = to;
		add_edge_into_list(from, to, weight, index);
		//printf("%d,%d,%d,%d\n",index,from,to,weight);
	}
	/*	if (total_edge>50) {
	avg_weight = 5;
	}*/
	/*if (largest <= 200) { simple = true; }
	if (largest <= 150 && largest>125) { keyvalue = 0.29; min_len = 221; }
	if (largest <= 200 && largest>150)   min_len = 300;
	if (largest <= 250 && largest>200)   min_len = 300;
	if (largest <= 300 && largest>250)   min_len = 340;
	if (largest == 500 && edge_num>2025) min_len = 610;
	if (largest == 500 && edge_num<2025) min_len = 320;
	if (largest>500) { keyvalue = 0.1;   min_len = 800; finalflag = 1; }*/

	//voilent_sovle(source, dest);
	//print_result();
	lpSolver();
	print_result();
	//return NULL;
}


