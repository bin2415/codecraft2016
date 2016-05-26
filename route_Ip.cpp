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
#define MAXVSIZE 610            /* ��󶥵���� */
#define MAXSIDES 3000
#define MAXPSIZE 85 			/* �ؾ��ڵ���� */
#define WEIGHT(k) m[u][k].length-20*isvia[k]  /* ���Ȩ�� */
#define INT_MAX 2000
//struct timeval start, end;
int total_edge = 0;
int avg_weight = 1;
int getted[MAXVSIZE][MAXVSIZE];   /* ��Ƕ�Ӧ���Ƿ������� */
int stacked[MAXVSIZE];  		  /* ����Ƿ���ջ */
int isvia[MAXVSIZE] = { 0 }; 		  /* ��Ǳؾ��ڵ� */
int path[MAXVSIZE] = { 0 };		  /* ����·�� */
int historylen[MAXVSIZE][MAXPSIZE] = { 0 };      /* ��������е�����·����� */
												 //int historyvia[MAXVSIZE] = {0};      /* ��������еıؾ��ڵ���� */
int largest = 0;                  /* ����������� */
int edge_num = 0;				  /* ·���߸��� */
int path_len = 0;				  /* ·��Ȩֵ���� */
int via_num = 0;    				  /* ��·�������ıؾ��ڵ���� */
int v_num = 0;					  /* ��·�������ĵ���ܸ��� */
int min_len = INT_MAX;            /* ��СȨֵ */
int finalflag = 0;
bool simple = false;
float keyvalue = 0.24;
stack<int> mystack, printstack;
int designatedP[MAXPSIZE];      /* ��¼�ؾ��Ľڵ�*/
int designatedN;                /* �ؾ��ڵ�ĸ���*/
int source = 0, dest = 0;            /* Դ�ڵ��Ŀ�Ľڵ� */
									 //int child_num[MAXVSIZE] = {0};        /* �ڵ�ĺ����ڵ���� */
									 //struct node {
									 //	int node_l;                 /* ������ͬ������һ���ڵ㣬0��ζ�� */
									 //	int node_r;                 /* ������ͬ������һ���ڵ㣬0��ζ�� */
									 //	int length;                 /* �ߵĳ���*/
									 //	int side;                   /* �ߵı��*/
									 //}m[MAXVSIZE][MAXVSIZE];         /* m����ģ���ڽӶ����������õ�һ�к͵�һ����Ϊ��ͷ */

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

//���Թ滮������
#define MAXROWNUM 10000
//#define SIDENUM 3000
bool cointainSides[MAXSIDES] = { false };    //���Թ滮������ıߵļ���
int row_count = 0; //�����ļ���
int row_variable_count = 0; //����ϵ���ļ���
int ia[MAXROWNUM], ja[MAXROWNUM];
double ar[MAXROWNUM];
int eageNum;
vector<int> result;   //������Թ滮����ıߵ����
vector< vector<int> > tours; //��Ż�
vector<int> side_result; //��Ž��
void add_edge_into_list(int u, int v, int length, int side) /* �ѱ�(u,v)��ӵ����������� */
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
	//printf("����·����:"); 
	for (i = 0; i<side_result.size(); i++)
	{
		//printf("%d|",path[i]);
		record_result(side_result[i]);
	}
	//printf("ȨֵΪ:%d\n",min_len);
}

//////////////////////////////////////
/////////���Թ滮���/////////////////
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
		row += add_constrants;    //�ܵ�Լ������
		glp_prob *lp = glp_create_prob();
		glp_add_rows(lp, row); //��ӵ���������
		glp_add_cols(lp, eageNum);  //��ӵı�������
		glp_set_obj_dir(lp, GLP_MIN);      //ȡ��Сֵ


		for (i = 0; i < eageNum; i++)      //�����Թ滮�������������Χ
		{
			int tempSide = i + 1;
			glp_set_col_bnds(lp, tempSide, GLP_DB, 0, 1); //���ñ���Ϊ0��1֮��
			glp_set_obj_coef(lp, tempSide, sides[i].weight);
			glp_set_col_kind(lp, tempSide, GLP_IV);           //���ñ���Ϊ������


		}

		///��ӵĲ��óɻ���Լ��
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
			if (i == source)    //��i����ʼ��ʱ
			{
				glp_set_row_bnds(lp, ++row_count, GLP_FX, 1, 1);
				//glp_set_col_bnds(lp, i, GLP_DB, 0.0, 1.0);
				for (int j = 0; j < nodes[i].postSides.size(); j++)       //��Ϊ���ʱ���ܵĳ���Ϊ1
				{
					//int current_side = nodes[i].postSides[j];
					//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //���øñ����ķ�Χ
					//glp_set_col_kind(lp, current_side, GLP_IV);       //���øñ���Ϊ������
					ia[++row_variable_count] = row_count;
					ja[row_variable_count] = nodes[i].postSides[j] + 1;
					ar[row_variable_count] = 1;
				}

				if (nodes[i].preSides.size() > 0)
				{
					glp_set_row_bnds(lp, ++row_count, GLP_FX, 0, 0);
					for (int j = 0; j < nodes[i].preSides.size(); j++)       //��Ϊ���ʱ���ܵ����Ϊ0
					{
						//int current_side = nodes[i].preSides[j];
						//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //���øñ����ķ�Χ
						//glp_set_col_kind(lp, current_side, GLP_IV);       //���øñ���Ϊ������
						ia[++row_variable_count] = row_count;
						ja[row_variable_count] = nodes[i].preSides[j] + 1;
						ar[row_variable_count] = 1;
					}
				}

				continue;
			}
			if (i == dest)      //��i��Ŀ���ʱ���ܵ����Ϊ1
			{
				glp_set_row_bnds(lp, ++row_count, GLP_FX, 1, 1);
				for (int j = 0; j < nodes[i].preSides.size(); j++)       //��Ϊ�յ�ʱ���ܵ����Ϊ1
				{
					//int current_side = nodes[i].preSides[j];
					//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //���øñ����ķ�Χ
					//glp_set_col_kind(lp, current_side, GLP_IV);       //���øñ���Ϊ������
					ia[++row_variable_count] = row_count;
					ja[row_variable_count] = nodes[i].preSides[j] + 1;
					ar[row_variable_count] = 1.0;
				}

				if (nodes[i].postSides.size() > 0)
				{
					//glp_set_row_bnds(lp, ++row_count, GLP_FX, 0.0, 0.0);
					//glp_set_col_bnds(lp, i, GLP_DB, 0.0, 1.0);
					for (int j = 0; j < nodes[i].postSides.size(); j++)       //��Ϊ�յ�ʱ���ܵĳ���Ϊ0
					{
						//int current_side = nodes[i].postSides[j];
						//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //���øñ����ķ�Χ
						//glp_set_col_kind(lp, current_side, GLP_IV);       //���øñ���Ϊ������
						ia[++row_variable_count] = row_count;
						ja[row_variable_count] = nodes[i].postSides[j] + 1;
						ar[row_variable_count] = 1;
					}
				}
				continue;
			}
			if (isvia[i])       //��i�Ǳؾ��ڵ�ʱ,��Ⱥͳ��ȶ�Ϊ1
			{
				glp_set_row_bnds(lp, ++row_count, GLP_FX, 1, 1);
				for (int j = 0; j < nodes[i].postSides.size(); j++)       //��Ϊ�ؾ��ڵ�ʱ���ܵĳ���Ϊ1
				{
					//int current_side = nodes[i].postSides[j];
					//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //���øñ����ķ�Χ
					//glp_set_col_kind(lp, current_side, GLP_IV);       //���øñ���Ϊ������
					ia[++row_variable_count] = row_count;
					ja[row_variable_count] = nodes[i].postSides[j] + 1;
					ar[row_variable_count] = 1;
				}
				glp_set_row_bnds(lp, ++row_count, GLP_FX, 1, 1);
				for (int j = 0; j < nodes[i].preSides.size(); j++)       //��Ϊ�ؾ��ڵ�ʱ���ܵĳ���Ϊ1
				{
					//int current_side = nodes[i].preSides[j];
					//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //���øñ����ķ�Χ
					//glp_set_col_kind(lp, current_side, GLP_IV);       //���øñ���Ϊ������
					ia[++row_variable_count] = row_count;
					ja[row_variable_count] = nodes[i].preSides[j] + 1;
					ar[row_variable_count] = 1;
				}
				continue;
			}
			//ʣ�����Ϊ�Ǳؾ��ڵ㣬�޶�����Ϊ��ȵ��ڳ��ȣ��������С�ڵ���1
			{
				glp_set_row_bnds(lp, ++row_count, GLP_FX, 0, 0);       //��֤��Ⱥͳ������
				for (int j = 0; j < nodes[i].postSides.size(); j++)
				{
					//int current_side = nodes[i].postSides[j];
					//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //���øñ����ķ�Χ
					//glp_set_col_kind(lp, current_side, GLP_IV);       //���øñ���Ϊ������
					ia[++row_variable_count] = row_count;
					ja[row_variable_count] = nodes[i].postSides[j] + 1;
					ar[row_variable_count] = 1;
				}
				//int tempCount = nodes[i].postSides.size() + 1;
				for (int j = 0; j < nodes[i].preSides.size(); j++)
				{
					//int current_side = nodes[i].preSides[j];
					//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //���øñ����ķ�Χ
					//glp_set_col_kind(lp, current_side, GLP_IV);       //���øñ���Ϊ������
					ia[++row_variable_count] = row_count;
					ja[row_variable_count] = nodes[i].preSides[j] + 1;
					ar[row_variable_count] = -1.0;
				}

				glp_set_row_bnds(lp, ++row_count, GLP_DB, 0, 1); //��֤�����0��1֮��
				for (int j = 0; j < nodes[i].postSides.size(); j++)
				{
					//int current_side = nodes[i].postSides[j];
					//glp_set_col_bnds(lp, current_side, GLP_DB, 0.0, 1.0);   //���øñ����ķ�Χ
					//glp_set_col_kind(lp, current_side, GLP_IV);       //���øñ���Ϊ������
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

//��Ž��
void saveResult()
{
	int currentNode = source;
	int i;

	while (currentNode != dest)       //�ҵ�һ������㵽�յ��·
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

//�ж��Ƿ�ɻ��������سɻ��ĸ��������û�гɻ��򷵻�-1
int isRound()
{
	int countRound = 0; //��·�ĸ���
	int i;
	bool currentContain[MAXSIDES] = { false };
	int currentNode = source;
	while (currentNode != dest)       //�ҵ�һ������㵽�յ��·
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

	//����û����㵽�յ��·�ϵ�·���϶���һ����·��
	for (i = 0; i < result.size(); i++)
	{
		int currentSide = result[i];
		if (cointainSides[currentSide] && !currentContain[currentSide])  //�ҵ���һ����·�ϵĵ�
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
/*******************�������*******************/
//��Ҫ��ɵĹ��������
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
	char via_str[800];//����ؾ��ڵ�
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
		if (largest < from)                           // largest�ǵ�ĿǰΪֹ���ڵ� 
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


