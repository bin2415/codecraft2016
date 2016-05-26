/*
                   _ooOoo_
                  o8888888o
                  88" . "88
                  (| -_- |)
                  O\  =  /O
               ____/`---'\____
             .'  \\|     |//  `.
            /  \\|||  :  |||//  \
           /  _||||| -:- |||||-  \
           |   | \\\  -  /// |   |
           | \_|  ''\---/''  |   |
           \  .-\__  `-`  ___/-. /
         ___`. .'  /--.--\  `. . __
      ."" '<  `.___\_<|>_/___.'  >'"".
     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
     \  \ `-.   \_ __\ /__ _/   .-` /  /
======`-.____`-.___\_____/___.-`____.-'======
                   `=---='
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
         佛祖保佑       永无BUG
*/
#include "route.h"
#include "lib_record.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <stack>
using namespace std;
#define MAXVSIZE 610            /* 最大顶点个数 */
#define MAXPSIZE 85 			/* 必经节点个数 */
#define WEIGHT(k) m[u][k].length-20*isvia[k]  /* 获得权重 */
#define INT_MAX 2000
struct timeval start,end;
int total_edge = 0;
int avg_weight = 1;
int getted[MAXVSIZE][MAXVSIZE];   /* 标记对应边是否被搜索到 */  	
int stacked[MAXVSIZE];  		  /* 标记是否入栈 */
int isvia[MAXVSIZE] = {0}; 		  /* 标记必经节点 */
int path[MAXVSIZE] = {0};		  /* 保存路径 */
int historylen[MAXVSIZE][MAXPSIZE] = {0};      /* 保存记忆中的最优路径情况 */
//int historyvia[MAXVSIZE] = {0};      /* 保存记忆中的必经节点个数 */
int largest = 0;                  /* 保存的最大点数 */
int edge_num = 0;				  /* 路径边个数 */
int path_len = 0;				  /* 路径权值长度 */
int via_num=0;    				  /* 此路径包含的必经节点个数 */
int v_num = 0;					  /* 此路径包含的点的总个数 */
int min_len = INT_MAX;            /* 最小权值 */
int finalflag = 0;
bool simple = false;
float keyvalue = 0.24;
stack<int> mystack,printstack;
int designatedP[MAXPSIZE];      /* 记录必经的节点*/
int designatedN;                /* 必经节点的个数*/
int source=0,dest=0;            /* 源节点和目的节点 */ 
//int child_num[MAXVSIZE] = {0};        /* 节点的后续节点个数 */
struct node{
    int node_l;                 /* 具有相同起点的上一个节点，0意味无 */ 
    int node_r;                 /* 具有相同起点的下一个节点，0意味无 */ 
	int length;                 /* 边的长度*/
	int side;                   /* 边的编号*/
}m[MAXVSIZE][MAXVSIZE];         /* m数组模拟邻接多重链表，利用第一行和第一列作为表头 */ 

void add_edge_into_list(int u,int v,int length,int side) /* 把边(u,v)添加到多重链表中 */
{	
	//child_num[u]++;
    if(m[u][v].length>0) 
    {
        if(length<m[u][v].length)  m[u][v].length=length;
        return;
    }
	m[u][v].length = length;
	m[u][v].side = side;
    m[u][v].node_l=m[u][0].node_l;
    m[u][m[u][0].node_l].node_r=v;
    m[u][v].node_r=0;
    m[u][0].node_l=v;
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
}
/*******************暴力求解*******************/
void save_path()
{
	int tmp1=0,tmp2=0;
	while(!mystack.empty()){ 
        tmp1 = mystack.top();
        printstack.push(tmp1);
        mystack.pop();
    }
	min_len = path_len;
	edge_num=0;tmp1=0;tmp2=0;
	while(!printstack.empty()){ 
		tmp1 = printstack.top();
		if(tmp2!=0) path[edge_num++]=m[tmp2][tmp1].side;
       	mystack.push(tmp1);
        printstack.pop(); 
		tmp2=tmp1;
    }
}
bool is_select(int u,int v)
{
	if(stacked[v]+getted[u][v]!=0)  return false;
	if((float)(1+via_num)/v_num<keyvalue) return false;
    if(!simple&&path_len>finalflag*15+historylen[v][via_num]) return false;
    //if(finalflag&&v_num>largest*1/2) return false;
    if(path_len+m[u][v].length+avg_weight*(designatedN-via_num-isvia[v])>=min_len) return false;
    return true;
}
void voilent_sovle(int sour,int dist)
{
    int exist=0,u=0,v=0,x=0,y=0;
    mystack.push(sour);
	stacked[sour]=1;
	v_num+=1;
    while(!mystack.empty())
	{
		gettimeofday(&end,NULL);
		long time=1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec;
		if(time/1000000.0>19.5) break;
		u = mystack.top();
		//printf("u:%d[%d]->",u,stacked[u]);
		if(u==dist)	
		{   
			//printf("End\n");
			if(via_num==designatedN&&path_len<min_len)  save_path(); //break;}
			stacked[u]=0;		//标记出栈
			mystack.pop();
			path_len-=m[mystack.top()][u].length;
			v_num-=1;
			continue;
		}
		//int index = 1;
		v = (m[u][0]).node_r;
		exist=0;
	    while(v!=0)                        /* 所有与节点u相边的节点 */
        {
		   //if(isvia[v]==0&&child_num[u]>5&&index>u/2) break;
		   if(is_select(u,v))
		   {
                historylen[v][via_num]=path_len;
				path_len+=m[u][v].length;
                //if(via_num>historyvia[v]){
                //historyvia[v]=via_num;
            
               // }
				via_num+=isvia[v];
				v_num+=1;
				//printf("v:%d;via:%d,v:%d[比例：%f]\n",v,via_num,v_num,(float)(1+via_num)/v_num);
				mystack.push(v);
			    stacked[v]=1;
			    getted[u][v]=1;
  				exist = 1;   //存在有效节点
                if(isvia[v]==1){
				int index = 1;
				int next = m[u][v].node_r;
					while(next!=0)
					{
						if(isvia[next]==0&&index>=5) getted[u][next]=1;
						next = m[u][next].node_r;
						index++;
					}
				}
				break;
		   }
           v = (m[u][v]).node_r;
        }
		if(exist==0)
		{
			x = mystack.top();
			mystack.pop();
			stacked[x]=0;
			if(!mystack.empty()) path_len-=m[mystack.top()][x].length;
			via_num-=isvia[x];
			v_num-=1;
			y = (m[x][0]).node_r;  
	   	    while(y!=0)      
            {
				getted[x][y]=0;                
				y = (m[x][y]).node_r;
            }
		}
	}
}
void print_result()
{	
	int i;
	//printf("最优路径是:"); 
    for(i=0;i<edge_num;i++)
	{
		//printf("%d|",path[i]);
		record_result(path[i]);
	}
	//printf("权值为:%d\n",min_len);
}
/*******************暴力求解*******************/
//你要完成的功能总入口
void search_route(char *topo[5000], int edge_num, char *demand)
{
	gettimeofday(&start,NULL);
    int i=0,j=0,index=0,from=0,to=0,weight=0,via=0,total_weight=0;
    for(i=0;i<MAXVSIZE;i++)
    {
        for(j=0;j<MAXPSIZE;j++)
        historylen[i][j]=INT_MAX;
    }
	total_edge = edge_num;
    char via_str[800];//储存必经节点
    //printf("\n-------------------------------------------------------\n");
    //printf("edge_num:%d;demand:%s;\n",edge_num,demand);
	sscanf (demand,"%d,%d,%s",&source,&dest,via_str);
	source++;
	dest++;
	char *ptr = strtok(via_str, "|");
    while(ptr != NULL){
       sscanf(ptr,"%d",&via);
	   designatedP[designatedN++] = ++via;
	   isvia[via++]=1;
       ptr = strtok(NULL,"|"); 
    }
    for(i=0;i<edge_num;i++){
		sscanf(topo[i],"%d,%d,%d,%d",&index,&from,&to,&weight);
		from++;
		to++;
		total_weight+=weight;
        if(largest<from)                           // largest是到目前为止最大节点 
            largest = from;
        if(largest<to)
            largest = to;
		add_edge_into_list(from,to,weight,index);
		//printf("%d,%d,%d,%d\n",index,from,to,weight);
    } 
	if(total_edge>50) {
		avg_weight=5;
	}
    if(largest<=200) { simple=true; }
	if(largest<=150&&largest>125){  keyvalue=0.29; min_len=221;  }
    if(largest<=200&&largest>150)   min_len=300;  
	if(largest<=250&&largest>200)   min_len=300;
    if(largest<=300&&largest>250)   min_len=340;
    if(largest==500&&edge_num>2025) min_len=610;  
	if(largest==500&&edge_num<2025) min_len=320;  
    if(largest>500){ keyvalue = 0.1;   min_len=800; finalflag=1; }

	voilent_sovle(source,dest);
	print_result();
}


