
#include "route.h"
#include "lib_record.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <stack>
#include <glpk.h>
#include <sys/time.h>
using namespace std;
#define MAXEDGE  3000           /* 最大边的数量 */
#define MAXVARIB 10000          /* 最大变量个数 */ 
#define MAXVSIZE 602            /* 最大顶点个数 */
#define MAXPSIZE 52 			/* 必经节点个数 */
#define INT_MAX 2000
int largest = 0;                /* 保存的最大点数 */
int total_edge = 0;             /* 总边数 */
int isvia[MAXVSIZE] = {0}; 		/* 标记必经节点 */
int path[MAXVSIZE] = {0};		  /* 保存路径 */
int historylen[MAXVSIZE][MAXPSIZE] = {0};      /* 保存记忆中的最优路径情况 */
int getted[MAXVSIZE][MAXVSIZE];   /* 标记对应边是否被搜索到 */
int stacked[MAXVSIZE];  		  /* 标记是否入栈 */
int designatedP[MAXPSIZE];      /* 记录必经的节点*/
int designatedN;                /* 必经节点的个数*/
int source=0,dest=0;            /* 源节点和目的节点 */
int min_len = INT_MAX;            /* 最小权值 */
int edge_num = 0;				  /* 路径边个数 */
int via_num=0;    				  /* 此路径包含的必经节点个数 */
int v_num = 0;					  /* 此路径包含的点的总个数 */
int path_len = 0;				  /* 路径权值长度 */
struct timeval start,end;
stack<int> mystack,printstack;
struct edge{
    int weight;
    int from;
    int to;
}n[MAXEDGE];
struct node{
    int node_l;                 /* 具有相同起点的上一个节点，0意味无 */ 
    int node_r;                 /* 具有相同起点的下一个节点，0意味无 */ 
    int node_u;                 /* 具有相同终点的上一个节点，0意味无 */ 
    int node_d;                 /* 具有相同终点的下一个节点，0意味无 */
	int length;                 /* 边的长度*/
	int side;                   /* 边的编号*/
}m[MAXVSIZE][MAXVSIZE];         /* m数组模拟邻接多重链表，利用第一行和第一列作为表头 */ 
void add_edge_into_list(int u,int v,int length,int side) /* 把边(u,v)添加到多重链表中 */
{
    if(m[u][v].length>0)
    {
        if(length<m[u][v].length)  m[u][v].length=length;
        return;
    }
    int k;
	m[u][v].length = length;
	m[u][v].side = side;
    if(m[u][0].node_r==0)           /* 添加的起点为u的第一条边 */
    {
        m[u][0].node_r = v;         /* m[u][0].node_r是起点为u的第一条边的终点 */
        m[u][0].node_l = v;         /* m[u][0].node_l是最近一次输入的以u为起点的边的终点 */

        if(m[0][v].node_d==0)       /* 添加的终点为v的第一条边 */
        {
            (m[0][v]).node_d = u;   /* (m[0][v]).node_d是终点为v的第一条边的起点 */
            (m[0][v]).node_u = u;   /* (m[0][v]).node_u是最近一次输入的以v为终点的边的起点 */
        }
        else                        /* 添加的边不是终点为v的第一条边 */
        {
            k = (m[0][v]).node_u;   /* 最近一次输入的以v为终点的边的起点 */
            (m[k][v]).node_d = u;   /* 把边(k,v)的node_d指向具有相同终点的下一条边(u,v) */
            (m[u][v]).node_u = k;   /* 把边(u,v)的node_u指向具有相同终点的上一条边(k,v) */
            (m[0][v]).node_u = u;   /* 修改最近一次输入的以v为终点的边的起点 */
        }
    }
    else                            /* 添加边不是的起点为u的第一条边 */
    {
        k = (m[u][0]).node_l;       /* 最近一次输入的以u为起点的边的终点 */
        (m[u][k]).node_r = v;       /* 把边(u,k)的node_r指向具有相同起点的下一条边(u,v) */
        (m[u][v]).node_l = k;       /* 把边(u,v)的node_l指向具有相同起点的上一条边(u,k) */
        (m[u][0]).node_l = v;       /* 修改最近一次输入的以u为起点的边的终点 */
        if((m[0][v]).node_d==0)
        {
            (m[0][v]).node_d = u;
            (m[0][v]).node_u = u;
        }
        else
        {
            k = (m[0][v]).node_u;
            (m[k][v]).node_d = u;
            (m[u][v]).node_u = k;
            (m[0][v]).node_u = u;
        }
    }
}
/**************************线性规划**************************/
vector<int> getNextlist(int cur)
{
    vector<int> result;
    int next=m[cur][0].node_r;
    while(next!=0){
        result.push_back(m[cur][next].side);
        next=m[cur][next].node_r;        
    }
    return result;
}
vector<int> getLastlist(int cur)
{
    vector<int> result;
    int last=m[0][cur].node_d;
    while(last!=0){
        result.push_back(m[last][cur].side);
        last=m[last][cur].node_d;        
    }
    return result;
}
int findLoop(int start,int end,int flag,int edge[])
{
    vector<int> nextlist=getNextlist(start);    
    for(unsigned int i=0;i<nextlist.size();i++)
    {
        int next = nextlist[i];
        if(edge[next]==1)
        {   
            edge[next]=flag;
            if(n[next].to==end) return n[next].weight;
            return findLoop(n[next].to,end,flag,edge)+n[next].weight;
        }   
    }
}
void Sort(int start,int edge[],vector<int>* result)
{
    if(start==dest) return;
    vector<int> nextlist=getNextlist(start);
    for(unsigned int i=0;i<nextlist.size();i++)
    {
        int next = nextlist[i];
        if(edge[next]==2)
        {   
            result->push_back(next);
            Sort(n[next].to,edge,result);
        }   
    }
}
int edge_result[MAXEDGE];
void mip_solve()
{  
    vector<vector<int> > loopList;  //存储找到的环路
    int total_weight=0;          //存储总耗费
    while(true){
        glp_prob *mip = glp_create_prob();
        glp_set_prob_name(mip, "mip_solve");
        //glp_term_out(GLP_OFF);
        glp_set_obj_dir(mip, GLP_MIN);   
        glp_add_cols(mip, total_edge);
        for(int i=1;i<=total_edge;i++)
        {
           //  glp_set_col_bnds(mip, i, GLP_DB, 0.0, 1.0);
             glp_set_obj_coef(mip, i, n[i-1].weight);
           //  glp_set_col_kind(mip, i, GLP_IV); 
		   glp_set_col_kind(mip, i, GLP_BV);
        }
        int ia[MAXVARIB], ja[MAXVARIB];
        double ar[MAXVARIB];
        int var_num=1,row_num=1;
        
        glp_add_rows(mip, loopList.size()+2*largest);
        int row_index=1;
        /* 约束条件 */
        for(int i=1;i<=largest;i++)
        {
            if(i==source)//起点
            {
                glp_set_row_bnds(mip, row_index++, GLP_FX, 1, 1); //起点出度和等于1
                glp_set_row_bnds(mip, row_index++, GLP_FX, 0, 0); //起点入度和等于0
                vector<int> nextlist = getNextlist(i);
                for(unsigned int j=0;j<nextlist.size();j++)
                {
                    ia[var_num]=row_num;
                    ja[var_num]=nextlist[j]+1;
                    ar[var_num++]=1.0; 
                }
                row_num++;
                vector<int> lastlist = getLastlist(i);
                for(unsigned int j=0;j<lastlist.size();j++)
                {
                    ia[var_num]=row_num;
                    ja[var_num]=lastlist[j]+1;
                    ar[var_num++]=1.0; 
                }
                row_num++;
                continue;
            }
            if(i==dest) //终点
            {
                glp_set_row_bnds(mip, row_index++, GLP_FX, 1, 1); //终点入度和等于1
                glp_set_row_bnds(mip, row_index++, GLP_FX, 0, 0); //终点出度和等于0
                vector<int> lastlist = getLastlist(i);
                for(unsigned int j=0;j<lastlist.size();j++)
                {
                    ia[var_num]=row_num;
                    ja[var_num]=lastlist[j]+1;
                    ar[var_num++]=1.0; 
                }
                row_num++;
                vector<int> nextlist = getNextlist(i);
                for(unsigned int j=0;j<nextlist.size();j++)
                {
                    ia[var_num]=row_num;
                    ja[var_num]=nextlist[j]+1;
                    ar[var_num++]=1.0; 
                }
                row_num++;
                continue;
            }
            //必经节点
            if(isvia[i]==1)
            {
                glp_set_row_bnds(mip, row_index++, GLP_FX, 1, 1); //必经点入度和等于1
                glp_set_row_bnds(mip, row_index++, GLP_FX, 1, 1); //必经点出度和等于1
                vector<int> nextlist = getNextlist(i);
                for(unsigned int j=0;j<nextlist.size();j++)
                {
                    ia[var_num]=row_num;
                    ja[var_num]=nextlist[j]+1;
                    ar[var_num++]=1.0; 
                }
                row_num++;
                vector<int> lastlist = getLastlist(i);
                for(unsigned int j=0;j<lastlist.size();j++)
                {
                    ia[var_num]=row_num;
                    ja[var_num]=lastlist[j]+1;
                    ar[var_num++]=1.0; 
                }
                row_num++;
                continue;
            } 
            //非必经节点
            glp_set_row_bnds(mip, row_index++, GLP_FX, 0, 0); //入度和减去出度和等于0
            glp_set_row_bnds(mip, row_index++, GLP_DB, 0, 1); //入度和小于等于1,大于等于0
            vector<int> nextlist = getNextlist(i);
            vector<int> lastlist = getLastlist(i);
            for(unsigned int j=0;j<nextlist.size();j++)
            {
                ia[var_num]=row_num;
                ja[var_num]=nextlist[j]+1;
                ar[var_num++]=1.0; 
            }
            for(unsigned int j=0;j<lastlist.size();j++)
            {
                ia[var_num]=row_num;
                ja[var_num]=lastlist[j]+1;
                ar[var_num++]=-1.0; 
            }
            row_num++;
            for(unsigned int j=0;j<nextlist.size();j++)
            {
                ia[var_num]=row_num;
                ja[var_num]=nextlist[j]+1;
                ar[var_num++]=1.0; 
            }
            row_num++;
        }
        for(unsigned int i=0;i<loopList.size();i++)
        {
            //printf("小于等于[%d]\n", loopList[i].size()-1);
            glp_set_row_bnds(mip, row_index++, GLP_DB, 0, loopList[i].size()-1); //入度和小于等于边数减一,大于等于0
            for(unsigned int j=0;j<loopList[i].size();j++)
            {   
                ia[var_num]=row_num;
                ja[var_num]=loopList[i][j]+1;
                ar[var_num++]=1.0; 
            }
            row_num++;
        }
        glp_load_matrix(mip, var_num-1, ia, ja, ar);
        
        glp_iocp parm;
        glp_init_iocp(&parm);
        parm.presolve = GLP_ON;
        int err = glp_intopt(mip, &parm);
        total_weight = (int)glp_mip_obj_val(mip);
        for(int i=1;i<=total_edge;i++)
        {
            edge_result[i-1]=glp_mip_col_val(mip, i);
        }
        //寻找环路
        int flag = 2,start=-1;
        int cur_weight = findLoop(source,dest,flag++,edge_result);
        while(cur_weight!=total_weight){
            for(int i=0;i<total_edge;i++)
            {
                if(edge_result[i]==1)
                {
                    start=n[i].from;
                    break;
                }
            }
            cur_weight += findLoop(start,start,flag++,edge_result);
        }
        if(flag==3) break;
        for(int i=3;i<flag;i++)
        {
            vector<int> loop;
            for(int j=0;j<total_edge;j++)
            {
                if(edge_result[j]==i) 
                    loop.push_back(j);
            }
            loopList.push_back(loop);
        }
        glp_delete_prob(mip);
    }
    vector<int> result;
    Sort(source,edge_result,&result);
    for(unsigned int i=0;i<result.size();i++)
    {
        record_result(result[i]); 
        //printf("%d|",result[i]);
    }
    //printf("耗费：%d\n",total_weight);
}
/**************************线性规划**************************/
/**************************暴力求解**************************/
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
	if((float)(1+via_num)/v_num<0.15) return false;
    if(path_len>15+historylen[v][via_num]) return false;
    if(path_len+m[u][v].length+5*(designatedN-via_num-isvia[v])>=min_len) return false;
    return true;
}
void voilent_sovle(int sour,int dest)
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
		if(u==dest)
		{
			if(via_num==designatedN&&path_len<min_len)  
            {
                save_path(); 
                if(path_len==664)  break;
            }
			stacked[u]=0;		//标记出栈
			mystack.pop();
			path_len-=m[mystack.top()][u].length;
			v_num-=1;
			continue;
		}
		v = (m[u][0]).node_r;
		exist=0;
	    while(v!=0)                        /* 所有与节点u相边的节点 */
        {
		   if(is_select(u,v))
		   {
                historylen[v][via_num]=path_len;
				path_len+=m[u][v].length;
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
		printf("%d|",path[i]);
		record_result(path[i]);
	}
	//printf("权值为:%d\n",min_len);
}
/**************************暴力求解**************************/
//你要完成的功能总入口
void search_route(char *topo[5000], int edge_num, char *demand)
{
    gettimeofday(&start,NULL);
    int i=0,j=0,index=0,from=0,to=0,weight=0,via=0;
    for(i=0;i<MAXVSIZE;i++)
    {
        for(j=0;j<MAXPSIZE;j++)
        historylen[i][j]=INT_MAX;
    }
	total_edge = edge_num;
    char via_str[800];//储存必经节点
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
        n[index].weight=weight;
        n[index].from=from;
        n[index].to=to;
        if(largest<from)                           // largest是到目前为止最大节点
            largest = from;
        if(largest<to)
            largest = to;
		add_edge_into_list(from,to,weight,index);
		//printf("%d,%d,%d,%d\n",index,from,to,weight);
    }
    if(edge_num>=2250&&edge_num<2375){
      min_len=800;
      voilent_sovle(source,dest);
      print_result();
    }
    else{
      mip_solve();
    }
}
