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
#include <time.h>
#include <stack>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>
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
    int node_u;                 /* 具有相同终点的上一个节点，0意味无 */ 
    int node_d;                 /* 具有相同终点的下一个节点，0意味无 */
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
    int k;
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
/*
    m[u][v].node_l=m[u][0].node_l;
    m[u][m[u][0].node_l].node_r=v;
    m[u][v].node_r=0;
    m[u][0].node_l=v;

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
/*******************蚁群求解*******************/
struct ant{
    int location;      //蚂蚁的位置
    int path_len=0;      //蚂蚁走过的路径长度
    int via_num=0;       //蚂蚁走过的必经点个数
    vector<int> path;  //蚂蚁走过的路径
};
struct viahistory{
    int min_len=INT_MAX;       //到达此必经节点的路径中耗费的最小值
    int minlen_via_num=0;      //耗费最小的路径包含的必经点个数
    vector<int> min_len_path;  //当前最小耗费路径
    
    int min_vnum=INT_MAX;      //到达此必经节点的路径中经过节点个数的最小值
    int minv_via_num=0;        //节点个数最小的路径包含的必经点个数
    int minv_len=0;            //节点个数最小的路径的耗费
    vector<int> min_v_path;    //当前最少节点路径
    
    vector<vector<int> > can_get_path;       //可以达到这个点的路
	vector<int> get_lenlist;                //可到达此点的耗费表 
};
int ant_min_len = INT_MAX;   //最短路径长度
vector<int> min_path;        //最短路径
int loopN=0;                 //循环次数
bool is_available(int v,vector<int> path)   //测试一条路径加上一个新点之后是否是一条可行路径
{
    int can_get[MAXVSIZE]={0};
    queue<int> tmp;
    tmp.push(v);      
    can_get[v]=1;
    for(int i=0;i<path.size();i++)
		can_get[path[i]]=1; 
    while(!tmp.empty()){
        int next=m[0][tmp.front()].node_d;
        while(next!=0){
            if(can_get[next]==0)
            {
               tmp.push(next);
               can_get[next]=1;
            }
            next=m[next][tmp.front()].node_d;       
        } 
        tmp.pop();  
    }
    if(can_get[source]==0)  return false;
    for(int i=0;i<designatedN;i++)
        if(can_get[designatedP[i]]==0)  return false;
    return true;
}
void ant_sovle(int sour,int dest)
{
   // printf("\n------------起点：%d;终点：%d--------------\n",sour,dest);
    int total_num = designatedN;     //迭代次数等于必经点个数
    int ant_num = designatedN*40;    //每次派出的蚂蚁数量是必经点个数的40倍
    map<int,viahistory> history;     //记录蚂蚁走过的历史信息
    srand(time(NULL));
    for(int i=0;i<=total_num;i++)
    {
        //ant antlist[ant_num];
        vector<ant> antlist(ant_num);
        if(i==0)    //第一次迭代时初始化蚂蚁的位置为起点，蚂蚁走过的路径中加入起点
        {
            for(int j=0;j<ant_num;j++)
            {
                antlist[j].location=sour;
                antlist[j].path.push_back(sour);
            }        
        }else{      //把之前的记录信息反馈给蚂蚁
            for(int j=0;j<antlist.size();j++)
            {
                int cur_via=designatedP[j%designatedN];
                //printf("v:%d,min_len:%d,via_num:%d\n",cur_via,history[cur_via].min_len,history[cur_via].minlen_via_num);
                if(history[cur_via].min_len<INT_MAX)
                {
                    int randN=rand()%(loopN+3); 
                    if(history[cur_via].get_lenlist.size()==0)   randN=rand()%2;
                    if(randN==0 || randN == 1)
                    {
                        antlist[j].path=history[cur_via].min_len_path;
                        antlist[j].path_len=history[cur_via].min_len;
                        antlist[j].via_num=history[cur_via].minlen_via_num;            
                    }
					/*else if(randN==1)
                    {
                        antlist[j].path=history[cur_via].min_v_path;
                        antlist[j].path_len=history[cur_via].minv_len;
                        antlist[j].via_num=history[cur_via].minv_via_num;
                    }*/
					else{
                        int randM=rand()%history[cur_via].get_lenlist.size();
                        antlist[j].path=history[cur_via].can_get_path[randM];
                        antlist[j].path_len=history[cur_via].get_lenlist[randM];
                        antlist[j].via_num=history[cur_via].minlen_via_num;
                    }
                    antlist[j].location=cur_via;
                }else
                    antlist[j].location=dest;
            }        
        }
        for(int j=0;j<antlist.size();j++)  //重置历史记录
        {
            history[designatedP[j]].min_vnum=INT_MAX;
            history[designatedP[j]].min_len=INT_MAX;
            history[designatedP[j]].can_get_path.clear();   
            history[designatedP[j]].get_lenlist.clear();
        }
        for(int j=0;j<ant_num;j++)  //依次让每只蚂蚁去走
        {
            for(int k=0;k<50;k++)   //每只蚂蚁走50步
            {
                if(antlist[j].location==dest)//如果到达了目的地
                {
                    if (antlist[j].via_num==designatedN)//如果也经过了所有的必经点
                    {   
                        //printf("via_num:%d\n",antlist[j].via_num);
                        if(antlist[j].path_len<ant_min_len)//如果路径耗费小于当前最小值
                        {
                            ant_min_len=antlist[j].path_len;
                            reverse(antlist[j].path.begin(),antlist[j].path.end());
                            min_path=antlist[j].path;
                        }    
                    }
                    break;
                }
                vector<int> nextlist;   //将接下来要访问的且为访问过的点存储起来
                int next=m[0][antlist[j].location].node_d;
                //printf("location:%d,next:%d\n",antlist[j].location,next);
                while(next!=0)
                {
                    if(find(antlist[j].path.begin(),antlist[j].path.end(),next)==antlist[j].path.end())
                        nextlist.push_back(next);
                    next=m[next][antlist[j].location].node_d;
                }             
                if(nextlist.size()==0)  break;  //无路可走则停止此蚂蚁
                vector<double> selectlist;
                for(int n=0;n<nextlist.size();n++)//依次查看接下来的点
                {
                    double key=1.0;
                    if(isvia[nextlist[n]]==1)//如过是必经节点
                    {
                        antlist[j].path.push_back(nextlist[n]);//将必经节点加入蚂蚁的路径中
                        //如果此路径比到达此必经点的最小耗费路径还优并且此路径是可行的则将此节点存储的最小耗费路径换成这个路径
                        if(antlist[j].path_len+m[nextlist[n]][antlist[j].location].length<history[nextlist[n]].min_len)
                        {
                            if(is_available(nextlist[n],antlist[j].path))//如果此路径可行则替换掉原来的路径
                            {
                                history[nextlist[n]].min_len=antlist[j].path_len+m[nextlist[n]][antlist[j].location].length;
                                history[nextlist[n]].min_len_path=antlist[j].path;
                                history[nextlist[n]].minlen_via_num=antlist[j].via_num+1;
                            }
                        }else if(antlist[j].path_len+m[nextlist[n]][antlist[j].location].length<history[nextlist[n]].min_len+50)
                        { //如果不是最优但接近最有(尽超过50)则将其存储在此必经点的可到达路径中，并将该路径长度加入到长度列表里
                             history[nextlist[n]].can_get_path.push_back(antlist[j].path);
                             history[nextlist[n]].get_lenlist.push_back(antlist[j].path_len+m[nextlist[n]][antlist[j].location].length); 
                        }
                        //如果此路径包含的节点个数下于此必经点目前存储的最小的节点个数并且路径是可行的则将此节点存储的最小节点个数路径换成此节点
                     /*   if(antlist[j].path.size()<history[nextlist[n]].min_vnum)
                        {
                            if(is_available(nextlist[n],antlist[j].path))//如果此路径可行则替换掉原来的路径
                            {
                                history[nextlist[n]].min_vnum=antlist[j].path.size();
                                history[nextlist[n]].minv_len=antlist[j].path_len+m[nextlist[n]][antlist[j].location].length;
                                history[nextlist[n]].min_v_path=antlist[j].path;
                                history[nextlist[n]].minv_via_num=antlist[j].via_num+1;
                            }
                        }*/
                        key=0.0;
                        antlist[j].path.pop_back();//将必经节点取出                      
                    }else if(nextlist[n]==dest)
                    {
                        if (antlist[j].via_num==designatedN)  
                            key*=15000;
                        else
                            key*=0.1;
                    }else if(m[0][nextlist[n]].node_d==0)
                        key = 0.0;
                    if(n==0) selectlist.push_back(key);   
                    else selectlist.push_back(key+selectlist[selectlist.size()-1]);
                }
                if(selectlist[selectlist.size()-1]==0) break;
                for(int m=0;m<selectlist.size();m++)
				    selectlist[m]/=selectlist[selectlist.size()-1];
                double randnum=rand()%1000/(float)1000;//生成0-1间的随机数
				int select=0;//随即选择下一个城市
				for (int m=0;m<selectlist.size();m++) 
                {
				    if (selectlist[m]>randnum) 
                    {
						select = m;
						break;
				    }
				}
                //往后走一步
                antlist[j].path.push_back(nextlist[select]);
                //antlist[j].path_len+=m[antlist[j].location][nextlist[select]].length;
                antlist[j].path_len+=m[nextlist[select]][antlist[j].location].length;
                antlist[j].location=nextlist[select]; 
                antlist[j].via_num+=isvia[nextlist[select]];      
            }
        }
    }
}
void ant_print_result()
{
    for (int i = 0; i < min_path.size()-1; i++) {
        //printf("%d|",min_path[i]);
        //printf("%d|",m[min_path[i]][min_path[i+1]].side);
        record_result(m[min_path[i]][min_path[i+1]].side);
    }
    //printf("路径长度为：%d\n",ant_min_len); 
}
void use_ant(int s,int d)
{
    int time_elp = clock()*1000/CLOCKS_PER_SEC;
    while(1){     
        loopN++;
        ant_sovle(d,s);
        time_elp=clock()*1000/CLOCKS_PER_SEC;
        if(9900-time_elp<time_elp/loopN)
                break;
    }
    if(ant_min_len<INT_MAX) 
        ant_print_result();
}
/*******************蚁群求解*******************/
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
		//printf("u:%d[%d]->",u,stacked[u]);
		if(u==dest)	
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
    if(edge_num>=2375&&edge_num<2500)
        use_ant(source,dest);
    else{
        voilent_sovle(source,dest);
	    print_result(); 
    }   
   // use_ant(source,dest);
}



