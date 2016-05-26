#include <vector>
#include <math.h>
#include <stdio.h>
#include <ctime>
#include <map>
#include <cstdio>
#include <memory.h>
#include <algorithm>
#include <iostream>
#include <time.h>
#include <queue>
#include "route.h"
#include "lib_record.h"
//#pragma warning(disable:4996)
using namespace std;
#define INT_MAX 10000000
#define MAX 601
#define MAX_NODE_NEC 601
bool TestTargetInBranch(int index,vector<int> path) ;
bool RevTestTargetInBranch(int index,vector<int> path) ;
void search_init(char *graph[5000], int edge_num, char *condition);
int ReverseSearchInMIddleGraph(int startPoint, int targetPoint);
void dfs(int st, int cost, int num);
int pre_process();
int road_cost[MAX][MAX];
unsigned short road_id[MAX][MAX];
int st, en, node_nec_num;
int visited[MAX];
int min_cost = -1;
int max_node_num;
vector<int>record;//用于记录dfs的当前路径
vector<int>target;//用于记录dfs得到的当前cost最小的路径
vector<int> node_nec;

vector<int> best_Path;
int best_length = INT_MAX;
int aco_loop = 0;
struct Ant {
	int place;//蚂蚁的当前位置
	int targetFound = 0;//0:未找到目标点，1:找到目标点
	int nodeNecHave = 0;//表明蚂蚁的路径中包含了多少必经点
	vector<int> path;//蚂蚁的走过的路径
	int TotalLength = 0;//走过的距离总和
};
map<int, vector<int> > mp,rMp;//key: 图上每一个点 val: 该点下一步可到达的点集


struct ShortestPathToNodeNec
{

	int bestLength = INT_MAX;
	int nodeNecHave = 0;
	vector<vector<int> > PathCanGetThere;
	vector<int> PathLengthList;
	vector<int> bestPathNow;

	int bestPathSize = INT_MAX;
	int bestPathSizeNodeNecHave = 0;
	int bestPathSizeLength = 0;
	vector<int> bestPathSizePath;

};
void search_route(char *graph[5000], int edge_num, char *condition)
{

    search_init( &graph[0], edge_num, &condition[0]);
    if(1==pre_process())
        return;
    int num = min(edge_num,max_node_num);
    /*if(edge_num/max_node_num<4)
        record_result(12);
    else
        return;*/
    //取边数和最大节点编号中的最小值来作判定使用dfs还是蚁群算法的根据
    //若边数小于最大节点标号，则将该值除以2再来进行判定

    if(num==edge_num)
        num/=2;
	if(num>30){
        //record_result(12);
        int time_elp = clock()*1000/CLOCKS_PER_SEC;

        while(1){
            //if(0==SearchInBigGraph(st, en))
            aco_loop++;
            //if(aco_loop%2==1)
          //      SearchInMIddleGraph(st, en);
          //  else
                ReverseSearchInMIddleGraph(en,st);
            time_elp=clock()*1000/CLOCKS_PER_SEC;
            if(9900-time_elp<time_elp/aco_loop)
                break;
        }
        if (best_length < INT_MAX) {
            for (int i = 0; i < best_Path.size() - 1; i++) {
                record_result(road_id[best_Path[i]][best_Path[i + 1]]);
            }
        }

	}

    else{
        //开始从开始节点进行深度优先搜索，并且在dfs中不断更新最优的路径
        visited[st] = 1;
		record.push_back(st);
		dfs(st, 0, 0);
		vector<int>::iterator iter = target.begin();
		//将得到的最优路径进行保存
		for (; iter != target.end() - 1; iter++){
            int a, b;
            a = *iter;
            b = *(iter + 1);
            record_result(road_id[a][b]);
		}
		//cout <<"the best length: "<<min_cost <<endl;
    }

    return;
}
void search_init(char *graph[5000], int edge_num, char *condition)
{
    memset(&road_cost[0][0], 0, sizeof(road_cost));//初始值设置为20，是因为图中所有权重均在1到20之间
	memset(&road_id[0][0], -1, sizeof(road_id));//边的标号是大于等于0的，所以将边初始化为-1，
												//其实不初始化在使用dfs的时候也是可以的
	memset(&visited[0], 0, sizeof(visited));//初始化为0表示所有的节点均未被访问到
											//将图的信息保存到二维数组中
	//将topo.csv的信息读入
	max_node_num=0;
	for (int i = 0; i<edge_num; i++){
		int id, st, en, cost;
		sscanf(graph[i], "%d,%d,%d,%d", &id, &st, &en, &cost);
		//通过下面这个判断解决：两个节点之间同一个方向的有向边不超过一个条的问题，
		//即只将其中权重最小的一条有向边考虑为有效边
        if(road_cost[st][en]==0){
            road_cost[st][en] = cost;
            road_id[st][en] = id;
            mp[st].push_back(en);
            rMp[en].push_back(st);
        }
        else if(cost<road_cost[st][en]){
            road_cost[st][en]=cost;
            road_id[st][id]=id;
        }
        int a=max(st,en);
        if(a>max_node_num)
            max_node_num=a;

	}
	//将demand.csv的信息读入
	sscanf(condition, "%d,%d", &st, &en);
	node_nec_num = 0;
	char *tmp = condition + 3;
	while (*tmp != 0x0a && *tmp!='\n'&& *tmp!='\0')
	{
		if (*tmp == '|' || *tmp == ',')
		{
			int node;
			sscanf(tmp + 1, "%d|", &node);
			if(node!=st&&node!=en){
                node_nec.push_back(node);
                node_nec_num++;
            }
			tmp += 2;
		}
		else
			tmp++;
	}
}
//int road_cost[MAX][MAX];缺省的用0表示
//unsigned short road_id[MAX][MAX];缺省的用-1表示
//将图中出度为零的非终点节点删除
//判断所有的必经节点中是否有出度或者入度为零的节点，若有则返回1，使得找路算法终止
int pre_process()
{
    //将图中的出度为0的非终点节点删除
    for(int i=0;i<max_node_num+1;i++){
        if(i==en)
            continue;
        int j;
        for(j=0;j<max_node_num+1;j++){
            if(road_cost[i][j]!=0)
                break;
        }
        if(j==max_node_num){
            mp.erase(i);
        }
    }
    //遍历必经节点中是否有出度为零的节点，若有则返回1
    vector<int>::iterator iter=node_nec.begin();
    for(;iter!=node_nec.end();iter++){

        int i;
        for(i=0;i<max_node_num+1;i++){
            if(road_cost[*iter][i]!=0)
                break;
        }
        if(i==max_node_num)
            return 1;
    }
    //遍历必经节点中是否有入度为零的节点，若有则返回1
    iter=node_nec.begin();
    for(;iter!=node_nec.end();iter++){

        int i;
        for(i=0;i<max_node_num+1;i++){
            if(road_cost[i][*iter]!=0)
                break;
        }
        if(i==max_node_num)
            return 1;
    }

    //cout <<"the num of nodes whose output is zero: "<<cnt<<endl;
    return 0;
}

int ReverseSearchInMIddleGraph(int startPoint, int targetPoint) {


	int ANT_NUM = 250;
	int iter_max = node_nec_num;//迭代次数
	int iter_bestNotChange = -1;//表明最优的结果已经有多少代没有变化了
	map<int, ShortestPathToNodeNec> ShortestPathToNodeNecMp;

	srand(time(NULL));
	//开始进行迭代
	for (int iter = 0; iter<=iter_max; iter++) {
		//cout << iter << endl;

		//只要找到一个满足要求的解就结束迭代，
		//if (/*iter_bestNotChange > 3 &&*/ best_length < INT_MAX)
		//{
			//break;
		//}

		ANT_NUM = node_nec_num * 40;
		vector<Ant> ant(ANT_NUM);
		if (iter == 0) {
			for (int i = 0; i < ant.size(); i++) {
				ant[i].place = startPoint;
				ant[i].path.push_back(startPoint);
			}

		}
		else {
			for (int i = 0; i < ant.size(); i++) {
				if (ShortestPathToNodeNecMp[node_nec[i%node_nec_num]].bestLength < INT_MAX) {
                    int randP = rand()%(aco_loop+3);
                    if(ShortestPathToNodeNecMp[node_nec[i%node_nec_num]].PathLengthList.size()==0)
                        randP = rand()%2;
                    if(randP==0){
                         ant[i].path = ShortestPathToNodeNecMp[node_nec[i%node_nec_num]].bestPathNow;
                        ant[i].TotalLength = ShortestPathToNodeNecMp[node_nec[i%node_nec_num]].bestLength;
                        ant[i].nodeNecHave = ShortestPathToNodeNecMp[node_nec[i%node_nec_num]].nodeNecHave;
                        ant[i].place = node_nec[i%node_nec_num];
                    }
                    else if(randP==1){
                         ant[i].path = ShortestPathToNodeNecMp[node_nec[i%node_nec_num]].bestPathSizePath;
                        ant[i].TotalLength = ShortestPathToNodeNecMp[node_nec[i%node_nec_num]].bestPathSizeLength;
                        ant[i].nodeNecHave = ShortestPathToNodeNecMp[node_nec[i%node_nec_num]].bestPathSizeNodeNecHave;
                        ant[i].place = node_nec[i%node_nec_num];
                    }
                    else{
                        int randPlace = rand() % ShortestPathToNodeNecMp[node_nec[i%node_nec_num]].PathLengthList.size();
                        ant[i].path = ShortestPathToNodeNecMp[node_nec[i%node_nec_num]].PathCanGetThere[randPlace];
                        ant[i].TotalLength = ShortestPathToNodeNecMp[node_nec[i%node_nec_num]].PathLengthList[randPlace];
                        ant[i].nodeNecHave = ShortestPathToNodeNecMp[node_nec[i%node_nec_num]].nodeNecHave;
                        ant[i].place = node_nec[i%node_nec_num];
                    }

				}
				else
					ant[i].place = targetPoint;
			}
		}


		for (int i = 0; i < node_nec_num; i++) {
			ShortestPathToNodeNecMp[node_nec[i]].PathCanGetThere.clear();
			ShortestPathToNodeNecMp[node_nec[i]].PathLengthList.clear();
			ShortestPathToNodeNecMp[node_nec[i]].bestLength = INT_MAX;
            ShortestPathToNodeNecMp[node_nec[i]].bestPathSize = INT_MAX;
		}

		//逐个蚂蚁路径选择
		for (int i = 0; i<ANT_NUM; i++) {

			for (int j = 0; j<50; j++) {
				//若第i只蚂蚁已经到达了目标终点，则检测是不是通过了所有的必经点
				if (ant[i].place == targetPoint) {
					ant[i].targetFound = 1;
					//通过了所有的必经点，则判断路径的cost是不是低于best_length
					if (ant[i].nodeNecHave == node_nec.size()) {
						//小于最小路径长度，则更新最小路径长度，以及次数标记量iter_bestNotChange
						if (ant[i].TotalLength<best_length) {
							best_length = ant[i].TotalLength;
							reverse(ant[i].path.begin(),ant[i].path.end());
							best_Path = ant[i].path;
							iter_bestNotChange = 0;
						}
						//等于最小路径长度，则只更新次数表计量iter_bestNotChange
						else if (ant[i].TotalLength == best_length) {
							iter_bestNotChange++;
						}
						//大于最小路径长度，则不错处理
						else {}
					}
					break;//当第i只蚂蚁到达目标节点后，则停止第i只蚂蚁的路径搜索
				}
			/*	else if (j>0&&find(node_nec.begin(), node_nec.end(), ant[i].place) != node_nec.end()) {
                  //  if(iter>node_nec_num*0.75&&TestTargetInBranch(ant[i].place,ant[i].path)==false)
                  //      break;
					if (ShortestPathToNodeNecMp[ant[i].place].bestLength>ant[i].TotalLength) {
						ShortestPathToNodeNecMp[ant[i].place].bestLength = ant[i].TotalLength;
						ShortestPathToNodeNecMp[ant[i].place].bestPathNow = ant[i].path;
						ShortestPathToNodeNecMp[ant[i].place].nodeNecHave = ant[i].nodeNecHave;
					}
					ShortestPathToNodeNecMp[ant[i].place].PathCanGetThere.push_back(ant[i].path);
					ShortestPathToNodeNecMp[ant[i].place].PathLengthList.push_back(ant[i].TotalLength);
					break;
				}
*/

				//获得该蚂蚁下一步可到达的点
				vector<int> nextAllow;
				//获得第i只蚂蚁可以走的下一个节点，排除掉已经走过的节点
				for (int q = 0; q<rMp[ant[i].place].size(); q++) {
					if (find(ant[i].path.begin(), ant[i].path.end(), rMp[ant[i].place][q]) == ant[i].path.end())
							nextAllow.push_back(rMp[ant[i].place][q]);
				}
				//第i只蚂蚁没有要走的下一个节点，则停止第i只蚂蚁的路径搜索
				if (nextAllow.size() == 0) {
					break;

				}
				//第i只蚂蚁有可以选择走的下一个节点
				if (nextAllow.size() > 0) {
					//计算去每一个城市的概率,在这里是将每个可以通过的下一个节点均等看待的，
					//可以适当增大通过必经节点的概率
					vector<double> P;
					double Pmax = 0.0;
					for (int q = 0; q < nextAllow.size(); q++) {
						double tempP = 1.0/*10.0 / road_cost[ant[i].place][nextAllow[q]]tau[nextAllow[q]] pow(10.0 / road_cost[ant[i].place][nextAllow[q]], beta)*/;
						if (find(node_nec.begin(), node_nec.end(), nextAllow[q]) != node_nec.end()) {
							ant[i].path.push_back(nextAllow[q]);

                            if (ShortestPathToNodeNecMp[nextAllow[q]].bestLength>ant[i].TotalLength+road_cost[nextAllow[q]][ant[i].place]) {
                              //如果从nextAllow[q]开始宽度优先遍历看能否到达target，并且和path在一起是否包括所有的必经节点 
							   if(RevTestTargetInBranch(nextAllow[q],ant[i].path)==true){
                                    ShortestPathToNodeNecMp[nextAllow[q]].bestLength = ant[i].TotalLength+road_cost[nextAllow[q]][ant[i].place];
                                    ShortestPathToNodeNecMp[nextAllow[q]].bestPathNow = ant[i].path;
                                    ShortestPathToNodeNecMp[nextAllow[q]].nodeNecHave = ant[i].nodeNecHave+1;
                                }
                            }
                            else if(ShortestPathToNodeNecMp[nextAllow[q]].bestLength+50>ant[i].TotalLength+road_cost[nextAllow[q]][ant[i].place]){
                                ShortestPathToNodeNecMp[nextAllow[q]].PathCanGetThere.push_back(ant[i].path);
                                ShortestPathToNodeNecMp[nextAllow[q]].PathLengthList.push_back(ant[i].TotalLength+road_cost[nextAllow[q]][ant[i].place]);
                            }

                            if(ShortestPathToNodeNecMp[nextAllow[q]].bestPathSize>ant[i].path.size()){
                             //   if(ant[i].path!=ShortestPathToNodeNecMp[nextAllow[q]].bestPathNow){
                                    if(RevTestTargetInBranch(nextAllow[q],ant[i].path)==true){
                                        ShortestPathToNodeNecMp[nextAllow[q]].bestPathSizeLength = ant[i].TotalLength+road_cost[nextAllow[q]][ant[i].place];
                                        ShortestPathToNodeNecMp[nextAllow[q]].bestPathSizePath = ant[i].path;
                                        ShortestPathToNodeNecMp[nextAllow[q]].bestPathSizeNodeNecHave = ant[i].nodeNecHave+1;
                                        ShortestPathToNodeNecMp[nextAllow[q]].bestPathSize=ant[i].path.size();
                                    }
                               // }
                            }
                            tempP = 0;

							ant[i].path.pop_back();
						}
						else if (nextAllow[q] == targetPoint) {
							if (ant[i].nodeNecHave == node_nec.size())
								tempP *= 15000;
							else
								tempP *= 0.1;
						}
						else if(rMp[nextAllow[q]].size()==0){
                            tempP = 0;
						}
						if (q == 0)
							P.push_back(tempP);
						else
							P.push_back(P[P.size() - 1] + tempP);
					}
                    if(P[P.size() - 1]==0)
                        break;
					//归一化
					for (int q = 0; q < P.size(); q++)
						P[q] /= P[P.size() - 1];

					//轮盘赌选择下一个要去的城市
#define RAND 999 //三位小数。
					//srand(time(NULL));//设置随机数种子，使每次获取的随机序列不同。
					double randNum = rand() % (RAND + 1) / (float)(RAND + 1);//生成0-1间的随机数。

					int choice = 0;
					for (int q = 0; q<P.size(); q++) {

						if (P[q] > randNum) {
							choice = q;
							break;
						}
					}
					//前往下一个城市
					ant[i].path.push_back(nextAllow[choice]);
					ant[i].TotalLength += road_cost[nextAllow[choice]][ant[i].place];
					ant[i].place = nextAllow[choice];
					if (find(node_nec.begin(), node_nec.end(), nextAllow[choice]) != node_nec.end()) {
						ant[i].nodeNecHave++;
					}

				}
			}
		}


	}
	return 0;
}

//检查从path开始从index点到达target并且包含所有的必经节点
bool RevTestTargetInBranch(int index,vector<int> path) {
	queue<int> q;
	int alreadyPush[601] = { 0 };
	q.push(index);
	alreadyPush[index] = 1;
	for (int i = 0; i < path.size(); i++)
		alreadyPush[path[i]] = 1;
	while(!q.empty()) {
		for (int i = 0; i < rMp[q.front()].size(); i++) {
			if (alreadyPush[rMp[q.front()][i]] == 0) {
				q.push(rMp[q.front()][i]);
				alreadyPush[rMp[q.front()][i]] = 1;
			}
		}
		q.pop();
	}
	if(alreadyPush[st]==0)
        return false;
    for(int i=0;i<node_nec.size();i++){
        if(alreadyPush[node_nec[i]]==0)
            return false;
    }
    return true;
}

