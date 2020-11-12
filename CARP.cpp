#include<stdio.h>
#include<time.h>
#include<algorithm>
#include<map>
#include<vector>
using namespace std;
struct task_edge//储存任务边的两端点
{
    int i, j;
    friend operator < (const task_edge &a, const task_edge &b)
    {
        if(a.i == b.i)
            return a.j < b.j;
        return a.i < b.i;
    }
    task_edge(int _i, int _j): i(_i), j(_j) {};
    task_edge() {};
};
struct task_info//储存任务边的相应信息
{
    int cost, demand;
    bool serviced;
    task_info(int c, int d, bool s): cost(c), demand(d), serviced(s) {};
    task_info() {};
};
int ub, vern, ern, enrn, vehn, capa, tc, depot;

vector<vector<int> > cost;//储存结点间的最短路径长度
map<task_edge, task_info> task;//任务边到其对应信息的映射表
void load_data(const char* file_addr)
{
    FILE *fp = fopen(file_addr, "r");
    if(fp == NULL)
    {
        printf("Can't find the file.");
        return ;
    }
    //若干整体信息的读入
    char row[100];
    fgets(row, 100, fp);
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &ub);//upper bound 结点间路径长度的上界
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &vern);//vertex number 结点数目
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &ern);//edge required number 任务边数目
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &enrn);//edge non-required number 非任务边数目
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &vehn);//vehicle number 车辆数
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &capa);//capacity 车辆容量
    fgets(row, 100, fp);
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &tc);//total cost 任务边的开销总和
    fgets(row, 100, fp);
    vector<int> temp;
    for(int i = 0; i <= vern; i++) temp.push_back(0);
    for(int i = 0; i <= vern; i++) cost.push_back(temp);//创建相应大小的路径长度表
    for(int i = 0; i <= vern; i++)//初始化路径长度，相同点的距离设为零，不同点的距离设为无穷大
    {
        for(int j = 0; j <= vern; j++)
            cost[i][j] = ub * vern;
        cost[i][i] = 0;
    }
    task.clear();
    for(int t = 1; t <= ern; t++)//任务边信息的读入
    {
        int i, j, c, d;
        fgets(row, 100, fp);
        sscanf(row, "%*[^0-9]%d%*[^0-9]%d%*[^0-9]%d%*[^0-9]%d", &i, &j, &c, &d);
        cost[i][j] = cost[j][i] = c;
        task[task_edge(i, j)] = task[task_edge(j, i)] = task_info(c, d, false);
    }
    if(enrn)//非任务边信息的读入
    {
        fgets(row, 100, fp);
        for(int t = 0; t < enrn; t++)
        {
            int i, j, c;
            fgets(row, 100, fp);
            sscanf(row, "%*[^0-9]%d%*[^0-9]%d%*[^0-9]%d", &i, &j, &c);
            cost[i][j] = cost[j][i] = c;
        }
    }
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &depot);
    fclose(fp);
}

vector<task_edge> construct_giant_tour()//构造一个大路径，并返回它
{
    for(int k = 1; k <= vern; k++)//Floyd算法，求解最短路径
        for(int i = 1; i <= vern; i++)
            for(int j = 1; j <= vern; j++)
                if(cost[i][j] > cost[i][k] + cost[k][j])
                    cost[i][j] = cost[i][k] + cost[k][j];
    vector<task_edge> tour;
    for(int current = depot, load = 0; tour.size() < ern;)//采用随机决策方法构造大路径
    {
        int mincost = ub, maxcost = -1;
        vector<task_edge> nearest;//离当前结点最近的结点集合，下一个结点的选取来源
        for(map<task_edge, task_info>::iterator it = task.begin(); it != task.end(); it++)
        {
            if(cost[current][it->first.i] < mincost && it->second.serviced == false)
                mincost = cost[current][it->first.i];
            if(cost[current][it->first.i] > maxcost && it->second.serviced == false)
                maxcost = cost[current][it->first.i];
        }
        for(map<task_edge, task_info>::iterator it = task.begin(); it != task.end(); it++)
            if(cost[current][it->first.i] <= mincost + 0.1 * (maxcost - mincost) && it->second.serviced == false)
                nearest.push_back(it->first);
        task_edge next;
        srand((unsigned) time(NULL));
        int rc = rand() % 5 + 1;
        if(rc == 5) rc = load % capa > capa / 2 ? 1:2;//决策5：根据当前已载重的大小选择决策1或2
        switch (rc)
        {
        case 1://决策1：选取离车站最近的结点
            for(int i = 0, m = ub;i < nearest.size(); i++)
                if(cost[nearest[i].j][depot] < m)
                {
                    m = cost[nearest[i].j][depot];
                    next = nearest[i];
                }
            break;
        case 2://决策2：选取离车站最远的结点
            for(int i = 0, m = -1;i < nearest.size(); i++)
                if(cost[nearest[i].j][depot] > m)
                {
                    m = cost[nearest[i].j][depot];
                    next = nearest[i];
                }
            break;
        case 3://决策3：选取需求量最大的结点
            for(int i = 0, m = -1;i < nearest.size(); i++)
                if(task[nearest[i]].demand > m)
                {
                    m = task[nearest[i]].demand;
                    next = nearest[i];
                }
            break;
        case 4://决策4：选取需求量最小的结点
            for(int i = 0, m = capa;i < nearest.size(); i++)
                if(task[nearest[i]].demand < m)
                {
                    m = task[nearest[i]].demand;
                    next = nearest[i];
                }
            break;
        }
        tour.push_back(next);
        load += task[next].demand;
        task[next].serviced = task[task_edge(next.j, next.i)].serviced = true;
    }
    return tour;
}

vector<vector<task_edge> > split_and_output(vector<task_edge> tour)//对大路径采用位移优化进行分解
{//此部分的代码难以用注释描述清楚
    int v[ern + 1], p[ern + 1], r[ern + 1];
    /*v代表走到此结点的最小开销（路径长度）
        p代表此结点的最优前驱结点
        r代表此结点与其最优前驱结点构成的小路径的最优位移起始点*/
    v[0] = 0, p[0] = 0;
    for(int i = 1; i <= ern; i++) v[i] = ub * ern;
    for(int i = 1; i <= ern; i++)
    {
        for(int j = i, load = 0, co, c1, f, cf, best;j <= ern; j++)
        {
            load += task[tour[j - 1]].demand;
            if(load > capa) break;
            if(i == j)
            {
                co = cost[depot][tour[i - 1].i] + task[tour[j - 1]].cost + cost[tour[j - 1].j][depot];
                best = i;
            }
            else if(j == i + 1)
            {
                c1 = co - cost[tour[j - 2].j][depot] + cost[tour[j - 2].j][tour[j - 1].i] + task[tour[j - 1]].cost + cost[tour[j - 1].j][depot];
                int c2 = cost[depot][tour[j - 1].i] + task[tour[j - 1]].cost + cost[tour[j - 1].j][tour[i - 1].i] + task[tour[i - 1]].cost + cost[tour[i - 1].j][depot];
                co = min(c1, c2);
                best = c1 < c2 ? i : j;
                f = j;
                cf = c2;
            }
            else
            {
                int c2 = c1 - cost[depot][tour[i - 1].i] + cost[depot][tour[j - 1].i] + task[tour[j - 1]].cost + cost[tour[j - 1].j][tour[i - 1].i];
                c1 += -cost[tour[j - 2].j][depot] + cost[tour[j - 2].j][tour[j - 1].i] + task[tour[j - 1]].cost + cost[tour[j - 1].j][depot];
                int c3 = cf - cost[tour[j - 2].j][tour[i - 1].i] + cost[tour[j - 2].j][tour[j - 1].i] + task[tour[j - 1]].cost + cost[tour[j - 1].j][tour[i - 1].i];
                if(c3 > c2) f = j;
                co = min(c1, min(c2, c3));
                if(co == c1) best = i;
                else if(co == c2) best = j;
                else best = f;
            }
            if(v[i - 1] + co < v[j])
            {
                v[j] = v[i - 1] + co;
                p[j] = i - 1;
                r[j] = best;
            }
        }
    }
    /*printf("\n");
    for(int i = 0; i <= ern; i++) printf("%d %d\n", v[i], p[i]);*/
    /*Trips*/
    vector<vector<task_edge> > trips;
    for(int i, j = ern; i > 0; j = i)//根据之前得到的数组数据生成小路径即问题的最终解
    {
        i = p[j];
        int e = r[j];
        vector<task_edge> temp;
        for(int k = e; k <= j; k++) temp.push_back(tour[k - 1]);
        for(int k = i + 1; k < e; k++) temp.push_back(tour[k - 1]);
        trips.push_back(temp);
    }
    return trips;
}
int main()
{
    //FILE *fp = fopen("instances\\CARP\\bccm\\val1A.dat", "r");
    load_data("instances\\CARP\\bccm\\val1A.dat");
    //printf("%d %d %d %d %d %d %d %d\n", ub, vern, ern, enrn, vehn, capa, tc, depot);
    /*Giant tour with random criterion*/
    vector<task_edge> tour = construct_giant_tour();
    /*printf("\n");
    for(int i = 0; i < tour.size(); i++) printf("(%d, %d)\n", tour[i].i, tour[i].j);*/
    /*Split with shifts*/
    vector<vector<task_edge> > trips = split_and_output(tour);
    for(int i = trips.size() - 1; i >= 0; i--)//结果输出
    {
        printf("%d.", trips.size() - i);
        for(int j = 0; j < trips[i].size(); j++) printf("(%d, %d) ", trips[i][j].i, trips[i][j].j);
        printf("\n");
    }
    return 0;
}
