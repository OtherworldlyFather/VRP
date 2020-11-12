#include<stdio.h>
#include<time.h>
#include<algorithm>
#include<map>
#include<vector>
using namespace std;
struct task_edge//��������ߵ����˵�
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
struct task_info//��������ߵ���Ӧ��Ϣ
{
    int cost, demand;
    bool serviced;
    task_info(int c, int d, bool s): cost(c), demand(d), serviced(s) {};
    task_info() {};
};
int ub, vern, ern, enrn, vehn, capa, tc, depot;

vector<vector<int> > cost;//�����������·������
map<task_edge, task_info> task;//����ߵ����Ӧ��Ϣ��ӳ���
void load_data(const char* file_addr)
{
    FILE *fp = fopen(file_addr, "r");
    if(fp == NULL)
    {
        printf("Can't find the file.");
        return ;
    }
    //����������Ϣ�Ķ���
    char row[100];
    fgets(row, 100, fp);
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &ub);//upper bound ����·�����ȵ��Ͻ�
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &vern);//vertex number �����Ŀ
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &ern);//edge required number �������Ŀ
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &enrn);//edge non-required number ���������Ŀ
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &vehn);//vehicle number ������
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &capa);//capacity ��������
    fgets(row, 100, fp);
    fgets(row, 100, fp);
    sscanf(row, "%*[^0-9]%d", &tc);//total cost ����ߵĿ����ܺ�
    fgets(row, 100, fp);
    vector<int> temp;
    for(int i = 0; i <= vern; i++) temp.push_back(0);
    for(int i = 0; i <= vern; i++) cost.push_back(temp);//������Ӧ��С��·�����ȱ�
    for(int i = 0; i <= vern; i++)//��ʼ��·�����ȣ���ͬ��ľ�����Ϊ�㣬��ͬ��ľ�����Ϊ�����
    {
        for(int j = 0; j <= vern; j++)
            cost[i][j] = ub * vern;
        cost[i][i] = 0;
    }
    task.clear();
    for(int t = 1; t <= ern; t++)//�������Ϣ�Ķ���
    {
        int i, j, c, d;
        fgets(row, 100, fp);
        sscanf(row, "%*[^0-9]%d%*[^0-9]%d%*[^0-9]%d%*[^0-9]%d", &i, &j, &c, &d);
        cost[i][j] = cost[j][i] = c;
        task[task_edge(i, j)] = task[task_edge(j, i)] = task_info(c, d, false);
    }
    if(enrn)//���������Ϣ�Ķ���
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

vector<task_edge> construct_giant_tour()//����һ����·������������
{
    for(int k = 1; k <= vern; k++)//Floyd�㷨��������·��
        for(int i = 1; i <= vern; i++)
            for(int j = 1; j <= vern; j++)
                if(cost[i][j] > cost[i][k] + cost[k][j])
                    cost[i][j] = cost[i][k] + cost[k][j];
    vector<task_edge> tour;
    for(int current = depot, load = 0; tour.size() < ern;)//����������߷��������·��
    {
        int mincost = ub, maxcost = -1;
        vector<task_edge> nearest;//�뵱ǰ�������Ľ�㼯�ϣ���һ������ѡȡ��Դ
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
        if(rc == 5) rc = load % capa > capa / 2 ? 1:2;//����5�����ݵ�ǰ�����صĴ�Сѡ�����1��2
        switch (rc)
        {
        case 1://����1��ѡȡ�복վ����Ľ��
            for(int i = 0, m = ub;i < nearest.size(); i++)
                if(cost[nearest[i].j][depot] < m)
                {
                    m = cost[nearest[i].j][depot];
                    next = nearest[i];
                }
            break;
        case 2://����2��ѡȡ�복վ��Զ�Ľ��
            for(int i = 0, m = -1;i < nearest.size(); i++)
                if(cost[nearest[i].j][depot] > m)
                {
                    m = cost[nearest[i].j][depot];
                    next = nearest[i];
                }
            break;
        case 3://����3��ѡȡ���������Ľ��
            for(int i = 0, m = -1;i < nearest.size(); i++)
                if(task[nearest[i]].demand > m)
                {
                    m = task[nearest[i]].demand;
                    next = nearest[i];
                }
            break;
        case 4://����4��ѡȡ��������С�Ľ��
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

vector<vector<task_edge> > split_and_output(vector<task_edge> tour)//�Դ�·������λ���Ż����зֽ�
{//�˲��ֵĴ���������ע���������
    int v[ern + 1], p[ern + 1], r[ern + 1];
    /*v�����ߵ��˽�����С������·�����ȣ�
        p����˽�������ǰ�����
        r����˽����������ǰ����㹹�ɵ�С·��������λ����ʼ��*/
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
    for(int i, j = ern; i > 0; j = i)//����֮ǰ�õ���������������С·������������ս�
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
    for(int i = trips.size() - 1; i >= 0; i--)//������
    {
        printf("%d.", trips.size() - i);
        for(int j = 0; j < trips[i].size(); j++) printf("(%d, %d) ", trips[i][j].i, trips[i][j].j);
        printf("\n");
    }
    return 0;
}
