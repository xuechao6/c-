#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <vector>


struct UAV
{
    int id;
    int arrive_point;
    double k; //所走的路程和整段路程的占比
    double lon;
    double lat;
    double alt;
    double vel;
};
struct point
{
    double lon;
    double lat;
    double alt;
    double v;
};
double k_zhuji;
UAV uav_self;
UAV uav_zhuji;
point now;
std::vector<point> points;
//角度转化为弧度
double toRadians(double degree) {
    return degree * (M_PI/ 180.0);
}
//计算两点距离
double dis(const point& p1, const point& p2) {
    // 将角度转换为弧度
    double lat1 = toRadians(p1.lat);
    double lon1 = toRadians(p1.lon);
    double lat2 = toRadians(p2.lat);
    double lon2 = toRadians(p2.lon);
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    // 使用Haversine公式计算大距离
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    double distance = 6371000 * c;
    // 考虑高度差
    double alt_diff = p2.alt - p1.alt;
    double distance_ = std::sqrt(distance * distance + alt_diff * alt_diff);
    return distance_;
}

void control(std::vector<point> points)
{
    if(!points.empty())
    {
        for(int i=0; i<points.size()-1; i++)
        {
            bool congji_is_arrive = 0;
            point last = points[i];        //上一个出发点的位置
            point expect = points[i+1];    //期望点的位置
            double dis_last_expect = dis(last,expect); //计算已知的两个点的距离
            while(1)
            {
                //1.避免出现主机先到，从机未到后，主机的k和从机的k不一致
                if(uav_zhuji.arrive_point == i) 
                {
                    double dis_now_last = dis(now,last);
                    double k_self = dis_now_last/dis_last_expect;       //根据目前的位置，计算所走过的比例
                    double error = (k_zhuji - k_self)*dis_last_expect;  //根据走过的比例和总距离，计算距离误差
                    double pid = 0.5*error;                             //只有比例项，p控制
                    //限制幅度
                    if(pid>10){
                        pid = 10;}
                    if(pid<-10){
                        pid=-10;}
                    uav_self.vel +=pid; //输出从机的速度
                }

                //2.主机还没到，从机已经到了的情况。从机速度为0，等待主机，主机到达后跳出while循环，前往下一点
                double dis_now_expect = dis(now,expect);  
                if(dis_now_expect < 1) //如果从机当前点和期望点之间的距离小于1m
                {
                    congji_is_arrive = 1; //考虑惯性，赋予标志位到达期望点
                } 
                if(congji_is_arrive ==1)
                {
                    if(uav_zhuji.arrive_point == i+1) //判断主机是否到达期望点，如果到达期望点，跳出循环
                    {
                        break;
                    }
                    else
                    {
                        uav_self.vel = 0;
                    }
                }
            }
        }
    }
}
// 1.如果主机已经到达，发过来的主机k和现在从机的k不一致
// 如果主机没到，从机到了？
// 2.如果主机到了第二个点，需要发送标志位，从机切换第二个点的条件
// 3.输出只需要输出速度吗，还是需要输出目标点

//回调函数
// void callback_point()
// {
//     points.push_back({1.0,1.0,1.0,1.0});
//     points.push_back({2.0,2.0,2.0,2.0});
//     points.push_back({3.0,3.0,3.0,3.0});
// }
int main()
{
    points.push_back({1.0,1.0,1.0,1.0});
    points.push_back({2.0,2.0,2.0,2.0});
    points.push_back({3.0,3.0,3.0,3.0});

    //todo
    //1.订阅主机发布的消息，消息包括：速度、考虑通信延迟的k、arrive_point
    //2.从机发布速度指令，是否需要发布下一个期望点的位置？
    return 0;
}