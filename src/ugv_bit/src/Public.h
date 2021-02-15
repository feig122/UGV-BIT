#ifndef PUBLIC_H
#define PUBLIC_H

#include <ros/ros.h>
#include <sys/time.h>
#include <vector>

using namespace std;

class TTimer
{
private:
    struct timeval tv1, tv2;
    struct timezone tz;
    float t_off;
public:
    TTimer();
    void Clear(float t=0);
    double GetValue(void);
};

TTimer::TTimer()
{
   Clear();
}

void TTimer::Clear(float t)
{
    gettimeofday(&tv1, &tz);
    tv2=tv1;
    t_off=t;
}//设置时间起点

double TTimer::GetValue(void)
{
    double ret;
    gettimeofday(&tv2, &tz);
    ret = (tv2.tv_sec - tv1.tv_sec) + (tv2.tv_usec - tv1.tv_usec)* 0.000001+t_off;
    return ret;
}//获取执行时间

class TDataFilter
{
private:
    double buf[200];
public:
    int filternum;
    double value;

    TDataFilter(int n)
    {
        filternum=n;
        for(int i=0;i<200;i++) buf[i]=0;
    }

    float GetValue(double v)
    {
        int i;
        for(i=0;i<filternum-1;i++) buf[i]=buf[i+1];
        buf[filternum-1]=v;
        value=0;
        for(i=0;i<filternum;i++) value=value+buf[i];
        value=value/filternum;
        return value;
    }
};



int FindMinID(vector<float> datalist)
{
    if(datalist.size()==0) return -1;

    vector<float>::iterator itMin = min_element(datalist.begin(), datalist.end());
    return distance(datalist.begin(), itMin);
}



#endif 