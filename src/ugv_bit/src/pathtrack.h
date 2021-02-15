#include <vector>
#include "Public.h"

using namespace std;

#define Rad2Deg 180/3.1415926
#define Deg2Rad 3.1415926/180
#define pi 3.1415926

struct _Point
{
    double x,y;
};

struct _Spcs
{
    float r, a;
};

struct Position_Err  // 航向误差， 横向误差，纵向误差
{
    float dx, dy;
    float angle;
};


class TPathTrack
{
private:
      
    int work_state;
    bool enabled;

    bool CheckPoint(_Point p)
    {
        if(fabs(p.x)+fabs(p.y)>10) return true;
        return false;
    }

public:
    TDataFilter *df_turnangle, *df_speed;
    _Point cur_pos;
    float angle2north;
    int speed, turn_angle;
    Position_Err pos_err;
    int aim_range;    //  瞄准距离
    float fine_error;   //  定位精度

    _Point track_p, target_p;
    double target_distance;
    //Position_Err pos_err;   //  航向误差， 横向误差，纵向误差

    bool record_track_flag;
    vector<_Point> track_points, record_track_points; 
    
    TTimer tmr_work;
    TPathTrack();
    _Point Coordinate_Fix2Mov(_Point fix_p);
    _Point Coordinate_Mov2Fix(_Point mov_p);

    double sat(double s);
    int CheckTargetShot();
    void CheckTrackPoint();
    void SaveTrack(char* filename);
    void LoadTrack(char* filename);
    void Control(_Point* p);
    void Stop();

    void run();
    
    int TrackPoint_ID, TargetPoint_ID, Track_Direction;
    bool cycle_flag;
    int max_speed, min_speed, max_angle;
    
    int FindNearestTracePointID(_Point p);   //  获取最近轨迹点
    _Point FindNearestTargetInTrace(_Point p);    //  在轨迹上搜索最近的炮点
    int FindTrackDirection();  //  寻找轨迹方向　　
    void FindTrackPointDirection();
    bool CheckCycle(float d);   //  判断是否构成闭环
    void InterpolateTrackPoints();   //  直线差值轨迹点
    void FitTrackToTrack();   //  轨迹去炮点
    void UpdateTurnAngle(_Point* p);
};

// double P2P_r(_Point p1,_Point p2);
// _Spcs P2P(_Point p1,_Point p2);
// _Spcs P2P(_Point p);

