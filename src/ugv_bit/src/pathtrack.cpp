#include "pathtrack.h"
#include "ugv_bit/gps_data.h"  
#include "std_msgs/Int32.h"

_Spcs P2P(_Point p1,_Point p2)
{
    _Spcs p;
    double x=p2.x-p1.x, y=p2.y-p1.y;

    p.r=x*x+y*y;
    if(fabs(p.r)<0.000001) p.r=0;
    else p.r=sqrt(p.r);

    if (fabs(y) < 0.0001)
    {
        if (x >= 0) p.a = pi / 2;
        else p.a = -pi / 2;
    }
    else if (y > 0) p.a = atan(x / y);
    //else if (x >= 0) p.a = pi - atan(x / y);
    else p.a = -pi + atan(x / y);     //  计算误差角度

    return p;
}

double P2P_r(_Point p1,_Point p2)
{
    _Spcs p=P2P(p1,p2);
    return p.r;
}

_Spcs P2P(_Point p)
{
    _Point p0={0,0};
    return P2P(p0,p);
}

TPathTrack::TPathTrack()
{
    speed=turn_angle=0;
    aim_range=3;      //  预先瞄准距离
    fine_error=1;     //  定位精度
    enabled= false;    
	record_track_flag=false;
    target_p=track_p={0, 0};
    work_state=0;
    df_turnangle=new TDataFilter(100);
    df_speed=new TDataFilter(100);
    max_speed=1200,  min_speed=200;
    max_angle=1200;
}

_Point TPathTrack::Coordinate_Fix2Mov(_Point fix_p)
{
    float s=sin(angle2north*Deg2Rad), c=cos(angle2north*Deg2Rad);
    _Point tmp_p, mov_p;
    tmp_p.x=fix_p.x-cur_pos.x;    tmp_p.y=fix_p.y-cur_pos.y;
    mov_p.x=c*tmp_p.x+s*tmp_p.y;
    mov_p.y=-s*tmp_p.x+c*tmp_p.y;
    return mov_p;
}

_Point TPathTrack::Coordinate_Mov2Fix(_Point mov_p)
{
    float s=sin(angle2north*Deg2Rad), c=cos(angle2north*Deg2Rad);

    _Point fix_p;
    fix_p.x=c*mov_p.x-s*mov_p.y+cur_pos.x;
    fix_p.y=s*mov_p.x+c*mov_p.y+cur_pos.y;
    return fix_p;
}

int TPathTrack::FindNearestTracePointID(_Point p) //  获取最近轨迹点
{
    vector<float> datalist;
    for(int i=0;i<track_points.size();i++)
        datalist.push_back(P2P_r(p, track_points.at(i)));

    int r=FindMinID(datalist);   //  获得距离最小ID
    if(r>=0 && datalist.at(r)>20) r=-1;    //  判断最小距离是否小
    return r;
}


int TPathTrack::FindTrackDirection()  //  寻找轨迹方向　　
{
    _Point near_p0, near_p1;
    if(TrackPoint_ID<=track_points.size()-2)
    {
        near_p0=Coordinate_Fix2Mov(track_points.at(TrackPoint_ID));
        near_p1=Coordinate_Fix2Mov(track_points.at(TrackPoint_ID+1));
        if(near_p1.y>near_p0.y) return 1;
        else return -1;
    }
    else
    {
        near_p0=Coordinate_Fix2Mov(track_points.at(TrackPoint_ID));
        near_p1=Coordinate_Fix2Mov(track_points.at(TrackPoint_ID-1));
        if(near_p1.y>near_p0.y) return -1;
        else return 1;
    }
}

void TPathTrack::FindTrackPointDirection()
{
    TrackPoint_ID=FindNearestTracePointID(cur_pos);
    if(TrackPoint_ID>=0)
    {
        Track_Direction=FindTrackDirection();
        cycle_flag=CheckCycle(5);    //  判断是否构成闭环
    }
    else Track_Direction=0;   //  表示无方向

        //printf("Track_ID=%d Direction=%d\n", TrackPoint_ID, Track_Direction);
}

void TPathTrack::CheckTrackPoint()  //  检测是否有可用轨迹
{
    track_p={0, 0};

    bool find_flag=false;
    FindTrackPointDirection();
    if(TrackPoint_ID<0 || track_points.size()<3) return;

    TrackPoint_ID+=Track_Direction*aim_range;
    if(TrackPoint_ID<0)
    {
        if(cycle_flag)  TrackPoint_ID=track_points.size()-1;
        else  TrackPoint_ID=0;
    }

    else if(TrackPoint_ID>=track_points.size())
    {
        if(cycle_flag)  TrackPoint_ID=0;
        else TrackPoint_ID=track_points.size()-1;
    }

    _Point mp=Coordinate_Fix2Mov(track_points.at(TrackPoint_ID));
    if(mp.y>0.3)  track_p=track_points.at(TrackPoint_ID);
    printf("%d\n", TrackPoint_ID);
}


void TPathTrack::Stop()
{
    speed=turn_angle=0;
}

bool TPathTrack::CheckCycle(float d)   //  判断是否构成闭环
{
    bool rt=false;
    if(track_points.size()<3) rt=false;

    float r=P2P_r(track_points.at(0), track_points.at(track_points.size()-1));
    rt=(r<d);
        //printf("cy=%d\n", rt);
    return rt;
}

void TPathTrack::UpdateTurnAngle(_Point* p)
{
    // _Point mp=Coordinate_Fix2Mov(*p);
    // _Spcs sp=P2P(mp);
    // error_angle=sp.a * Rad2Deg;   //  计算误差角度

    //turn_angle=Rad2Deg*atan(2.0*10*sin(Deg2Rad*pos_err.angle)/sp.r);
    float k=max_angle/90.0;
    turn_angle=pos_err.angle*k;

    //转角限位　
    if (turn_angle > max_angle) turn_angle = max_angle;
    else if (turn_angle < -max_angle) turn_angle = -max_angle;
}


void TPathTrack::Control(_Point* p)
{
    if(!CheckPoint(*p)) return;   //  不是有效点

    _Point mp=Coordinate_Fix2Mov(*p);
    _Spcs sp=P2P(mp);
    pos_err.angle=sp.a * Rad2Deg;   //  计算误差角度
    pos_err.dx=mp.x;
    pos_err.dy=mp.y;

    //纯跟踪算法求偏转角度
    UpdateTurnAngle(p);

    if(pos_err.dy>fine_error)   //  没有到达误差距离
    {
        float myangle=fabs(turn_angle);
        speed=max_speed-(max_speed-min_speed)*fabs(myangle)/max_angle;
        //  限制速度增加幅度
        //float old_speed=df_speed->value;
        //float dspeed=1;
        //if(speed-old_speed>dspeed) speed=old_speed+dspeed;
        if(speed<min_speed) speed=min_speed;
        else if(speed > max_speed) speed = max_speed;
        //  动态调整预瞄准距离

        aim_range=6-5.0*fabs(turn_angle)/max_angle;
        if(aim_range<1) aim_range=1;

        //aim_range=2;  //6*pow(e,-1*fabs(turn_angle/14))+6;

        //if(CheckTargetShot()>=2) speed=0;    //  到达炮点停车
        //else if(target_distance<6) speed=min_speed;    //  接近炮点要降速
    }
    else speed=turn_angle=0, *p={0, 0};    //  把跟踪点或炮点清掉

    speed=df_speed->GetValue(speed);
    turn_angle=df_turnangle->GetValue(turn_angle);//滤波处理
}

void TPathTrack::run()
{
    if(enabled)
    {
        CheckTrackPoint();
        //CheckTargetShot();  //  0 表示没有炮点，  1 表示 跟踪炮点，  2 表示炮点到位  3 经过不准确
        if(CheckPoint(track_p))  Control(&track_p);
        else Stop();
    }
    //else printf("not enable\n");

  if(record_track_flag && record_track_points.size()<10000)    //记录轨迹数据
  {
      if(record_track_points.empty())  record_track_points.push_back(cur_pos);
      else if(P2P_r(cur_pos,record_track_points.back())>1)  record_track_points.push_back(cur_pos);    
      
      //printf("%d %.2f %.2f\n", record_track_points.size(), cur_pos.x, cur_pos.y);  
  }
}

void TPathTrack::SaveTrack(char* filename)
{
    //if(record_track_points.size()==0)  return;

    FILE *fp=fopen(filename, "w");
    for(int i=0;i<record_track_points.size();i++)
    {
        fprintf(fp, "%.2f %.2f\n", record_track_points.at(i).x, record_track_points.at(i).y);
    }
    fclose(fp);

    ROS_INFO("I heard: [%d]", record_track_points.size());    
}

void TPathTrack::LoadTrack(char* filename)
{
    FILE *fp=fopen(filename, "r");
    _Point p;

    // for(int i=0;i<record_track_points.size();i++)
    // {
    //     fscanf(fp, "%f %f\n", p.x, p.y);

    // }
    // fclose(fp);
}


TPathTrack pathtrack;

void GPSDataCallback(const ugv_bit::gps_data::ConstPtr& msg)
{
    pathtrack.cur_pos.x=msg->utm_x;
    pathtrack.cur_pos.y=msg->utm_y;
    pathtrack.angle2north=msg->angle2north;
}

void PathRecordCallback(const std_msgs::Int32::ConstPtr& msg)
{
    //ROS_INFO("I heard: [%d]", msg->data);
    
    if(msg->data==1 && !pathtrack.record_track_flag)
    {
        pathtrack.record_track_flag=true;
        pathtrack.record_track_points.clear();
    } 
    else if(msg->data==0 && pathtrack.record_track_flag)
    {
        pathtrack.record_track_flag=false;
        pathtrack.SaveTrack("/home/wsk/abc.txt");
    }
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "pathtrack");
	ros::NodeHandle n;
	//ros::Publisher pub = n.advertise<std_msgs::Int32>("path_record", 10);
	ros::Subscriber sub1 = n.subscribe("path_record", 1000, PathRecordCallback);
    ros::Subscriber sub2 = n.subscribe("GPS_Data", 1000, GPSDataCallback);

	ros::Rate loop_rate(100);
	while (ros::ok())
	{
        pathtrack.run();

		ros::spinOnce();
        loop_rate.sleep();
	}

	return 0;
}