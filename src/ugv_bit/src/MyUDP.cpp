#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <vector>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include "Public.h"
#include <std_msgs/String.h>
#include "ugv_bit/motor_state.h"   
#include "ugv_bit/motor_ctr.h"   
#include "ugv_bit/gps_data.h"   
#include "visualization_msgs/Marker.h"
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

using namespace std;

//TTimer timer_udp_connect[6];  //自定义时间类：用于获取UDP连接失败的时间

const float Pi=3.14159;

int  sock;
char UDP_recvbuf[1024] = {0};
int  UDP_recvlen=0;

ros::Publisher motor_state;
ros::Publisher gps_data;
ros::Publisher marker_pub;
ros::Publisher path_pub;

ros::Subscriber motor_ctr;
 
double base_x=0, base_y=0;

#define ERR_EXIT(m) \
    do { \
    perror(m); \
    exit(EXIT_FAILURE); \
    } while (0)

//函数作用：将字符串按照特定字符分割成片段，
//s：字符串指针，sep:字符串中的分隔字符
vector<string> split(const string& s, const string& sep)
{
    vector<string> v;
    string::size_type pos1, pos2;
    pos2 = s.find(sep);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));

        pos1 = pos2 + sep.size();
        pos2 = s.find(sep, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
    return v;
}

nav_msgs::Path path;
void PubCarPath(double x,double y)
{
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1;

    pose.header.stamp=ros::Time::now();
    pose.header.frame_id="/odom";

    if(path.poses.size()>1000)  path.poses.erase(path.poses.begin());

    if(path.poses.size()==0)  path.poses.push_back(pose);
    else
    {
        geometry_msgs::PoseStamped p0=path.poses.back();
        float ds=pow(p0.pose.position.x-pose.pose.position.x, 2)+pow(p0.pose.position.y-pose.pose.position.y, 2);
        if(ds>=0.001) ds=sqrt(ds);
        if(ds>0.4) path.poses.push_back(pose);  
    }
		
    path.header.stamp=ros::Time::now();
    path.header.frame_id="/odom";
    path_pub.publish(path);
}
        
void PubCarPosByMarker(double x,double y,double th)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;

	geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(th);
    marker.pose.orientation.x = q.x;
    marker.pose.orientation.y = q.y;
    marker.pose.orientation.z = q.z;
    marker.pose.orientation.w = q.w;

    marker.scale.x = 1.2;
    marker.scale.y = 0.7;
    marker.scale.z = 0.4;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_pub.publish(marker);
}

//解析数据包：从数据包中读出各部分信息
void GetUGVState(char *recbuf)
{
   vector<string> strs,substrs;
   
   strs=split(recbuf,"\n");
   for(int i=0;i<strs.size();i++)
   {
       substrs=split(strs.at(i)," ");
       if(substrs.size()>=32 && substrs.at(0)=="$Motor")  
       {
            ugv_bit::motor_state msg;

            msg.mode=atoi(substrs.at(1).c_str());
            msg.speed=atoi(substrs.at(2).c_str());
            msg.angle=atoi(substrs.at(3).c_str());
            for(int j=0;j<4;j++)
            {
                msg.enabled[j]=atoi(substrs.at(4+7*j).c_str());
                msg.WNvalue[j]=atoi(substrs.at(5+7*j).c_str());
                msg.DIvalue[j]=atoi(substrs.at(6+7*j).c_str());
                msg.DOvalue[j]=atoi(substrs.at(7+7*j).c_str());
                msg.bv[j]=atof(substrs.at(8+7*j).c_str());
                msg.iq[j]=atof(substrs.at(9+7*j).c_str());
                msg.spd[j]=atof(substrs.at(10+7*j).c_str());
            }

            motor_state.publish(msg);
       }
       else if(substrs.size()>=9 && substrs.at(0)=="$GPS" && substrs.at(1)=="B")  
       {
            ugv_bit::gps_data msg;

            msg.quality = atoi(substrs.at(2).c_str());
            msg.lat = atof(substrs.at(3).c_str());
            msg.lon = atof(substrs.at(4).c_str());
            msg.utm_x = atof(substrs.at(5).c_str());
            msg.utm_y = atof(substrs.at(6).c_str());
            msg.angle2north = atof(substrs.at(7).c_str());
            msg.speed = atof(substrs.at(8).c_str());

            gps_data.publish(msg);

            if(fabs(base_x+base_y)<10)  base_x=msg.utm_x, base_y=msg.utm_y;
            float x=msg.utm_x-base_x, y=msg.utm_y-base_y;
            
            printf("%.2f %.2f  \n ", x, y);
            
            PubCarPosByMarker(y, x, msg.angle2north/180*Pi);
            PubCarPath(y, x);

       }
   }
}

/*线程：同过UDP读取底层数据*/
void *echo_ser(void *arg)
{
    int len;
    char recvbuf[1024] = {0};
    struct sockaddr_in peeraddr;
    socklen_t peerlen;
    while (1)
    {
        peerlen = sizeof(peeraddr);
        memset(recvbuf, 0, sizeof(recvbuf));
        len = recvfrom(sock, recvbuf, sizeof(recvbuf), 0, (struct sockaddr *)&peeraddr, &peerlen);
        string ip=inet_ntoa(peeraddr.sin_addr);   

        if(ip=="192.168.3.103")  GetUGVState(recvbuf);
               
        if (len <= 0)
        {
            if (errno == EINTR)
                continue;
            ERR_EXIT("recvfrom error");
        }
    }
    close(sock);
    pthread_exit(NULL);
}


void UDP_Init(int port)
{
    if ((sock = socket(PF_INET, SOCK_DGRAM, 0)) < 0)   ERR_EXIT("socket error");

 
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    printf("监听%d端口\n",port);
    if (bind(sock, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
        ERR_EXIT("bind error");

    pthread_t thread;
    int res=pthread_create(&thread,NULL,echo_ser,NULL);//创建一个线程，用于发布话题
    if(res!=0)
    {
        printf("Create thread failed\n");
        exit(res);
    }
  
}

void Close_UDP(void)
{
    close(sock);
}

void UDP_Send(char *ip, int port, char *str)
{
    struct sockaddr_in sendaddr;
	
    sendaddr.sin_family = AF_INET;
    sendaddr.sin_port = htons(port);
    sendaddr.sin_addr.s_addr = inet_addr(ip);
	sendto(sock, str, strlen(str), 0, (struct sockaddr *)&sendaddr, sizeof(struct sockaddr_in));
}

void MyCallback(const ugv_bit::motor_ctr::ConstPtr& msg)
{
    char buf[1000];
    sprintf(buf, "$Motor B %d %d %d", msg->mode, msg->speed, msg->angle);
    
    //printf("%s\n",buf);
    UDP_Send("192.168.3.103", 8080, buf);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "MyUDP");  //初始化节点
	ros::NodeHandle n;
   
    motor_state = n.advertise<ugv_bit::motor_state>("Motor_State",10);
    gps_data = n.advertise<ugv_bit::gps_data>("GPS_Data",10);
    motor_ctr = n.subscribe("Motor_Ctr", 200, MyCallback);
    
    path_pub = n.advertise<nav_msgs::Path>("car_path",1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("car_marker",1);

    ros::Rate loop_rate(100);   //定义程序循环的频率
	UDP_Init(8090);  //初始化UDP
    int count;
	while (ros::ok())
	{
        //PubCurPosByMarker(0,0,0);

        //count++;
        //PubCarPath(count*0.1, count*0.1);


        ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
