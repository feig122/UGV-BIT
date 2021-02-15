	#include <ros/ros.h>
	#include <std_msgs/Char.h>
	#include <std_msgs/Int32.h>
	#include <stdio.h>
	#include <termios.h>
	#include <fcntl.h>
	#include "ugv_bit/motor_ctr.h"

	int kbhit(void)
	{
	    struct termios oldt, newt;
	    int ch;
	    int oldf;
	    tcgetattr(STDIN_FILENO, &oldt);
	    newt = oldt;
	    newt.c_lflag &= ~(ICANON | ECHO);
	    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
	    ch = getchar();
	    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	    fcntl(STDIN_FILENO, F_SETFL, oldf);
	    if(ch != EOF)
	    {
		ungetc(ch, stdin);
		return 1;
	    }
	    return 0;
	}

	int getch()
	{
	    struct termios oldt,newt;
	    int ch;
	    tcgetattr( STDIN_FILENO, &oldt );
	    newt = oldt;
	    newt.c_lflag &= ~( ICANON | ECHO );
	    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	    ch = getchar();
	    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	    return ch;
	}


	int main(int argc,char **argv)
	{
	    
	    ros::init(argc, argv, "key_op");                              //解析参数，命名节点
	    ros::NodeHandle nh;                                               //创建句柄，实例化node
	    ros::Publisher pub1 = nh.advertise<ugv_bit::motor_ctr>("Motor_Ctr", 1);//创建publisher
		ros::Publisher pub2 = nh.advertise<std_msgs::Int32>("path_record", 10);//创建publisher

        int angle_count=0;
        ugv_bit::motor_ctr mctr;
		mctr.enabled=true;
		mctr.mode=1;
		mctr.speed=mctr.angle=0;

	    ros::Rate loop_rate(100);         //循环频率
		std_msgs::Int32 msg;
		while(ros::ok())
	    {
            //pub.publish(msg);

			if(kbhit())
			{
		        char c=getch();

				if(c=='1')  
				{
					msg.data=1;
					pub2.publish(msg);
					//printf("wsk\n");
				}
				else if(c=='0')  
				{
					std_msgs::Int32 msg;
					msg.data=0;
					pub2.publish(msg);
				}
				
                int step=100;
				bool flag=false;
				if(c==65)  mctr.speed+=step,  flag=true;
				else if(c==66)  mctr.speed-=2*step,  flag=true; 
				// else if(c==67)  mctr.angle+=step,  angle_count=0,  flag=true;
				// else if(c==68)  mctr.angle-=step,  angle_count=0,  flag=true;
				// else if(c=='1')  mctr.mode=1,  flag=true;  
                // else if(c=='2')  mctr.mode=-1,  flag=true;

				if(flag)
				{
					if(mctr.speed>3200) mctr.speed=3200;
					else if(mctr.speed<0) mctr.speed=0;
                    if(mctr.angle>1200) mctr.angle=1200;
					else if(mctr.angle<-1200) mctr.angle=-1200;

					pub1.publish(mctr);
				}
        
			}
		    loop_rate.sleep();
	    }
	    return 0;

	}