#include <ros/ros.h>
#include <uxa_sam_msgs/std_position_move.h>
#include <uxa_uic_msgs/motion.h>
#include <uxa_uic_msgs/remocon.h>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <stdio.h>
#include <iostream>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>

#include <math.h>




void send_msg(std::string STR);
void send_remocon(unsigned char remocon);
void send_std_position(unsigned char ID, unsigned int POS);
void send_position(unsigned char ID, unsigned char TORQLEVEL, unsigned char POS);
int Motor_Control();

ros::Publisher motion_pub;
ros::Publisher remocon_pub;
ros::Publisher std_pos_move_pub;
ros::Publisher pos_move_pub;

uxa_uic_msgs::motion uic_motion_msg;
uxa_uic_msgs::remocon uic_remocon_msg;
uxa_sam_msgs::std_position_move sam_std_pos_move_msg;
uxa_sam_msgs::position_move sam_pos_move_msg;



int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_opencv");

    if(Init_Ball_Tracking(__WINDOW_MODE) == __WEBCAM_ERROR) return __WEBCAM_ERROR;

    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    motion_pub = nh.advertise<uxa_uic_msgs::motion>
            ("uic_driver_motion", 100);
    remocon_pub = nh.advertise<uxa_uic_msgs::remocon>
            ("uic_driver_remocon", 100);

    std_pos_move_pub = nh.advertise<uxa_sam_msgs::std_position_move>
            ("sam_driver_std_position_move", 100);
    pos_move_pub = nh.advertise<uxa_sam_msgs::position_move>
            ("sam_driver_position_move", 100);

    sleep(2);
    send_msg("pc_control");
    sleep(1);
    send_msg("stop");
    sleep(7);
    send_msg("basic_motion");
    send_position(23, 2, 63);
    sleep(2);

    while(ros::ok())
    {
        loop_rate.sleep();
       

    }

     // Release the capture device housekeeping
     return 0;

}

void send_msg(std::string STR)
{
    uic_motion_msg.motion_name = STR;
    motion_pub.publish(uic_motion_msg);
}

void send_remocon(unsigned char remocon)
{
    uic_remocon_msg.btn_code = remocon;
    remocon_pub.publish(uic_remocon_msg);
}

void send_std_position(unsigned char ID ,unsigned int POS)
{
    sam_std_pos_move_msg.id = ID;
    sam_std_pos_move_msg.pos14 = POS;
    std_pos_move_pub.publish(sam_std_pos_move_msg);
}

void send_position(unsigned char ID, unsigned char TORQLEVEL, unsigned char POS)
{
    sam_pos_move_msg.id = ID;
    sam_pos_move_mss.torqlevel = TORQLEVEL;
    sam_pos_move_msg.pos = POS;
    pos_move_pub.publish(sam_pos_move_msg);
}



