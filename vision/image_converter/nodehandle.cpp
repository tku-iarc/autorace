#include "nodehandle.h"

NodeHandle::NodeHandle()
{
    read_yaml();
    HSV_mode = 0;
    save_srv = nh.advertiseService("/tb3/save", &NodeHandle::savecall, this);
    connect_srv = nh.advertiseService("/tb3/connect", &NodeHandle::connectcall, this);

    color_sub = nh.subscribe("/tb3/color", 1, &NodeHandle::colorcall, this);
    center_sub = nh.subscribe("/tb3/center", 1, &NodeHandle::centercall, this);
    center_sub2 = nh.subscribe("/tb3/center2", 1, &NodeHandle::centercall2, this);

    src_pub = nh.advertise<sensor_msgs::Image>("/tb3/src", 1);
    src_pub2 = nh.advertise<sensor_msgs::Image>("/tb3/src2", 1);
    mask_pub = nh.advertise<sensor_msgs::Image>("/tb3/mask", 1);
    mask_pub2 = nh.advertise<sensor_msgs::Image>("/tb3/mask2", 1);
    monitor_pub = nh.advertise<sensor_msgs::Image>("/tb3/monitor", 1);
    monitor_pub2 = nh.advertise<sensor_msgs::Image>("/tb3/monitor2", 1);
    sign_pub = nh.advertise<sensor_msgs::Image>("/detect/image_sign", 1);

    fps_pub = nh.advertise<std_msgs::Float32>("/tb3/fps", 1);
    fps_pub2 = nh.advertise<std_msgs::Float32>("/tb3/fps2", 1);
    obj_pub = nh.advertise<std_msgs::Int32MultiArray>("/tb3/object", 1);
    obj_pub2 = nh.advertise<std_msgs::Int32MultiArray>("/tb3/object2", 1);
    catch_pub = nh.advertise<std_msgs::Int32MultiArray>("/tb3/catch", 1);
	ball_pub = nh.advertise<std_msgs::Int32MultiArray>("/tb3/ball", 1);
}
NodeHandle::~NodeHandle()
{
}
void NodeHandle::read_yaml()
{
    std::string param = YAML_PATH;
    const char *parampath = param.c_str();
    if (ifstream(parampath))
    {
        std::string temp = "rosparam load " + param + " /tb3";
        const char *load = temp.c_str();
        system(load);
        cout << "Read the yaml file" << endl;
    }
    get_param();
}
void NodeHandle::save_yaml()
{
    std::string param = YAML_PATH;
    std::string temp = "rosparam dump " + param + " /tb3";
    const char *save = temp.c_str();
    system(save);
    cout << "Save the yaml file" << endl;
    get_param();
}
void NodeHandle::colorcall(const vision::color msg)
{
    HSV_mode = msg.mode;
    switch (HSV_mode)
    {
    case 0:
        HSV_red.assign(msg.data.begin(), msg.data.end());
        break;
    case 1:
        HSV_blue.assign(msg.data.begin(), msg.data.end());
        break;
    case 2:
        HSV_yellow.assign(msg.data.begin(), msg.data.end());
        break;
    case 3:
        HSV_white.assign(msg.data.begin(), msg.data.end());
        break;
    case 4:
        HSV_black.assign(msg.data.begin(), msg.data.end());
        break;
    case 5:
        HSV_redgoal.assign(msg.data.begin(), msg.data.end());
        break;
    case 6:
        HSV_bluegoal.assign(msg.data.begin(), msg.data.end());
        break;
    case 7:
        HSV_yellowgoal.assign(msg.data.begin(), msg.data.end());
        break;
    }
    //for(int i=0; i<6; ++i)
    //  std::cout << msg.data[i] << ' ';
    //cout<<endl;
}
bool NodeHandle::savecall(std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res)
{
    //cout<<"Save\n";
    save_yaml();
    return true;
}
bool NodeHandle::connectcall(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res)
{
    //cout<<"program running\n";
    return true;
}
void NodeHandle::centercall(const std_msgs::Int32MultiArray msg)
{
    Setting[0] = msg.data[0];
    Setting[1] = msg.data[1];
    Setting[2] = msg.data[2];
    Setting[3] = msg.data[3];

    CenterXMsg = msg.data[0];
    //cout<<"CenterXMsg"<<endl;
    CenterYMsg = msg.data[1];
    //cout<<CenterYMsg<<endl;
    CatchDistanceMsg = msg.data[2];
	SizeFilterMsg = msg.data[3];
}
void NodeHandle::centercall2(const std_msgs::Int32MultiArray msg)
{
    CenterXMsg2 = msg.data[0];
    //cout<<"CenterXMsg"<<endl;
    CenterYMsg2 = msg.data[1];
    //cout<<CenterYMsg<<endl;
    CatchDistanceMsg2 = msg.data[2];
	SizeFilterMsg2 = msg.data[3];

    Setting2[0] = msg.data[0];
    Setting2[1] = msg.data[1];
    Setting2[2] = msg.data[2];
    Setting2[3] = msg.data[3];
}
void NodeHandle::get_param()
{
    cout << "Get parameter" << endl;
    nh.getParam("/tb3/HSV/Red", HSV_red);
    nh.getParam("/tb3/HSV/Blue", HSV_blue);
    nh.getParam("/tb3/HSV/Yellow", HSV_yellow);
    nh.getParam("/tb3/HSV/White", HSV_white);
    nh.getParam("/tb3/HSV/Black", HSV_black);
    nh.getParam("/tb3/HSV/RedGoal", HSV_redgoal);
    nh.getParam("/tb3/HSV/BlueGoal", HSV_bluegoal);
    nh.getParam("/tb3/HSV/YellowGoal", HSV_yellowgoal);

    nh.getParam("/tb3/center", Setting);
    if (Setting.size())
    {
        CenterXMsg = Setting[0];
        CenterYMsg = Setting[1];
        CatchDistanceMsg = Setting[2];
		SizeFilterMsg = Setting[3];
    }
    nh.getParam("/tb3/center2", Setting2);
    if (Setting.size())
    {
        CenterXMsg2 = Setting2[0];
        CenterYMsg2 = Setting2[1];
        CatchDistanceMsg2 = Setting2[2];
		SizeFilterMsg2 = Setting2[3];
    }
}

void NodeHandle::pub_sign(Mat image)
{
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    sign_pub.publish(msg);
}
void NodeHandle::pub_monitor(Mat Monitor)
{
    sensor_msgs::ImagePtr monitormsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Monitor).toImageMsg();
    monitor_pub.publish(monitormsg);
}
void NodeHandle::pub_monitor2(Mat Monitor)
{
    sensor_msgs::ImagePtr monitormsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Monitor).toImageMsg();
    monitor_pub2.publish(monitormsg);
}
void NodeHandle::pub_mask(Mat Mask)
{
    sensor_msgs::ImagePtr maskmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Mask).toImageMsg();
    mask_pub.publish(maskmsg);
}
void NodeHandle::pub_mask2(Mat Mask)
{
    sensor_msgs::ImagePtr maskmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Mask).toImageMsg();
    mask_pub2.publish(maskmsg);
}
void NodeHandle::pub_src(Mat Src)
{
    sensor_msgs::ImagePtr srcmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Src).toImageMsg();
    src_pub.publish(srcmsg);
}
void NodeHandle::pub_src2(Mat Src)
{
    sensor_msgs::ImagePtr srcmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Src).toImageMsg();
    src_pub2.publish(srcmsg);
}
void NodeHandle::pub_fps(double fps)
{
    std_msgs::Float32 msg;
    msg.data = fps;
    fps_pub.publish(msg);
}
void NodeHandle::pub_fps2(double fps)
{
    std_msgs::Float32 msg;
    msg.data = fps;
    fps_pub2.publish(msg);
}
void NodeHandle::pub_object(Object red, Object blue, Object yellow, Object white, Object black)
{
    std_msgs::Int32MultiArray msg;
    msg.data.push_back(red.angle);
    msg.data.push_back(red.distance);
    msg.data.push_back(red.dis_point.x);
    msg.data.push_back(red.dis_point.y);

    msg.data.push_back(blue.angle);
    msg.data.push_back(blue.distance);
    msg.data.push_back(blue.dis_point.x);
    msg.data.push_back(blue.dis_point.y);

    msg.data.push_back(yellow.angle);
    msg.data.push_back(yellow.distance);
    msg.data.push_back(yellow.dis_point.x);
    msg.data.push_back(yellow.dis_point.y);

    msg.data.push_back(white.angle);
    msg.data.push_back(white.distance);
    msg.data.push_back(white.dis_point.x);
    msg.data.push_back(white.dis_point.y);

    msg.data.push_back(black.angle);
    msg.data.push_back(black.distance);
    msg.data.push_back(black.dis_point.x);
    msg.data.push_back(black.dis_point.y);

    obj_pub.publish(msg);
}
void NodeHandle::pub_object2(Object red, Object blue, Object yellow)
{
    std_msgs::Int32MultiArray msg;
    msg.data.push_back(red.offset);
    msg.data.push_back(red.distance);
    msg.data.push_back(red.size);
    msg.data.push_back(0);

    msg.data.push_back(blue.offset);
    msg.data.push_back(blue.distance);
    msg.data.push_back(blue.size);
    msg.data.push_back(0);

    msg.data.push_back(yellow.offset);
    msg.data.push_back(yellow.distance);
    msg.data.push_back(yellow.size);
    msg.data.push_back(0);

    obj_pub2.publish(msg);
}
void NodeHandle::pub_catch(Object red, Object blue, Object yellow, Object white, Object black)
{
    std_msgs::Int32MultiArray msg;
    if (red.distance < CatchDistanceMsg)
    {
        msg.data.push_back(1);
        msg.data.push_back(0);
    }
    else if (blue.distance < CatchDistanceMsg)
    {
        msg.data.push_back(1);
        msg.data.push_back(1);
    }
    else if (yellow.distance < CatchDistanceMsg)
    {
        msg.data.push_back(1);
        msg.data.push_back(2);
    }
    else if (white.distance < CatchDistanceMsg)
    {
        msg.data.push_back(1);
        msg.data.push_back(3);
    }
    else if (black.distance < CatchDistanceMsg)
    {
        msg.data.push_back(1);
        msg.data.push_back(4);
    }
    else
    {
        msg.data.push_back(0);
        msg.data.push_back(9999);
    }

    catch_pub.publish(msg);
}
void NodeHandle::pub_ball(Object red, Object blue, Object yellow, Object white, Object black)
{
	vector<int> ball[5];
	int near[3]={999,999,999};
    ball[0].push_back(red.angle);
    ball[0].push_back(red.distance);

    ball[1].push_back(blue.angle);
    ball[1].push_back(blue.distance);

    ball[2].push_back(yellow.angle);
    ball[2].push_back(yellow.distance);

    ball[3].push_back(white.angle);
    ball[3].push_back(white.distance);

    ball[4].push_back(black.angle);
    ball[4].push_back(black.distance);

    for(int i=0; i<5; i++){
        if(ball[i][1] < near[2]){
            near[0] = i;
            near[1] = ball[i][0];
            near[2] = ball[i][1];
        }
    }

	std_msgs::Int32MultiArray msg;
    msg.data.push_back(near[0]);
    msg.data.push_back(near[1]);
    msg.data.push_back(near[2]);

	ball_pub.publish(msg);
}
