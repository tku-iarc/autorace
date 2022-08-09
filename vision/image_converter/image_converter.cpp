#include "image_converter.h"
#define PI 3.14159
#define RPICAM "/raspicam_node/image/compressed"
#define USBCAM "/usb_cam/image_raw"
// #define USBCAM1 "camera1/usb_cam1/image_raw"
#define USBCAM1 "camera/image"
#define USBCAM2 "camera2/usb_cam2/image_raw"
Coord directions[8] = {
    Coord(0, 1),
    Coord(0, -1),
    Coord(-1, 0),
    Coord(1, 0),
    Coord(-1, 1),
    Coord(1, 1),
    Coord(-1, -1),
    Coord(1, -1)};
Vision::Vision()
{
    //image_sub = nh.subscribe("/raspicam_node/image/compressed", 1,&Vision::imageCb,this);
    image_sub = nh.subscribe(USBCAM1, 1, &Vision::imageCb, this);
    // image_sub2 = nh.subscribe(USBCAM2, 1, &Vision::imageCb2, this);

    FrameRate = 0.0;
}
Vision::~Vision()
{
    source.release();
    destroyAllWindows();
}
//==================================影像接收與處理=============================
void Vision::imageCb(const sensor_msgs::ImageConstPtr& msg)
//void Vision::imageCb(const sensor_msgs::CompressedImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //convert image data
        //cv::flip(cv_ptr->image, cv_ptr->image, 1); // reverse image
        if (!cv_ptr->image.empty())
        {
            FrameRate = Rate();

            //cv::imshow("view", cv_ptr->image);
            //cv::waitKey(10);
            //source = cv_ptr->image.clone();

            double scale = 0.5;
            Mat frame = cv_ptr->image.clone();
            // Size dsize = Size(frame.cols * scale, frame.rows * scale);
            // resize(frame, frame, dsize);
            // source = CutFrame(frame, 0, frame.rows * 0.4, frame.cols, frame.rows);
            source = frame.clone();
            monitor = source.clone();
            //cv::imshow("view", source);
            //cv::waitKey(10);

            //=============================================
            Object red, blue, yellow, white, black;
/*
			red = ColorMoldel(Red);			
			blue = ColorMoldel(Blue);
			yellow = ColorMoldel(Yellow);
			black = ColorMoldel(Black);
			*/
#pragma omp parallel sections
            {
#pragma omp section
                {
                    red = ColorMoldel(Red, Setting);
                    //cout<<"1";
                }
#pragma omp section
                {
                    blue = ColorMoldel(Blue, Setting);
                    //cout<<"2";
                }
// #pragma omp section
//                 {
//                     yellow = ColorMoldel(Yellow, Setting);
//                     //cout<<"3";
//                 }
                /*
#pragma omp section
                {
                    white = ColorMoldel(White);
                    //cout<<"4";
                }
#pragma omp section
                {
                    black = ColorMoldel(Black);
                    //cout<<"5";
                }*/
            }
            //center line
            line(monitor, Point(CenterXMsg, 0), Point(CenterXMsg, monitor.rows), Scalar(0, 0, 255), 1);
            monitor = DrawMonior(monitor, red, Red, Setting);
            monitor = DrawMonior(monitor, blue, Blue, Setting);
            // monitor = DrawMonior(monitor, yellow, Yellow, Setting);
            //monitor = DrawMonior(monitor, white, White, Setting);
            //monitor = DrawMonior(monitor, black, Black, Setting);
            switch (HSV_mode)
            {
            case 0:
                mask = red.mask;
                break;
            case 1:
                mask = blue.mask;
                break;
            case 2:
                mask = yellow.mask;
                break;
            case 3:
                mask = white.mask;
                break;
            case 4:
                mask = black.mask;
                break;
            }
            pub_src(frame);
            pub_mask(red.mask);
            pub_mask2(blue.mask);
            pub_monitor(monitor);
            pub_fps(FrameRate);

            if(red.size>0)
            {
                pub_sign(red.rect);
            }
            else if(blue.size>0)
            {
                pub_sign(blue.rect);
            }
            //cout<<"red.size"<<red.size<<"  blue.size"<<blue.size<<endl;
            // pub_object(red, blue, yellow, white, black);
            // pub_catch(red, blue, yellow, white, black);
            // pub_ball(red, blue, yellow, white, black);
            //imshow("mask",mask);
            //imshow("monitor",monitor);
            //waitKey(10);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert to image!");
    }
}
void Vision::imageCb2(const sensor_msgs::ImageConstPtr& msg)
{
//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //convert image data
//         if (!cv_ptr->image.empty())
//         {
//             FrameRate2 = Rate();

//             //cv::imshow("view", cv_ptr->image);
//             //cv::waitKey(10);
//             //source = cv_ptr->image.clone();

//             double scale = 0.3;
//             Mat frame = cv_ptr->image.clone();
//             Size dsize = Size(frame.cols * scale, frame.rows * scale);
//             resize(frame, frame, dsize);
//             source = CutFrame(frame, 0, 0, frame.cols, frame.rows * 0.4);
//             //source = frame.clone();
//             monitor2 = source.clone();
//             //cv::imshow("view", source);
//             //cv::waitKey(10);

//             //=============================================
//             Object red_goal, blue_goal, yellow_goal;

// #pragma omp parallel sections
//             {
// #pragma omp section
//                 {
//                     red_goal = ColorMoldel(RedGoal, Setting2);
//                     //cout<<"5";
//                 }
// #pragma omp section
//                 {
//                     blue_goal = ColorMoldel(BlueGoal, Setting2);
//                     //cout<<"6";
//                 }
// #pragma omp section
//                 {
//                     yellow_goal = ColorMoldel(YellowGoal, Setting2);
//                     //cout<<"7";
//                 }

//             }
//             //center line
//             line(monitor2, Point(CenterXMsg2, 0), Point(CenterXMsg2, monitor2.rows), Scalar(0, 0, 255), 1);
//             monitor2 = DrawMonior(monitor2, red_goal, RedGoal, Setting2);
//             monitor2 = DrawMonior(monitor2, blue_goal, BlueGoal, Setting2);
//             monitor2 = DrawMonior(monitor2, yellow_goal, YellowGoal, Setting2);

//             switch (HSV_mode)
//             {
//             case 5:
//                 mask2 = red_goal.mask;
//                 break;
//             case 6:
//                 mask2 = blue_goal.mask;
//                 break;
//             case 7:
//                 mask2 = yellow_goal.mask;
//                 break;
//             }
//             pub_src2(frame);
//             pub_mask2(mask2);
//             pub_monitor2(monitor2);
//             pub_fps2(FrameRate2);
//             pub_object2(red_goal, blue_goal, yellow_goal);

//             //imshow("mask2",mask2);
//             //imshow("red_goal.mask",red_goal.mask);
//             //imshow("blue_goal.mask",blue_goal.mask);
//             //imshow("yellow_goal.mask",yellow_goal.mask);
//             //imshow("monitor2",monitor2);
//             waitKey(10);
//         }
//     }
//     catch (cv_bridge::Exception &e)
//     {
//         ROS_ERROR("Could not convert to image!");
//     }
}
//===========================================================================
double Vision::Rate()
{
    double ALPHA = 0.5;
    double dt;
    static int frame_counter = 0;
    static double frame_rate = 0;
    static double StartTime = ros::Time::now().toNSec();
    ;
    double EndTime;

    frame_counter++;
    if (frame_counter == 10)
    {
        EndTime = ros::Time::now().toNSec();
        dt = (EndTime - StartTime) / frame_counter;
        StartTime = EndTime;
        if (dt != 0)
        {
            frame_rate = (1000000000.0 / dt) * ALPHA + frame_rate * (1.0 - ALPHA);
            //cout << "FPS: " << frame_rate << endl;
        }
        frame_counter = 0;
    }
    return frame_rate;
}
cv::Mat Vision::get_source()
{
    //Mat output = source.clone();
    //if(output.empty()){
    //	cout<<"no frame\n";
    //}
    return source;
}
cv::Mat Vision::CutFrame(Mat frame, int upx, int upy, int downx, int downy)
{
    Mat output(downy - upy, downx - upx, CV_8UC3);
    for (int i = 0; i < downy - upy; i++)
    {
        for (int j = 0; j < downx - upx; j++)
        {
            output.data[(i * output.cols * 3) + (j * 3) + 0] = frame.data[((i + upy) * frame.cols * 3) + ((j + upx) * 3) + 0];
            output.data[(i * output.cols * 3) + (j * 3) + 1] = frame.data[((i + upy) * frame.cols * 3) + ((j + upx) * 3) + 1];
            output.data[(i * output.cols * 3) + (j * 3) + 2] = frame.data[((i + upy) * frame.cols * 3) + ((j + upx) * 3) + 2];
        }
    }
    return output;
}
Object Vision::ColorMoldel(Color index, vector<int> setting)
{
    int CenterX=setting[0];
    int CenterY=setting[1];
    int CatchDistance=setting[2];
    int SizeFilter=setting[3];

    Mat inputMat = source.clone();
    Mat hsv(inputMat.rows, inputMat.cols, CV_8UC3, Scalar(0, 0, 0));
    Mat mask(inputMat.rows, inputMat.cols, CV_8UC1, Scalar(0, 0, 0));
    Mat mask2(inputMat.rows, inputMat.cols, CV_8UC1, Scalar(0, 0, 0));
    Mat dst(inputMat.rows, inputMat.cols, CV_8UC3, Scalar(0, 0, 0));
    vector<int> HSV;
    switch (index)
    {
    case Red:
        HSV.assign(HSV_red.begin(), HSV_red.end());
        break;
    case Blue:
        HSV.assign(HSV_blue.begin(), HSV_blue.end());
        break;
    case Yellow:
        HSV.assign(HSV_yellow.begin(), HSV_yellow.end());
        break;
    case White:
        HSV.assign(HSV_white.begin(), HSV_white.end());
        break;
    case Black:
        HSV.assign(HSV_black.begin(), HSV_black.end());
        break;
    case RedGoal:
        HSV.assign(HSV_redgoal.begin(), HSV_redgoal.end());
        break;
    case BlueGoal:
        HSV.assign(HSV_bluegoal.begin(), HSV_bluegoal.end());
        break;
    case YellowGoal:
        HSV.assign(HSV_yellowgoal.begin(), HSV_yellowgoal.end());
        break;
    }

    Object ball;
    //cout<<HSV.size()<<endl;

    int hmin, hmax, smin, smax, vmin, vmax;
    if (HSV.size() == 6)
    {
        hmin = HSV[0];
        hmax = HSV[1];
        smin = HSV[2];
        smax = HSV[3];
        vmin = HSV[4];
        vmax = HSV[5];
        
        cvtColor(inputMat, hsv, CV_BGR2HSV);
        if (HSV[0] <= HSV[1])
        {
            inRange(hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), mask);
        }
        else
        {
            inRange(hsv, Scalar(hmin, smin, vmin), Scalar(255, smax, vmax), mask);
            inRange(hsv, Scalar(0, smin, vmin), Scalar(hmax, smax, vmax), mask2);
            mask = mask + mask2;
        }

        //開操作 (去除一些噪點)
        //Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        //morphologyEx(mask, mask, MORPH_OPEN, element);

        //閉操作 (連接一些連通域)
        //morphologyEx(mask, mask, MORPH_CLOSE, element);
        
        Mat black(inputMat.rows, inputMat.cols, CV_8UC3, Scalar(255, 255, 255));
        // whilte.copyTo(dst, (cv::Mat::ones(mask.size(), mask.type()) * 255 - mask));
        black.copyTo(dst, mask);
        // cv::imshow("dst", dst);
        // waitKey(10);
        ball = SearchObject(mask, setting);
        ball.mask = dst.clone();
        // ball.mask = mask.clone();
        // if(index==0)
        // {
        //     imshow("mask",ball.mask);
        //     waitKey(1);
        // }
    }
    else
    {
        cout << "HSV vector size: " << HSV.size() << " error\n";
    }
    return ball;
}
Object Vision::SearchObject(Mat mask, vector<int> setting)
{
    int CenterX=setting[0];
    int CenterY=setting[1];
    int CatchDistance=setting[2];
    int SizeFilter=setting[3];

    Mat img = convertTo3Channels(mask.clone());
    int width = img.cols, length = img.rows;
    int obj_size = SizeFilter;
    Point start = Point(0, 0);
    Point end = Point(img.cols, img.rows);
    vector<vector<Coord> > obj; //物件列表
    Object ball;

    bool check[width][length]; //確認搜尋過的pix

    //清理標記與物件
    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < length; j++)
        {
            check[i][j] = false;
        }
    }

    //搜尋範圍顯示
    //rectangle(img, start, end, Scalar(255,0,0),2);

    //選取的範圍做搜尋
    //255為白
    for (int i = start.x; i < end.x; i++)
    {
        for (int j = start.y; j < end.y; j++)
        {
            //std::cout << i << " "	<< j << std::endl
            if (!check[i][j])
            {
                check[i][j] = true;

                if (is_white(img, Coord(i, j)))
                {

                    queue<Coord> bfs_list;
                    bfs_list.push(Coord(i, j));

                    vector<Coord> dot;
                    dot.push_back(Coord(i, j)); //搜尋到的物件

                    //放入佇列
                    while (!bfs_list.empty())
                    {

                        Coord ori = bfs_list.front();
                        bfs_list.pop();

                        //搜尋八方向
                        for (int k = 0; k < 8; k++)
                        {
                            Coord dst = ori + directions[k];

                            //處理邊界
                            if ((dst.get_x() < 0) || (dst.get_x() >= width) || (dst.get_y() < 0) || (dst.get_y() >= length))
                                continue;

                            if (check[dst.get_x()][dst.get_y()])
                                continue;

                            if (is_white(img, dst))
                            {
                                bfs_list.push(dst);
                                dot.push_back(dst);
                            }
                            check[dst.get_x()][dst.get_y()] = true;
                        }
                    }
                    if (dot.size() > obj_size)
                    {
                        obj.push_back(dot);
                    }
                }
            }
        }
    }

    ////上色
    //for(int i = 0; i < obj.size(); i++){
    //	//需要的物體大小
    //	//if(obj[i].size()>obj_size) continue;
    //	for(int j = 0; j < obj[i].size(); j++){
    //		Coord point = obj[i][j];
    //		line(img, Point(point.get_x(), point.get_y()), Point(point.get_x(), point.get_y()) ,Scalar(0,0,255), 1);
    //	}
    //}

    //imshow("window", img);
    //waitKey(10);

    for (int i = 0; i < obj.size(); i++)
    {
        int min_x = img.cols;
        int min_y = img.rows;
        int max_x = 0;
        int max_y = 0;
        Point center;
        int offset;
        Point dis_point;
        int distance;
        int size = 0;
        int radius = 0;
        int center_x = CenterX;
        int center_y = CenterY;
       // double PI = 3.14159;
        for (int j = 0; j < obj[i].size(); j++)
        {
            Coord point = obj[i][j];
            if (point.get_x() < min_x)
                min_x = point.get_x();
            if (point.get_y() < min_y)
                min_y = point.get_y();
            if (point.get_x() > max_x)
                max_x = point.get_x();
            if (point.get_y() > max_y)
                max_y = point.get_y();

            //circle(img, Point(point.get_x(), point.get_y()), 1, Scalar(255,255,255), -1);
        }
        radius = (max_x - min_x) / 2;
//==========================
        //if(min_y > img.rows*0.7 && radius > img.cols*0.6)
        //{
            //cout<<radius<<endl;
            //radius = img.cols*0.9;
        //}
//==========================
        center = Point((max_x + min_x) / 2, (max_y + min_y) / 2);
        offset = center.x - center_x;
        dis_point = Point(center.x, min_y + (radius * 2));
        //dis_point = Point(center.x, min_y);
        //distance = img.rows-dis_point.y;
        distance = sqrt(pow((dis_point.x - center_x), 2) + pow((dis_point.y - center_y), 2));
        //cout<<dis_point.x<<" "<<center_x <<" "<<dis_point.y<<" "<<center_y<<" "<<distance<<endl;
        // size = pow(radius, 2) * PI;
        size = abs(max_x-min_x)*abs(max_y-min_y);

        //過慮長條角錐
        //if( (down_y-up_y)*0.8 > (down_x-up_x) )continue;
        if (ball.size < size && size > 2000)
        {
            ball.upleft = Point(min_x, min_y);
            ball.downright = Point(max_x, max_y);
            ball.center = center;
            ball.offset = offset;
            ball.distance = distance;
            ball.dis_point = dis_point;
            ball.size = size;
            ball.radius = radius;
		
            int y = -(center.x - center_x);
            int x = -(center.y - center_y*1.5);
            ball.angle = atan2(y,x)*180/PI;

           //====================
            static int count=0;
            count++;

            std::string X;
            std::stringstream X_out;
            X_out << count;
            X = X_out.str();
            X+=".png";

            Mat image((max_y-min_y), (max_x-min_x), CV_8UC3, Scalar(0, 0, 0));
            for(int x=0; x<abs(max_x-min_x); x++)
            {
                for(int y=0; y<abs(max_y-min_y); y++)
                {
                    image.data[(y * image.cols * 3) + (x * 3) + 0] = source.data[((y + min_y) * source.cols * 3) + ((x + min_x) * 3) + 0];
                    image.data[(y * image.cols * 3) + (x * 3) + 1] = source.data[((y + min_y) * source.cols * 3) + ((x + min_x) * 3) + 1];
                    image.data[(y * image.cols * 3) + (x * 3) + 2] = source.data[((y + min_y) * source.cols * 3) + ((x + min_x) * 3) + 2];
                }
            }
            ball.rect=image.clone();
            //imwrite(X, image);

           //==========
        }
    }
    return ball;
}
Mat Vision::convertTo3Channels(const Mat &binImg)
{
    Mat three_channel = Mat::zeros(binImg.rows, binImg.cols, CV_8UC3);
    vector<Mat> channels;
    for (int i = 0; i < 3; i++)
    {
        channels.push_back(binImg);
    }
    merge(channels, three_channel);
    return three_channel;
}

cv::Mat Vision::DrawMonior(Mat frame, Object ball, Color index, vector<int> setting)
{
    int CenterX=setting[0];
    int CenterY=setting[1];
    int CatchDistance=setting[2];
    int SizeFilter=setting[3];

    Mat outputframe = frame.clone();
    int center_x = CenterX;
    int center_y = CenterY;
    if (ball.size != -1)
    {
        string output;
        Scalar color;
        switch (index)
        {
        case 0:
            output = "Red";
            color = Scalar(0, 0, 255);
            break;
        case 1:
            output = "Blue";
            color = Scalar(255, 0, 0);
            break;
        case 2:
            output = "Yellow";
            color = Scalar(0, 255, 255);
            break;
        case 3:
            output = "White";
            color = Scalar(255, 255, 255);
            break;
        case 4:
            output = "Black";
            color = Scalar(0, 0, 0);
            break;
        case 5:
            output = "RedGoal";
            color = Scalar(0, 0, 255);
            break;
        case 6:
            output = "BlueGoal";
            color = Scalar(255, 0, 0);
            break;
        case 7:
            output = "YellowGoal";
            color = Scalar(0, 255, 255);
            break;
        default:
            output = "None";
            color = Scalar(0, 0, 0);
            break;
        }
        
        rectangle(outputframe, ball.upleft, ball.downright, color, 2);
        //outer
        //circle(outputframe, ball.center, ball.radius, color, 2);
        //center
        circle(outputframe, ball.center, 2, Scalar(0, 255, 0), -1);
        circle(outputframe, ball.center, 1, Scalar(0, 0, 0), -1);

        //dis_point
        line(outputframe, ball.center, ball.dis_point, Scalar(255, 255, 255), 1);
        circle(outputframe, ball.dis_point, 2, Scalar(0, 255, 0), -1);

        putText(outputframe, output, ball.center, 0, 0.3, Scalar(255, 255, 255), 2);
        putText(outputframe, output, ball.center, 0, 0.3, Scalar(0, 0, 0), 1);
    }
    return outputframe;
}
