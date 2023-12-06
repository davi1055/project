#include<iostream>
#include"../include/serial/serial_port.hpp"
#include"../include/serial/serial.hpp"
int main()
{
    Detector detector;
    SerialPort serial_port;
    VisionData vision_data;
    cv::VideoCapture capture("../1.png");
    //cv::VideoCapture capture(0);
    while(1)
    {
        cv::Mat src;
        capture >>src;
        if(src.empty())
        {
            std::cout<<"src empty!"<<"\n";
        }
        detector.GetSourceImage(src);
        detector.Detect(src);
        detector.SentVisionData(vision_data);

        // serial_port.TransformData(vision_data);
        
        // serial_port.OpenPort();
        // bool robot_cmd_required_stop = false;
        // bool robot_cmd_is_ok = true;
        // robot_cmd_loop(serial_port,robot_cmd_required_stop,vision_data);

        if(cv::waitKey(0)=='q')
            return 0;
    }
    
}