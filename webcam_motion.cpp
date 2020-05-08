#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;

Mat image_treat(Mat subtracted_frame)
{
    threshold(subtracted_frame, subtracted_frame, 45 , 255 , 0 );
    int dilation_size = 7;
    dilate(subtracted_frame, subtracted_frame, getStructuringElement(MORPH_RECT, Size(1.25*dilation_size,1.25*dilation_size), Point(dilation_size, dilation_size)));
    Canny(subtracted_frame, subtracted_frame,75, 225, 3);
    return subtracted_frame;
}
int main()
{
    VideoCapture video(0); //captures video from default cam
    Mat old_frame,old_frame_gray, current_frame,current_frame_gray, cv_frame;
    video >> current_frame;
    cvtColor(current_frame, current_frame_gray, CV_RGB2GRAY);
    while(true)
    {
        video >> old_frame;
        cvtColor(old_frame, old_frame_gray, CV_RGB2GRAY);
        cvtColor(current_frame, current_frame_gray, CV_RGB2GRAY);
        cv_frame = image_treat(current_frame_gray-old_frame_gray);
        //rectangle(current_frame, boundingRect(cv_frame), Scalar(0,255,0),3,8,0);
        Scalar t = sum(cv_frame);
        if(t[0] > 2000)
        {
            rectangle(current_frame, boundingRect(cv_frame), Scalar(0,255,0),3,8,0);
            putText(current_frame, "movement detected", Point(275,25),  FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0), 3, 8,false);
        }
        imshow("Display", current_frame);
        waitKey(30);
        video >> current_frame;
    } 
}
