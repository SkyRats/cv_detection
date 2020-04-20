#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;
Mat detect (Mat frame)
{
    Mat frame2 = frame;
    cvtColor(frame, frame, CV_RGB2GRAY);
    GaussianBlur(frame, frame, Size(9,9), 0);
    GaussianBlur(frame, frame, Size(9,9), 0);
    threshold(frame, frame, 120, 255, 1); //invertido pelo exemplo TROCAR!
    vector<vector<Point>> contour;
    findContours(frame, contour, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    float kernel[3][3] = {{-1, 3.5 ,-1}, {-1, -1, -1}, {-1, 3.5, -1}};
    for(int i = 0; i<contour.size(); i++)
    {
        int peri = arcLength(contour[i], true);
        vector<Point> approx;
        approxPolyDP(contour[i], approx, 0.02*peri, true);
        if (approx.size() == 12)
        {
            polylines(frame2, approx, true, Scalar(0,255,0), 5, 8, 0);
            imshow("", frame2);
        }
    }
    return frame;
}
int main()
{
    Mat frame;
    VideoCapture video(0);
    video >> frame;
    while (true)
    {
        imshow("base", frame);
        imshow("display", detect(frame));
        waitKey(30);
        video >> frame;
    }
}