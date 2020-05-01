#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;

#define vp vector<Point>

/* Prototipos das funcoes nas outras branches */
vp order_points(vp pts);
Mat four_points_transform(Mat image, vp pts);

Mat detect (Mat frame)
{
    /* Acho que deveriamos renomear frame2 para original
    e frame para processed ou alguma coisa assim */
    Mat frame2 = frame;
    cvtColor(frame, frame, CV_RGB2GRAY);
    GaussianBlur(frame, frame, Size(9,9), 0);
    GaussianBlur(frame, frame, Size(9,9), 0);
    threshold(frame, frame, 120, 255, 1); //invertido pelo exemplo TROCAR!
    
    vector<vp> contour;
    findContours(frame, contour, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    /* Usei cv::Mat porque já tem multiplicação de matrizes definida */
    Mat kernel = (Mat_<double>(3,3) << 
    -1, 3.5, -1 ,
    -1, -1, -1 ,
    -1, 3.5, -1
    );
    Mat kernel2 = (Mat_<double>(3,3) <<
    5, -0.35, -1 ,
    -0.35, -1, -1 ,
    -1, -1, 5
    );
    Mat kernel3 = (Mat_<double>(3,3) << 
    -1, -1, -1 ,
    5, -1, 5 ,
    -1, -1, -1
    );
    Mat kernel4 = (Mat_<double>(3,3) << 
    -0.35, -1, 5 ,
    -1, -1, -1 ,
    5, -0.35, -1
    );

    for(auto cnt : contour)
    {
        int peri = arcLength(cnt, true);
        vp approx;
        approxPolyDP(cnt, approx, 0.02*peri, true);
        if (approx.size() == 12)
        {
            polylines(frame2, approx, true, Scalar(0,255,0), 5, 8, 0);
            imshow("", frame2);

            Mat box = (Mat_<double>(3,3) <<
            0,0 ,
            frame.rows, 0 ,
            frame.rows, frame.cols ,
            0, frame.cols
            );
            Mat edge_pts = (Mat_<double>(3,3) <<
            approx[0].x, approx[0].y ,
            approx[11].x, approx[11].y , 
            approx[5].x, approx[5].y ,
            approx[6].x, approx[6].y
            );
            
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