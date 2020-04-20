#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>

#include <bits/stdc++.h>

using namespace cv;
using namespace std;

array<double, 2> order_points(array<double, 2> pts[4]){

    sort(pts, pts+4);
    array<double, 2> bl;

    if(pts[0][1] > pts[1][1]){
        bl = pts[1];
    }else{
        bl = pts[0];
        pts[0] = pts[1];
    }
    
    if(pts[2][1] < pts[3][1]){
        pts[1] = pts[3];
    }else{
        pts[1] = pts[2];
        pts[2] = pts[3];
    }
    pts[3] = bl;

    return *pts;
}

int main(){

    while(true){

        VideoCapture cap(0);
        Mat frame, gray;

        cap >> frame;
        cvtColor(frame, frame, CV_RGB2GRAY);

        //detect(gray);

        imshow("frame", frame);

        if(waitKey(1) == 27) break;
    
    }

    return 0;
    
}
