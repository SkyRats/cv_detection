#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>

#include <bits/stdc++.h>

using namespace cv;
using namespace std;

#define vp vector<Point>

vp order_points(vp pts);

Mat four_points_transform(Mat image, vp pts){

    vp rect = order_points(pts);

    /* Se alguem souber de um jeito mais eficiente de fazer isso,
    me avisa */ 
    Point tl = rect[0];
    Point tr = rect[1];
    Point br = rect[2];
    Point bl = rect[3];

    double widthA =  sqrt( pow( br.x - bl.x, 2.0) - pow( br.y - bl.y, 2.0 ) );  
    double widthB =  sqrt( pow( tr.x - tl.x, 2.0) - pow( tr.y - tl.y, 2.0 ) );
    /* Precisa de mencao explicita porque existe cv::max */
    double maxWidth = std::max(widthA, widthB);

    double heightA =  sqrt( pow( br.x - tr.x, 2.0) - pow( br.y - tr.y, 2.0 ) );  
    double heightB =  sqrt( pow( tl.x - bl.x, 2.0) - pow( tl.y - bl.y, 2.0 ) );
    double maxHeight = std::max(heightA, heightB);

    Mat dst = (Mat_<double> (4,2) << 
        0,0 ,
        maxWidth-1, 0 , 
        maxWidth-1, maxHeight-1 ,
        0, maxHeight-1);

    Mat M = getPerspectiveTransform(rect, dst);
    Mat warped; 
    warpPerspective(image, warped, M, Size(maxWidth, maxHeight));

    return warped;

}

int main(){
    return 0;
}