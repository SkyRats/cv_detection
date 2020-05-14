#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>

#include <bits/stdc++.h>

using namespace cv;
using namespace std;

#define vp vector<Point>
#define vpf vector<Point2f>

vpf order_points(vpf pts);

Mat four_points_transform(Mat image, vpf pts){

    vpf rect = order_points(pts);

    /* Se alguem souber de um jeito mais eficiente de fazer isso,
    me avisa */ 
    Point tl = rect[0];
    Point tr = rect[1];
    Point br = rect[2];
    Point bl = rect[3];

    float widthA =  sqrt( abs ( pow( br.x - bl.x, 2.0) - pow( br.y - bl.y, 2.0 ) ) );  
    float widthB =  sqrt( abs ( pow( tr.x - tl.x, 2.0) - pow( tr.y - tl.y, 2.0 ) ) );
    /* Precisa de mencao explicita porque existe cv::max */
    float maxWidth = std::max(widthA, widthB);

    float heightA =  sqrt( abs ( pow( br.y - tr.y, 2.0 ) - pow( br.x - tr.x, 2.0) ) ); 
    float heightB =  sqrt( abs ( pow( tl.y - bl.y, 2.0 ) - pow( tl.x - bl.x, 2.0) ) );

    float maxHeight = std::max(heightA, heightB);

    /* Mat dst = (Mat_<float> (4,2) << 
        0.0,0.0 ,
        maxWidth, 0.0 , 
        maxWidth, maxHeight ,
        0.0, maxHeight);
    */

    vp dst = {
        Point2f(0.0, 0.0) ,
        Point2f(maxWidth, 0.0) ,
        Point2f(maxWidth, maxHeight) ,
        Point2f(0.0, maxHeight)
    };

    Mat M = getPerspectiveTransform(rect, dst);
    Mat warped; 
    warpPerspective(image, warped, M, Size(maxWidth, maxHeight));

    return warped;

}

int main(){

    Mat frame;
    VideoCapture feed(-1);

    while(waitKey(30) != 27){
        feed >> frame;

        vp pts { Point2f(0.0,0.0), Point2f(0.0,50.0), Point2f(50.0,50.0), Point2f(50.0,0.0) };

        imshow("Transform", four_points_transform(frame, pts));
    }

    return 0;
}