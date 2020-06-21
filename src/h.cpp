#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h> 
#include <cv_bridge/cv_bridge.h>

#include "cv_detection/H_info.h"

#define ANGLE_THRESH_MIN 1
#define ANGLE_THRESH_MAX 2.2
#define KERNEL_THRESH_MAX 1.2
#define KERNEL_THRESH_MIN 0.9
#define PI 3.14159

#define vp vector<Point>
#define vpf vector<Point2f>

// Set true for debugging purposes, showing internals of algorithm
#define DEBUG true

// Sorts points based on y coordinate
struct comparison {
    bool operator() (Point2f pt1, Point2f pt2) { return (pt1.y > pt2.y);}
} comparing;

class HDetector {
    private:
        vpf edge_pts = { Point2f(0,0), Point2f(0,0), Point2f(0,0), Point2f(0,0)};
        Rect bounds;
        float area_ratio;
        void order_points();
        void four_points_transform(Mat image);
        float angle(Point2f v1, Point2f v2, Point2f relative = Point2f(0,0) );
        bool angle_check(vpf pts);
    public:
        Mat warped; 
        HDetector();
        bool detect (Mat frame);
        float getArea();
        void setArea(vp contour, Mat frame);
        int getCenter_X();
        int getCenter_Y();

};

HDetector::HDetector(){
}

/* Order points in edge_pts so that the first exit is the top-left, the second 
top-right, the third bottom-right, and the fourth bottom-left */   
void HDetector::order_points(){

    sort(this->edge_pts.begin(), this->edge_pts.end(), comparing);
    Point2f p1, p2;

    /* Switch the position of the first and second points
        if the second is to the right of the second */
    if(this->edge_pts[0].x > this->edge_pts[1].x ){
        p1 = this->edge_pts[1];
        this->edge_pts[1] = this->edge_pts[0];
        this->edge_pts[0] = p1;
    }

    /* Same as above for the third and fourth */
    if(this->edge_pts[2].x < this->edge_pts[3].x){
        p2 = this->edge_pts[3];
        this->edge_pts[3] = this->edge_pts[2];
        this->edge_pts[2] = p2;
    }

}

/* Takes an image as argument and returns warped perspective, moving edge_pts to
the edge of the frame */
void HDetector::four_points_transform(Mat image){

    order_points();

    Point tl = this->edge_pts[0];
    Point tr = this->edge_pts[1];
    Point br = this->edge_pts[2];
    Point bl = this->edge_pts[3];

    float widthA =  sqrt( abs ( pow( br.x - bl.x, 2.0) - pow( br.y - bl.y, 2.0 ) ) );  
    float widthB =  sqrt( abs ( pow( tr.x - tl.x, 2.0) - pow( tr.y - tl.y, 2.0 ) ) );
    // Requires explicit reference due to cv::max
    float maxWidth = std::max(widthA, widthB);

    float heightA =  sqrt( abs ( pow( br.y - tr.y, 2.0 ) - pow( br.x - tr.x, 2.0) ) ); 
    float heightB =  sqrt( abs ( pow( tl.y - bl.y, 2.0 ) - pow( tl.x - bl.x, 2.0) ) );

    float maxHeight = std::max(heightA, heightB);

    vpf dst = {
        Point2f(0.0, 0.0) ,
        Point2f(maxWidth, 0.0) ,
        Point2f(maxWidth, maxHeight),
        Point2f(0.0, maxHeight)
    };

    Mat M = getPerspectiveTransform(this->edge_pts, dst);
    
    warpPerspective(image, this->warped, M, Size(maxWidth, maxHeight));

}

// Determines angle between vectors 'v1' and 'v2' using 'relative' as origin
float HDetector::angle(Point2f v1, Point2f v2, Point2f relative){
    
    float Vlenght1, Vlenght2; // Length of v1 and v2
    
    Vlenght1 = sqrt((v1.x - relative.x)*(v1.x - relative.x) + (v1.y - relative.y)*(v1.y - relative.y));
    Vlenght2 = sqrt((v2.x - relative.x)*(v2.x - relative.x) + (v2.y - relative.y)*(v2.y - relative.y));
    // Takes dot product and divides by vector lengths to get cos of the angle
    float a = ((v1.x - relative.x)*(v2.x - relative.x) + (v1.y - relative.y)*(v2.y -relative.y))/(Vlenght1*Vlenght2);

    return acos(a);
}

/* Checks if all sides of a 12 sided shape 'pts' are perpendicular, 
    using ANGLE_THRESH */
bool HDetector::angle_check(vpf pts){
    
    // First has to be done manually
    float a = angle(pts[1], pts[11], pts[0]);
    if (a < ANGLE_THRESH_MIN && a > ANGLE_THRESH_MAX){
        return false;
    }else{
        for (int i = 1; i <= 10; i++){
            // General term
            a = angle(pts[i+1], pts[i-1], pts[i]);
            if (a < ANGLE_THRESH_MIN && a > ANGLE_THRESH_MAX)
                return false;
        }
    }

}

float HDetector::getArea(){
    return this->area_ratio;
}

void HDetector::setArea(vp contour, Mat frame){
    this->area_ratio = contourArea(contour, false)/(frame.cols*frame.rows);
}

int HDetector::getCenter_X(){
    return (this->bounds.x + this->bounds.width/2);
}

int HDetector::getCenter_Y(){
    return (this->bounds.y + this->bounds.height/2);
}

// Takes an image 'frame' and detects whether it contains the letter H
bool HDetector::detect (Mat frame){
    bool detected = false;

    Mat frame2 = frame;
    
    if(DEBUG){
        imshow("Lines", frame2);
    }

    cvtColor(frame, frame, CV_RGB2GRAY);
    // Blur and threshold remove noise from image
    GaussianBlur(frame, frame, Size(9,9), 0);
    GaussianBlur(frame, frame, Size(9,9), 0);
    GaussianBlur(frame, frame, Size(9,9), 0);
    GaussianBlur(frame, frame, Size(9,9), 0);
    GaussianBlur(frame, frame, Size(9,9), 0);
    GaussianBlur(frame, frame, Size(9,9), 0);
    threshold(frame, frame, 150, 255, 0);

    vector<vp> contour;
    findContours(frame, contour, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    for(vp cnt : contour){

        int peri = arcLength(cnt, true);
        vpf approx;
        approxPolyDP(cnt, approx, 0.02*peri, true);
        
        if (approx.size() == 12){

            this->bounds = boundingRect(approx); // Precisa ser Rect2f?
            
            float a1 = angle(approx[0] - approx[1], Point2f(0,1) );
            float a2 = angle(approx[1] - approx[2], Point2f(0,1) );

            /* If the sides of the H are very close to parallel to its bounds,
                use the bounding rect vertices for warp */
            if( a1 < 0.1 || a2 < 0.1
               || abs(a1 - PI) < 0.1 || abs(a1 - PI) < 0.1 ){
                
                this->edge_pts = {
                    Point2f (bounds.x, bounds.y) ,
                    Point2f (bounds.x + bounds.width, bounds.y) , 
                    Point2f (bounds.x, bounds.y + bounds.height) ,
                    Point2f (bounds.x + bounds.width, bounds.y + bounds.height)
                };
            
            /* If they are far, use the vertices that are closest to the bounding
                rect sides */
            }else{
            
                for(Point2f v : approx){

                    // Close on left side of bound
                    if( abs(v.x - bounds.x) <= 1) edge_pts[0] = v;
                    // On right side
                    else if( abs(v.x - (bounds.x + bounds.width) ) <= 1) edge_pts[1] = v;

                    // On top
                    else if( abs(v.y - bounds.y) <= 1) edge_pts[2] = v;
                    //On bottom
                    else if( abs(v.y - (bounds.y + bounds.height) ) <= 1) edge_pts[3] = v;

                }

            }

            four_points_transform(frame);
            
            if(DEBUG){
                imshow("warped", frame);
            }
            
            if (angle_check(approx)){
                                
                // Maximizes the sum for an image that looks like an H
                
                Mat kernel_vertical = (Mat_<float>(3,3) <<
                -1, 5,-1
                -1, -1, -1
                -1, 5, -1
                );
                Mat kernel_horizontal = (Mat_<float>(3,3) <<
                -1, -1,-1,
                5, -1, 5
                -1, -1, -1
                );
                
                /* Resizes processed image to 3x3 for lighter processing and
                    converts to compatible format */
                Mat small_img;
                resize(this->warped, small_img, Size(3,3), INTER_AREA);
                small_img.convertTo(small_img, CV_32FC2);

                if(DEBUG){ 
                    Mat big_small_img;
                    // Increase size of small_img for human analysis
                    resize(small_img, big_small_img, Size(60,60));
                    imshow("small_img", big_small_img);
                    // Draws bound
                    rectangle(frame2, bounds, (0,255,0));
                }

                // As a greyscale image, the sum of its pixel values is in channel 0
                int kernel_sum_vertical = sum((small_img)*(kernel_vertical))[0];
                int kernel_sum_horizontal = sum((small_img)*(kernel_horizontal))[0];
                
                if((kernel_sum_vertical >=1000 && kernel_sum_vertical <= 1300) || ((kernel_sum_horizontal <= -1000 && kernel_sum_horizontal >= -1300))){
                    cout << "H detectado"<< endl;
                    detected = true;
                    HDetector::setArea(cnt,frame2);
                    if(DEBUG){

                        float cx = 0, cy = 0;

                        for(Point2f point : edge_pts){
                            cx += point.x;
                            cy += point.y;
                        }
                        cx /= 4.0;
                        cy /= 4.0;
                
                        imshow("warped", frame);

                        // Shows captures edge of H in black
                        circle(frame2, edge_pts[0], 3, (255,0,0), 3 );
                        circle(frame2, edge_pts[1], 3, (255,0,0), 3 );
                        circle(frame2, edge_pts[2], 3, (255,0,0), 3 );                
                        circle(frame2, edge_pts[3], 3, (255,0,0), 3 );
                        //Shows H center
                        circle(frame2, Point2f(this->bounds.x + this->bounds.width/2, this->bounds.y + this->bounds.height/2), 3, (0, 0, 255), 3 );
                
                        imshow("Lines", frame2);
                    }             
                }else cout << endl;
                

            }else cout << endl;
            
        }
    }
    return detected;
}

void callback(const sensor_msgs::ImageConstPtr& img_msg){

    cv_bridge::CvImagePtr cv_ptr;
    ros::NodeHandle n;
    ros::Publisher h_pub = n.advertise<cv_detection::H_info>("h_detection",0);
    
    try{
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_detection::H_info msg;
    HDetector* detector = new HDetector();

    if ( detector->detect(cv_ptr->image) ){
        msg.detected = true;
        msg.center_x = detector->getCenter_X();
        msg.center_y = detector->getCenter_Y();
        msg.area_ratio = detector->getArea();
    
    }else{
        msg.detected = false;
        msg.center_x = -1;
        msg.center_y = -1;
        msg.side_diff = 0;
        msg.area_ratio = -1;
    }

    h_pub.publish(msg);

}

// For testing
int main(int argc, char** arvg){
    ros::init(argc, arvg, "h_node");
    ros::NodeHandle n;
    ros::Subscriber h_sub = n.subscribe("/iris_fpv_cam/usb_cam/image_raw", 3, callback);
    
    while(ros::ok()){
        ros::spinOnce();
        if(waitKey(30) == 27) break;
    }

    /* ros::Publisher h_pub = n.advertise<cv_detection::H_info>("h_detection",0);
    cv_detection::H_info msg;
    Mat frame;

    VideoCapture video(0);
    video >> frame;
    HDetector* detector = new HDetector();
    while (ros::ok()){

        ros::spinOnce();

        if ( detector->detect(frame) ){
            msg.detected = true;
            msg.center_x = detector->getCenter_X();
            msg.center_y = detector->getCenter_Y();
            msg.area_ratio = detector->getArea();
        
        }else{
            msg.detected = false;
            msg.center_x = -1;
            msg.center_y = -1;
            msg.side_diff = 0;
            msg.area_ratio = -1;
        }

        h_pub.publish(msg);
        if (waitKey(30) == 27) break;
        video >> frame; 
    } */
}

