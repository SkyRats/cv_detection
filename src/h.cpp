#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;
#include "ros/ros.h"
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

    cvtColor(frame, frame, CV_RGB2GRAY);
    // Blur and threshold remove noise from image
    GaussianBlur(frame, frame, Size(9,9), 0);
    GaussianBlur(frame, frame, Size(9,9), 0);
    threshold(frame, frame, 150, 255, 1);

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
                // Draws bound
                rectangle(frame2, bounds, (0,255,0));
                //Shows H center
                circle(frame2, Point2f(this->bounds.x + this->bounds.width/2, this->bounds.y + this->bounds.height/2), 3, (0, 0, 255), 3 );
                
                imshow("Lines", frame2);
            }
            
            if (angle_check(approx)){
                                
                /* Maximizes the sum for an image that is composed of two vertical
                    strips on each side, each spanning 1/3 of the images length 
                    Like so:
                    111000111
                    111000111
                    111000111
                */
                Mat sides_kernel = (Mat_<float>(12,1) <<
                1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1 
                );
                /* Maximizes the sum for an image that is composed of one vertical 
                    strip in the middle spanning 1/3 of the images length 
                    Like so:
                    000111000
                    000111000
                    000111000
                */                
                Mat middle_kernel = (Mat_<float>(12,1) <<
                0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0
                );

                /* Resizes processed image to 12x12 for lighter processing and
                    converts to compatible format */
                Mat small_img;
                resize(this->warped, small_img, Size(12,12), INTER_AREA);
                small_img.convertTo(small_img, CV_32FC2);

                if(DEBUG){ 
                    Mat big_small_img;
                    // Increase size of small_img for analysis
                    resize(small_img, big_small_img, Size(300,300));
                    imshow("small_img", big_small_img);
                }

                // As a greyscale image, the sum of its pixel values is in channel 0
                int sides = sum( (small_img)*(sides_kernel) ) [0];
                int middle = sum( (small_img)*(middle_kernel) ) [0];

                /*
                Index 0 goes for vertical H and 1, for horizontal (only positions 
                    possible after processing)
                
                VERTICAL
                Ideally has two full vertical strips in its sides, which are 12x4 
                pixels, each with value 255
                Also has one 4x4 square in its center, each at 255
                Like so:
                111000111
                111111111
                111000111

                HORIZONTAL
                Ideally has 4 4x4, 255 pixel squares in its sides (one in each corner)
                Also has one full vertical strip in its centre, which is 12x4 at 255
                Like so:
                111111111
                000111000
                111111111

                The sum of these arrangements is stores in ideal_sides 
                    and ideal_middle
                */
                int ideal_sides[2] = {2*(255*12*4), 4*(255*4*4)};
                int ideal_middle[2] = {255*4*4, 255*12*4};

                /* 
                KERNEL_THRESH_MIN is < 1, which represents the smallest acceptable
                    fraction of the ideal sum above.
                KERNEL_THRESH_MAX is > 1, the greatest acceptable fraction 
                */
                if( (sides >= ideal_sides[0]*KERNEL_THRESH_MIN && middle <= ideal_middle[0]*KERNEL_THRESH_MAX) 
                || (sides <= ideal_sides[1]*KERNEL_THRESH_MAX && middle >= ideal_middle[1]*KERNEL_THRESH_MIN) ){
                    cout << "H detectado"<< endl;
                    detected = true;
                    HDetector::setArea(cnt,frame2);               
                }else cout << endl;
                

            }else cout << endl;
            
        }
    }
    return detected;
}

// For testing
int main(int argc, char** arvg){
    ros::init(argc, arvg, "h_node");
    ros::NodeHandle n;
    ros::Publisher h_pub = n.advertise<cv_detection::H_info>("h_detection",0);
    cv_detection::H_info msg;
    Mat frame;
    
    //getCenter_X()
    //getCenter_Y()

    VideoCapture video(0);
    HDetector* detector = new HDetector();
    video >> frame;
    while (ros::ok()){
        if ( detector->detect(frame) ){
            msg.detected = true;
            msg.center_x = detector->getCenter_X();
            msg.center_y = detector->getCenter_Y();
            // msg.side_diff
            msg.area_ratio = detector->getArea();
        }
        else{
            msg.detected = false;
            msg.center_x = -1;
            msg.center_y = -1;
            msg.side_diff = 0;
            msg.area_ratio = -1;
        }
        h_pub.publish(msg);
        if (waitKey(30) == 27) break;
        video >> frame;
    }
}

