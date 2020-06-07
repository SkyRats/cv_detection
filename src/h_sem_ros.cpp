#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;

#define ANGLE_THRESH_MIN 1
#define ANGLE_THRESH_MAX 2.2
#define KERNEL_THRESH_LOW 1.2
#define KERNEL_THRESH_HIGH 0.9
#define PI 3.14159

#define vp vector<Point>
#define vpf vector<Point2f>

#define DEBUG true

struct comparison {
    bool operator() (Point2f pt1, Point2f pt2) { return (pt1.y > pt2.y);}
} comparing;

class HDetector {
    private:
        vpf edge_pts = { Point2f(0,0), Point2f(0,0), Point2f(0,0), Point2f(0,0) };
        void order_points();
        void four_points_transform(Mat image);
        float dot_product_angle(Point2f p1, Point2f p2, Point2f relative = Point2f(0,0) );
        bool dot_product_check(vpf pts);
    public:
        Mat warped; 
        HDetector();
        Mat detect (Mat frame);

};

HDetector::HDetector() {}

void HDetector::order_points(){

    sort(this->edge_pts.begin(), this->edge_pts.end(), comparing);
    Point2f p1, p2;

    if(this->edge_pts[0].x > this->edge_pts[1].x ){
        p1 = this->edge_pts[1];
        this->edge_pts[1] = this->edge_pts[0];
        this->edge_pts[0] = p1;
    }

    if(this->edge_pts[2].x < this->edge_pts[3].x){
        p2 = this->edge_pts[3];
        this->edge_pts[3] = this->edge_pts[2];
        this->edge_pts[2] = p2;
    }

}

void HDetector::four_points_transform(Mat image){

    order_points();

    Point tl = this->edge_pts[0];
    Point tr = this->edge_pts[1];
    Point br = this->edge_pts[2];
    Point bl = this->edge_pts[3];

    float widthA =  sqrt( abs ( pow( br.x - bl.x, 2.0) - pow( br.y - bl.y, 2.0 ) ) );  
    float widthB =  sqrt( abs ( pow( tr.x - tl.x, 2.0) - pow( tr.y - tl.y, 2.0 ) ) );
    /* Precisa de mencao explicita porque existe cv::max */
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

float HDetector::dot_product_angle(Point2f p1, Point2f p2, Point2f relative){
    
    float Vlenght1, Vlenght2; //modulo dos vetores
    
    Vlenght1 = sqrt((p1.x - relative.x)*(p1.x - relative.x) + (p1.y - relative.y)*(p1.y - relative.y));
    Vlenght2 = sqrt((p2.x - relative.x)*(p2.x - relative.x) + (p2.y - relative.y)*(p2.y - relative.y));
    float a = ((p1.x - relative.x)*(p2.x - relative.x) + (p1.y - relative.y)*(p2.y -relative.y))/(Vlenght1*Vlenght2);

    return acos(a);
}

bool HDetector::dot_product_check(vpf pts){
    
    float a = dot_product_angle(pts[1], pts[11], pts[0]);
    //agora o termo geral
    if (a < ANGLE_THRESH_MIN && a > ANGLE_THRESH_MAX){
        return false;
    }else{
        for (int i = 1; i <= 10; i++){

            a = dot_product_angle(pts[i+1], pts[i-1], pts[i]);
            if (a < ANGLE_THRESH_MIN && a > ANGLE_THRESH_MAX)
                return false;
        }
    }

}

Mat HDetector::detect (Mat frame){
    Mat frame2 = frame;
    cvtColor(frame, frame, CV_RGB2GRAY);
    GaussianBlur(frame, frame, Size(9,9), 0);
    GaussianBlur(frame, frame, Size(9,9), 0);
    threshold(frame, frame, 150, 255, 1); //invertido pelo exemplo TROCAR! //@caio-freitas trocaram?
    //@caio-freitas seria bom parametrizar esses valores de threshold
    vector<vp> contour;
    findContours(frame, contour, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    for(vp cnt : contour)
    {
        int peri = arcLength(cnt, true);
        vpf approx;
        approxPolyDP(cnt, approx, 0.02*peri, true);
        
        if (approx.size() == 12)
        {

            Rect2f bounds = boundingRect(approx);
            
            float a1 = dot_product_angle(approx[0] - approx[1], Point2f(0,1) );
            float a2 = dot_product_angle(approx[1] - approx[2], Point2f(0,1) );

            if( a1 < 0.1 || a2 < 0.1
               || abs(a1 - PI) < 0.1 || abs(a1 - PI) < 0.1 ){
                
                this->edge_pts = {
                    Point2f (bounds.x, bounds.y) ,
                    Point2f (bounds.x + bounds.width, bounds.y) , 
                    Point2f (bounds.x, bounds.y + bounds.height) ,
                    Point2f (bounds.x + bounds.width, bounds.y + bounds.height)
                };
            
            }else{
            
                for(Point2f v : approx){

                    if( abs(v.x - bounds.x) <= 1) edge_pts[0] = v;
                    else if( abs(v.x - (bounds.x + bounds.width) ) <= 1) edge_pts[1] = v;

                    else if( abs(v.y - bounds.y) <= 1) edge_pts[2] = v;
                    else if( abs(v.y - (bounds.y + bounds.height) ) <= 1) edge_pts[3] = v;

                }

            }

            four_points_transform(frame);
            
            if(DEBUG){
                imshow("warped", frame);

                circle(frame2, edge_pts[0], 3, (255,0,0), 3 );
                circle(frame2, edge_pts[1], 3, (255,0,0), 3 );
                circle(frame2, edge_pts[2], 3, (255,0,0), 3 );                
                circle(frame2, edge_pts[3], 3, (255,0,0), 3 );

                circle(frame2, approx[3], 3, (0,0,255), 3 );
                circle(frame2, approx[4], 3, (0,0,255), 3 );
                circle(frame2, approx[5], 3, (0,0,255), 3 );

                rectangle(frame2, bounds, (0,255,0));
                imshow("Lines", frame2);
            }
            
            if (dot_product_check(approx)) {

                Mat kernels[2];
                kernels[0] = (Mat_<float>(12,1) <<
                1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1 
                );
                kernels[1] = (Mat_<float>(12,1) <<
                0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0
                );

                Mat small_img;
                resize(this->warped, small_img, Size(12,12), INTER_AREA);
                small_img.convertTo(small_img, CV_32FC2);

                if(DEBUG){ 
                    Mat big_small_img;
                    resize(small_img, big_small_img, Size(300,300));
                    imshow("small_img", big_small_img);
                }

                int sides = sum( (small_img)*(kernels[0]) ) [0];
                int middle = sum( (small_img)*(kernels[1]) ) [0];

                int exp_sides[2] = {2*(255*12*4), 4*(255*4*4)};
                int exp_middle[2] = {255*4*4, 255*12*4};

                if( (sides >= exp_sides[0]*KERNEL_THRESH_HIGH && middle <= exp_middle[0]*KERNEL_THRESH_LOW) 
                || (sides <= exp_sides[1]*KERNEL_THRESH_LOW && middle >= exp_middle[1]*KERNEL_THRESH_HIGH) ){
                    cout << "H detectado"<< endl;                    
                }else cout << endl;
                

            }else cout << endl;
            
        }
    }
    return frame2;
}

int main(){
    Mat frame;
    VideoCapture video(0);
    HDetector* detector = new HDetector();
    video >> frame;
    while (true){
        imshow("display", detector->detect(frame));
        if (waitKey(30) == 27) break;
        video >> frame;
    }
}

