#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;

#define ANGLE_THRESH_MIN 1
#define ANGLE_THRESH_MAX 2.2
<<<<<<< HEAD
#define KERNEL_THRESH_MAX 1.2
#define KERNEL_THRESH_MIN 0.9
=======
>>>>>>> master
#define PI 3.14159

#define vp vector<Point>
#define vpf vector<Point2f>

// Set true for debugging purposes, showing internals of algorithm
#define DEBUG true

<<<<<<< HEAD
// Sorts points based on y coordinate
=======
// Prototipos das funcoes
vpf order_points(vpf pts);
void four_points_transform(Mat *image, vpf pts);
Mat detect(Mat frame);
float dot_product_angle(Point2f p1, Point2f p2);
bool scalar_product_check(vpf pts);


>>>>>>> master
struct comparison {
    bool operator() (Point2f pt1, Point2f pt2) { return (pt1.y > pt2.y);}
} comparing;

class HDetector {
    private:
        vpf edge_pts = { Point2f(0,0), Point2f(0,0), Point2f(0,0), Point2f(0,0) };
        void order_points();
        void four_points_transform(Mat image);
        float angle(Point2f v1, Point2f v2, Point2f relative = Point2f(0,0) );
        bool angle_check(vpf pts);
    public:
        Mat warped; 
        HDetector();
        Mat detect (Mat frame);

};

HDetector::HDetector() {}

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

<<<<<<< HEAD
}

/* Takes an image as argument and returns warped perspective, moving edge_pts to
the edge of the frame */
void HDetector::four_points_transform(Mat image){
=======
    //if(DEBUG) cout << pts << endl;

    return pts;
}

void four_points_transform(Mat *image, vpf pts){
>>>>>>> master

    order_points();

    Point tl = this->edge_pts[0];
    Point tr = this->edge_pts[1];
    Point br = this->edge_pts[2];
    Point bl = this->edge_pts[3];

<<<<<<< HEAD
    float widthA =  sqrt( abs ( pow( br.x - bl.x, 2.0) - pow( br.y - bl.y, 2.0 ) ) );  
    float widthB =  sqrt( abs ( pow( tr.x - tl.x, 2.0) - pow( tr.y - tl.y, 2.0 ) ) );
    // Requires explicit reference due to cv::max
=======
    float widthA =  sqrt( abs ( pow( br.x - bl.x, 2) - pow( br.y - bl.y, 2 ) ) );  
    float widthB =  sqrt( abs ( pow( tr.x - tl.x, 2) - pow( tr.y - tl.y, 2 ) ) );
    /* Precisa de mencao explicita porque existe cv::max */
>>>>>>> master
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

<<<<<<< HEAD
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
=======
    Mat M = getPerspectiveTransform(rect, dst);
    Mat warped;
    warpPerspective(*image, warped, M, Size(maxWidth, maxHeight));
    *image = warped;
    //if(DEBUG) imshow("warped", warped); // @caio-freitas ver a imagem cropada tbm é útil pra debugar
    return M;

}

float dot_product_angle(Point2f p1, Point2f p2){
    
    float Vlenght1, Vlenght2; //modulo dos vetores
    // a primeira vez é diferente das outras e precisa ser feita manualmente
    
    Vlenght1 = sqrt((p1.x)*(p1.x) + (p1.y)*(p1.y));
    Vlenght2 = sqrt((p2.x)*(p2.x) + (p2.y)*(p2.y));
    float a = ((p1.x)*(p2.x) + (p1.y)*(p2.y))/(Vlenght1*Vlenght2);

    return acos(a);
    }

>>>>>>> master

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
<<<<<<< HEAD
        }
=======
            } 
        }
    }
    else{
        return false;
>>>>>>> master
    }

}

<<<<<<< HEAD
// Takes an image 'frame' and detects whether it contains the letter H
Mat HDetector::detect (Mat frame){
=======
Mat detect (Mat frame){

>>>>>>> master
    Mat frame2 = frame;
    cvtColor(frame, frame, CV_RGB2GRAY);
    // Blur and threshold remove noise from image
    GaussianBlur(frame, frame, Size(9,9), 0);
    GaussianBlur(frame, frame, Size(9,9), 0);
    threshold(frame, frame, 150, 255, 1);

    vector<vp> contour;
    findContours(frame, contour, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

<<<<<<< HEAD
    for(vp cnt : contour){

=======
    for(vp cnt : contour)
    {
>>>>>>> master
        int peri = arcLength(cnt, true);
        vpf approx;
        approxPolyDP(cnt, approx, 0.02*peri, true);
        
<<<<<<< HEAD
        if (approx.size() == 12){

            Rect2f bounds = boundingRect(approx);
            
            float a1 = angle(approx[0] - approx[1], Point2f(0,1) );
            float a2 = angle(approx[1] - approx[2], Point2f(0,1) );

            /* If the sides of the H are very close to parallel to its bounds,
                use the bounding rect vertices for warp */
            if( a1 < 0.1 || a2 < 0.1
               || abs(a1 - PI) < 0.1 || abs(a1 - PI) < 0.1 ){
                
                this->edge_pts = {
                    Point2f (bounds.x, bounds.y) ,
                    Point2f (bounds.x + bounds.width, bounds.y) , 
=======
        if (approx.size() == 12)
        {

            //if(DEBUG) polylines(frame2, approx, true, Scalar(0,255,0), 5, 8, 0);

            Rect2f bounds = boundingRect(approx);

            vpf edge_pts = {Point2f(0,0), Point2f(0,0), Point2f(0,0), Point2f(0,0)};
            
            float a1 = dot_product_angle(approx[0] - approx[1], Point2f(0,1) );
            float a2 = dot_product_angle(approx[1] - approx[2], Point2f(0,1) );

            if( a1 < 0.1 || a2 < 0.1
               || abs(a1 - PI) < 0.1 || abs(a1 - PI) < 0.1 ){
                
                edge_pts = {
                    Point2f (bounds.x, bounds.y) ,
                    Point2f (bounds.x + bounds.width, bounds.y) , 
                    Point2f (bounds.x + bounds.width, bounds.y) , 
                    Point2f (bounds.x + bounds.width, bounds.y) , 
>>>>>>> master
                    Point2f (bounds.x, bounds.y + bounds.height) ,
                    Point2f (bounds.x + bounds.width, bounds.y + bounds.height)
                };
            
<<<<<<< HEAD
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
=======
            else
            
                for(Point2f v : approx){

                    if( abs(v.x - bounds.x) <= 1) edge_pts[0] = v;
                    else if( abs(v.x - (bounds.x + bounds.width) ) <= 1) edge_pts[1] = v;

                    else if( abs(v.y - bounds.y) <= 1) edge_pts[2] = v;
>>>>>>> master
                    else if( abs(v.y - (bounds.y + bounds.height) ) <= 1) edge_pts[3] = v;

                }

            }

<<<<<<< HEAD
            four_points_transform(frame);
            
            if(DEBUG){
                imshow("warped", frame);

                // Shows captures edge of H in black
=======
            four_points_transform(&frame, edge_pts);
            
            if(DEBUG){

                imshow("warped", frame);

>>>>>>> master
                circle(frame2, edge_pts[0], 3, (255,0,0), 3 );
                circle(frame2, edge_pts[1], 3, (255,0,0), 3 );
                circle(frame2, edge_pts[2], 3, (255,0,0), 3 );                
                circle(frame2, edge_pts[3], 3, (255,0,0), 3 );
<<<<<<< HEAD
                // Draws bound
=======

                circle(frame2, approx[3], 3, (0,0,255), 3 );
                circle(frame2, approx[4], 3, (0,0,255), 3 );
                circle(frame2, approx[5], 3, (0,0,255), 3 );

>>>>>>> master
                rectangle(frame2, bounds, (0,255,0));
                imshow("Lines", frame2);
            }
            
<<<<<<< HEAD
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
=======
            if (scalar_product_check(approx)) {

                Mat kernels[2];
                kernels[0] = (Mat_<float>(12,1) <<
                1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1 
                );
                kernels[1] = (Mat_<float>(12,1) <<
                0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0
                );

                Mat small_img;
                resize(frame, small_img, Size(12,12), INTER_AREA);
                small_img.convertTo(small_img, CV_32FC2);


                //polylines(frame2, approx, true, Scalar(0,255,0), 5, 8, 0); <<-- Da core dump                
                if(DEBUG){ 

                    //cout << small_img << endl;

                    Mat big_small_img;
>>>>>>> master
                    resize(small_img, big_small_img, Size(300,300));
                    imshow("small_img", big_small_img);
                }

<<<<<<< HEAD
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
=======
                int sides = sum( (small_img)*(kernels[0]) ) [0];
                int middle = sum( (small_img)*(kernels[1]) ) [0];

                int exp_sides[2] = {2*(255*12*4), 4*(255*4*4)};
                int exp_middle[2] = {255*4*4, 255*12*4};

                //Precisam ser melhor ajustados
                float HIGH_THRESH = 0.9;
                float LOW_THRESH = 1.2;

                if( (sides >= exp_sides[0]*HIGH_THRESH && middle <= exp_middle[0]*LOW_THRESH) 
                || (sides <= exp_sides[1]*LOW_THRESH && middle >= exp_middle[1]*HIGH_THRESH) ){
>>>>>>> master
                    cout << "H detectado"<< endl;                    
                }else cout << endl;
                

            }else cout << endl;
            
        }
    }
    return frame2;
}

// For testing
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

