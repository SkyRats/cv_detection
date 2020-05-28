#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;

#define ANGLE_THRESH_MIN 1
#define ANGLE_THRESH_MAX 2.2
#define vp vector<Point>
#define vpf vector<Point2f>

#define DEBUG true

// Prototipos das funcoes nas outras branches
vpf order_points(vpf pts);
Mat four_points_transform(Mat *image, vpf pts);
Mat detect(Mat frame);
float dot_product_angle(Point2f p1, Point2f p2);
bool scalar_product_check(vpf pts);


struct comparison {
    bool operator() (Point2f pt1, Point2f pt2) { return (pt1.y > pt2.y);}
} comparing;

vpf order_points(vpf pts){

    sort(pts.begin(), pts.end(), comparing);
    Point2f p1, p2;

    if(pts[0].x > pts[1].x ){
        p1 = pts[1];
        pts[1] = pts[0];
        pts[0] = p1;
    }

    if(pts[2].x < pts[3].x){
        p2 = pts[3];
        pts[3] = pts[2];
        pts[2] = p2;
    }

    //if(DEBUG) cout << pts << endl;

    return pts;
}

Mat four_points_transform(Mat *image, vpf pts){

    vpf rect = order_points(pts);

    /* Se alguem souber de um jeito mais eficiente de fazer isso,
    me avisa */ 
    Point tl = rect[0];
    Point tr = rect[1];
    Point br = rect[2];
    Point bl = rect[3];

    float widthA =  sqrt( abs ( pow( br.x - bl.x, 2) - pow( br.y - bl.y, 2 ) ) );  
    float widthB =  sqrt( abs ( pow( tr.x - tl.x, 2) - pow( tr.y - tl.y, 2 ) ) );
    /* Precisa de mencao explicita porque existe cv::max */
    float maxWidth = std::max(widthA, widthB);

    float heightA =  sqrt( abs ( pow( br.y - tr.y, 2.0 ) - pow( br.x - tr.x, 2.0) ) ); 
    float heightB =  sqrt( abs ( pow( tl.y - bl.y, 2.0 ) - pow( tl.x - bl.x, 2.0) ) );

    float maxHeight = std::max(heightA, heightB);

    vpf dst = {
        Point2f(0.0, 0.0) ,
        Point2f(maxWidth, 0.0) ,
        Point2f(maxWidth, maxHeight) ,
        Point2f(0.0, maxHeight)
        // @caio-freitas talvez mostrar esses pontos na imagem final numerados ajude a debugar o código
    };

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


bool scalar_product_check(vpf pts){
    float relativeX;
    float relativeY;
    float Vlenght1, Vlenght2; //modulo dos vetores
    // a primeira vez é diferente das outras e precisa ser feita manualmente
    relativeX = pts[0].x; 
    relativeY = pts[0].y;
    Vlenght1 = sqrt((pts[1].x - relativeX)*(pts[1].x - relativeX) + (pts[1].y - relativeY)*(pts[1].y - relativeY));
    Vlenght2 = sqrt((pts[11].x - relativeX)*(pts[11].x - relativeX) + (pts[11].y - relativeY)*(pts[11].y - relativeY));
    float a = ((pts[1].x - relativeX)*(pts[11].x - relativeX) + (pts[1].y - relativeY)*(pts[11].y - relativeY))/(Vlenght1*Vlenght2);
    //agora o termo geral
    if (acos(a) > ANGLE_THRESH_MIN and acos(a) < ANGLE_THRESH_MAX){
        for (int i = 1; i <= 10; i++){
            relativeX = pts[i].x; 
            relativeY = pts[i].y;
            Vlenght1 = sqrt((pts[i+1].x - relativeX)*(pts[i+1].x - relativeX) + (pts[i+1].y - relativeY)*(pts[i+1].y - relativeY));
            Vlenght2 = sqrt((pts[i-1].x - relativeX)*(pts[i-1].x - relativeX) + (pts[i-1].y - relativeY)*(pts[i-1].y - relativeY));
            a = ((pts[i+1].x - relativeX)*(pts[i-1].x - relativeX) + (pts[i+1].y - relativeY)*(pts[i-1].y - relativeY))/(Vlenght1*Vlenght2);
            if (acos(a) > ANGLE_THRESH_MIN and acos(a) < ANGLE_THRESH_MAX){
                continue;
            }
            else{
                return false;
            } 
        }
    }
    else{
        return false;
    }
    return true;
}

Mat detect (Mat frame)
{
    /* Acho que deveriamos renomear frame2 para original
    e frame para processed ou alguma coisa assim */
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

            //if(DEBUG) polylines(frame2, approx, true, Scalar(0,255,0), 5, 8, 0);

            Rect2f bounds = boundingRect(approx);

            vpf edge_pts = {Point2f(0,0), Point2f(0,0), Point2f(0,0), Point2f(0,0)};
            
            float a1 = dot_product_angle(approx[0] - approx[1], Point2f(0,1) );
            float a2 = dot_product_angle(approx[1] - approx[2], Point2f(0,1) );

            if( a1 < 0.1 || a2 < 0.1
               || abs(a1 - PI) < 0.1 || abs(a1 - PI) < 0.1 )
                
                edge_pts = {
                    Point2f (bounds.x, bounds.y) ,
                    Point2f (bounds.x + bounds.width, bounds.y) , 
                    Point2f (bounds.x + bounds.width, bounds.y) , 
                    Point2f (bounds.x + bounds.width, bounds.y) , 
                    Point2f (bounds.x, bounds.y + bounds.height) ,
                    Point2f (bounds.x + bounds.width, bounds.y + bounds.height)
                };
            
            else
            
                for(Point2f v : approx){

                    if( abs(v.x - bounds.x) <= 1) edge_pts[0] = v;
                    else if( abs(v.x - (bounds.x + bounds.width) ) <= 1) edge_pts[1] = v;

                    else if( abs(v.y - bounds.y) <= 1) edge_pts[2] = v;
                    else if( abs(v.y - (bounds.y + bounds.height) ) <= 1) edge_pts[3] = v;

                }

            Mat perspective = four_points_transform(&frame, edge_pts);
            if(DEBUG) imshow("warped", frame);

            vpf h_approx;
            approxPolyDP(approx, h_approx, 0.02*peri, true);

             if(DEBUG){
                circle(frame2, h_approx[0], 3, (255,0,0), 3 );
                circle(frame2, h_approx[1], 3, (255,0,0), 3 );
                circle(frame2, h_approx[2], 3, (255,0,0), 3 );
                circle(frame2, h_approx[3], 3, (255,0,0), 3 );
                circle(frame2, h_approx[4], 3, (255,0,0), 3 );
                circle(frame2, h_approx[5], 3, (255,0,0), 3 );
                circle(frame2, h_approx[6], 3, (255,0,0), 3 );
                circle(frame2, h_approx[7], 3, (255,0,0), 3 );
                circle(frame2, h_approx[8], 3, (255,0,0), 3 );
                circle(frame2, h_approx[9], 3, (255,0,0), 3 );
                circle(frame2, h_approx[10], 3, (255,0,0), 3 );
                circle(frame2, h_approx[11], 3, (255,0,0), 3 );
                imshow("Circles", frame2);
            }
            
            if (scalar_product_check(approx)) {

                Mat kernels[4];
                kernels[0] = (Mat_<float>(3,3) << 
                -1, 3.5, -1 ,
                -1, -1, -1 ,
                -1, 3.5, -1
                );
                kernels[1] = (Mat_<float>(3,3) <<
                5, -0.35, -1 ,
                -0.35, -1, -1 ,
                -1, -1, 5
                );
                kernels[2] = (Mat_<float>(3,3) << 
                -1, -1, -1 ,
                5, -1, 5 ,
                -1, -1, -1
                );
                kernels[3] = (Mat_<float>(3,3) << 
                -0.35, -1, 5 ,
                -1, -1, -1 ,
                5, -0.35, -1
                );

                Mat small_img;
                resize(frame, small_img, Size(3,3), /*(0,0), (0,0),*/ INTER_CUBIC);
                small_img.convertTo(small_img, CV_32FC2);


                //polylines(frame2, approx, true, Scalar(0,255,0), 5, 8, 0); <<-- Da core dump                
                if(DEBUG){ 
                    Mat big_small_img;
                    resize(small_img, big_small_img, Size(300,300));
                    imshow("small_img", big_small_img);
                }

                for(int i = 0; i < 4; i++){
                    float soma = sum( (small_img)*(kernels[i]) ) [0];
                    if(soma >= 1240.0){
                        cout << "H detectado : \t" << soma << endl;
                        /* cv::putText(frame, "Eh um H", ); <<-- COMPLETAR 
                        Alem disso, aqui deve ser frame ou frame2 ?
                        drawContours(frame, approx, 0, (0,255,0), 2); */ 
                    }
                    soma = 0.0;
                }
            }else cout << endl;
            
        }
    }
    /* frame (processado) ou frame2 (original)? */
    return frame;
}

int main()
{
    Mat frame;
    VideoCapture video(0);
    video >> frame;
    while (true)
    {
        imshow("display", detect(frame));
        if (waitKey(30) == 27) break;
        video >> frame;
    }
}

