#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;

#define vp vector<Point>
#define vpf vector<Point2f>

/* Prototipos das funcoes nas outras branches */
vpf order_points(vpf pts);
Mat four_points_transform(Mat image, vpf pts);

struct comparison {
    bool operator() (Point2f pt1, Point2f pt2) { return (pt1.x < pt2.x);}
} comparing;

vpf order_points(vpf pts){

    sort(pts.begin(), pts.end(), comparing);
    Point2f bl;

    if( pts[0].y > pts[1].y ){
        bl = pts[1];
    }else{
        bl = pts[0];
        pts[0] = pts[1];
    }
    
    if(pts[2].y < pts[3].y){
        pts[1] = pts[3];
    }else{
        pts[1] = pts[2];
        pts[2] = pts[3];
    }
    pts[3] = bl;

    return pts;
}

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

    for(vp cnt : contour)
    {
        int peri = arcLength(cnt, true);
        vp approx;
        approxPolyDP(cnt, approx, 0.02*peri, true);
        if (approx.size() == 12)
        {
            polylines(frame2, approx, true, Scalar(0,255,0), 5, 8, 0);
            imshow("frame", frame2);

            /* Desnecessário sem debugging
            Mat box = (Mat_<float>(3,3) <<
            0,0 ,
            frame.rows, 0 ,
            frame.rows, frame.cols ,
            0, frame.cols
            );
            */
            vpf edge_pts = {
            Point2f (approx[0].x, approx[0].y) ,
            Point2f (approx[11].x, approx[11].y) , 
            Point2f (approx[5].x, approx[5].y ),
            Point2f (approx[6].x, approx[6].y)
            };

            Mat transformed = four_points_transform(frame, edge_pts);

            Mat small_img;
            resize(transformed, small_img, Size(3,3), (0,0), (0,0), INTER_AREA);

            small_img.convertTo(small_img, CV_32FC2);
            
            for(int i = 0; i < 4; i++){
                float soma = sum( (small_img)*(kernels[i]) ) [0];
                if(soma >= 1240){
                    cout << "H detectado" << endl;
                    /* cv::putText(frame, "Eh um H", ); <<-- COMPLETAR 
                    Alem disso, aqui deve ser frame ou frame2 ?
                    drawContours(frame, approx, 0, (0,255,0), 2); */ 
                }else cout << endl;
            }
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
        imshow("base", frame);
        imshow("display", detect(frame));
        if (waitKey(30) == 27) break;
        video >> frame;
    }
}
