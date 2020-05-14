#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;

#define ANGLE_THRESH_MIN 1
#define ANGLE_THRESH_MAX 2.2
#define vp vector<Point>
#define vpf vector<Point2f>

#define DEBUG true

/* Prototipos das funcoes nas outras branches */
vpf order_points(vpf pts);
Mat four_points_transform(Mat image, vpf pts);
Mat detect(Mat frame);
bool scalar_product_check(vpf pts);

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

    vpf dst = {
        Point2f(0.0, 0.0) ,
        Point2f(maxWidth, 0.0) ,
        Point2f(maxWidth, maxHeight) ,
        Point2f(0.0, maxHeight)
        // @caio-freitas talvez mostrar esses pontos na imagem final numerados ajude a debugar o código
    };

    Mat M = getPerspectiveTransform(rect, dst);
    Mat warped; 
    warpPerspective(image, warped, M, Size(maxWidth, maxHeight));
    if(DEBUG) imshow("warped", warped); // @caio-freitas ver a imagem cropada tbm é útil pra debugar
    return warped;

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
            vpf all_points  = {
                Point2f (approx[0].x, approx[0].y) ,
                Point2f (approx[1].x, approx[1].y) ,
                Point2f (approx[2].x, approx[2].y) ,
                Point2f (approx[3].x, approx[3].y) ,
                Point2f (approx[4].x, approx[4].y) ,
                Point2f (approx[5].x, approx[5].y) ,
                Point2f (approx[6].x, approx[6].y) ,
                Point2f (approx[7].x, approx[7].y) ,
                Point2f (approx[8].x, approx[8].y) ,
                Point2f (approx[9].x, approx[9].y) ,
                Point2f (approx[10].x, approx[10].y) ,
                Point2f (approx[11].x, approx[11].y)};
            if (scalar_product_check(all_points)){
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
                if(DEBUG) imshow("small_img", small_img); // @caio-freitas mostrar o kernel tbm é bom
                /*
                @caio-freitas problema:
                A imagem 3x3 é mostrada nessa mesma resolução, oq deixa impossível de ler
                isso não acontece no python...
                Para mostra-la, acho que teremos que mudar sua escala
                */
                for(int i = 0; i < 4; i++){
                    float soma = sum( (small_img)*(kernels[i]) ) [0];
                    if(soma >= 1240){ // @caio-freitas esse valor de threshold pode não estar certo, podemos brincar com ele
                        cout << "H detectado" << endl;
                        /* cv::putText(frame, "Eh um H", ); <<-- COMPLETAR 
                        Alem disso, aqui deve ser frame ou frame2 ?
                        drawContours(frame, approx, 0, (0,255,0), 2); */ 
                    }else cout << endl;
                }
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
        imshow("display", detect(frame));
        if (waitKey(30) == 27) break;
        video >> frame;
    }
}

