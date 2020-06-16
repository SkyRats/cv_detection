#include <iostream>
#include <opencv2/opencv.hpp>
#include <list>

#define MIN_PIX_VALUE 60
#define MOTION_THRESHOLD 2000 // no quarto do @pedro-fuoco, com todas as luzes acesas
#define DEBUG true
#define DILATION_SIZE 20 //mexer aqui para mudar a precisão (solução suficiente por enquanto, talvez precise mudar para aplicaões especificas)
using namespace std;
using namespace cv;


class MotionDetector {
    private:
        bool primeira;
        int total_sum;
        Mat old_gray, new_gray, cv_frame;
        void image_treat();
        void bounding_recs();
        bool is_relevant(vector<Point> rect);

    public:
<<<<<<< HEAD
        MotionDetector();
=======
        MotionDetecter();
        void frame_reset(Mat frame);
>>>>>>> master
        bool detect(Mat new_image); // ver se é a primera, se for a primeira retornar false
        vector<Rect> relevant_rectangles;
};

<<<<<<< HEAD
MotionDetector::MotionDetector(){
    this->primeira = true;
};

bool MotionDetector::detect(Mat new_image) {
=======
void MotionDetecter::frame_reset(Mat frame){
    cvtColor(frame, this->old_gray, CV_RGB2GRAY);
};
MotionDetecter::MotionDetecter(){
    this->primeira = true;
};

bool MotionDetecter::detect(Mat frame) {
    Mat new_image = frame;
>>>>>>> master
    if (this->primeira == true) {
        cvtColor(new_image, this->old_gray, CV_RGB2GRAY);
        this->primeira = false;
        return false;
    }
    else {
        cvtColor(new_image, this->new_gray, CV_RGB2GRAY);
        image_treat();
        bounding_recs();
        //if(this->total_sum != 0) {
        if(DEBUG){
            if (DEBUG) {
                for(int i = 0; i < relevant_rectangles.size(); i++){
                    rectangle(new_image, relevant_rectangles[i], Scalar(0,255,0),3,8,0);
                }
                //putText(new_image, "movement detected", Point(275,25),  FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0), 3, 8, false);
                imshow("Display", new_image);
            }
            //return true;
        }
    }
};

<<<<<<< HEAD
bool MotionDetector::is_relevant(vector<Point> rect) {
    if(contourArea(rect) > 1000){ //precisamos definir como veremos a area minima
=======

bool MotionDetecter::is_relevant(vector<Point> rect) {
    if(contourArea(rect) > 10000){ //precisamos definir como veremos a area minima
>>>>>>> master
        return true;
    }
};

void MotionDetector::image_treat() {
    Mat subtracted_frame = this->old_gray - this->new_gray;
    imshow("new", old_gray);
    threshold(subtracted_frame, subtracted_frame, MIN_PIX_VALUE , 255 , 0 );
    dilate(subtracted_frame, subtracted_frame, getStructuringElement(MORPH_RECT, Size(1.25*DILATION_SIZE,1.25*DILATION_SIZE), Point(DILATION_SIZE, DILATION_SIZE)));
    Canny(subtracted_frame, subtracted_frame, 75, 225, 3);
    this->cv_frame = subtracted_frame;
    imshow("tratado", this->cv_frame);
};

void MotionDetector::bounding_recs() {
    relevant_rectangles.clear();
    vector<vector<Point>> stored_contours;
    vector<Point> poly;
    findContours(this->cv_frame, stored_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE); 
    // aqui achamos os contornos que existem na imagem, eles são armazenados em stored_contours
    this->total_sum = 0;
    for(int i = 0; i < stored_contours.size(); i++){ 
        approxPolyDP(stored_contours[i], poly, 20, true); //transforma o contorno detectado em um poligono poly
        if (is_relevant(poly)){
            this->relevant_rectangles.push_back(boundingRect(poly)); //envia o rect relevante para a lista
            this->total_sum += sum(sum(poly))[0];
        }
    }
};


int main() {
    Mat frame, reseter;
    VideoCapture video(-1); // captures video from default cam

<<<<<<< HEAD
    MotionDetector* detecter = new MotionDetector;
=======
    MotionDetecter* detecter = new MotionDetecter;
    int i = 0;
>>>>>>> master
    while (true) {
        video >> frame;
        video >> reseter;
        detecter->detect(frame);
        waitKey(30);
        detecter->frame_reset(reseter);
    }   
}