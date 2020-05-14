#include <iostream>
#include <opencv2/opencv.hpp>
#include <list>

#define MIN_PIX_VALUE 40
#define MOTION_THRESHOLD 2000 // no quarto do @pedro-fuoco, com todas as luzes acesas
#define DEBUG true
#define DILATION_SIZE 7
using namespace std;
using namespace cv;


class MotionDetecter {
    private:
        bool primeira;
        int total_sum;
        Mat old_gray, new_gray, cv_frame;
        void image_treat();
        void bounding_recs();
        bool is_relevant(vector<Point> rect);
    public:
        MotionDetecter();
        bool detect(Mat new_image); // ver se é a primera, se for a primeira retornar false
        vector<Rect> relevant_rectangles;
};

MotionDetecter::MotionDetecter(){
    this->primeira = true;
};

bool MotionDetecter::detect(Mat new_image) {
    if (this->primeira == true) {
        cvtColor(new_image, this->old_gray, CV_RGB2GRAY);
        this->primeira = false;
        return false;
    }
    else {
        cvtColor(new_image, this->new_gray, CV_RGB2GRAY);
        image_treat();
        bounding_recs();
        if(this->total_sum != 0) {
            if (DEBUG) {
                for(int i = 0; i < relevant_rectangles.size(); i++){
                    rectangle(new_image, relevant_rectangles[i], Scalar(0,255,0),3,8,0);
                }
                putText(new_image, "movement detected", Point(275,25),  FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0), 3, 8, false);
                imshow("Display", new_image);
            }
            //return true;
        }
    }
};


bool MotionDetecter::is_relevant(vector<Point> rect) {
    if(contourArea(rect) > 1000){ //precisamos definir como veremos a area minima
        return true;
    }
};

void MotionDetecter::image_treat() {
    Mat subtracted_frame = this->old_gray - this->new_gray;
    threshold(subtracted_frame, subtracted_frame, MIN_PIX_VALUE , 255 , 0 );
    dilate(subtracted_frame, subtracted_frame, getStructuringElement(MORPH_RECT, Size(1.25*DILATION_SIZE,1.25*DILATION_SIZE), Point(DILATION_SIZE, DILATION_SIZE)));
    Canny(subtracted_frame, subtracted_frame, 75, 225, 3);
    this->cv_frame = subtracted_frame;
    imshow("tratado", this->cv_frame);
};

void MotionDetecter::bounding_recs() {
    relevant_rectangles.clear();
    vector<vector<Point>> stored_contours;
    vector<Point> poly;
    findContours(this->cv_frame, stored_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE); 
    // aqui achamos os contornos que existem na imagem, eles são armazenados em stored_contours
    this->total_sum = 0;
    // existe um possível bug que o findContours detecta os cantos da tela, pode ser resolvido com int i = 1
    for(int i = 0; i < stored_contours.size(); i++){ 
        approxPolyDP(stored_contours[i], poly, 20, true); //transforma o contorno detectado em um poligono poly
        if (is_relevant(poly)){
            this->relevant_rectangles.push_back(boundingRect(poly)); //envia o rect relevante para a lista
            this->total_sum += sum(sum(poly))[0];
        }
    }
};



/* movarea = boundingRect(result);

        if(2*movarea.area() >= (result.cols*result.rows)){
            
            Size size = result.size();

            resize(result, result, Size(100, 100), 2, 2);
            Rect checkarea = boundingRect(result);
            thresh = 100;

        }else checkarea = movarea;
    
        x0 = checkarea.x;
        y0 = checkarea.y;

        w = checkarea.width + x0;
        h = checkarea.height + y0;
        
        if(sum(result)[0]/255 > thresh){
            std::cout << "MOVING\n";
            rectangle(frame1, movarea, Scalar(255, 0, 0), 2);
        }else std::cout << std::endl;   */

/*
int main()  {
    Mat old_frame,old_frame_gray, current_frame,current_frame_gray, cv_frame;
    
    
    
    cvtColor(current_frame, current_frame_gray, CV_RGB2GRAY);
    
    while(true) {
        video >> old_frame; // Stores current video frame

        cvtColor(old_frame, old_frame_gray, CV_RGB2GRAY);           // Converts from
        cvtColor(current_frame, current_frame_gray, CV_RGB2GRAY);   // BGR to Gray

        cv_frame = image_treat(current_frame_gray - old_frame_gray);

        Scalar t = sum(cv_frame);


        if(t[0] > MOTION_THRESHOLD) {
            rectangle(current_frame, boundingRect(cv_frame), Scalar(0,255,0),3,8,0);
            putText(current_frame, "movement detected", Point(275,25),  FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0), 3, 8, false);
        }
        
        if (DEBUG) imshow("Display", current_frame);
        waitKey(30);
        video >> current_frame;

    }
}
*/


int main() {
    Mat frame;
    VideoCapture video(-1); // captures video from default cam

    MotionDetecter* detecter = new MotionDetecter;
    while (true) {
        video >> frame;
        detecter->detect(frame);
        waitKey(30);
    }   
}