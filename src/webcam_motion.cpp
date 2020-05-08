#include <iostream>
#include <opencv2/opencv.hpp>

#define MIN_PIX_VALUE 45
#define MOTION_THRESHOLD 2000 // no quarto do @pedro-fuoco, com todas as luzes acesas
#define DEBUG true
#define DILATION_SIZE 7

using namespace cv;


class MotionDetecter {
    private:
        bool primeira = true;
        Scalar sum;
        Mat old_gray, new_gray, cv_frame;
        void image_treat(Mat subtracted_frame);
        void bounding_recs(Mat treated_image);
    public:
        bool detect(Mat new_image); // ver se é a primera, se for a primeira retornar false
}

bool MotionDetecter::detect(Mat new_image) {
    if (this->primeira == true) {
        cvtColor(new_image, this->old_gray, CV_RGB2GRAY);
        return false;
    }
    else {
        cvtColor(new_image, this->new_gray, CV_RGB2GRAY);
        this->image_treat();
        this->bounding_recs();

        if(this->sum != 0) {
            if (DEBUG) {
                rectangle(this->new_gray, boundingRect(cv_frame), Scalar(0,255,0),3,8,0);
                putText(this->new_gray, "movement detected", Point(275,25),  FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0), 3, 8, false);
                imshow("Display", this->new_gray);
            }
            return true;
        }
        // todo o resto
    }
}

void MotionDetecter::image_treat() {
    Mat subtracted_frame = this->new_gray - this->old_gray;
    threshold(subtracted_frame, subtracted_frame, MIN_PIX_VALUE , 255 , 0 );
    dilate(subtracted_frame, subtracted_frame, getStructuringElement(MORPH_RECT, Size(1.25*DILATION_SIZE,1.25*DILATION_SIZE), Point(DILATION_SIZE, DILATION_SIZE)));
    Canny(subtracted_frame, subtracted_frame, 75, 225, 3);
    this->cv_frame = subtracted_frame;
}

void MotionDetecter::bounding_recs(Mat)    {
    // Usar o this->cv_frame
    // Retorno por imagem
    // retornar soma no atributo this->sum





    // Todo retorno vetor
}



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

    MotionDetecter detecter = new MotionDetecter;
    while (true) {
        video >> frame;
        detecter.detect(frame);
        waitKey(30);
    }   
}