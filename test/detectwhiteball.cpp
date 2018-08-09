// ConsoleApplication1.cpp : Defines the entry point for the console application.
//


#include <opencv2/opencv.hpp>
#include <iostream>
#include<opencv2/video/video.hpp>
#include<opencv2/video/background_segm.hpp>


using namespace std;
using namespace cv;

//global variables

Mat frame;

int main() {
    
    // Create a VideoCapture object and open the input file
    // If the input is the web camera, pass 0 instead of the video file name
    VideoCapture cap("ball.mp4");
    
    // Default resolution of the frame is obtained.The default resolution is system dependent.
    int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    
    
    // Check if camera opened successfully
    if (!cap.isOpened()) {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
    
    bool Play = true;
    
    while (1) {
        
        // Capture frame-by-frame
        if (Play){
            cap >> frame;
            
            // If the frame is empty, break immediately
            if (frame.empty())
                break;

            
        

            // H channel
            Mat HSVframe;
            vector<Mat> channels(3);
            cv::cvtColor(frame, HSVframe, CV_BGR2HSV);
            cv::split(HSVframe, channels);
            Mat hframe;
            hframe = channels[0];

            Mat gray;
            cv::cvtColor(frame, gray, CV_BGR2GRAY);

            frame.convertTo(frame, CV_64FC1);
            frame = frame /255.f;


            vector<Mat> channels2(3);
            cv::split(frame, channels2);
            Mat B = channels2[0];
            Mat G = channels2[1];
            Mat R = channels2[2];
            Mat I=R+B+G;


            double ImaxVal=0;
            cv::Point ImaxPoint;
            minMaxLoc(I, 0, &ImaxVal, 0, &ImaxPoint);
            I=(I/ImaxVal)*255;

            I.convertTo(I,CV_8UC1);

            Mat Ithreshold;

            cv::threshold(I,Ithreshold,179,255,THRESH_BINARY);

            imshow("thres",Ithreshold);

            Mat Icontours;

            Canny(Ithreshold,Icontours,100,100);


            std::vector<cv::Vec3f> Icircles;
            HoughCircles(Icontours,Icircles, CV_HOUGH_GRADIENT,
                            2, //accumulator resolution 
                            100, //min distance between two circles
                            100, //Canny high threshold
                            50, // min number of votes
                            10,60); // min and max radius
    

            
            
            std::vector<Vec3f>::const_iterator Icir = Icircles.begin();

            while(Icir!=Icircles.end())
            {
                circle(Icontours, cv::Point((* Icir)[0],(* Icir)[1]),(* Icir)[2],Scalar{100},3);
                ++Icir;
            }

            imshow("Icontours",Icontours);
            



            /*

            Mat gc = Mat(Size(hframe.cols, hframe.rows), CV_64FC1);
            for (int i = 0 ; i < hframe.rows; ++i)
            {
                for (int j = 0 ; j < hframe.cols; ++j)
                {
                    double g = G.at<double>(i, j);
                    double r = R.at<double>(i, j);
                    double b = B.at<double>(i, j);
                    double sRGB = b+g+r;
                    gc.at<double>(i, j) = (g/sRGB)*255;
                }
            }
            
            
            gc.convertTo(gc,CV_8UC1);

            //GGC HISTOGRAM 

            int histSize = 255;

	        // Set the range
	        float range[] = { 0, 255 };
	        const float* histRange = { range };

	        bool uniform = true; bool accumulate = false;

	        Mat hist;

	        // Compute the histogram
	        calcHist(&gc, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
            
            
            // Draw the histogram
	        int hist_w = 512; int hist_h = 400;
	        int bin_w = cvRound((double)hist_w / histSize);

	        Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(255, 255, 255));

            // Normalize the result to [ 0, histImage.rows ]
            normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

            for (int i = 1; i < histSize; i++)
            {
                line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
                    Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))),
                    Scalar(0, 0, 255), 2, 8, 0);
            }

            double maxVal=0;
            cv::Point maxPoint;
            minMaxLoc(hist, 0, &maxVal, 0, &maxPoint);
    
            int GCmax=maxPoint.y;

            Mat threshold;

            cv::threshold(gc,threshold,GCmax-11,255,THRESH_BINARY);


            Mat contours;

           
        
            Canny(threshold,contours,100,100);


            std::vector<cv::Vec3f> circles;
            HoughCircles(contours,circles, CV_HOUGH_GRADIENT,
                            2, //accumulator resolution 
                            100, //min distance between two circles
                            100, //Canny high threshold
                            50, // min number of votes
                            10,100); // min and max radius
    

            
            
            std::vector<Vec3f>::const_iterator cir = circles.begin();

            while(cir!=circles.end())
            {
                circle(contours, cv::Point((* cir)[0],(*cir)[1]),(*cir)[2],Scalar{100},3);
                ++cir;
            }

            

       
        //imshow("hist",histImage);
        //imshow("contours",contours);
        //imshow("gc",gc);
        //imshow("frame",frame);

        */
            
        }
        
        //Press Space to pause
        char b = (char)waitKey(1);
        if (b == 'p')
            Play = !Play;
        
        
        // Press  ESC on keyboard to exit
        char c = (char)waitKey(25);
        if (c == 27)
            break;
        
    }
    
    // When everything done, release the video capture object
    cap.release();
    //video.release();
    
    // Closes all the frames
    destroyAllWindows();
    
    return 0;
}

