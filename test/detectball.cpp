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

            vector<Mat> channels2(3);
            cv::split(frame, channels2);
            Mat B = channels2[0];
            Mat G = channels2[1];
            Mat R = channels2[2];

            Mat I=R+B+G;

            cv::blur(frame, frame, cv::Size(10, 10));

            /*
            Mat div;
            Mat RR;
            Mat BB;
            G.convertTo(div,CV_64F);
            R.convertTo(RR,CV_64F);
            B.convertTo(BB,CV_64F);
        
            Mat r = RR/div;
            Mat b = BB/div;

            Scalar bm = mean(b);
            float bmean = bm[0];
            Scalar rm = mean(r);
            float rmean = rm[0];

            Mat r_mean;
            Mat r_std;
            meanStdDev(r,r_mean,r_std);

            Mat b_mean;
            Mat b_std;
            meanStdDev(r,b_mean,b_std);


            double sigma_r = 3.;
            double sigma_b = 3.;
            double r_minthres = r_mean.at<double>(0,0) - sigma_r*r_std.at<double>(0,0);
            double r_maxthres = r_mean.at<double>(0,0) + sigma_r*r_std.at<double>(0,0);
            double b_minthres = b_mean.at<double>(0,0) - sigma_b*b_std.at<double>(0,0);
            double b_maxthres = b_mean.at<double>(0,0) + sigma_b*b_std.at<double>(0,0);
            */


            //G HISTOGRAM 

            int histSize = 255;

	        // Set the range
	        float range[] = { 0, 255 };
	        const float* histRange = { range };

	        bool uniform = true; bool accumulate = false;

	        Mat hist;

	        // Compute the histogram
	        calcHist(&G, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
            
            
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

            





            //H HISTOGRAM SEGMENTATION

            int hhistSize = 180;

	        // Set the range
	        float hrange[] = { 0, 180 };
	        const float* hhistRange = { range };

	        Mat hhist;

	        // Compute the histogram
	        calcHist(&hframe, 1, 0, Mat(), hhist, 1, &histSize, &histRange, uniform, accumulate);
            
        

	        Mat hhistImage(hist_h, hist_w, CV_8UC3, Scalar(255, 255, 255));

            // Normalize the result to [ 0, histImage.rows ]
            normalize(hhist, hhist, 0, hhistImage.rows, NORM_MINMAX, -1, Mat());

            for (int i = 1; i < histSize; i++)
            {
                line(hhistImage, Point(bin_w*(i - 1), hist_h - cvRound(hhist.at<float>(i - 1))),
                    Point(bin_w*(i), hist_h - cvRound(hhist.at<float>(i))),
                    Scalar(0, 0, 255), 2, 8, 0);
            }

            // Display and Save
        

            double maxVal=0;
            cv::Point maxPoint;
            minMaxLoc(hhist, 0, &maxVal, 0, &maxPoint);
    

            int Hmax=maxPoint.y;

         
            //threshold on the Hue channel
            double minthres = Hmax-8;
            double maxthres = Hmax+8;
            Scalar mintable = Scalar{ minthres,0,0 };
            Scalar maxtable = Scalar{ maxthres,255,255};
            Mat threshold;
            cv::inRange(HSVframe, mintable, maxtable, threshold);

            imshow("threshold", threshold);

            cout << " hmax = " << Hmax;

            //calculate mean Hue channel
		    Scalar tempval = mean(hframe);
		    double Hmean = tempval.val[0];

		    //threshold on the Hue channel
		    double minthres2 = Hmean - 8;
		    double maxthres2 = Hmean + 8;
		    Scalar mintable2 = Scalar{ minthres2,0,0 };
		    Scalar maxtable2 = Scalar{ maxthres2,255,255 };
		    Mat threshold2;
		    cv::inRange(HSVframe, mintable2, maxtable2, threshold2);

            imshow("threshold2", threshold2);

            cout << " Hmean = " << Hmean;
            

            
            /*
            //GREEN HISTOGRAM SEGMENTATION

            

            int GhistSize = 255;

	        // Set the range
	        float Grange[] = { 0, 255 };
	        const float* GhistRange = { Grange };

	        Mat Ghist;

	        // Compute the histogram
	        calcHist(&Gframe, 1, 0, Mat(), Ghist, 1, &GhistSize, &GhistRange, uniform, accumulate);
            

            // Draw the histogram
	        int Ghist_w = 512; int Ghist_h = 400;
	        int Gbin_w = cvRound((double)Ghist_w / GhistSize);

	        Mat GhistImage(Ghist_h, Ghist_w, CV_8UC3, Scalar(255, 255, 255));

            // Normalize the result to [ 0, histImage.rows ]
            normalize(Ghist, Ghist, 0, GhistImage.rows, NORM_MINMAX, -1, Mat());

            for (int i = 1; i < histSize; i++)
            {
                line(GhistImage, Point(Gbin_w*(i - 1), Ghist_h - cvRound(hist.at<float>(i - 1))),
                    Point(Gbin_w*(i), Ghist_h - cvRound(Ghist.at<float>(i))),
                    Scalar(0, 0, 255), 2, 8, 0);
            }

            // Display and Save
            namedWindow("Original Green Image Histogram", CV_WINDOW_AUTOSIZE);
            imshow("Original Green Image Histogram", GhistImage);

            double GmaxVal=0;
            cv::Point GmaxPoint;
            minMaxLoc(Ghist, 0, &GmaxVal, 0, &GmaxPoint);
    
            cout << GmaxPoint << " " ;

            int Gmax=GmaxPoint.y;
         
         
            //threshold on the Hue channel
            double Gminthres = Gmax - 5;
            double Gmaxthres = Gmax + 5;
            Scalar Gmintable = Scalar{ 0,Gminthres, 0};
            Scalar Gmaxtable = Scalar{ 255, Gminthres, 255};
            Mat Gthreshold;
            cv::inRange(frame, Gmintable, Gmaxtable, Gthreshold);



             imshow("Gthreshold", Gthreshold);

            */
              /*

            
            // Create a structuring element
            int erosion_size = 4;
            Mat element = getStructuringElement(cv::MORPH_CROSS,
            cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
            cv::Point(erosion_size, erosion_size));

            // Apply erosion or dilation on the image
            //cv::erode(threshold, threshold, element);
            //cv::dilate(threshold, threshold, element);

           

          

            //Detect contours avec FindContours
            vector<vector<cv::Point> > contours;
            vector<Vec4i> hierarchy;
            cv::findContours(threshold, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

            vector<vector<cv::Point> > contours_poly(contours.size());
            vector<Point2f>center(contours.size());
            vector<float>radius(contours.size());

            
            double maxRadius = 1.;
            double MinRadius = 80.;
            Scalar color_ball = Scalar(255, 255, 255);
            vector<vector<cv::Point> >hull(contours.size());


            for (size_t i = 0; i < contours.size(); i++)
            {
                convexHull(Mat(contours[i]), hull[i]);
                approxPolyDP(hull[i], contours_poly[i], 3, true);
                minEnclosingCircle(contours_poly[i], center[i], radius[i]);

                // Pick the ball that best matches the users gaze.
                if (radius[i]<maxRadius && radius[i]>MinRadius)
                {
                    circle(frame, center[i], (int)radius[i], color_ball, 2, 8, 0 );
                }
            }


        */

            

            
            //imshow("threshold", threshold);
                // Write the frame into the file 'outcpp.avi'
                //video.write(frame);

           
            
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

