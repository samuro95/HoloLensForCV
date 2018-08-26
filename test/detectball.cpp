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
    VideoCapture cap("t1.mp4");
    
    // Default resolution of the frame is obtained.The default resolution is system dependent.
    int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    
    
    // Check if camera opened successfully
    if (!cap.isOpened()) {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
    
    bool Play = true;
    int i=0;
    while (1 && i<2) {
        
        // Capture frame-by-frame
        if (Play){
            cap >> frame;
            
            // If the frame is empty, break immediately
            if (frame.empty())
                break;
            
            i=i+1;

            imwrite("situation.png",frame);

            // H channel
            Mat HSVframe;
            vector<Mat> channels(3);
            cv::cvtColor(frame, HSVframe, CV_BGR2HSV);
            cv::split(HSVframe, channels);
            Mat hframe;
            hframe = channels[0];
            Mat S = channels[1];
            Mat V = channels[2];

            /*
            frame.convertTo(frame, CV_64FC4);
            frame = frame /255.f;

            vector<Mat> channels2(3);
            cv::split(frame, channels2);
            Mat B = channels2[0];
            Mat G = channels2[1];
            Mat R = channels2[2];

            Mat I=R+B+G;

            Mat gc = Mat(G.size(), CV_64FC1);
            Mat bc = Mat(B.size(), CV_64FC1);
            Mat rc = Mat(R.size(), CV_64FC1);

           
            for (int i = 0 ; i < G.rows; ++i)
            {
                for (int j = 0 ; j < G.cols; ++j)
                {
                    double g = G.at<double>(i, j);
                    double r = R.at<double>(i, j);
                    double b = B.at<double>(i, j);
                    double sRGB = b+g+r;
                    
                    if (sRGB>0.001)
                    {
                        gc.at<double>(i, j) = (g/sRGB)*255.;
                        bc.at<double>(i, j) = (b/sRGB)*255.;
                        rc.at<double>(i, j) = (r/sRGB)*255.;
                    }
                    else
                    {
                        gc.at<double>(i, j) = 0.0;
                        bc.at<double>(i, j) = 0.0;
                        rc.at<double>(i, j) = 0.0;
                    }
                }
            }

            gc.convertTo(gc,CV_8UC1);
            bc.convertTo(bc,CV_8UC1);
            rc.convertTo(rc,CV_8UC1);
            */
             /*
            R=R*255;
            B=B*255;
            G=G*255;
            
           

            R.convertTo(R,CV_8UC1);
            G.convertTo(G,CV_8UC1);
            B.convertTo(B,CV_8UC1);

           
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


            /*
            
            //RGB HISTOGRAM 

            int histSize = 255;

	        // Set the range
	        float range[] = { 0, 255 };
	        const float* histRange = { range };

	        bool uniform = true; bool accumulate = false;

	        Mat GChist;
            Mat BChist;
            Mat RChist;

	        // Compute the histograms
            calcHist(&gc, 1, 0, Mat(), GChist, 1, &histSize, &histRange, uniform, accumulate);
            calcHist(&bc, 1, 0, Mat(), BChist, 1, &histSize, &histRange, uniform, accumulate);
            calcHist(&rc, 1, 0, Mat(), RChist, 1, &histSize, &histRange, uniform, accumulate);
            
            // Draw the histogram
	        int hist_w = 512; int hist_h = 400;
	        int bin_w = cvRound((double)hist_w / histSize);

            Mat GChistImage(hist_h, hist_w, CV_8UC3, Scalar(255, 255, 255));
            Mat BChistImage(hist_h, hist_w, CV_8UC3, Scalar(255, 255, 255));
            Mat RChistImage(hist_h, hist_w, CV_8UC3, Scalar(255, 255, 255));
            
            // Normalize the result to [ 0, histImage.rows ]
           
            normalize(GChist, GChist, 0, GChistImage.rows, NORM_MINMAX, -1, Mat());
            normalize(BChist, BChist, 0, BChistImage.rows, NORM_MINMAX, -1, Mat());
            normalize(RChist, RChist, 0, RChistImage.rows, NORM_MINMAX, -1, Mat());
            

            for (int i = 1; i < histSize; i++)
            {
                line(GChistImage, Point(bin_w*(i - 1), hist_h - cvRound(GChist.at<float>(i - 1))),
                   Point(bin_w*(i), hist_h - cvRound(GChist.at<float>(i))),
                    Scalar(0, 0, 255), 2, 8, 0);

                line(BChistImage, Point(bin_w*(i - 1), hist_h - cvRound(BChist.at<float>(i - 1))),
                    Point(bin_w*(i), hist_h - cvRound(BChist.at<float>(i))),
                    Scalar(0, 0, 255), 2, 8, 0);

                line(RChistImage, Point(bin_w*(i - 1), hist_h - cvRound(RChist.at<float>(i - 1))),
                    Point(bin_w*(i), hist_h - cvRound(RChist.at<float>(i))),
                    Scalar(0, 0, 255), 2, 8, 0);
            }



            */
            /*
            imwrite("gc.png",gc);
            imwrite("bc.png",bc);
            imwrite("rc.png",rc);





            imshow("GChistImage", GChistImage);
            imwrite("GChistImage.png",GChistImage);
            imshow("BChistImage", BChistImage);
            imwrite("BChistImage.png",GChistImage);
            imshow("RChistImage", RChistImage);
            imwrite("RChistImage.png",RChistImage);
            */
           
           /*
            
            double GmaxVal=0;
            cv::Point GmaxPoint;
            minMaxLoc(GChist, 0, &GmaxVal, 0, &GmaxPoint);

            double BmaxVal=0;
            cv::Point BmaxPoint;
            minMaxLoc(BChist, 0, &BmaxVal, 0, &BmaxPoint);

            double RmaxVal=0;
            cv::Point RmaxPoint;
            minMaxLoc(RChist, 0, &RmaxVal, 0, &RmaxPoint);


            int Gmax=GmaxPoint.y;
            int Bmax=BmaxPoint.y;
            int Rmax=RmaxPoint.y;
        
            double Gminthres = Gmax - 14;
            double Gmaxthres = Gmax + 14;
            Scalar Gmintable = Scalar{Gminthres};
            Scalar Gmaxtable = Scalar{Gmaxthres};
            Mat Gthreshold;
            cv::inRange(gc, Gmintable, Gmaxtable, Gthreshold);


            double Bminthres = Bmax - 14;
            double Bmaxthres = Bmax + 14;
            Scalar Bmintable = Scalar{Bminthres};
            Scalar Bmaxtable = Scalar{Bmaxthres};
            Mat Bthreshold;
            cv::inRange(bc, Bmintable, Bmaxtable, Bthreshold);

            double Rminthres = Rmax - 10;
            double Rmaxthres = Rmax + 10;
            Scalar Rmintable = Scalar{Rminthres};
            Scalar Rmaxtable = Scalar{Rmaxthres};
            Mat Rthreshold;
            cv::inRange(rc, Rmintable, Rmaxtable, Rthreshold);

            Bthreshold.convertTo(Bthreshold, CV_64FC1);
            Bthreshold=Bthreshold/255.;
            
            Gthreshold.convertTo(Gthreshold, CV_64FC1);
            Gthreshold=Gthreshold/255.;

            Rthreshold.convertTo(Rthreshold, CV_64FC1);
            Rthreshold=Rthreshold/255.;
    
            Mat threshold = Gthreshold.mul(Bthreshold).mul(Rthreshold);
            threshold=threshold*255.;
            threshold.convertTo(threshold, CV_8UC1);
           
        

            
            
            Mat threshold2;
            // Create a structuring element
            int erosion_size = 3;
            Mat element = getStructuringElement(cv::MORPH_CROSS,
            cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
            cv::Point(erosion_size, erosion_size));

            // Apply erosion or dilation on the image
            cv::erode(threshold, threshold2, element);
            cv::dilate(threshold2, threshold2, element);
            cv::erode(threshold, threshold2, element);
            cv::dilate(threshold2, threshold2, element);
            cv::erode(threshold, threshold2, element);
            cv::dilate(threshold2, threshold2, element);
            
            //cv::blur(threshold, threshold2, cv::Size(5,5));


            Mat Icontours, Icontours2;

            Canny(threshold2,Icontours,50,100);

            std::vector<cv::Vec3f> Icircles;
            HoughCircles(Icontours,Icircles, CV_HOUGH_GRADIENT,
                            2, //accumulator resolution 
                            15, //min distance between two circles
                            100, //Canny high threshold
                            50, // min number of votes
                            10,60); // min and max radius

            std::vector<Vec3f>::const_iterator Icir = Icircles.begin();

            frame = frame * 255.f;
            frame.convertTo(frame, CV_64FC4);
            

            while(Icir!=Icircles.end())
            {
                //circle(Icontours, cv::Point((* Icir)[0],(* Icir)[1]),(* Icir)[2],Scalar{150},2);
                
                circle(frame, cv::Point((* Icir)[0],(* Icir)[1]),(* Icir)[2], Scalar{0,0,0},2);
                ++Icir;
            }
          

            imshow("frame", frame);

            imwrite("frame.png", frame);
    
            //imwrite("thres.png", Bthreshold);
            
            */
            
            //frame = frame * 255.f;
            //frame.convertTo(frame, CV_64FC4);
        
            
            
            //H HISTOGRAM SEGMENTATION

            int hhistSize = 180;

            bool uniform = true; bool accumulate = false;

            // Draw the histogram
	        int hist_w = 512; int hist_h = 400;
	        int bin_w = cvRound((double)hist_w / hhistSize);

	        // Set the range
	        float hrange[] = { 0, 180 };
	        const float* hhistRange = { hrange };

	        Mat hhist;

	        // Compute the histogram
	        calcHist(&hframe, 1, 0, Mat(), hhist, 1, &hhistSize, &hhistRange, uniform, accumulate);
            
        
	        Mat hhistImage(hist_h, hist_w, CV_8UC3, Scalar(255, 255, 255));

            // Normalize the result to [ 0, histImage.rows ]
            normalize(hhist, hhist, 0, hhistImage.rows, NORM_MINMAX, -1, Mat());

            for (int i = 1; i < hhistSize; i++)
            {
                line(hhistImage, Point(bin_w*(i - 1), hist_h - cvRound(hhist.at<float>(i - 1))),
                    Point(bin_w*(i), hist_h - cvRound(hhist.at<float>(i))),
                    Scalar(0, 0, 255), 2, 8, 0);
            }


            double maxVal=0;
            cv::Point maxPoint;
            minMaxLoc(hhist, 0, &maxVal, 0, &maxPoint);

            int hmax=maxPoint.y;

            int histSize2 = 255;

            // Set the range
            float range2[] = { 0, 255 };
            const float* histRange2 = { range2 };

            Mat vhist;

            // Compute the histogram
            calcHist(&V, 1, 0, Mat(), vhist, 1, &histSize2, &histRange2, uniform, accumulate);

            cv::Point vmaxPoint;
            minMaxLoc(vhist, 0, &maxVal, 0, &vmaxPoint);
            int vmax = vmaxPoint.y;

            double thresvmax;
            double thresvmin;

            if (vmax < 205)
                thresvmax = vmax + 10;
            else
                thresvmax = 255;

            if (vmax > 50)
                thresvmin = vmax - 50;
            else
                thresvmin = 0;


            

    /*
            hframe.convertTo(hframe, CV_64FC1);
           
            Mat h=(hframe/180.);

            for (int i = 0 ; i < h.rows; ++i)
            {
                for (int j = 0 ; j < h.cols; ++j)
                {
                    double hval = h.at<double>(i, j);
                    if (hval>1.)
                        h.at<double>(i, j) = 0.99;
                    if (hval<0.)
                        h.at<double>(i, j) = 0.01;
                }
            }

            h=h*255.;

            h.convertTo(h,CV_8UC1);
            
        */
            
            
         
            Scalar mintable = Scalar{ double(hmax - 10), 2 , thresvmin };
		    Scalar maxtable = Scalar{ double(hmax + 10) , 240 , thresvmax };
		    Mat threshold,threshold2;
		    cv::inRange(HSVframe, mintable, maxtable, threshold);

            imshow("thresholdHSV",threshold);
            imwrite("thresholdHSV.png",threshold);

        
            // Create a structuring element
            int erosion_size = 3;
            Mat element = getStructuringElement(cv::MORPH_CROSS,
            cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
            cv::Point(erosion_size, erosion_size));

            // Apply erosion or dilation on the image
            cv::erode(threshold, threshold2, element);
            cv::dilate(threshold2, threshold2, element);
            cv::erode(threshold2, threshold2, element);
            cv::dilate(threshold2, threshold2, element);
            cv::erode(threshold2, threshold2, element);
            cv::dilate(threshold2, threshold2, element);
            cv::erode(threshold2, threshold2, element);
            cv::dilate(threshold2, threshold2, element);

            cv::blur(threshold2, threshold2, cv::Size(5,5));

            Mat Icontours, Icontours2;

            //Canny(threshold2,Icontours,50,100);

            std::vector<cv::Vec3f> Icircles;
            HoughCircles(threshold2,Icircles, CV_HOUGH_GRADIENT,
                            2, //accumulator resolution 
                            15, //min distance between two circles
                            100, //Canny high threshold
                            50, // min number of votes
                            10,60); // min and max radius

            std::vector<Vec3f>::const_iterator Icir = Icircles.begin();

            while(Icir!=Icircles.end())
            {
                //circle(Icontours, cv::Point((* Icir)[0],(* Icir)[1]),(* Icir)[2],Scalar{150},2);
                
                circle(frame, cv::Point((* Icir)[0],(* Icir)[1]),(* Icir)[2], Scalar{0,0,0},2);
                ++Icir;
            }
          
            
              
            //imshow("IcontoursHSV",Icontours);
            //imwrite("IcontoursHSV.png",Icontours);

            imshow("frameHSV",frame);
            //imwrite("frameHSV.png",frame);

            
           

          /*

            //Detect contours avec FindContours
            vector<vector<cv::Point> > contours;
            vector<Vec4i> hierarchy;
            cv::findContours(threshold, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

            vector<vector<cv::Point> > contours_poly(contours.size());
            vector<Point2f>center(contours.size());
            vector<float>radius(contours.size());

        
            int MaxRadius = 200;
            int MinRadius = 10;
            Scalar color_ball = Scalar(255, 255, 255);
            vector<vector<cv::Point> >hull(contours.size());
            


            for (size_t i = 0; i < contours.size(); i++)
            {
                convexHull(Mat(contours[i]), hull[i]);
                approxPolyDP(hull[i], contours_poly[i], 3, true);
                minEnclosingCircle(contours_poly[i], center[i], radius[i]);
                

                if (radius[i]<MaxRadius && radius[i]>MinRadius)
                {
                    circle(frame, center[i], (int)radius[i], color_ball, 2, 8, 2);
                    cout << "a ";
                }

            }

*/
        //imshow("frame", frame);
        //imshow("threshold2", threshold2);
        //imshow("Icontours", Icontours);
        //imshow("Icontours2", Icontours2);

        
      Play = !Play;
           
            
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
    //destroyAllWindows();
    
    return 0;
}

