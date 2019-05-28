

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


//This is the function to be filled in C3
void processImage_c3(cv::Mat &in, cv::Mat &out)
{
	double k[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};

	cv::Mat kernel = cv::Mat(3, 3, CV_32F, k);

	//cv::filter2D(in,out,-1,kernel);

	cv::Sobel(in, out, -1, 1, 1);

	//cv::threshold(in,out,140.0,255.0,cv::THRESH_TOZERO_INV);
	cv::threshold(in, out, 50.0, 255.0, cv::THRESH_BINARY);

	//cv::erode(out,out,cv::Mat());
	//cv::dilate(out,out,cv::Mat());
}

//This is the function to be filled in C4
void processImageColor_c4(cv::Mat &in, cv::Mat &out)
{

	cv::Mat r;
	cv::Mat g;
	cv::Mat b;
	cv::Mat alpha;

	cv::Mat wallOut;
	cv::Mat GreenThreshold;
	cv::Mat BlueThreshold;
	cv::Mat RedThreshold;
	cv::Mat o1;
	cv::Mat oGreen;

	cv::Mat o[] = {b, g, r, alpha};
	//in is a color image. We split it into the 3 colors (in order BGR) and the alpha channel:
	cv::split(in, o);

	//From now on, we process just one of the channels. For instance, o[2] is the red channel

	//This is an example:
	//cv::filter2D(o[1],out,-1,cv::getGaussianKernel(5,2));

	//PICKS YELLOW RGB(255,255,0)
	cv::threshold(o[0], BlueThreshold, 255.0, 255.0, cv::THRESH_BINARY);
	bitwise_not(BlueThreshold, BlueThreshold);
	cv::threshold(o[1], GreenThreshold, 175.0, 255.0, cv::THRESH_BINARY);
	cv::threshold(o[2], RedThreshold, 175.0, 255.0, cv::THRESH_BINARY);

	bitwise_and(GreenThreshold, BlueThreshold, o1);
	bitwise_and(o1, RedThreshold, oGreen);
	

	//PICKS WALL
	cv::Mat onlyRed;
	cv::Mat onlyGreen;
	cv::Mat onlyBlue;

	cv::threshold(o[0], onlyBlue, 200.0, 255.0, cv::THRESH_BINARY);
	cv::threshold(o[1], onlyGreen, 200.0, 255.0, cv::THRESH_BINARY);
	cv::threshold(o[2], onlyRed, 200.0, 255.0, cv::THRESH_BINARY);
	bitwise_and(onlyBlue,onlyGreen,out);
	bitwise_and(out,onlyRed,wallOut);
	bitwise_not(wallOut,wallOut);

	bitwise_and(oGreen, wallOut, out);
	int erosion_type =0;
	int erosion_size = 3;

	cv::Mat kernel = cv::getStructuringElement(erosion_type,cv::Size(2*erosion_size+1,2*erosion_size+1),cv::Point(erosion_size,erosion_size) );
	erode(out, out, kernel);
}
void processImageCircle_c4(cv::Mat &in, cv::Mat &out)
{
	cv::Moments moment = cv::moments(in);
	double x = moment.m10/moment.m00;
	double y = moment.m01/moment.m00;
	cv:circle(out, cv::Point(x,y),25,cv::Scalar(0,0,255),3);
}

void processImageChallenge(cv::Mat &in, cv::Mat &out)
{
	cv::Mat r;
	cv::Mat g;
	cv::Mat b;
	cv::Mat alpha;

	cv::Mat wallOut;
	cv::Mat GreenThreshold1;
	cv::Mat BlueThreshold1;
	cv::Mat RedThreshold1;
	cv::Mat o1;
	cv::Mat oRed;
	cv::Mat oBook;

	cv::Mat o[] = {b, g, r, alpha};
	//in is a color image. We split it into the 3 colors (in order BGR) and the alpha channel:
	cv::split(in, o);

	//From now on, we process just one of the channels. For instance, o[2] is the red channel

	//This is an example:
	//cv::filter2D(o[1],out,-1,cv::getGaussianKernel(5,2));
	
	//RED
	cv::threshold(o[0], BlueThreshold1, 80.0, 255.0, cv::THRESH_BINARY);
	bitwise_not(BlueThreshold1, BlueThreshold1);
	cv::threshold(o[1], GreenThreshold1, 80.0, 255.0, cv::THRESH_BINARY);
	bitwise_not(GreenThreshold1, GreenThreshold1);
	cv::threshold(o[2], RedThreshold1, 120.0, 255.0, cv::THRESH_BINARY);

	bitwise_and(GreenThreshold1, BlueThreshold1, o1);
	bitwise_and(o1, RedThreshold1, out);
/*
	//PICKS WALL
	cv::Mat onlyRed;
	cv::Mat onlyGreen;
	cv::Mat onlyBlue;

	cv::threshold(o[0], onlyBlue, 200.0, 255.0, cv::THRESH_BINARY);
	cv::threshold(o[1], onlyGreen, 200.0, 255.0, cv::THRESH_BINARY);
	cv::threshold(o[2], onlyRed, 200.0, 255.0, cv::THRESH_BINARY);
	bitwise_and(onlyBlue,onlyGreen,out);
	bitwise_and(out,onlyRed,wallOut);
	bitwise_not(wallOut,wallOut);
	bitwise_and(oBook, wallOut, out);
*/
	int erosion_type =0;
	int erosion_size = 3;

	cv::Mat kernel = cv::getStructuringElement(erosion_type,cv::Size(2*erosion_size+1,2*erosion_size+1),cv::Point(erosion_size,erosion_size) );
	erode(out, out, kernel);
}





























