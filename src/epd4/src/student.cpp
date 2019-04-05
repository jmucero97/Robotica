

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

	//PICKS GREEN
	cv::threshold(o[0], BlueThreshold, 152.0, 255.0, cv::THRESH_BINARY);
	cv::threshold(o[1], GreenThreshold, 255.0, 255.0, cv::THRESH_BINARY);
	bitwise_not(GreenThreshold, GreenThreshold);
	cv::threshold(o[2], RedThreshold, 172.0, 255.0, cv::THRESH_BINARY);

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
	

}
