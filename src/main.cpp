#include <stdio.h>
#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <sstream>
#include <time.h>

#define DEPTH_SENSOR		2
#define RGB_SENSOR			0
#define EXT_SENSOR			1
#define NUM_COLOR_SENSORS	2
#define NUM_SENSORS			3

using namespace cv;
using namespace std;	

// Interpolate color of a point with non-integer coordinates in an image
// You can use this function to get smoother outputs, but it is optional
Vec3b avSubPixelValue8U3( const Point2f pt, const Mat img )
{
	int floorx = (int)floor( pt.x );
	int floory = (int)floor( pt.y );

	if( floorx < 0 || floorx >= img.cols-1 || 
		floory < 0 || floory >= img.rows-1 )
		return 0;

	float px = pt.x - floorx;
	float py = pt.y - floory;

	Vec3b tl = img.at<Vec3b>(floory,floorx);
	Vec3b tr = img.at<Vec3b>(floory,floorx+1);
	Vec3b bl = img.at<Vec3b>(floory+1,floorx);
	Vec3b br = img.at<Vec3b>(floory+1,floorx+1);
	Vec3b result;
	for (int i=0;i<3;i++)
		result[i] = (unsigned char)floor(tl[i] * (1-px)*(1-py) + tr[i]*px*(1-py) + bl[i]*(1-px)*py + br[i]*px*py + 0.5 );

	return result;
}

// Load the depth image, the RGB & the external image for a specific frame
// If all images can be loaded, return true
// Otherwise, return false
bool loadImages(char* src, int frame, Mat& depthMat, Mat& rgbMat, Mat& extMat){
	char fname[10];
	char fpath[100];

	// load depth image
	printf("Frame %04d\n",frame);
	sprintf(fname,"%04d.png",frame);
	sprintf(fpath,"%s/Depth/%s",src,fname);
	depthMat = imread(fpath, CV_LOAD_IMAGE_ANYDEPTH);
	if (!depthMat.data ) {
		return false;
	}
	
	// load RGB image
	sprintf(fpath,"%s/RGB/%s",src,fname);
	rgbMat = imread(fpath);
	if (!rgbMat.data) {
		return false;
	}
	
	// load external image
	sprintf(fpath,"%s/Ext/%s",src,fname);
	extMat = imread(fpath);
	if (!extMat.data) {
		return false;
	}
	// TODO 0: In the sample dataset, the external images need to be filipped vertically to have the same direction with the others
	// In your own dataset, however, you may not need to do it. Check your images and comment out the statement below if not needed.
	flip(extMat,extMat,1);
	return true;
}

// Extract board information from the input file
bool getboardInfor(char* boardInforPath, Size &boardSize, float &squareSize, Rect_<float> &boardRegion){
	FILE* file = fopen(boardInforPath, "r");
	if (!file) {
		fprintf(stderr,"Error! Can't find the board information file!\n");
		return  false;
	}

	float right, bottom;
	fscanf(file, "%d %d %f %f %f %f %f",&boardSize.width,&boardSize.height,&squareSize, &boardRegion.x, &boardRegion.y, &right, &bottom);
	boardRegion.width = right - boardRegion.x;
	boardRegion.height = bottom - boardRegion.y;
	fclose(file);
	return true;
}

// Compute 3-D coordinates of the checker inner corners in {B}
// Input:
//		boardSize : checker size
//		squareSize: side of each cell in the checker pattern
// Output:
//		inCorners3D: list of computed 3-D coordinates
//		matInCorners3D: matrices 4xN of computed 3-D homogeneous coordinates
//
void calcInnerCorner(Size boardSize, float squareSize, vector<Point3f>& inCorners3D, Mat &matInCorners3D)
{
	// TODO I.1.a: Compute 3-D coordinates of the checker inner corners in {B}
	//             Remember to list them in order from top-left corner to the right-bottom one
	//
	// fill your code here
	//

	int ncellsX = boardSize.width+1 ;
	int ncellsY = boardSize.height+1;
	
	float endPosX	= ((float)ncellsX / 2) - 1;
	float endPosY	= ((float)ncellsY / 2) - 1;

	int k = 0;

	for(float i = -endPosY; i <= endPosY; i = i+1)
	{
		for(float j = -endPosX; j <= endPosX; j = j+1)
		{
			Point3f p(j * squareSize, i * squareSize, 0.0);
			inCorners3D.push_back(p);
		}
	}

	convertPointsToHomogeneous(inCorners3D, matInCorners3D);
}

// Compute 3-D coordinates of the board outer corners in {B}
// Input:
//		boardRegion
// Output:
//		boardCorners3D
//		matBoardCorners3D
//
void calcOuterCorner(Rect_<float> boardRegion, vector<Point3f>& boardCorners3D, Mat &matBoardCorners3D)
{
	// TODO I.1.b: Compute 3-D coordinates of the board 4 outer corners in {B}
	//
	// fill your code here
	//
	
	int k = 0;

	Point3f p1(boardRegion.x,boardRegion.y, 0.0);
	boardCorners3D.push_back(p1);
	Point3f p2(boardRegion.x + boardRegion.width, boardRegion.y, 0.0);
	boardCorners3D.push_back(p2);
	Point3f p3(boardRegion.x, boardRegion.y + boardRegion.height, 0.0);
	boardCorners3D.push_back(p3);
	Point3f p4(boardRegion.x + boardRegion.width, boardRegion.y + boardRegion.height, 0.0);
	boardCorners3D.push_back(p4);

	convertPointsToHomogeneous(boardCorners3D, matBoardCorners3D);
}

// calibrate a camera using Zhang's method
bool runCalibration( vector<Point3f> inCorners3D, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, 
	vector<vector<Point2f> > incorner2DPoints, vector<Mat>& rvecs, vector<Mat>& tvecs)
{
	// TODO II.1.2: Calibrate the camera using function calibrateCamera of OpenCV
	//              Choose suitable flags 
	//
	// fill your code here
	//

	vector<vector<Point3f>> arr_incorner3DPoints;
	for(int i = 0; i < incorner2DPoints.size(); i++)
	{
		arr_incorner3DPoints.push_back(inCorners3D);
	}

	int flags = CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K1|CV_CALIB_FIX_K2 |
								CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4|CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6;

	double rpe = calibrateCamera(arr_incorner3DPoints, incorner2DPoints, imageSize,cameraMatrix, distCoeffs,
								rvecs, tvecs, flags);
	return true;
}

// Find a non-zero point from a starting point in a thresholded image
// Input:
//		img    : the thresholded image
//		start  : starting point
//		maxDist: maximum distance to search
// Output:
//		dst    : a non-zero point
// If no point is found, return false
bool findClosestPoint(Mat img, Point2f start, Point2f &dst, int maxDist = 10){
	int x = floor(start.x + 0.5);
	int y = floor(start.y + 0.5);
	if (img.at<unsigned char>(y,x) > 0){
		dst = start;
		return true;
	}
	for (int i=1;i<=maxDist;i++){
		if (x - i >= 0){				// Search on the left
			for (int j=-i;j<=i;j++){
				if (y + j < 0 || y + j > img.rows || img.at<unsigned char>(y + j,x-i) == 0) continue;
				dst.x = x -i;
				dst.y = y + j;
				return true;
			}
		}
		
		if (x + i < img.cols){			// Search on the right
			for (int j=-i;j<=i;j++){
				if (y + j < 0 || y + j > img.rows || img.at<unsigned char>(y + j,x+i) == 0) continue;
				dst.x = x + i;
				dst.y = y + j;
				return true;
			}
		}
		
		if (y - i >= 0){				// Search upward
			for (int j=-i;j<=i;j++){
				if (x + j < 0 || x + j > img.cols || img.at<unsigned char>(y - i,x+j) == 0) continue;
				dst.x = x + j;
				dst.y = y - i;
				return true;
			}
		}
		
		if (y + i < img.rows){			// Search downward
			for (int j=-i;j<=i;j++){
				if (x + j < 0 || x + j > img.cols || img.at<unsigned char>(y + i,x+j) == 0) continue;
				dst.x = x + j;
				dst.y = y + i;
				return true;
			}
		}
	}
	dst = start;
	return false;
}



// Write the pointcloud extracted from a depth image & its colored image to PLY file
// Input:
//		count  : the number of points in the cloud
//		f      : focal length of the depth sensor
//		color  : the colored depth image
//		fname  : file name
void writePLY(int count, float f, Mat color, Mat depth, char* fname){
	FILE* file = fopen( fname, "w");

	if ( !file )
    {
		std::cerr << "Creation Error\n";
        return;
    }

	fprintf( file, "ply\n");
	fprintf( file, "format ascii 1.0\n" );
	fprintf( file, "element vertex %d\n", count );
	fprintf( file, "property float x\nproperty float y\nproperty float z\n" );
	fprintf( file, "property uchar blue\nproperty uchar green\nproperty uchar red\n");
	fprintf( file, "end_header\n");

	for (int i=0;i<depth.cols;i++){
		for (int j=0;j<depth.rows;j++){
			if (depth.at<short>(j,i) > 0){
				float Z = depth.at<short>(j,i);
				float X = (i-depth.cols/2) * Z/f;
				float Y = (j-depth.rows/2) * Z/f;
				Vec3b colors = color.at<Vec3b>(j,i);
				fprintf( file, "%f %f %f %d %d %d\n",X, Y, Z, colors[0], colors[1], colors[2]);
			}
		}
	}
	fclose(file);
}

int main(int argc, char** argv){
	char src[80] = "../Data";					// path to the dataset
	char dst[80] = "../Output";				// path to the output directory
	char boardInfor[80] = "../Data/board.txt";  // the board information file

	int start = 10;								// start frame
	int end = 60;								// the last frame can be accessed in the dataset
	int step = 1;								// step in frame numbers to process
	int nFrame2Use = 10;						// number of frames can be used to calibrate
	Rect_<float> boardRegion(-150, -103.5, 138, 110.5);		// boardRegion

	vector<int> goodFrames;						// list of good frames to use for calibration
	Size boardSize;
	boardSize.width = 5;
	boardSize.height = 4;
	float squareSize = 35;
	float fc = 285.171;
	float fd = 285.171;

	int arg = 0;								// Process input parameters
	while( ++arg < argc) 
	{ 
		// Input directory
		if( !strcmp(argv[arg], "-i") )
			strcpy(src, argv[++arg] );
		
		// Output directory
		if( !strcmp(argv[arg], "-o") )
			strcpy(dst, argv[++arg] );

		// First frame
		if( !strcmp(argv[arg], "-b") )
			start = atoi( argv[++arg] );
		
		// Step
		if( !strcmp(argv[arg], "-s") )
			step = atoi( argv[++arg] );

		// Last frame
		if( !strcmp(argv[arg], "-e") )
			end = atoi( argv[++arg] );

		// Number of frames to calibrate
		if( !strcmp(argv[arg], "-n") )
			nFrame2Use = atoi( argv[++arg] );

		// Focal length
		if( !strcmp(argv[arg], "-fc") )
			fc = atof( argv[++arg] );
		
		// Focal length
		if( !strcmp(argv[arg], "-fd") )
			fd = atof( argv[++arg] );

		// Board information file
		if( !strcmp(argv[arg], "-v") )
			strcpy(boardInfor, argv[++arg] );
	}

	// get board information from file
	getboardInfor(boardInfor, boardSize, squareSize, boardRegion);
	
	bool gotSize = false;
	bool extCalibrated = false;
	Size imageSize[NUM_SENSORS];
	Mat cameraMatrix[NUM_SENSORS];										// cameras' intrinsic matrices
	vector<Mat> rvecs[NUM_COLOR_SENSORS], tvecs[NUM_COLOR_SENSORS];		// board pose regarding 2 color cameras
	vector<vector<Point2f> > incorner2DPoints[NUM_COLOR_SENSORS];		// extracted inner corners
	vector<vector<Point2f> > boardCorner2DPoints;						// extracted board outer corners from depth images

	vector<float> reprojErrs[NUM_SENSORS];
	vector<Point3f> inCorners3D;											// 3D coordinates of inner corners in {B}
	Mat matInCorners3D;
	vector<Point3f> boardCorners3D;										// 3D coordinates of the board outer corners in {B}
	Mat matBoardCorners3D;
	vector<vector<short>> allBoardCornerDepth;

	// prepared reference 3-D coordinates
	calcInnerCorner(boardSize, squareSize, inCorners3D, matInCorners3D);
	calcOuterCorner(boardRegion, boardCorners3D,matBoardCorners3D);
	
	
	// Initiate camera matrices
	float _cam[9] = {fc, 0 , 0, 0, fc, 0, 0 , 0, 1};
	cameraMatrix[RGB_SENSOR] = Mat(3,3,CV_32F,_cam);
	float _extCam[9] = {0, 0 , 0, 0, 0, 0, 0 , 0, 1};
	cameraMatrix[EXT_SENSOR] = Mat(3,3,CV_32F,_extCam);	
	float _depthcam[9] = {fd, 0 , 0, 0, fd, 0, 0 , 0, 1};
	cameraMatrix[DEPTH_SENSOR] = Mat(3,3,CV_32F,_depthcam);
	// null distortation parameters
	Mat distCoeffs(1,4,CV_32F);
	Mat distCoeffRGB(1,4,CV_32F);
	distCoeffs = 0;
	distCoeffRGB = 0;
	////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////   STEP 1: Scan over images & collect useful information    ////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	char fpath[100];
	for (int frame=start;frame<=end;frame+=step){
		Mat depthMat, rgbMat, extMat;

		vector<Point2f> rgbPointBuf;			// temporary extracted 2-D inner corners
		vector<Point2f> extPointBuf;

		/// Load images & validate
		if (!loadImages(src, frame, depthMat, rgbMat, extMat)){
			continue;
		}

		if (!gotSize) {
			// get image sizes
			imageSize[RGB_SENSOR].height = rgbMat.rows;
			imageSize[RGB_SENSOR].width  = rgbMat.cols;
			imageSize[DEPTH_SENSOR].height = depthMat.rows;
			imageSize[DEPTH_SENSOR].width  = depthMat.cols;
			imageSize[EXT_SENSOR].height = extMat.rows;
			imageSize[EXT_SENSOR].width  = extMat.cols;

			// set cameras' principal points
			_cam[2] = imageSize[RGB_SENSOR].width/2;
			_cam[5] = imageSize[RGB_SENSOR].height/2;
			_extCam[2] = imageSize[EXT_SENSOR].width/2;
			_extCam[5] = imageSize[EXT_SENSOR].height/2;
			_depthcam[2] = imageSize[DEPTH_SENSOR].width/2;
			_depthcam[5] = imageSize[DEPTH_SENSOR].height/2;
			gotSize = true;
		}

		//////  Process the color images
		// TODO II.1.1: Extract inner corners of the checkerboard (extPointBuf) from the external image (extMat)
		//              If no corner is detected, skip this frame
		//			    If the detected corners are in a wrong order, revert it
		//
		// fill your code here
		//

		bool patternfound = findChessboardCorners(extMat, boardSize, extPointBuf);
		
		if(extPointBuf.size() == 0)
			continue;

		if(!patternfound)
			extPointBuf.empty();
		else
		{
			Mat ExtMatGray;
            //cvtColor(extMat, ExtMatGray, CV_BGR2GRAY);
			//cornerSubPix( ExtMatGray, extPointBuf, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
			if(((extPointBuf[extPointBuf.size() - 1].x - extPointBuf[0].x) < 0) &&
				((extPointBuf[extPointBuf.size() - 1].y - extPointBuf[0].y) < 0))
				reverse(extPointBuf.begin(), extPointBuf.end());
		}


		if (extPointBuf.size() > 0)
			drawChessboardCorners( extMat, boardSize, Mat(extPointBuf), true );

		//resize extMat
		Mat extResized;
		resize(extMat,extResized,rgbMat.size());
		cv::namedWindow( "External image", 10 );
		cv::imshow( "External image", extResized );
		cv::waitKey( 20 );
		// TODO I.2.1: Extract inner corners of the checkerboard (extPointBuf) from the external(COLOR) image (extMat)
		//             If no corner is detected, skip this frame
		//			   If the detected corners are in a wrong order, revert it
		//
		// fill your code here
		//
		
		patternfound = findChessboardCorners(rgbMat, boardSize, rgbPointBuf);
		if(rgbPointBuf.size() == 0)
			continue;

		if(!patternfound)
			rgbPointBuf.empty();
		else
		{
			Mat rgbMatGray;
            //cvtColor(rgbMat, rgbMatGray, CV_BGR2GRAY);
			//cornerSubPix( rgbMatGray, rgbPointBuf, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
			
			if(((rgbPointBuf[rgbPointBuf.size() - 1].x - rgbPointBuf[0].x) < 0) &&
				((rgbPointBuf[rgbPointBuf.size() - 1].y - rgbPointBuf[0].y) < 0))
				reverse(rgbPointBuf.begin(), rgbPointBuf.end());
		}

		
		if (rgbPointBuf.size() > 0)
			drawChessboardCorners(rgbMat, boardSize, Mat(rgbPointBuf), true );
		cv::namedWindow( "RGB image", 11 );
		cv::imshow( "RGB image", rgbMat);
		cv::waitKey( 20 );

		Mat rVec, tVec;
		// TODO I.2.2: From the detected 2D inner corners (rgbPointBuf) & their 3D coordinates in {B} (inCorners3D), estimate the board pose 
		//             (regarding the color sensor coordinate system {C}) using function solvePnPRansac() of OpenCV. 
		//			   Save your result into rVec and tVec for later use
		//
		// fill your code here
		//
		solvePnPRansac(inCorners3D, rgbPointBuf, cameraMatrix[RGB_SENSOR], distCoeffRGB, rVec, tVec);

		vector<Point2f> rgbPoints2;
		projectPoints(inCorners3D,rVec,tVec,cameraMatrix[RGB_SENSOR],distCoeffRGB,rgbPoints2);	// back-project inner corners into the RGB image
		for (int i=0;i<rgbPoints2.size();i++)
			circle(rgbMat,Point(floor(rgbPoints2[i].x + 0.5),floor(rgbPoints2[i].y + 0.5)),2,Scalar(128,128,0),2);
	    cv::imshow( "RGB image", rgbMat );
		cv::waitKey( 20 );

		rVec.convertTo(rVec, CV_32F);
		tVec.convertTo(tVec, CV_32F);

		//////  Process the depth image
		Mat depthMat0 = depthMat / 4;					// scale depth map
		depthMat0.convertTo(depthMat0, CV_8U);			// gray-scale depth image

		// duplicate channels
		Mat depthMat1(depthMat.rows,depthMat.cols,CV_8UC3);	// true-color depth image
		insertChannel(depthMat0,depthMat1,0);
		insertChannel(depthMat0,depthMat1,1);
		insertChannel(depthMat0,depthMat1,2);

		cv::namedWindow( "Depth image", 12 );		
		cv::imshow( "Depth image", depthMat1 );
		cv::waitKey( 20 );
		
		// plot locations of the inner corners (from the RGB image) onto the depth image
		for (int i=0;i<boardSize.height;i++){
			for (int j=0;j<boardSize.width;j++){
				circle(depthMat1,Point(floor(rgbPointBuf[i*boardSize.width +j].x),floor(rgbPointBuf[i*boardSize.width +j].y)),2,Scalar(0,0,255));
			}
		}
		cv::imshow( "Depth image", depthMat1 );
		cv::waitKey( 20 );
		
		Mat segmentedDepthMat;
		// TODO I.2.3.a: Detect the board region using a segmentation technique (thresholding, watershed, mean-shift ...). 
		//					Input: the gray-scale depth map (depthMat0)
		//					Output: a depth map with only the extracted board region (segmentedDepthMat)
		//               Note that you can get hints about the board position from the inner corners location in the color image.
		//
		// fill your code here
		//	
		/*
		cv::Mat markers(depthMat0.size(), CV_8U, cv::Scalar(-1));
		Point2f p0	= rgbPoints2[0];
		Point2f p1	= rgbPoints2[boardSize.width - 1];
		Point2f p2	= rgbPoints2[rgbPoints2.size() - boardSize.width];
		Point2f p3  = rgbPoints2[rgbPoints2.size() - 1];
		int minX  = (p0.x < p2.x)? p0.x : p2.x;
		int minY  = (p0.y < p1.y)? p0.y : p1.y;
		int maxX  = (p1.x > p3.x)? p1.x : p3.x;
		int maxY  = (p2.y > p3.y)? p2.y : p3.y;

		Mat tempdst(depthMat0.size(), depthMat0.type());
		int d[4];
		d[0] =  depthMat0.at<uchar>(floor(p0.y),floor(p0.x));
		d[1] =  depthMat0.at<uchar>(floor(p1.y),floor(p1.x));
		d[2] =  depthMat0.at<uchar>(floor(p2.y),floor(p2.x));
		d[3] =  depthMat0.at<uchar>(floor(p3.y),floor(p3.x));
		int min = INT_MAX, max = -1;
		for(int i = 0; i < 4; i++)
		{
			if(d[i] < min)min = d[i];
			if(d[i] > max)max = d[i];
		}
		
		markers(Rect(0, 0, 100, 50)) = Scalar::all(1);
		markers(Rect(0, markers.rows - 50, 100, 50)) = Scalar::all(1);
		markers(Rect(markers.cols - 100, markers.rows - 50, 100, 50)) = Scalar::all(1);
		markers(Rect(markers.cols - 100, 0, 100, 50)) = Scalar::all(1);

		double scaleX = (boardSize.width + 1.0)/ boardSize.width;
		double scaleY = (boardSize.height + 1.0)/ boardSize.height;
		double offsetX =  (maxX - minX) * scaleX;
		double offsetY =  (maxY - minY) * scaleY;
		markers(Rect(minX+5, minY+5,offsetX-10, offsetY-10)) = Scalar::all(200);
		
		
		markers.convertTo(segmentedDepthMat, CV_32S);
		
		Mat temp(depthMat0.size(), CV_8UC3);
		insertChannel(depthMat0,temp,0);
		insertChannel(depthMat0,temp,1);
		insertChannel(depthMat0,temp,2);
		
		watershed(temp, segmentedDepthMat);

		//pyrMeanShiftFiltering(temp, segmentedDepthMat, 30, 30, 2);
		segmentedDepthMat.convertTo(segmentedDepthMat, CV_8U);
		threshold(segmentedDepthMat, segmentedDepthMat, 1, 255, THRESH_TOZERO); 
		*/

		Point2f p0	= rgbPoints2[0];
		Point2f p1	= rgbPoints2[boardSize.width - 1];
		Point2f p2	= rgbPoints2[rgbPoints2.size() - boardSize.width];
		Point2f p3  = rgbPoints2[rgbPoints2.size() - 1];


		short ThresholdDepth[4];
		ThresholdDepth[0] = depthMat0.at<unsigned char>(p0);
		ThresholdDepth[1] = depthMat0.at<unsigned char>(p1);
		ThresholdDepth[2] = depthMat0.at<unsigned char>(p2);
		ThresholdDepth[3] = depthMat0.at<unsigned char>(p3);
		
		int min = INT_MAX, max = -1;
		for(int i = 0; i < 4; i++)
		{
			if(ThresholdDepth[i] < min)min = ThresholdDepth[i];
			if(ThresholdDepth[i] > max)max = ThresholdDepth[i];
		}

		threshold(depthMat0, segmentedDepthMat, max + 10, 255, THRESH_TOZERO_INV);
		threshold(segmentedDepthMat, segmentedDepthMat, min - 10, 255, THRESH_TOZERO);
		
		//threshold(segmentedDepthMat, segmentedDepthMat, ThresholdDepth - boardSize.width, 255, THRESH_TOZERO);
		
		//cv::imshow( "Depth image", segmentedDepthMat);
		//cv::waitKey( 20 );
		
		vector<Vec4i> lines;
		// TODO I.2.3.b: From segmentedDepthMat, extract edges by Canny (or other) detector
		//				 Detect lines using HoughLinesP	
		//
		// fill your code here
		//
		cv::Mat cannyOutput;
		int threshold1 = 50;
		int threshold2 = 200;

		Canny(segmentedDepthMat, cannyOutput, threshold1, threshold2);

		HoughLinesP(cannyOutput, lines, 1, CV_PI/360, 20, 20.0,10);

		bool boardEdgeFound[4] = {0};
		Vec4i boardEdges[4];
		// TODO I.2.3.c: Use the known inner corners to filter the detected lines. Save them into vector boardEdges
		//
		// fill your code here
		//
		
		/*
		Point2f p0	= rgbPoints2[0];
		Point2f p1	= rgbPoints2[boardSize.width - 1];
		Point2f p2	= rgbPoints2[rgbPoints2.size() - boardSize.width];
		Point2f p3  = rgbPoints2[rgbPoints2.size() - 1];
		*/
		double slopetop		= atan(abs((p1.y - p0.y)/(p1.x - p0.x)));
		double slopebottom	= atan(abs((p3.y - p2.y)/(p3.x - p2.x)));
		double slopeleft	= ((p2.x - p0.x) != 0) ? atan(abs((p2.y - p0.y)/(p2.x - p0.x))) : CV_PI / 2;
		double sloperight	= ((p3.x - p1.x) != 0) ? atan(abs((p3.y - p1.y)/(p3.x - p1.x))) : CV_PI / 2;
		
		int idx[4] = {-1};
		float nearTop = FLT_MAX, nearBottom = FLT_MAX, nearLeft = FLT_MAX, nearRight =  FLT_MAX;
		
		for( size_t i = 0; i < lines.size(); i++ )
		{
			//If the slope of the edges is not nearly vertical or //horizontal we discard them.
			double slopeLine = ((lines[i][2] - lines[i][0]) != 0)?atan(double(abs(lines[i][3] - lines[i][1]) / (lines[i][2] - lines[i][0]))): CV_PI / 2;
			int minX = (lines[i][0] < lines[i][2])? lines[i][0] : lines[i][2] ;
			int maxX = (lines[i][0] > lines[i][2])? lines[i][0] : lines[i][2] ;
			int minY = (lines[i][1] < lines[i][3])? lines[i][1] : lines[i][3] ;
			int maxY = (lines[i][1] > lines[i][3])? lines[i][1] : lines[i][3] ;
			double limit = (10 / 180.0) * CV_PI;
			if((abs(slopeLine - slopetop)  < limit) && (minY < p0.y) && ((p0.y - minY)  < nearTop) &&
				!((p0.x > maxX) || (p3.x < minX))) 
			{
				nearTop = (p0.y - minY);
				idx[0] = i;
			}
			
			if((abs(slopeLine - sloperight)  < limit) && (maxX > p3.x) && ((maxX - p3.x)  < nearRight) &&
				!((p0.y > maxY) || (p3.y < minY)))
			{
				nearRight = (maxX - p3.x);
				idx[1] = i;
			}
			
			if((abs(slopeLine - slopebottom)  < limit) && (maxY > p3.y) && ((maxY - p3.y)  < nearBottom) &&
				!((p0.x > maxX) || (p3.x < minX)))
			{
				nearBottom = (maxY - p3.y);
				idx[2] = i;
			}

			if((abs(slopeLine - slopeleft)  < limit) && (minX < p0.x) && ((p0.x - minX )  < nearLeft) &&
				!((p0.y > maxY) || (p3.y < minY)))
			{
				nearLeft = (p0.x - minX );
				idx[3] = i;
			}
		}

		for(int i = 0; i < 4; i++)
		{
			if(idx[i] != -1)
			{
				boardEdgeFound[i] = true;
				boardEdges[i] = lines[idx[i]];
			}
		}

		bool allBorderFound = true;
		for (int i=0;i<4;i++){
			if (!boardEdgeFound[i]) {
				allBorderFound = false;
				break;
			}
			else
			{
				line(depthMat1,Point(boardEdges[i][0],boardEdges[i][1]),Point(boardEdges[i][2],boardEdges[i][3]),Scalar(0,0,255),2);
			}
		}
		if (!allBorderFound) continue;
		cv::imshow( "Depth image", depthMat1 );
		cv::waitKey( 20 );

		vector<Point2f> board2DCorners;
		// TODO I.2.3.d: Compute the board outer corners as the intersections of the board edges (in boardEdges)
		//				 Save them into board2DCorners
		//				 Make sure that your corners are sorted in the correct order regarding the computed 3-D coordinates in part I.1.b
		//
		// fill your code here
		//

		Vec4i edge1, edge2;
		{
			edge1 = boardEdges[0];
			edge2 = boardEdges[3];
			int a1 = edge1[0] - edge1[2];
			int b1 = edge1[3] - edge1[1];
			int c1 = edge1[2] * edge1[1] - edge1[0] * edge1[3];
			int a2 = edge2[0] - edge2[2];
			int b2 = edge2[3] - edge2[1];
			int c2 = edge2[2] * edge2[1] - edge2[0] * edge2[3];
			if((a1*b2 - a2*b1) == 0) continue;
			Point p((c1*a2 - a1*c2)/(a1*b2 - a2*b1),(b1*c2 - c1*b2)/(a1*b2 - a2*b1));
			board2DCorners.push_back(p);
		}
		{
			edge1 = boardEdges[0];
			edge2 = boardEdges[1];
			int a1 = edge1[0] - edge1[2];
			int b1 = edge1[3] - edge1[1];
			int c1 = edge1[2] * edge1[1] - edge1[0] * edge1[3];
			int a2 = edge2[0] - edge2[2];
			int b2 = edge2[3] - edge2[1];
			int c2 = edge2[2] * edge2[1] - edge2[0] * edge2[3];
			if((a1*b2 - a2*b1) == 0) continue;
			Point p((c1*a2 - a1*c2)/(a1*b2 - a2*b1),(b1*c2 - c1*b2)/(a1*b2 - a2*b1));
			board2DCorners.push_back(p);
		}
		{
			edge1 = boardEdges[2];
			edge2 = boardEdges[3];
			int a1 = edge1[0] - edge1[2];
			int b1 = edge1[3] - edge1[1];
			int c1 = edge1[2] * edge1[1] - edge1[0] * edge1[3];
			int a2 = edge2[0] - edge2[2];
			int b2 = edge2[3] - edge2[1];
			int c2 = edge2[2] * edge2[1] - edge2[0] * edge2[3];
			if((a1*b2 - a2*b1) == 0) continue;
			Point p((c1*a2 - a1*c2)/(a1*b2 - a2*b1),(b1*c2 - c1*b2)/(a1*b2 - a2*b1));
			board2DCorners.push_back(p);
		}
		{
			edge1 = boardEdges[1];
			edge2 = boardEdges[2];
			int a1 = edge1[0] - edge1[2];
			int b1 = edge1[3] - edge1[1];
			int c1 = edge1[2] * edge1[1] - edge1[0] * edge1[3];
			int a2 = edge2[0] - edge2[2];
			int b2 = edge2[3] - edge2[1];
			int c2 = edge2[2] * edge2[1] - edge2[0] * edge2[3];
			if((a1*b2 - a2*b1) == 0) continue;
			Point p((c1*a2 - a1*c2)/(a1*b2 - a2*b1),(b1*c2 - c1*b2)/(a1*b2 - a2*b1));
			board2DCorners.push_back(p);
		}


		if (board2DCorners.size() < 4) continue;

		// estimate the outer corners' depth
		vector<short> boardCornersDepth;
		for (int j=0;j<4;j++){
			Point2f dstPoint;
			if (!findClosestPoint(segmentedDepthMat, board2DCorners[j], dstPoint, 5)) 
				break;
			boardCornersDepth.push_back(depthMat.at<short>(floor(dstPoint.y+0.5),floor(dstPoint.x+0.5)));
			circle(depthMat1,board2DCorners[j],2,Scalar(0,255,0),2);
			circle(depthMat1,dstPoint,2,Scalar(255,0,0),2);
		}
		if (boardCornersDepth.size() < 4) continue;
		cv::imshow( "Depth image", depthMat1 );
		cv::waitKey( 20 );

		// a good frame -> save all useful information
		goodFrames.push_back(frame);
		incorner2DPoints[RGB_SENSOR].push_back(rgbPointBuf);
		incorner2DPoints[EXT_SENSOR].push_back(extPointBuf);
		boardCorner2DPoints.push_back(board2DCorners);
		allBoardCornerDepth.push_back(boardCornersDepth);
		rvecs[RGB_SENSOR].push_back(rVec);
		tvecs[RGB_SENSOR].push_back(tVec);

		// combine the processed images
		Mat tmpOut;
		hconcat(rgbMat,extResized,tmpOut);
		//
		Mat outView;
		hconcat(depthMat1,tmpOut,outView);
		cv::namedWindow( "Kinect Calibration", 1 );
		cv::imshow( "Kinect Calibration", outView );
		cv::waitKey( 100 );

		if (goodFrames.size() >= nFrame2Use){
			runCalibration(inCorners3D, imageSize[EXT_SENSOR],  cameraMatrix[EXT_SENSOR], distCoeffs, incorner2DPoints[EXT_SENSOR],rvecs[EXT_SENSOR],tvecs[EXT_SENSOR]);
			extCalibrated = true;
			break;
		}
	}

	if (!extCalibrated) {
		runCalibration(inCorners3D, imageSize[EXT_SENSOR],  cameraMatrix[EXT_SENSOR], distCoeffs, incorner2DPoints[EXT_SENSOR],rvecs[EXT_SENSOR],tvecs[EXT_SENSOR]);
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////             STEP 2: Calibrate the depth camera             ////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	double cx = imageSize[DEPTH_SENSOR].width/2;		// the depth sensor's principal point
	double cy = imageSize[DEPTH_SENSOR].height/2;
	
	// Compute translation t1 between the RGB & depth sensor
	Mat t1 = Mat::zeros(3,1,CV_32F);
	for (int i=0;i<goodFrames.size();i++){
		Mat matR;
		// TODO I.3: Compute translation t1 between the RGB & depth sensor
		//           (see project decription for more details)
		// Inputs:
		//	  - Rotation vector for RGB image (rvecs[RGB_SENSOR][i])
		//	  - Translation vector for RGB image (tvecs[RGB_SENSOR][i])
		//    - 3-D coordinates of the board corners in {B} (matBoardCorners3D)
		//    - Detected 2-D board corners in the depth image (boardCorner2DPoints)
		//    - Board corners' depth in {D} (allBoardCornerDepth)
		// Task:
		//	  update t1 according to equation (6)
		//
		// fill your code here
		//

		Rodrigues(rvecs[RGB_SENSOR][i], matR);

		vector<Point3f> RcPbm;
		vector<Point3f> P_d;
		
		Mat MatP_b;
		
		for(int j = 0; j < 4; j++)
		{
			float P_dj[] =	{((boardCorner2DPoints[i][j].x - cx) * allBoardCornerDepth[i][j] / fd),
							 ((boardCorner2DPoints[i][j].y - cy) * allBoardCornerDepth[i][j] / fd),
							 allBoardCornerDepth[i][j]};

			Mat MatP_dj(3, 1, CV_32F, P_dj);
			Mat MatP_bj(boardCorners3D[j]);
			Mat tempResult1 = matR * MatP_bj + tvecs[RGB_SENSOR][i];
			Mat tempResult2 = cameraMatrix[RGB_SENSOR] * tempResult1;
			Mat tempResult = MatP_dj - tempResult1;
			t1 += tempResult;
			
		}
	}
	t1 = t1/(4*goodFrames.size());
	
	Point3f vt1(t1);

	////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////             STEP 3: Calibrate the external camera          ////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	Mat r2;
	Mat t2;
	vector<Point3f> point3D;
	vector<Point2f> point2D;
	for (int i=0;i<goodFrames.size();i++){
		// TODO II.1.3: collect the inner corners' 3-D coordinates in {D} and their projection in {E}
		//              (see project decription for more details)
		// Inputs:
		//	  - Rotation vector for RGB image (rvecs[RGB_SENSOR][i])
		//	  - Translation vector for RGB image (tvecs[RGB_SENSOR][i])
		//	  - Translation vector between the RGB & the depth sensor (t1)
		//    - Detected inner corners in the external image (incorner2DPoints[EXT_SENSOR])
		// Task:
		//	  update point3D & point2D
		//
		// fill your code here
		//
		Mat matR;
		Rodrigues(rvecs[RGB_SENSOR][i], matR);
		for(int j = 0; j < incorner2DPoints[EXT_SENSOR][i].size(); j++)
		{
			point2D.push_back(incorner2DPoints[EXT_SENSOR][i][j]);

			Mat MatP_bj(inCorners3D[j]);
			Mat tempResult1 = matR * MatP_bj + tvecs[RGB_SENSOR][i];
			Mat tempResult2 = tempResult1 + t1;
			point3D.push_back(Point3f(tempResult2));
		}
	}
	// TODO II.1.4: use solvePnPRansac to compute r2 & t2
	//
	// fill your code here
	//
	solvePnPRansac(point3D, point2D, cameraMatrix[EXT_SENSOR], distCoeffs, r2, t2);

	Point3f vv1(t2);
	Point3f vvr(r2);

	////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////         STEP 4: Paint depth images by RGB & external images        ////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int i=0;i<goodFrames.size();i++){
		int frame = goodFrames.at(i);
		Mat depthMat, rgbMat0, extMat0, rgbMat, extMat;

		if (!loadImages(src, frame, depthMat, rgbMat0, extMat0)){
			continue;
		}

		// comput 3D coordinates of every pixel in the depth map in {D}
		vector<Point3f> _3DPoints;
		vector<int> pSet, qSet;							// saved row & col indices of _3DPoints
		for (int p=0;p<depthMat.rows;p++){				// row index
			for (int q=0;q<depthMat.cols;q++){			// col index
				if (depthMat.at<short>(p,q) != 0){
					float X, Y, Z;
					// TODO I.4.a: compute 3D coordinate (X,Y,Z) of each pixel in the depth image according to {D}
					//
					// fill your code here
					//
					X = (q - cx) * depthMat.at<short>(p,q) / fd;
					Y = (p - cy) * depthMat.at<short>(p,q) / fd;
					Z = depthMat.at<short>(p,q);

					_3DPoints.push_back(Point3f(X,Y,Z));
					pSet.push_back(p);
					qSet.push_back(q);
				}
			}
		}

		////////   From the RGB image
		printf("Projecting the RGB image...\n");
		vector<Point2f> projectedPoints;
		Mat outTest = Mat::zeros(rgbMat0.size(),rgbMat0.type());
		int pointCount = 0;
		// TODO I.4.b: 
		//     - Project these 3D points (_3DPoints) onto the color image using transformation [I, -t1]
		//	   - If the projected point is inside the RGB image, pick its color & paint the corresponding point 
		//       (see pSet & qSet) in the output image (outTest)
		//       You may want to use avSubPixelValue8U3 to get a smoother result (optional)
		//	   - Count the number of the ploted points
		//
		// fill your code here
		//

		Mat zerovec = Mat::zeros(rvecs[RGB_SENSOR][i].size(),rvecs[RGB_SENSOR][i].type());
		projectPoints(_3DPoints,zerovec, -t1, cameraMatrix[RGB_SENSOR],distCoeffRGB, projectedPoints);

		
		for(int m = 0; m < projectedPoints.size(); m++)
		{
			if(projectedPoints[m].x >= 0 && projectedPoints[m].x < rgbMat0.cols && 
				projectedPoints[m].y >= 0 && projectedPoints[m].y < rgbMat0.rows)
			{
				outTest.at<Vec3b>(pSet[m], qSet[m]) = avSubPixelValue8U3(projectedPoints[m],rgbMat0);
				pointCount++;
			}
		}
		
		// save the output image
		sprintf(fpath,"%s/rgb_%04d.png",dst,frame);
		imwrite(fpath, outTest);
		// save the output point cloud
		sprintf(fpath,"%s/rgb_%04d.ply",dst,frame);
		writePLY(pointCount,fd,outTest,depthMat,fpath);

		
		////////   From the external image
		printf("Projecting the external image...\n");

		vector<Point2f> projectedPoints2;
		Mat outTest2 = Mat::zeros(rgbMat0.size(),rgbMat0.type());
		Mat extDepth = Mat::ones(extMat0.rows/4,extMat0.cols/4,CV_32F) * 1E+10;
		int pointCount2 = 0;
		Mat outDepth = Mat::zeros(depthMat.size(),depthMat.type());		// save depth value of ploted points
																		// used when writing the point cloud
		// TODO II.2: 
		//     - Project these 3D points (_3DPoints) onto the external image using transformation [r2, t2]
		//	   - Update the Z-buffer (extDepth).
		//     - Project these 3D points (_3DPoints) again onto the external image. 
		//		 If a 3D point has depth < 120% the stored depth in Z-buffer:
		//			+ Pick color & paint the corresponding point (see pSet & qSet) in the output image (outTest2)
		//			  You may want to use avSubPixelValue8U3 to get a smoother result (optional)
		//			+ Set its depth in outDepth
		//			+ Count the number of the ploted points
		//       Otherwise, skip the point
		//
		// fill your code here
		//

		projectPoints(_3DPoints,r2, t2, cameraMatrix[EXT_SENSOR],distCoeffs, projectedPoints2);
		
		for(int m = 0; m < projectedPoints2.size(); m++)
		{
			if(projectedPoints2[m].x >= 0 && projectedPoints2[m].x < extMat0.cols && 
				projectedPoints2[m].y >= 0 && projectedPoints2[m].y < extMat0.rows)
			{
				int X = (projectedPoints2[m].x / 4);
				int Y = (projectedPoints2[m].y / 4);

				if(extDepth.at<float>(Y, X) > _3DPoints[m].z)
					extDepth.at<float>(Y, X) = _3DPoints[m].z;
			}
		}

		for(int m = 0; m < projectedPoints2.size(); m++)
		{
			if(projectedPoints2[m].x >= 0 && projectedPoints2[m].x < extMat0.cols && 
				projectedPoints2[m].y >= 0 && projectedPoints2[m].y < extMat0.rows)
			{
				int X = (projectedPoints2[m].x / 4);
				int Y = (projectedPoints2[m].y / 4);
				float extDepthAtXY =  extDepth.at<float>(Y, X);

				if(_3DPoints[m].z <= (1.2 * extDepthAtXY))
				{
					outTest2.at<Vec3b>(pSet[m], qSet[m]) = avSubPixelValue8U3(projectedPoints2[m],extMat0);
					outDepth.at<short>(pSet[m], qSet[m]) = floor(extDepthAtXY);
					pointCount2++;
				}
			}

		}
		

		// save the output image
		sprintf(fpath,"%s/ext_%04d.png",dst,frame);
		imwrite(fpath,outTest2);
		// save the output point cloud
		sprintf(fpath,"%s/ext_%04d.ply",dst,frame);
		writePLY(pointCount2,fd,outTest2,outDepth,fpath);
		
		// output the depth map
		Mat depthMat0 = depthMat / 4;
		depthMat0.convertTo(depthMat0, CV_8U);

		Mat depthMat1(depthMat.rows,depthMat.cols,CV_8UC3);
		insertChannel(depthMat0,depthMat1,0);
		insertChannel(depthMat0,depthMat1,1);
		insertChannel(depthMat0,depthMat1,2);

		// combine output & display
		Mat tmpOut,outfinal;
		hconcat(outTest,outTest2,tmpOut);
		hconcat(depthMat1,tmpOut,outfinal);

		cv::imshow( "Kinect Calibration", outfinal );
		cv::waitKey( 50 );
	}
	return 0;
}