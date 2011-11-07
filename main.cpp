#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <vector>

#include "LinePrediction.h"
#include "ModelFitting.h"
#include "Player.h"

using namespace std;
using namespace cv;

IplImage* lineImage;

int main(int argc, char** argv) {
  
  const char* movie = "trimmed.avi";
  Line * myLine;
  
  if(argc == 2) {
    movie = argv[1];
  }
  
  CvCapture* capture = cvCaptureFromAVI(movie);
  IplImage* image = 0;
  IplImage* prevFrame = 0, *currFrame = 0;

  cvNamedWindow("White Ps", CV_WINDOW_AUTOSIZE);
  
  //Grabbing first frame
  image = cvQueryFrame(capture);
  prevFrame = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
  if(!image) {
    cout << "Unable to parse video: " << movie << endl;
    return 0;
  }
  
  IplImage* smat = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
  whitePixelExtraction(image, smat);
  
  IplImage* res = cvCloneImage(smat);
  calculateGradients(smat, res);  

  Line* Image_Lines_From_Kevin = houghDetectLines(res);
  cvShowImage("White Ps", res);
    
 // myLine = Image_Lines_From_Kevin;
  //Fit_Model_To_Image(Image_Lines_From_Kevin);

  while(image && prevFrame) {
    prevFrame = cvCloneImage(image);
    image = cvQueryFrame(capture);
    if(!image || !prevFrame)
      break;
    char c = cvWaitKey(33);
    if(c == 27)
      break;
    
    IplImage* smat = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
 		cvSet(smat, cvScalar(0,0,0));
 		whitePixelExtraction(image, smat);
		cvShowImage("White Ps", smat);
    currFrame = smat;
        
    track(prevFrame, image);
  }

  return(0);
}
