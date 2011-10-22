#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <string.h>

using namespace std;

int main(int argc, char** argv) {
  const char* movie = "truck2.avi";

  CvCapture* capture = cvCaptureFromAVI(movie);
  IplImage* image = 0;

  cvNamedWindow("Display", CV_WINDOW_AUTOSIZE);

  while(true) {
    image = cvQueryFrame(capture);

    if(!image)
      break;

    cvShowImage("Display", image);
    
    char key = cvWaitKey(30);
    if(key == 27) break;
  }
  cvReleaseCapture(&capture);
  cvReleaseImage(&image);
}
