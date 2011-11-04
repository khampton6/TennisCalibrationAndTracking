#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <vector>

#include "LinePrediction.h"
#include "ModelFitting.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
  
  const char* movie = "trimmed.avi";
  
  if(argc == 2) {
    movie = argv[1];
  }
  
  CvCapture* capture = cvCaptureFromAVI(movie);
  IplImage* image = 0;
  
  //Grabbing first frame
  image = cvQueryFrame(capture);
  if(!image) {
    cout << "Unable to parse video: " << movie << endl;
    return 0;
  }
  
  cout << image->height << " " << image->width << endl;
  
  int mat_dims[2] = {image->height, image->width};
  SparseMat smat = SparseMat(2, mat_dims, CV_8UC1);
  whitePixelExtraction(image, smat);
  
  Mat m;
  smat.convertTo(m, CV_8UC1);
  IplImage i = m;
  
  IplImage* res;
  Mat grads = calculateGradients(&i,res, &i);  
  
  IplImage j = grads;
  
  Line* Image_Lines_From_Kevin = houghDetectLines(&j);
  
  Fit_Model_To_Image(Image_Lines_From_Kevin);
  return(0);
}
