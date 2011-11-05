//
//  LinePrediction.h
//  
//
//  Created by Kevin Hampton on 11/3/11.
//  Copyright (c) 2011 Ga Tech. All rights reserved.
//

#ifndef _LinePrediction_h
#define _LinePrediction_h


#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <vector>
#include "ModelFitting.h"

using namespace std;
using namespace cv;

//typedef struct Line {
//  double nx;
//  double ny;
//  double d;
//  struct Line* next;
//} Line;

void whitePixelExtraction(IplImage* frame, SparseMat sparseMat);
void getPixelAt(IplImage* img, int x, int y, int* rgb[3]);
Line* houghDetectLines(IplImage* src);
double luminance(IplImage* image, int x, int y);
Mat calculateGradients(IplImage* src, IplImage* res, IplImage* wPixels);

#define PI 3.14159265

extern IplImage* lineImage;

#endif
