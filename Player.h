#ifndef _Player_h
#define _Player_h


#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <vector>

using namespace std;
using namespace cv;

int track(IplImage* img1, IplImage* img2);
void backgroundSub();

void flannFindPairs( const CvSeq*, const CvSeq* objectDescriptors,
    const CvSeq*, const CvSeq* imageDescriptors, vector<int>& ptpairs );
void findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
    const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs );
int naiveNearestNeighbor( const float* vec, int laplacian,
                     const CvSeq* model_keypoints,
                     const CvSeq* model_descriptors );
int locatePlanarObject( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors, const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, const CvPoint src_corners[4], CvPoint dst_corners[4] );

double compareSURFDescriptors( const float* d1, const float* d2, double best, int length );

#endif
