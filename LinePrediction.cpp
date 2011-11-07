//
//  LinePrediction.cpp
//  
//
//  Created by Kevin Hampton on 11/3/11.
//  Copyright (c) 2011 Ga Tech. All rights reserved.
//

#include "LinePrediction.h"

const int THRESHOLD = 150;

Line* houghDetectLines(IplImage* src) {
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* lines = 0;
  IplImage* color_dst = cvCreateImage(cvGetSize(src), 8, 3);
  lineImage = cvCreateImage(cvGetSize(src), 8, 3);
  cvSet(lineImage, cvScalar(0,0,0));
  
  lines = cvHoughLines2(src, storage, CV_HOUGH_PROBABILISTIC, 
                        1, CV_PI/180,80,15,10);
  
  vector<Vec<double, 7> > vecs = vector<Vec<double,7> >();
  
  for(int i = 0; i < MIN(lines->total, 100); i++) {
    
    CvPoint* lin = (CvPoint*)cvGetSeqElem(lines,i);
    
    //double xdist = lin[0].x-lin[1].x;
    //double ydist = lin[0].y-lin[1].y;
    double xdist = -(lin[1].y-lin[0].y);
    double ydist = lin[1].x-lin[0].x;
    double norm = sqrt(xdist*xdist + ydist*ydist);
    
    //double d = abs(xdist*lin[0].y - lin[0].x*ydist) / norm;
    double d = abs(lin[1].x*lin[0].y - lin[0].x*lin[1].y) / norm;
    
    Vec<double, 7> v(lin[0].x, lin[0].y, lin[1].x, lin[1].y, xdist/norm, ydist/norm, d);
    vecs.push_back(v);
    
    cvLine(lineImage, lin[0], lin[1], CV_RGB(255,0,0),1,8);
  }
  
  //Line Refinement
  for(int j = 0; j < vecs.size(); j++) {
    
    CvPoint tt, ttt;
    tt.x = vecs[j][0];
    tt.y = vecs[j][1];
    ttt.x = vecs[j][2];
    ttt.y = vecs[j][3];
    cvLine(lineImage, tt, ttt, CV_RGB(255,0,255),1,8);
    
    double x_diff = abs(tt.x-ttt.x);
    double y_diff = abs(tt.y-ttt.y);
    int horizontal = 0;
    
    if(x_diff > y_diff)
      horizontal = 1;
    else
      horizontal = 0;
    
    for(int i = j+1; i < vecs.size()-1; i++) {
      
      double dprod = vecs[i][4]*vecs[j][4] + vecs[i][5]*vecs[j][5];
      double angle = acos(dprod);
      
      double pt1x = vecs[i][0];
      double pt1y = vecs[i][1];
      double pt2x = vecs[i][2];
      double pt2y = vecs[i][3];
      
      double pt3x = vecs[j][0];
      double pt3y = vecs[j][1];
      double pt4x = vecs[j][2];
      double pt4y = vecs[j][3];
      
      double d1 = sqrt(pow(pt1x-pt3x,2) + pow(pt1y-pt3y,2));
      double d2 = sqrt(pow(pt1x-pt4x,2) + pow(pt1y-pt4y,2));
      double d3 = sqrt(pow(pt2x-pt3x,2) + pow(pt2y-pt3y,2));
      double d4 = sqrt(pow(pt2x-pt4x,2) + pow(pt2y-pt4y,2));
      
      int close = 0;
      int distThresh = 3;
      if(horizontal)
        close = (abs(pt1y-pt3y) < distThresh) || (abs(pt1y-pt4y) < distThresh) || (abs(pt2y-pt3y) < distThresh) || (abs(pt2y-pt3y) < distThresh);
      else
        close = (abs(pt1x-pt3x) < distThresh) || (abs(pt1x-pt4x) < distThresh) || (abs(pt2x-pt3x) < distThresh) || (abs(pt2x-pt3x) < distThresh);
      
      
      if(angle < PI/4 && close){
        CvPoint t1,t2;
        t1.x = vecs[i][0];
        t1.y = vecs[i][1];
        t2.x = vecs[i][2];
        t2.y = vecs[i][3];
        cvLine(lineImage, t1, t2, CV_RGB(0,0,255),1,8);
        vecs.erase(vecs.begin()+i);
        i--;
      }
    }
  }
  
  //Copy into Line list
  Vec<double, 7> vl;  
  Line * head = NULL, * currLine = NULL, * prevLine = NULL;
  
  for(int i = 0; i < vecs.size(); i++) {
    vl = vecs[i];
    currLine = (Line*) malloc(sizeof(struct Line));
    currLine->nx = vl[4]; 
    currLine->ny = vl[5];
    currLine->d  = vl[6];
    currLine->next = NULL;
//    printf ("X0: %10.6lf, Y0: %10.6lf, X1: %10.6lf, Y1: %10.6lf, NX: %10.6lf, NY: %10.6lf, D: %10.6lf\n", vl[0], vl[1], vl[2], vl[3], vl[4], vl[5], vl[6]);
    if (prevLine != NULL) {
    	prevLine->next = currLine;
    } else {
    	head = currLine;
    }
    prevLine = currLine;
  }
  return head;
}

IplImage* whitePixelExtraction(IplImage* frame, IplImage* dst) {
  
  double thetal = 128;
  double thetad = 20;
  int dist = 3;
  
  for(int j = 25; j < frame->height-25; j++) {
    for(int i = 25; i < frame->width-25; i++) {
      double lum = luminance(frame, i, j);
      int value = 0;
      
      if(lum >= thetal && lum-luminance(frame, i-dist, j) > thetad && 
         lum - luminance(frame,i+dist,j) > thetad) {
        value=1;
      }
      else if(lum >= thetal && lum-luminance(frame,i,j-dist) > thetad &&
              lum-luminance(frame, i, j+dist) > thetad) {
        value=1;
      }
      else {
        value=0;
      }
      ((uchar *)(dst->imageData + j*dst->widthStep))[i]=value*255;
    }
  }
  return dst;
}

double luminance(IplImage* src, int j, int i) {
	int height = src->height;
	int width = src->width;
	int step = src->widthStep/sizeof(float);
	int channels = src->nChannels; 
	float* data = (float *)src->imageData;
  
	int red = ((uchar *)(src->imageData + i*src->widthStep))[j*src->nChannels + 2];
	int green = ((uchar *)(src->imageData + i*src->widthStep))[j*src->nChannels + 1];
	int blue = ((uchar *)(src->imageData + i*src->widthStep))[j*src->nChannels + 0];
	double res = sqrt(.241*red*red + .691*green*green + .068*blue*blue);
  return res;
}

Mat calculateGradients(IplImage* src, IplImage* whitePixels) {
  IplImage* blur_src = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
  cvSmooth(src,blur_src);
  
  Mat m(whitePixels);
  IplImage* lamdas = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F, 6);
  cvCornerEigenValsAndVecs(src, lamdas, 3, 3);
  
  for(int i = 0; i < src->height; ++i) {
    for(int j = 0; j < src->width; ++j) {
      
      float whiteP = ((uchar *)(whitePixels->imageData + i*whitePixels->widthStep))[j*whitePixels->nChannels + 0];			
      
      if(whiteP == 255) {
				float l1 = ((uchar *)(lamdas->imageData + i*lamdas->widthStep))[j*lamdas->nChannels + 5];
				float l2 = ((uchar *)(lamdas->imageData + i*lamdas->widthStep))[j*lamdas->nChannels + 4];
				if(l2 != 0 && l1 >= 4*l2) {
	  			((uchar *)(whitePixels->imageData + i*whitePixels->widthStep))[j*whitePixels->nChannels + 0] = 0;
	  			m.at<int>(i,j) = 0;
          
				}
      }
      if(i < 50 || i > 300)
				m.at<int>(i,j) = 0;
    }
  }
  IplImage m_img = m;
  
  return m;
}

void getPixelAt(IplImage* src, int i, int j, int* rgb[3]) {
  
	int height = src->height;
	int width = src->width;
	int step = src->widthStep/sizeof(float);
	int channels = src->nChannels; 
	float* data = (float *)src->imageData;
	
	int r = data[i*step+j*channels+2];
	int g = data[i*step+j*channels+1];
	int b = data[i*step+j*channels+0];
	*rgb[0] = b;
	*rgb[1] = g;
	*rgb[2] = r;
}
