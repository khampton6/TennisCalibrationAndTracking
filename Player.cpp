#include "Player.h"

//int main(int params, char** argc) {
//  const char* file_name = "trimmed.avi";
//  
//  CvCapture* capture = cvCaptureFromAVI(file_name);
//  IplImage* frame = 0;
//  IplImage* prevFrame = 0;
//  
//  cvNamedWindow("Object", CV_WINDOW_AUTOSIZE);
//  cvNamedWindow("Image", CV_WINDOW_AUTOSIZE);
//
//  cvWaitKey(33);
//  
//  frame = cvQueryFrame(capture);
//  prevFrame = cvCloneImage(frame);
//  
//  if(!frame) {
//    cout << "Unable to parse video: " << file_name << endl;
//    return 0;
//  }
//  
//  while(frame && prevFrame) {
//    
//  prevFrame = cvCloneImage(frame);
//  frame = cvQueryFrame(capture);
//    
//  if(!frame || !prevFrame) {
//    return 0 ;
//  }
//    
//  track(prevFrame, frame);
//    cvWaitKey(33);
//    
//  }
//
//}


int track(IplImage* object, IplImage* image) {
  
  IplImage* greyO = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
  IplImage* greyI = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
  cvCvtColor(object, greyO, CV_BGR2GRAY);
  cvCvtColor(image, greyI, CV_BGR2GRAY);
  
  CvMemStorage* storage = cvCreateMemStorage(0);
  
  CvSeq* objectKeypoints = 0, *objectDescriptors = 0;
  CvSeq* imageKeypoints = 0, *imageDescriptors = 0;
  
  CvSURFParams params = cvSURFParams(500, 1);
  
  cvExtractSURF(greyO, 0, &objectKeypoints, &objectDescriptors, storage, params);
  
  cvExtractSURF(greyI, 0, &imageKeypoints, &imageDescriptors, storage, params);
  
  vector<int> ptpairs;
  
  //Finds pairs of points
  flannFindPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
  //findPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
  
  //Commented out code will draw circles around each feature point
  for(int i = 0; i < (objectKeypoints ? objectKeypoints->total : 0); i++) {
    CvSURFPoint* surfPt = (CvSURFPoint*)cvGetSeqElem(objectKeypoints, i);
    CvPoint pt = cvPointFrom32f(surfPt->pt);
    cvCircle(object, pt, 10, cvScalar(0,255,0), 1);
  }
  
  for(int i = 0; i < imageKeypoints->total; i++) {
    CvSURFPoint* surfPt = (CvSURFPoint*)cvGetSeqElem(imageKeypoints, i);
    CvPoint pt = cvPointFrom32f(surfPt->pt);
    cvCircle(image, pt, 10, cvScalar(255,0,0), 1);
  }

  //Draw Line correspondances
  for( int i = 0; i < (int)ptpairs.size(); i += 2 )
  {
    CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( objectKeypoints, ptpairs[i] );
    CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, ptpairs[i+1] );
    cvLine( image, cvPointFrom32f(r1->pt),
           cvPointFrom32f(r2->pt), cvScalar(0,0,255) );
  }
  
  cvShowImage("Object", object);
  cvShowImage("Image", image);
}


void
findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
          const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs )
{
  int i;
  CvSeqReader reader, kreader;
  cvStartReadSeq( objectKeypoints, &kreader );
  cvStartReadSeq( objectDescriptors, &reader );
  ptpairs.clear();
  
  for( i = 0; i < objectDescriptors->total; i++ )
  {
    const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
    const float* descriptor = (const float*)reader.ptr;
    CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
    CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
    int nearest_neighbor = naiveNearestNeighbor( descriptor, kp->laplacian, imageKeypoints, imageDescriptors );
    if( nearest_neighbor >= 0 )
    {
      ptpairs.push_back(i);
      ptpairs.push_back(nearest_neighbor);
    }
  }
}

int
locatePlanarObject( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
                   const CvSeq* imageKeypoints, const CvSeq* imageDescriptors,
                   const CvPoint src_corners[4], CvPoint dst_corners[4] )
{
  double h[9];
  CvMat _h = cvMat(3, 3, CV_64F, h);
  vector<int> ptpairs;
  vector<CvPoint2D32f> pt1, pt2;
  CvMat _pt1, _pt2;
  int i, n;
  
#ifdef USE_FLANN
  flannFindPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
#else
  findPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
#endif
  
  n = (int)(ptpairs.size()/2);
  if( n < 4 )
    return 0;
  
  pt1.resize(n);
  pt2.resize(n);
  for( i = 0; i < n; i++ )
  {
    pt1[i] = ((CvSURFPoint*)cvGetSeqElem(objectKeypoints,ptpairs[i*2]))->pt;
    pt2[i] = ((CvSURFPoint*)cvGetSeqElem(imageKeypoints,ptpairs[i*2+1]))->pt;
  }
  
  _pt1 = cvMat(1, n, CV_32FC2, &pt1[0] );
  _pt2 = cvMat(1, n, CV_32FC2, &pt2[0] );
  if( !cvFindHomography( &_pt1, &_pt2, &_h, CV_RANSAC, 5 ))
    return 0;
  
  for( i = 0; i < 4; i++ )
  {
    double x = src_corners[i].x, y = src_corners[i].y;
    double Z = 1./(h[6]*x + h[7]*y + h[8]);
    double X = (h[0]*x + h[1]*y + h[2])*Z;
    double Y = (h[3]*x + h[4]*y + h[5])*Z;
    dst_corners[i] = cvPoint(cvRound(X), cvRound(Y));
  }
  
  return 1;
}


double
compareSURFDescriptors( const float* d1, const float* d2, double best, int length )
{
  double total_cost = 0;
  assert( length % 4 == 0 );
  for( int i = 0; i < length; i += 4 )
  {
    double t0 = d1[i  ] - d2[i  ];
    double t1 = d1[i+1] - d2[i+1];
    double t2 = d1[i+2] - d2[i+2];
    double t3 = d1[i+3] - d2[i+3];
    total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
    if( total_cost > best )
      break;
  }
  return total_cost;
}


int
naiveNearestNeighbor( const float* vec, int laplacian,
                     const CvSeq* model_keypoints,
                     const CvSeq* model_descriptors )
{
  int length = (int)(model_descriptors->elem_size/sizeof(float));
  int i, neighbor = -1;
  double d, dist1 = 1e6, dist2 = 1e6;
  CvSeqReader reader, kreader;
  cvStartReadSeq( model_keypoints, &kreader, 0 );
  cvStartReadSeq( model_descriptors, &reader, 0 );
  
  for( i = 0; i < model_descriptors->total; i++ )
  {
    const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
    const float* mvec = (const float*)reader.ptr;
    CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
    CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
    if( laplacian != kp->laplacian )
      continue;
    d = compareSURFDescriptors( vec, mvec, dist2, length );
    if( d < dist1 )
    {
      dist2 = dist1;
      dist1 = d;
      neighbor = i;
    }
    else if ( d < dist2 )
      dist2 = d;
  }
  if ( dist1 < 0.6*dist2 )
    return neighbor;
  return -1;
}



void
flannFindPairs( const CvSeq*, const CvSeq* objectDescriptors,
               const CvSeq*, const CvSeq* imageDescriptors, vector<int>& ptpairs )
{
	int length = (int)(objectDescriptors->elem_size/sizeof(float));
  
  cv::Mat m_object(objectDescriptors->total, length, CV_32F);
	cv::Mat m_image(imageDescriptors->total, length, CV_32F);
  
  
	// copy descriptors
  CvSeqReader obj_reader;
	float* obj_ptr = m_object.ptr<float>(0);
  cvStartReadSeq( objectDescriptors, &obj_reader );
  for(int i = 0; i < objectDescriptors->total; i++ )
  {
    const float* descriptor = (const float*)obj_reader.ptr;
    CV_NEXT_SEQ_ELEM( obj_reader.seq->elem_size, obj_reader );
    memcpy(obj_ptr, descriptor, length*sizeof(float));
    obj_ptr += length;
  }
  CvSeqReader img_reader;
	float* img_ptr = m_image.ptr<float>(0);
  cvStartReadSeq( imageDescriptors, &img_reader );
  for(int i = 0; i < imageDescriptors->total; i++ )
  {
    const float* descriptor = (const float*)img_reader.ptr;
    CV_NEXT_SEQ_ELEM( img_reader.seq->elem_size, img_reader );
    memcpy(img_ptr, descriptor, length*sizeof(float));
    img_ptr += length;
  }
  
  // find nearest neighbors using FLANN
  cv::Mat m_indices(objectDescriptors->total, 2, CV_32S);
  cv::Mat m_dists(objectDescriptors->total, 2, CV_32F);
  cv::flann::Index flann_index(m_image, cv::flann::KDTreeIndexParams(4));  // using 4 randomized kdtrees
  flann_index.knnSearch(m_object, m_indices, m_dists, 2, cv::flann::SearchParams(64) ); // maximum number of leafs checked
  
  int* indices_ptr = m_indices.ptr<int>(0);
  float* dists_ptr = m_dists.ptr<float>(0);
  for (int i=0;i<m_indices.rows;++i) {
    if (dists_ptr[2*i]<0.6*dists_ptr[2*i+1]) {
      ptpairs.push_back(i);
      ptpairs.push_back(indices_ptr[2*i]);
    }
  }
}


void backgroundSub() {
  const char* file_name = "trimmed.avi";
  int n = 100;
  
  CvCapture* capture = cvCaptureFromAVI(file_name);
  IplImage* frame = 0;
  IplImage* firstFrame = 0;
  
  cvNamedWindow("Frame", CV_WINDOW_AUTOSIZE);
  
  frame = cvQueryFrame(capture);
  firstFrame = cvCloneImage(frame);
  if(!frame) {
    cout << "Unable to parse video: " << file_name << endl;
    return;
  }
  
  int avgs[frame->height][frame->width][3];
  
  for(int in = 0; in < n; in++) {
    for(int i = 0; i < frame->height; i++) {
      for(int j = 0; j < frame->width; j++) {
        uchar red = ((uchar *)(frame->imageData + i*frame->widthStep))[j*frame->nChannels + 2];
        uchar green = ((uchar *)(frame->imageData + i*frame->widthStep))[j*frame->nChannels + 1];
        uchar blue = ((uchar *)(frame->imageData + i*frame->widthStep))[j*frame->nChannels + 0];
        
        avgs[i][j][2] += red;
        avgs[i][j][1] += green;
        avgs[i][j][0] += blue;
      }
    }
    frame=cvQueryFrame(capture);
  }
  
  if(!firstFrame)
    cout << "No First" << endl;
  
  Mat m(frame->height, frame->width, CV_32F);
  IplImage* bsub = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U,3);
  
  for(int i = 0; i < frame->height; i++) {
    for(int j = 0; j < frame->width; j++) {
      unsigned char r = avgs[i][j][2]/n;
      unsigned char b = avgs[i][j][0]/n;
      unsigned char g = avgs[i][j][1]/n;
      
      int rgb = (r << 16) + (g << 8) + b;
      
      ((uchar *)(bsub->imageData + i*bsub->widthStep))[j*bsub->nChannels + 2] = (int)r;
      ((uchar *)(bsub->imageData + i*bsub->widthStep))[j*bsub->nChannels + 1] = (int)g;
      ((uchar *)(bsub->imageData + i*bsub->widthStep))[j*bsub->nChannels + 0] = (int)b;
    }
  }
  Mat one = firstFrame;
  Mat two = bsub;
  Mat three = one-two;
  IplImage output = three;
  
  cvShowImage("Frame", &output);
  cvWaitKey(0);
}

