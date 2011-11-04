//
//  ModelFitting.h
//  
//
//  Created by Kevin Hampton on 11/3/11.
//  Copyright (c) 2011 Ga Tech. All rights reserved.
//

#ifndef _ModelFitting_h
#define _ModelFitting_h

//#include <sys/types.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <netdb.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>

//TODO: De-allocate dynamic memory
/* #define TODO_FROM_KEVIN 0.0 */

typedef struct Line {
	double nx;
	double ny;
	double d;
	struct Line * next;
} Line;

typedef struct CourtLines {
	Line * hor;
	Line * ver;
} CourtLines;

int Fit_Model_To_Image (Line *);
int Build_Model_Data_Structure (void);
int Build_Image_Data_Structure (Line *);
int compare_routine_hor (const void *, const void *);
int compare_routine_ver (const void *, const void *);
int Compute_Intersection_Of_2Lines (CvPoint2D64f *, Line *, Line *);
int Evaluate_Model_Support (void);
int Compute_Matching_Score (double *);

//extern int best_matching_score;

#endif
