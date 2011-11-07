
#include "ModelFitting.h"
#include "LinePrediction.h"

double calib_params[3][3];
double best_calib_params[3][3];
int best_matching_score = -1000000;
int best_match_count = 0;
CourtLines CModel;
CourtLines CImage;
int image_hor_line_count = 0, image_ver_line_count = 0;
int best_var1, best_var2, best_var3, best_var4, best_var5, best_var6, best_var7,best_var8;

CvPoint2D64f Left_Border_Center, Top_Border_Center;
CvMat matrix_1, matrix_2, matrix_3, matrix_4, matrix_5, matrix_6;
Line ** Model_Hor_Line_Array, ** Model_Ver_Line_Array;
Line ** Image_Hor_Line_Array, ** Image_Ver_Line_Array;

//IplImage* lineImage;

void setLineImage(IplImage* line) {
	lineImage = line;
}

//int main(int argc, char * argv[]) {
//	//Build model data structure
//	//Obtain image data structure
//	//Classify lines in both data structures as either hor or ver
//	//Order ver lines from left to right in both
//	//Order hor lines from top to bottom in both
//	Line * Image_Lines_From_Kevin;
//	Fit_Model_To_Image(Image_Lines_From_Kevin);
//	return (0);
//}

//Build, classify and order lines
int Build_Model_Data_Structure (void) {
	int bmds_var1;
	Line * bmds_prev, * bmds_curr;

	bmds_prev = NULL; bmds_curr = NULL;
	for (bmds_var1=0;bmds_var1<5;bmds_var1++) {
		bmds_curr = (Line *) malloc(sizeof(struct Line));
		bmds_curr->nx = 0.0;
		bmds_curr->ny = 1.0;
		bmds_curr->next = NULL;
		if (bmds_prev != NULL) {
			bmds_prev->next = bmds_curr;
		} else {
			CModel.hor = bmds_curr;
		}
		bmds_prev = bmds_curr;
	}
	//Origin is assumed to be at top-right corner
	//Distance 'd' measured in feet
	//Hor lines are ordered from top to bottom
	bmds_curr = CModel.hor;
	bmds_curr->d = 21;
	bmds_curr = bmds_curr->next;
	bmds_curr->d = 39;
	bmds_curr = bmds_curr->next;
	bmds_curr->d = 60;
	bmds_curr = bmds_curr->next;
	bmds_curr->d = 81;
	bmds_curr = bmds_curr->next;
	bmds_curr->d = 99;
	bmds_curr = bmds_curr->next; //bmds_curr should point to NULL here

	bmds_prev = NULL; bmds_curr = NULL;
	for (bmds_var1=0;bmds_var1<5;bmds_var1++) {
		bmds_curr = (Line *) malloc(sizeof(struct Line));
		bmds_curr->nx = 1.0;
		bmds_curr->ny = 0.0;
		bmds_curr->next = NULL;
		if (bmds_prev != NULL) {
			bmds_prev->next = bmds_curr;
		} else {
			CModel.ver = bmds_curr;
		}
		bmds_prev = bmds_curr;
	}
	//Ver lines are ordered from left to right
	bmds_curr = CModel.ver;
	bmds_curr->d = 12;
	bmds_curr = bmds_curr->next;
	bmds_curr->d = 16.5;
	bmds_curr = bmds_curr->next;
	bmds_curr->d = 30;
	bmds_curr = bmds_curr->next;
	bmds_curr->d = 43.5;
	bmds_curr = bmds_curr->next;
	bmds_curr->d = 48;
	bmds_curr = bmds_curr->next; //bmds_curr should point to NULL here
	
	Model_Hor_Line_Array = (Line**) malloc(5*sizeof(Line *));
	bmds_var1 = 0;
	bmds_curr = CModel.hor;
	while (bmds_curr != NULL) {
		*(Model_Hor_Line_Array+bmds_var1) = bmds_curr;
		bmds_var1++;
		bmds_curr = bmds_curr->next;
	}
	
	Model_Ver_Line_Array = (Line**) malloc(5*sizeof(Line *));
	bmds_var1 = 0;
	bmds_curr = CModel.ver;
	while (bmds_curr != NULL) {
		*(Model_Ver_Line_Array+bmds_var1) = bmds_curr;
		bmds_var1++;
		bmds_curr = bmds_curr->next;
	}
	
	return (0);
}

int compare_routine_hor (const void * crh_param_a, const void * crh_param_b) {
	double crh_w_a, crh_w_b;
	Line * crh_line_a = *((Line **) crh_param_a);
	Line * crh_line_b = *((Line **) crh_param_b);
	crh_w_a = (crh_line_a->nx)*(Top_Border_Center.x) + 
	          (crh_line_a->ny)*(Top_Border_Center.y) + 
		  (crh_line_a->d)*1;
	crh_w_b = (crh_line_b->nx)*(Top_Border_Center.x) + 
	          (crh_line_b->ny)*(Top_Border_Center.y) + 
		  (crh_line_b->d)*1;
	return (crh_w_a - crh_w_b);
}

int compare_routine_ver (const void * crv_param_a, const void * crv_param_b) {
	double crv_w_a, crv_w_b;
	Line * crv_line_a = *((Line **) crv_param_a);
	Line * crv_line_b = *((Line **) crv_param_b);
	crv_w_a = (crv_line_a->nx)*(Left_Border_Center.x) + 
	          (crv_line_a->ny)*(Left_Border_Center.y) + 
		  (crv_line_a->d)*1;
	crv_w_b = (crv_line_b->nx)*(Left_Border_Center.x) + 
	          (crv_line_b->ny)*(Left_Border_Center.y) + 
		  (crv_line_b->d)*1;
	return (crv_w_a - crv_w_b);
}

//Build, classify and order lines
int Build_Image_Data_Structure (Line * BIDS_Image_Lines) {
	Line * bids_curr_hor = NULL, * bids_prev_hor = NULL;
	Line * bids_curr_ver = NULL, * bids_prev_ver = NULL;
	double bids_nx, bids_ny, bids_d;
	int bids_var1, bids_is_hor_line;
	
	while (BIDS_Image_Lines != NULL) {
		bids_nx = BIDS_Image_Lines->nx;
		bids_ny = BIDS_Image_Lines->ny;
		bids_d  = BIDS_Image_Lines->d;
		printf ("NX: %lf, NY: %lf, D: %lf, ", bids_nx, bids_ny, bids_d);
		
		bids_is_hor_line = 1;
		printf("Angle: %lf, ", fabs(atan2(fabs(bids_ny), fabs(bids_nx))));
		if ( fabs(atan2(fabs(bids_ny), fabs(bids_nx))) < (((double) 44)/63) ) {
			bids_is_hor_line = 0;
		}
		printf ("bids_is_hor_line: %d\n", bids_is_hor_line);
		
		if (bids_is_hor_line) {
			bids_curr_hor = (Line *) malloc(sizeof(struct Line));
			bids_curr_hor->nx = bids_nx;
			bids_curr_hor->ny = bids_ny;
			bids_curr_hor->d  = bids_d;
			bids_curr_hor->next = NULL;
			if (bids_prev_hor != NULL) {
				bids_prev_hor->next = bids_curr_hor;
			} else {
				CImage.hor = bids_curr_hor;
			}
			bids_prev_hor = bids_curr_hor;
			image_hor_line_count++;
		} else {
			bids_curr_ver = (Line *) malloc(sizeof(struct Line));
			bids_curr_ver->nx = bids_nx;
			bids_curr_ver->ny = bids_ny;
			bids_curr_ver->d  = bids_d;
			bids_curr_ver->next = NULL;
			if (bids_prev_ver != NULL) {
				bids_prev_ver->next = bids_curr_ver;
			} else {
				CImage.ver = bids_curr_ver;
			}
			bids_prev_ver = bids_curr_ver;
			image_ver_line_count++;
		}
		
		BIDS_Image_Lines = BIDS_Image_Lines->next;
	}
	
	//printf ("HERE1\n");
	
	Image_Hor_Line_Array = (Line**) malloc(image_hor_line_count*sizeof(Line *));
	bids_var1 = 0;
	bids_curr_hor = CImage.hor;
	while (bids_curr_hor != NULL) {
		*(Image_Hor_Line_Array+bids_var1) = bids_curr_hor;
		bids_var1++;
		//printf ("bids_var1: %d\n", bids_var1);
		bids_curr_hor = bids_curr_hor->next;
	}
	
	//printf ("HERE2\n");
	
	Image_Ver_Line_Array = (Line**) malloc(image_ver_line_count*sizeof(Line *));
	bids_var1 = 0;
	bids_curr_ver = CImage.ver;
	while (bids_curr_ver != NULL) {
		*(Image_Ver_Line_Array+bids_var1) = bids_curr_ver;
		bids_var1++;
		bids_curr_ver = bids_curr_ver->next;
	}

	qsort(Image_Hor_Line_Array, image_hor_line_count, sizeof(Line *), compare_routine_hor);
	qsort(Image_Ver_Line_Array, image_ver_line_count, sizeof(Line *), compare_routine_ver);
	
	return (0);
}

int Compute_Intersection_Of_2Lines (CvPoint2D64f * cio2_point, Line * cio2_line_1, Line * cio2_line_2) {
	double cio2_var1, cio2_var2, cio2_var3;
	
	cio2_var1 = (cio2_line_1->ny)*(cio2_line_2->d) - (cio2_line_2->ny)*(cio2_line_1->d);
	cio2_var2 = -((cio2_line_1->nx)*(cio2_line_2->d) - (cio2_line_2->nx)*(cio2_line_1->d));
	cio2_var3 = (cio2_line_1->nx)*(cio2_line_2->ny) - (cio2_line_2->nx)*(cio2_line_1->ny);
	
	//To account for (nx, ny, -d) representation of a line
	cio2_var1 *= -1;
	cio2_var2 *= -1;
	
	cio2_point->x = cio2_var1/cio2_var3;
	cio2_point->y = cio2_var2/cio2_var3;
	
	//printf ("%10.6lf, %10.6lf, %10.6lf\n", cio2_var1, cio2_var2, cio2_var3);
	
	return (0);
}

int Compute_Matching_Score (double * cms_image_point) {
	//TODO: Kevin
  /*Given a point, 
   2 if on a line
   -1 if not on line
   0 if outside image boundary
   */
  int j = (int) round(cms_image_point[0]);
  int i = (int) round(cms_image_point[1]);
 
  if( (j < 0) || (j >= lineImage->width) || (i < 0) || (i >= lineImage->height)) {
    return 0;
  }
	if(lineImage == NULL)
		std::cout << "Blah" << endl;

  float* data = (float *)lineImage->imageData;
  
	int red = ((uchar *)(lineImage->imageData + i*lineImage->widthStep))[j*lineImage->nChannels + 2];
	int green = ((uchar *)(lineImage->imageData + i*lineImage->widthStep))[j*lineImage->nChannels + 1];
	int blue = ((uchar *)(lineImage->imageData + i*lineImage->widthStep))[j*lineImage->nChannels + 0];
  
  if(red > 200) {
    return 2;
  }
  else {
    return -1;
  }
}

int Evaluate_Model_Support (void) {
	int ems_matching_score = 0;
	int ems_var1, ems_var2;
	int did_best_match_happen = 0;
	double ems_model_point[3], ems_image_point[3];
	CvMat ems_model_matrix, ems_image_matrix;
	
	ems_var2 = 21;
	for (ems_var1=12; ems_var1<48; ems_var1++) {
		ems_model_point[0] = ems_var1;
		ems_model_point[1] = ems_var2;
		ems_model_point[2] = 1;
		ems_model_matrix = cvMat(3, 1, CV_64FC1, ems_model_point);
		ems_image_matrix = cvMat(3, 1, CV_64FC1, ems_image_point);
		cvMatMul (&matrix_5, &ems_model_matrix, &ems_image_matrix);
		if (ems_image_point[2] != 0.0) {
			ems_image_point[0] = (ems_image_point[0])/(ems_image_point[2]);
			ems_image_point[1] = (ems_image_point[1])/(ems_image_point[2]);
		} 
		ems_matching_score += Compute_Matching_Score(ems_image_point);
	}
	
	ems_var2 = 39;
	for (ems_var1=12; ems_var1<48; ems_var1++) {
		ems_model_point[0] = ems_var1;
		ems_model_point[1] = ems_var2;
		ems_model_point[2] = 1;
		ems_model_matrix = cvMat(3, 1, CV_64FC1, ems_model_point);
		ems_image_matrix = cvMat(3, 1, CV_64FC1, ems_image_point);
		cvMatMul (&matrix_5, &ems_model_matrix, &ems_image_matrix);
		if (ems_image_point[2] != 0.0) {
			ems_image_point[0] = (ems_image_point[0])/(ems_image_point[2]);
			ems_image_point[1] = (ems_image_point[1])/(ems_image_point[2]);
		} 
		ems_matching_score += Compute_Matching_Score(ems_image_point);
	}
	
	ems_var2 = 60;
	for (ems_var1=12; ems_var1<48; ems_var1++) {
		ems_model_point[0] = ems_var1;
		ems_model_point[1] = ems_var2;
		ems_model_point[2] = 1;
		ems_model_matrix = cvMat(3, 1, CV_64FC1, ems_model_point);
		ems_image_matrix = cvMat(3, 1, CV_64FC1, ems_image_point);
		cvMatMul (&matrix_5, &ems_model_matrix, &ems_image_matrix);
		if (ems_image_point[2] != 0.0) {
			ems_image_point[0] = (ems_image_point[0])/(ems_image_point[2]);
			ems_image_point[1] = (ems_image_point[1])/(ems_image_point[2]);
		} 
		ems_matching_score += Compute_Matching_Score(ems_image_point);
	}
	
	ems_var2 = 81;
	for (ems_var1=12; ems_var1<48; ems_var1++) {
		ems_model_point[0] = ems_var1;
		ems_model_point[1] = ems_var2;
		ems_model_point[2] = 1;
		ems_model_matrix = cvMat(3, 1, CV_64FC1, ems_model_point);
		ems_image_matrix = cvMat(3, 1, CV_64FC1, ems_image_point);
		cvMatMul (&matrix_5, &ems_model_matrix, &ems_image_matrix);
		if (ems_image_point[2] != 0.0) {
			ems_image_point[0] = (ems_image_point[0])/(ems_image_point[2]);
			ems_image_point[1] = (ems_image_point[1])/(ems_image_point[2]);
		} 
		ems_matching_score += Compute_Matching_Score(ems_image_point);
	}
	
	ems_var2 = 99;
	for (ems_var1=12; ems_var1<48; ems_var1++) {
		ems_model_point[0] = ems_var1;
		ems_model_point[1] = ems_var2;
		ems_model_point[2] = 1;
		ems_model_matrix = cvMat(3, 1, CV_64FC1, ems_model_point);
		ems_image_matrix = cvMat(3, 1, CV_64FC1, ems_image_point);
		cvMatMul (&matrix_5, &ems_model_matrix, &ems_image_matrix);
		if (ems_image_point[2] != 0.0) {
			ems_image_point[0] = (ems_image_point[0])/(ems_image_point[2]);
			ems_image_point[1] = (ems_image_point[1])/(ems_image_point[2]);
		} 
		ems_matching_score += Compute_Matching_Score(ems_image_point);
	}
	
	ems_var1 = 12;
	for (ems_var2=21; ems_var2<99; ems_var2++) {
		ems_model_point[0] = ems_var1;
		ems_model_point[1] = ems_var2;
		ems_model_point[2] = 1;
		ems_model_matrix = cvMat(3, 1, CV_64FC1, ems_model_point);
		ems_image_matrix = cvMat(3, 1, CV_64FC1, ems_image_point);
		cvMatMul (&matrix_5, &ems_model_matrix, &ems_image_matrix);
		if (ems_image_point[2] != 0.0) {
			ems_image_point[0] = (ems_image_point[0])/(ems_image_point[2]);
			ems_image_point[1] = (ems_image_point[1])/(ems_image_point[2]);
		} 
		ems_matching_score += Compute_Matching_Score(ems_image_point);
	}
	
	ems_var1 = 16.5;
	for (ems_var2=21; ems_var2<99; ems_var2++) {
		ems_model_point[0] = ems_var1;
		ems_model_point[1] = ems_var2;
		ems_model_point[2] = 1;
		ems_model_matrix = cvMat(3, 1, CV_64FC1, ems_model_point);
		ems_image_matrix = cvMat(3, 1, CV_64FC1, ems_image_point);
		cvMatMul (&matrix_5, &ems_model_matrix, &ems_image_matrix);
		if (ems_image_point[2] != 0.0) {
			ems_image_point[0] = (ems_image_point[0])/(ems_image_point[2]);
			ems_image_point[1] = (ems_image_point[1])/(ems_image_point[2]);
		} 
		ems_matching_score += Compute_Matching_Score(ems_image_point);
	}
	
	ems_var1 = 30;
	for (ems_var2=21; ems_var2<99; ems_var2++) {
		ems_model_point[0] = ems_var1;
		ems_model_point[1] = ems_var2;
		ems_model_point[2] = 1;
		ems_model_matrix = cvMat(3, 1, CV_64FC1, ems_model_point);
		ems_image_matrix = cvMat(3, 1, CV_64FC1, ems_image_point);
		cvMatMul (&matrix_5, &ems_model_matrix, &ems_image_matrix);
		if (ems_image_point[2] != 0.0) {
			ems_image_point[0] = (ems_image_point[0])/(ems_image_point[2]);
			ems_image_point[1] = (ems_image_point[1])/(ems_image_point[2]);
		} 
		ems_matching_score += Compute_Matching_Score(ems_image_point);
	}
	
	ems_var1 = 43.5;
	for (ems_var2=21; ems_var2<99; ems_var2++) {
		ems_model_point[0] = ems_var1;
		ems_model_point[1] = ems_var2;
		ems_model_point[2] = 1;
		ems_model_matrix = cvMat(3, 1, CV_64FC1, ems_model_point);
		ems_image_matrix = cvMat(3, 1, CV_64FC1, ems_image_point);
		cvMatMul (&matrix_5, &ems_model_matrix, &ems_image_matrix);
		if (ems_image_point[2] != 0.0) {
			ems_image_point[0] = (ems_image_point[0])/(ems_image_point[2]);
			ems_image_point[1] = (ems_image_point[1])/(ems_image_point[2]);
		} 
		ems_matching_score += Compute_Matching_Score(ems_image_point);
	}
	
	ems_var1 = 48;
	for (ems_var2=21; ems_var2<99; ems_var2++) {
		ems_model_point[0] = ems_var1;
		ems_model_point[1] = ems_var2;
		ems_model_point[2] = 1;
		ems_model_matrix = cvMat(3, 1, CV_64FC1, ems_model_point);
		ems_image_matrix = cvMat(3, 1, CV_64FC1, ems_image_point);
		cvMatMul (&matrix_5, &ems_model_matrix, &ems_image_matrix);
		if (ems_image_point[2] != 0.0) {
			ems_image_point[0] = (ems_image_point[0])/(ems_image_point[2]);
			ems_image_point[1] = (ems_image_point[1])/(ems_image_point[2]);
		} 
		ems_matching_score += Compute_Matching_Score(ems_image_point);
	}
	
	if (ems_matching_score == best_matching_score) {
		best_match_count++;
	}
	if (ems_matching_score > best_matching_score) {
		did_best_match_happen = 1;
		best_match_count = 1;
		best_matching_score = ems_matching_score;
		for (ems_var1=0; ems_var1<3; ems_var1++) {
			for (ems_var2=0; ems_var2<3; ems_var2++) {
				best_calib_params[ems_var1][ems_var2] = calib_params[ems_var1][ems_var2];
			}
		}
	}
	
	return (did_best_match_happen);
}

int Fit_Model_To_Image (Line * FMTI_Image_Lines) {
	
	int fmti_var1, fmti_var2, fmti_var3, fmti_var4, fmti_var5, fmti_var6, fmti_var7, fmti_var8;
	Line * model_hor_1, * model_hor_2, * model_ver_1, * model_ver_2;
	Line * image_hor_1, * image_hor_2, * image_ver_1, * image_ver_2;
	CvPoint2D64f model_point_1, model_point_2, model_point_3, model_point_4;
	CvPoint2D64f image_point_1, image_point_2, image_point_3, image_point_4;
	double fmti_matrix_1[8][8], fmti_matrix_2[8], fmti_matrix_3[8], fmti_matrix_4[8][8] = {0.0};
	double fmti_matrix_5[3][3] = {0.0};
	double fmti_f2, fmti_b2_nr, fmti_b2_dr, fmti_b2;
	int temp_var_1, temp_var_2; Line * temp_line;
	
	Left_Border_Center.x = 0;
	Left_Border_Center.y = 176;
	Top_Border_Center.x = 320;
	Top_Border_Center.y = 0;
	Build_Model_Data_Structure();
	Build_Image_Data_Structure(FMTI_Image_Lines);
	
	printf ("Hor lines:%d, Ver lines:%d\n", image_hor_line_count, image_ver_line_count);	
	printf ("Hor Lines:\n");
	for (temp_var_1=0; temp_var_1<image_hor_line_count; temp_var_1++) {
		temp_line = Image_Hor_Line_Array[temp_var_1];
		printf ("NX: %lf, NY: %lf, D: %lf\n", temp_line->nx, temp_line->ny, temp_line->d);
	}
	printf ("Ver Lines:\n");
	for (temp_var_1=0; temp_var_1<image_ver_line_count; temp_var_1++) {
		temp_line = Image_Ver_Line_Array[temp_var_1];
		printf ("NX: %lf, NY: %lf, D: %lf\n", temp_line->nx, temp_line->ny, temp_line->d);
	}
	
	for (fmti_var1=0; fmti_var1<(5-1); fmti_var1++) {
		for (fmti_var2=(fmti_var1+1); fmti_var2<5; fmti_var2++) {
			for (fmti_var3=0; fmti_var3<(image_hor_line_count-1); fmti_var3++) {
				for (fmti_var4=(fmti_var3+1); fmti_var4<image_hor_line_count; fmti_var4++) {
	for (fmti_var5=0; fmti_var5<(5-1); fmti_var5++) {
		for (fmti_var6=(fmti_var5+1); fmti_var6<5; fmti_var6++) {
			for (fmti_var7=0; fmti_var7<(image_ver_line_count-1); fmti_var7++) {
				for (fmti_var8=(fmti_var7+1); fmti_var8<image_ver_line_count; fmti_var8++) {
					
					model_hor_1 = *(Model_Hor_Line_Array+fmti_var1);
					model_hor_2 = *(Model_Hor_Line_Array+fmti_var2);
					model_ver_1 = *(Model_Ver_Line_Array+fmti_var5);
					model_ver_2 = *(Model_Ver_Line_Array+fmti_var6);
					image_hor_1 = *(Image_Hor_Line_Array+fmti_var3);
					image_hor_2 = *(Image_Hor_Line_Array+fmti_var4);
					image_ver_1 = *(Image_Ver_Line_Array+fmti_var7);
					image_ver_2 = *(Image_Ver_Line_Array+fmti_var8);
					
					Compute_Intersection_Of_2Lines (&model_point_1, model_hor_1, model_ver_1);
					Compute_Intersection_Of_2Lines (&model_point_2, model_hor_1, model_ver_2);
					Compute_Intersection_Of_2Lines (&model_point_3, model_hor_2, model_ver_1);
					Compute_Intersection_Of_2Lines (&model_point_4, model_hor_2, model_ver_2);
					
					Compute_Intersection_Of_2Lines (&image_point_1, image_hor_1, image_ver_1);
					Compute_Intersection_Of_2Lines (&image_point_2, image_hor_1, image_ver_2);
					Compute_Intersection_Of_2Lines (&image_point_3, image_hor_2, image_ver_1);
					Compute_Intersection_Of_2Lines (&image_point_4, image_hor_2, image_ver_2);
					
					fmti_matrix_1[0][0] = image_point_1.x;
					fmti_matrix_1[0][1] = image_point_1.y;
					fmti_matrix_1[0][2] = 1.0;
					fmti_matrix_1[0][3] = 0.0;
					fmti_matrix_1[0][4] = 0.0;
					fmti_matrix_1[0][5] = 0.0;
					fmti_matrix_1[0][6] = -(image_point_1.x)*(model_point_1.x);
					fmti_matrix_1[0][7] = -(image_point_1.y)*(model_point_1.x);
					
					fmti_matrix_1[1][0] = 0.0;
					fmti_matrix_1[1][1] = 0.0;
					fmti_matrix_1[1][2] = 0.0;
					fmti_matrix_1[1][3] = image_point_1.x;
					fmti_matrix_1[1][4] = image_point_1.y;
					fmti_matrix_1[1][5] = 1.0;
					fmti_matrix_1[1][6] = -(image_point_1.x)*(model_point_1.y);
					fmti_matrix_1[1][7] = -(image_point_1.y)*(model_point_1.y);
					
					fmti_matrix_1[2][0] = image_point_2.x;
					fmti_matrix_1[2][1] = image_point_2.y;
					fmti_matrix_1[2][2] = 1.0;
					fmti_matrix_1[2][3] = 0.0;
					fmti_matrix_1[2][4] = 0.0;
					fmti_matrix_1[2][5] = 0.0;
					fmti_matrix_1[2][6] = -(image_point_2.x)*(model_point_2.x);
					fmti_matrix_1[2][7] = -(image_point_2.y)*(model_point_2.x);
					
					fmti_matrix_1[3][0] = 0.0;
					fmti_matrix_1[3][1] = 0.0;
					fmti_matrix_1[3][2] = 0.0;
					fmti_matrix_1[3][3] = image_point_2.x;
					fmti_matrix_1[3][4] = image_point_2.y;
					fmti_matrix_1[3][5] = 1.0;
					fmti_matrix_1[3][6] = -(image_point_2.x)*(model_point_2.y);
					fmti_matrix_1[3][7] = -(image_point_2.y)*(model_point_2.y);
					
					fmti_matrix_1[4][0] = image_point_3.x;
					fmti_matrix_1[4][1] = image_point_3.y;
					fmti_matrix_1[4][2] = 1.0;
					fmti_matrix_1[4][3] = 0.0;
					fmti_matrix_1[4][4] = 0.0;
					fmti_matrix_1[4][5] = 0.0;
					fmti_matrix_1[4][6] = -(image_point_3.x)*(model_point_3.x);
					fmti_matrix_1[4][7] = -(image_point_3.y)*(model_point_3.x);
					
					fmti_matrix_1[5][0] = 0.0;
					fmti_matrix_1[5][1] = 0.0;
					fmti_matrix_1[5][2] = 0.0;
					fmti_matrix_1[5][3] = image_point_3.x;
					fmti_matrix_1[5][4] = image_point_3.y;
					fmti_matrix_1[5][5] = 1.0;
					fmti_matrix_1[5][6] = -(image_point_3.x)*(model_point_3.y);
					fmti_matrix_1[5][7] = -(image_point_3.y)*(model_point_3.y);
					
					fmti_matrix_1[6][0] = image_point_4.x;
					fmti_matrix_1[6][1] = image_point_4.y;
					fmti_matrix_1[6][2] = 1.0;
					fmti_matrix_1[6][3] = 0.0;
					fmti_matrix_1[6][4] = 0.0;
					fmti_matrix_1[6][5] = 0.0;
					fmti_matrix_1[6][6] = -(image_point_4.x)*(model_point_4.x);
					fmti_matrix_1[6][7] = -(image_point_4.y)*(model_point_4.x);
					
					fmti_matrix_1[7][0] = 0.0;
					fmti_matrix_1[7][1] = 0.0;
					fmti_matrix_1[7][2] = 0.0;
					fmti_matrix_1[7][3] = image_point_4.x;
					fmti_matrix_1[7][4] = image_point_4.y;
					fmti_matrix_1[7][5] = 1.0;
					fmti_matrix_1[7][6] = -(image_point_4.x)*(model_point_4.y);
					fmti_matrix_1[7][7] = -(image_point_4.y)*(model_point_4.y);

// 					printf ("model_point_1.x:%10.6lf, model_point_1.y:%10.6lf\n", model_point_1.x, model_point_1.y);
// 					printf ("image_point_1.x:%10.6lf, image_point_1.y:%10.6lf\n", image_point_1.x, image_point_1.y);
// 					printf ("model_point_2.x:%10.6lf, model_point_2.y:%10.6lf\n", model_point_2.x, model_point_2.y);
// 					printf ("image_point_2.x:%10.6lf, image_point_2.y:%10.6lf\n", image_point_2.x, image_point_2.y);
// 					printf ("model_point_3.x:%10.6lf, model_point_3.y:%10.6lf\n", model_point_3.x, model_point_3.y);
// 					printf ("image_point_3.x:%10.6lf, image_point_3.y:%10.6lf\n", image_point_3.x, image_point_3.y);
// 					printf ("model_point_4.x:%10.6lf, model_point_4.y:%10.6lf\n", model_point_4.x, model_point_4.y);
// 					printf ("image_point_4.x:%10.6lf, image_point_4.y:%10.6lf\n", image_point_4.x, image_point_4.y);
// 					for (temp_var_1=0; temp_var_1<8; temp_var_1++) {
// 						temp_var_2=0;
// 						printf ("%lf", fmti_matrix_1[temp_var_1][temp_var_2]);
// 						for (temp_var_2=1; temp_var_2<8; temp_var_2++) {
// 							printf (" %lf", fmti_matrix_1[temp_var_1][temp_var_2]);
// 						}
// 						printf ("\n");
// 					}
// 					printf ("\n");
					
					//cvInitMatHeader(&matrix_1, 8, 8, CV_64FC1, fmti_matrix_1, 8*sizeof(double));
					matrix_1 = cvMat(8, 8, CV_64FC1, fmti_matrix_1);
					matrix_4 = cvMat(8, 8, CV_64FC1, fmti_matrix_4);
					cvInv(&matrix_1, &matrix_4, CV_LU);
// 					for (temp_var_1=0,temp_var_2=0; temp_var_1<8; temp_var_1++) {
// 						temp_var_2=0;
// 						printf ("%lf", fmti_matrix_4[temp_var_1][temp_var_2]);
// 						for (temp_var_2=1; temp_var_2<8; temp_var_2++) {
// 							printf (" %lf", fmti_matrix_4[temp_var_1][temp_var_2]);
// 						}
// 						printf ("\n");
// 					}
// 					printf ("\n");
					
					fmti_matrix_2[0] = model_point_1.x;
					fmti_matrix_2[1] = model_point_1.y;
					fmti_matrix_2[2] = model_point_2.x;
					fmti_matrix_2[3] = model_point_2.y;
					fmti_matrix_2[4] = model_point_3.x;
					fmti_matrix_2[5] = model_point_3.y;
					fmti_matrix_2[6] = model_point_4.x;
					fmti_matrix_2[7] = model_point_4.y;
					
					matrix_2 = cvMat(8, 1, CV_64FC1, fmti_matrix_2);
					matrix_3 = cvMat(8, 1, CV_64FC1, fmti_matrix_3);
					cvMatMul( &matrix_4, &matrix_2, &matrix_3);
					calib_params[0][0] = fmti_matrix_3[0];
					calib_params[0][1] = fmti_matrix_3[1];
					calib_params[0][2] = fmti_matrix_3[2];
					calib_params[1][0] = fmti_matrix_3[3];
					calib_params[1][1] = fmti_matrix_3[4];
					calib_params[1][2] = fmti_matrix_3[5];
					calib_params[2][0] = fmti_matrix_3[6];
					calib_params[2][1] = fmti_matrix_3[7];
					calib_params[2][2] = 1.0;
					
// 					printf ("%10.6lf %10.6lf %10.6lf\n", calib_params[0][0], calib_params[0][1], calib_params[0][2]);
// 					printf ("%10.6lf %10.6lf %10.6lf\n", calib_params[1][0], calib_params[1][1], calib_params[1][2]);
// 					printf ("%10.6lf %10.6lf %10.6lf\n", calib_params[2][0], calib_params[2][1], calib_params[2][2]);
// 					printf ("\n");
					
					matrix_5 = cvMat(3, 3, CV_64FC1, fmti_matrix_5);
					matrix_6 = cvMat(3, 3, CV_64FC1, calib_params);
					cvInv(&matrix_6, &matrix_5, CV_LU);
// 					printf ("%10.6lf %10.6lf %10.6lf\n", fmti_matrix_5[0][0], fmti_matrix_5[0][1], fmti_matrix_5[0][2]);
// 					printf ("%10.6lf %10.6lf %10.6lf\n", fmti_matrix_5[1][0], fmti_matrix_5[1][1], fmti_matrix_5[1][2]);
// 					printf ("%10.6lf %10.6lf %10.6lf\n", fmti_matrix_5[2][0], fmti_matrix_5[2][1], fmti_matrix_5[2][2]);
					
// 					fmti_f2 = -((fmti_matrix_5[0][0])*(fmti_matrix_5[0][1])+(fmti_matrix_5[1][0])*(fmti_matrix_5[1][1]));
// 					fmti_f2 /= ((fmti_matrix_5[2][0])*(fmti_matrix_5[2][1]));
// 					
// 					fmti_b2_nr = (fmti_matrix_5[0][1])*(fmti_matrix_5[0][1]);
// 					fmti_b2_nr += (fmti_matrix_5[1][1])*(fmti_matrix_5[1][1]);
// 					fmti_b2_nr += (fmti_f2)*(fmti_matrix_5[2][1])*(fmti_matrix_5[2][1]);
// 					
// 					fmti_b2_dr = (fmti_matrix_5[0][0])*(fmti_matrix_5[0][0]);
// 					fmti_b2_dr += (fmti_matrix_5[1][0])*(fmti_matrix_5[1][0]);
// 					fmti_b2_dr += (fmti_f2)*(fmti_matrix_5[2][0])*(fmti_matrix_5[2][0]);
// 					fmti_b2 = fmti_b2_nr/fmti_b2_dr;
// 					
// 					if ((fmti_b2<=0.5) || (fmti_b2>=2)) {
// 						continue;
// 					}

					//return (0);
					if (Evaluate_Model_Support()) {
						best_var1 = fmti_var1;
						best_var2 = fmti_var2;
						best_var3 = fmti_var3;
						best_var4 = fmti_var4;
						best_var5 = fmti_var5;
						best_var6 = fmti_var6;
						best_var7 = fmti_var7;
						best_var8 = fmti_var8;
// 	if (best_matching_score > 200) {
// 	printf ("Best matching score: %d\n", best_matching_score);
// 	printf ("Best matching count: %d\n", best_match_count);
// 	printf ("%10.6lf, %10.6lf, %10.6lf\n", best_calib_params[0][0], best_calib_params[0][1], best_calib_params[0][2]);
// 	printf ("%10.6lf, %10.6lf, %10.6lf\n", best_calib_params[1][0], best_calib_params[1][1], best_calib_params[1][2]);
// 	printf ("%10.6lf, %10.6lf, %10.6lf\n", best_calib_params[2][0], best_calib_params[2][1], best_calib_params[2][2]);
// 	printf ("best_var1: %d, best_var2: %d, best_var3: %d, best_var4: %d\n", best_var1, best_var2, best_var3, best_var4);
// 	printf ("best_var5: %d, best_var6: %d, best_var7: %d, best_var8: %d\n", best_var5, best_var6, best_var7, best_var8);
// 	}
					}
				}
			}
		}
	}
				}
			}
		}
	}

	printf ("Best matching score: %d\n", best_matching_score);
	printf ("Best matching count: %d\n", best_match_count);
	printf ("%10.6lf, %10.6lf, %10.6lf\n", best_calib_params[0][0], best_calib_params[0][1], best_calib_params[0][2]);
	printf ("%10.6lf, %10.6lf, %10.6lf\n", best_calib_params[1][0], best_calib_params[1][1], best_calib_params[1][2]);
	printf ("%10.6lf, %10.6lf, %10.6lf\n", best_calib_params[2][0], best_calib_params[2][1], best_calib_params[2][2]);
	printf ("best_var1: %d, best_var2: %d, best_var3: %d, best_var4: %d\n", best_var1, best_var2, best_var3, best_var4);
	printf ("best_var5: %d, best_var6: %d, best_var7: %d, best_var8: %d\n", best_var5, best_var6, best_var7, best_var8);

	return (0);
}




























