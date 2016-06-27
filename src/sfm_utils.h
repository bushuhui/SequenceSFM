/*******************************************************************************
 *
 * Sequence SfM for UAV images
 *
 * Author:
 *    Shuhui Bu (bushuhui@nwpu.edu.cn)
 *
 ******************************************************************************/


#ifndef __SFM_UTILS_H__
#define __SFM_UTILS_H__

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "utils.h"

#include "sfm_data.h"

using namespace std;
using namespace cv;


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int load_camera_parameters(const char *fname, Mat &cam_k, Mat &cam_d);


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
std::vector<cv::DMatch> FlipMatches(const std::vector<cv::DMatch>& matches);
void KeyPointsToPoints(const std::vector<cv::KeyPoint>& kps, std::vector<cv::Point2f>& ps);
void PointsToKeyPoints(const std::vector<cv::Point2f>& ps, std::vector<cv::KeyPoint>& kps);

void GetAlignedPointsFromMatch(const std::vector<cv::KeyPoint>& imgpts1,
                               const std::vector<cv::KeyPoint>& imgpts2,
                               const std::vector<cv::DMatch>& matches,
                               std::vector<cv::KeyPoint>& pt_set1,
                               std::vector<cv::KeyPoint>& pt_set2);

std::vector<cv::Point3d> CloudPointsToPoints(const std::vector<CloudPoint> cpts);


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int MatchFeatures_of(Mat &img_1, Mat &img_2,
                     vector<DMatch> &matches,
                     vector<KeyPoint> &pts_1, vector<KeyPoint> &pts_2,
                     vector<KeyPoint> &pts_n1, vector<KeyPoint> &pts_n2,
                     Mat &fm,
                     CParamArray *pa=NULL);

int MatchFeatures_SURF(Mat &img_1, Mat &img_2,
                     vector<DMatch> &matches,
                     vector<KeyPoint> &pts_1, vector<KeyPoint> &pts_2,
                     vector<KeyPoint> &pts_n1, vector<KeyPoint> &pts_n2,
                     Mat &fm,
                     CParamArray *pa=NULL);

int MatchFeatures_OpenSURF(Mat &img_1, Mat &img_2,
                     vector<DMatch> &matches,
                     vector<KeyPoint> &pts_1, vector<KeyPoint> &pts_2,
                     vector<KeyPoint> &pts_n1, vector<KeyPoint> &pts_n2,
                     Mat &fm,
                     CParamArray *pa=NULL);

int MatchFeatures_SURF_CUDA(Mat &img_1, Mat &img_2,
                     vector<DMatch> &matches,
                     vector<KeyPoint> &pts_1, vector<KeyPoint> &pts_2,
                     vector<KeyPoint> &pts_n1, vector<KeyPoint> &pts_n2,
                     Mat &fm,
                     CParamArray *pa=NULL);

int MatchFeatures_SIFT(Mat &img_1, Mat &img_2,
                     vector<DMatch> &matches,
                     vector<KeyPoint> &pts_1, vector<KeyPoint> &pts_2,
                     vector<KeyPoint> &pts_n1, vector<KeyPoint> &pts_n2,
                     Mat &fm,
                     CParamArray *pa=NULL);


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int FindHomographyInliers2Views_t(std::vector<cv::KeyPoint> &kpt1, std::vector<cv::KeyPoint> &kpt2,
                                  std::vector<cv::DMatch> &matches);


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int PointsPlaneFit(std::vector<CloudPoint> &cp, double *w, double &e);

int PointsPlaneFit_RANSAC(std::vector<CloudPoint> &cp,
                          cv::Vec4d &plane, cv::Matx34d &plane_se3,
                          std::vector<int> &cpFilter,
                          int nRansacs=100,
                          double kThreshold=2.0);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

cv::Vec3d LocalCoord2G(cv::Matx34d &mse3);
cv::Matx34d GlobalCoord2L(cv::Vec3d &p);

double CalcCameraDistance(cv::Matx34d &cam1, cv::Matx34d &cam2);
int CalcGround2CameraDis(cv::Matx34d &gp, vector<cv::Matx34d> &cam, vector<double> &d);

int CalcCameraGround_axes(cv::Matx34d &cam, cv::Vec4d &p, cv::Matx34d &p_se3,
                          cv::Matx34d &axes_se3);

int TransCloudPoint(cv::Matx34d &t_se3, vector<CloudPoint> &cp);
int TransCamera(cv::Matx34d &t_se3, cv::Matx34d &cam_se3);
int TransCamera(cv::Matx34d &t_se3, map<int, cv::Matx34d> &cams);


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int CameraGPS_Fitting(std::vector<cv::Matx34d> &cams, std::vector<cv::Vec3d> &gps,
                      cv::Matx23d &trans);

int CameraGPS_Fitting_lm(std::vector<cv::Matx34d> &cams, std::vector<cv::Vec3d> &gps,
                      cv::Matx34d &trans);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int FilterData_NormDist(vector<double> &d, vector<int> &d_out,
                        int hist_n=100, double n_sig=2,
                        double *d_mean = NULL, double *d_sig = NULL);

int FilterData_Median(vector<double> &d, vector<int> &d_out,
                      double &mean, double &sig,
                      double r1=1.0/8.0, double r2=7.0/8.0,
                      double n_sig=2.0);

int FilterData_FMe(vector<double> &d, vector<int> &d_out,
                   double &mean, double &sig,
                   double t=6.0);

int CloudPoint_FilterZ(vector<DMatch> &matches,   vector<DMatch> &matches_out,
                       vector<CloudPoint> &cloud, vector<CloudPoint> &cloud_out,
                       double &r_sigmean);


#endif // end of __SFM_UTILS_H__
