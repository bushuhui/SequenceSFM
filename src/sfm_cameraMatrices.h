/*******************************************************************************
 *
 * Sequence SfM for UAV images
 *
 * Author:
 *    Shuhui Bu (bushuhui@nwpu.edu.cn)
 *
 ******************************************************************************/


#ifndef __SFM_CAMERAMATRICES_H__
#define __SFM_CAMERAMATRICES_H__

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "sfm_data.h"

bool CheckCoherentRotation(cv::Mat_<double>& R);
bool TestTriangulation(const std::vector<CloudPoint>& pcloud, const cv::Matx34d& P, std::vector<uchar>& status);

cv::Mat GetFundamentalMat(	const std::vector<cv::KeyPoint>& imgpts1,
                            const std::vector<cv::KeyPoint>& imgpts2,
                            std::vector<cv::KeyPoint>& imgpts1_good,
                            std::vector<cv::KeyPoint>& imgpts2_good,
                            std::vector<cv::DMatch>& matches
                          );


bool FindCameraMatrices(const cv::Mat& K,
                        const cv::Mat& Kinv,
                        const cv::Mat& distcoeff,
                        const std::vector<cv::KeyPoint>& imgpts1,
                        const std::vector<cv::KeyPoint>& imgpts2,
                        std::vector<cv::KeyPoint>& imgpts1_good,
                        std::vector<cv::KeyPoint>& imgpts2_good,
                        cv::Matx34d& P,
                        cv::Matx34d& P1,
                        std::vector<cv::DMatch>& matches,
                        std::vector<CloudPoint>& outCloud
                        );


#endif // end of __SFM_CAMERAMATRICES_H__
