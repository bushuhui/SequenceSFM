/*******************************************************************************
 *
 * Sequence SfM for UAV images
 *
 * Author:
 *    Shuhui Bu (bushuhui@nwpu.edu.cn)
 *
 ******************************************************************************/

#ifndef __SFM_BA_H__
#define __SFM_BA_H__

#include <vector>
#include <map>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "sfm_data.h"

using namespace std;
using namespace cv;


////////////////////////////////////////////////////////////////////////////////
/// SSBA
////////////////////////////////////////////////////////////////////////////////

int adjustBundle_ssba(vector<CloudPoint> &pointcloud,
                        Mat &camIntrinsic,
                        map<int, vector<KeyPoint> > &imgpts,
                        map<int, Matx34d>  &Pmats);


////////////////////////////////////////////////////////////////////////////////
/// PBA
////////////////////////////////////////////////////////////////////////////////

int adjustBundle_pba(vector<CloudPoint> &pointcloud,
                        Mat &camIntrinsic,
                        Mat &camDistortion,
                        map<int, vector<KeyPoint> > &imgpts,
                        map<int, Matx34d>  &Pmats);


#endif // end of __SFM_BA_H__
