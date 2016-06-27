/*******************************************************************************
 *
 * Sequence SfM for UAV images
 *
 * Author:
 *    Shuhui Bu (bushuhui@nwpu.edu.cn)
 *
 ******************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include <vector>
#include <iostream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <opencv2/gpu/gpu.hpp>
#include <opencv2/gpu/gpumat.hpp>
#include <opencv2/nonfree/gpu.hpp>


#include <Eigen/Dense>

#include "klt.h"
#include "surflib.h"

#include "utils.h"
#include "utils_math.h"
#include "utils_gui.h"

#include "sfm_utils.h"
#include "sfm_cameraMatrices.h"


using namespace std;
using namespace cv;
using namespace cv::gpu;
using namespace cv_klt;


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int load_camera_parameters(const char *fname, Mat &cam_k, Mat &cam_d)
{
    FileStorage     fs;

    fs.open(fname, FileStorage::READ);
    if( !fs.isOpened() ) {
        dbg_pe("Can not open file: %s\n", fname);
        return -1;
    }

    fs["camera_matrix"]           >> cam_k;
    fs["distortion_coefficients"] >> cam_d;

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

std::vector<cv::DMatch> FlipMatches(const std::vector<cv::DMatch>& matches)
{
    std::vector<cv::DMatch> flip;

    for(int i=0;i<matches.size();i++) {
        flip.push_back(matches[i]);
        swap(flip.back().queryIdx, flip.back().trainIdx);
    }

    return flip;
}

std::vector<cv::Point3d> CloudPointsToPoints(const std::vector<CloudPoint> cpts)
{
    std::vector<cv::Point3d> out;

    for (unsigned int i=0; i<cpts.size(); i++) {
        out.push_back(cpts[i].pt);
    }

    return out;
}

void GetAlignedPointsFromMatch(const std::vector<cv::KeyPoint>& imgpts1,
                               const std::vector<cv::KeyPoint>& imgpts2,
                               const std::vector<cv::DMatch>& matches,
                               std::vector<cv::KeyPoint>& pt_set1,
                               std::vector<cv::KeyPoint>& pt_set2)
{
    for (unsigned int i=0; i<matches.size(); i++) {
        pt_set1.push_back(imgpts1[matches[i].queryIdx]);
        pt_set2.push_back(imgpts2[matches[i].trainIdx]);
    }
}

void KeyPointsToPoints(const vector<KeyPoint>& kps, vector<Point2f>& ps)
{
    ps.clear();

    for (unsigned int i=0; i<kps.size(); i++)
        ps.push_back(kps[i].pt);
}

void PointsToKeyPoints(const vector<Point2f>& ps, vector<KeyPoint>& kps)
{
    kps.clear();

    for (unsigned int i=0; i<ps.size(); i++)
        kps.push_back(KeyPoint(ps[i],1.0f));
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int MatchFeatures_of(Mat &img_1, Mat &img_2,
                     vector<DMatch> &matches,
                     vector<KeyPoint> &pts_1, vector<KeyPoint> &pts_2,
                     vector<KeyPoint> &pts_n1, vector<KeyPoint> &pts_n2,
                     Mat &fm,
                     CParamArray *pa)
{
    string                  fd_name = "PyramidFAST";

    Ptr<FeatureDetector>    detector;
    int                     SURF_minHessian = 400;

    // load parameters
    if( pa != NULL ) {
        pa->s("fd_name", fd_name);
        pa->i("SURF_minHessian", SURF_minHessian);
    }

    matches.clear();

    /////////////////////////////////////////////////
    /// create feature detector
    /////////////////////////////////////////////////
    if( fd_name == "SURF" ) {
        detector = new SurfFeatureDetector( SURF_minHessian);
    } else if ( fd_name == "FAST" ) {
        detector = new FastFeatureDetector();
    } else if ( fd_name == "PyramidFAST" )  {
        detector = FeatureDetector::create("PyramidFAST");
    }

    detector->detect(img_1, pts_1);
    detector->detect(img_2, pts_2);

    /////////////////////////////////////////////////
    /// perform optical flow
    /////////////////////////////////////////////////
    vector<Point2f> pts_p1, pts_p2;

    KeyPointsToPoints(pts_1, pts_p1);
    pts_p2.resize(pts_p1.size());

    vector<uchar> vstatus(pts_p1.size());
    vector<float> verror(pts_p1.size());

    calcOpticalFlowPyrLK(img_1, img_2, pts_p1, pts_p2, vstatus, verror);


    vector<Point2f> to_find;
    vector<int> to_find_back_idx;
    for (unsigned int i=0; i<vstatus.size(); i++) {
        //printf("[%5d] vstatus=%d, verror=%f, p1=(%f %f), p2=(%f %f)\n",
        //       i, vstatus[i], verror[i],
        //       pts_p1[i].x, pts_p1[i].y,
        //       pts_p2[i].x, pts_p2[i].y);

        if (vstatus[i] && verror[i] < 12.0) {
            to_find_back_idx.push_back(i);
            to_find.push_back(pts_p2[i]);
        } else {
            vstatus[i] = 0;
        }
    }

    set<int> found_in_imgpts_j;
    Mat to_find_flat = Mat(to_find).reshape(1, to_find.size());

    vector<Point2f> j_pts_to_find;
    KeyPointsToPoints(pts_2, j_pts_to_find);

    Mat j_pts_flat = Mat(j_pts_to_find).reshape(1,j_pts_to_find.size());

    vector<vector<DMatch> > knn_matches;
    BFMatcher matcher(CV_L2);
    matcher.radiusMatch(to_find_flat, j_pts_flat, knn_matches, 2.0f);

    for(int i=0;i<knn_matches.size();i++) {
        DMatch _m;

        if(knn_matches[i].size() == 1) {
            _m = knn_matches[i][0];
        } else if(knn_matches[i].size() > 1) {
            if(knn_matches[i][0].distance / knn_matches[i][1].distance < 0.7) {
                _m = knn_matches[i][0];
            } else {
                continue; // did not pass ratio test
            }
        } else {
            continue; // no match
        }

        // prevent duplicates
        if (found_in_imgpts_j.find(_m.trainIdx) == found_in_imgpts_j.end()) {
            //back to original indexing of points for <i_idx>
            _m.queryIdx = to_find_back_idx[_m.queryIdx];
            matches.push_back(_m);
            found_in_imgpts_j.insert(_m.trainIdx);
        }
    }

    // Filter by Fundamental Matrix
    fm = GetFundamentalMat(pts_1, pts_2,
                      pts_n1, pts_n2,
                      matches);
    cout << "F = " << fm << "\n\n";


    return 0;
}

int iFileNo = 0;


struct SURF_parallel_data {
    FeatureDetector             *detector;
    SurfDescriptorExtractor     *extractor;

    Mat                         *img;
    vector<KeyPoint>            *pts;
    Mat                         *des;

    void print(void) {
        printf("detector:  %x\n", detector);
        printf("extractor: %x\n", extractor);
        printf("img:       %x\n", img);
        printf("pts:       %x\n", pts);
        printf("des:       %x\n", des);
    }
};

void *SURF_parallel_func(void *dat)
{
    SURF_parallel_data *p = (SURF_parallel_data *) dat;

    p->print();

    p->detector->detect(*(p->img), *(p->pts));
    printf("1\n"); fflush(stdout);
    p->extractor->compute(*(p->img), *(p->pts), *(p->des));
    printf("2\n"); fflush(stdout);

    return NULL;
}

int MatchFeatures_SURF(Mat &img_1, Mat &img_2,
                     vector<DMatch> &matches,
                     vector<KeyPoint> &pts_1, vector<KeyPoint> &pts_2,
                     vector<KeyPoint> &pts_n1, vector<KeyPoint> &pts_n2,
                     Mat &fm,
                     CParamArray *pa)
{
    string                  fd_name = "SURF";
    double                  SURF_minHessian = 400;

    FeatureDetector         *detector, *detector2;
    SurfDescriptorExtractor *extractor, *extractor2;
    Mat                     descriptors_1, descriptors_2;

    SURF_parallel_data      spd;

    // load parameters
    if( pa != NULL ) {
        pa->s("fd_name", fd_name);
        pa->d("SURF_minHessian", SURF_minHessian);
    }


    /////////////////////////////////////////////////
    /// create feature detector
    /////////////////////////////////////////////////
    if( fd_name == "SURF" ) {
        detector = new SurfFeatureDetector( SURF_minHessian );
        detector2 = new SurfFeatureDetector( SURF_minHessian );
    } else if ( fd_name == "FAST" ) {
        detector = new FastFeatureDetector();    
        detector2 = new FastFeatureDetector();
    } else if ( fd_name == "PyramidFAST" )  {
        detector = FeatureDetector::create("PyramidFAST");
        detector2 = FeatureDetector::create("PyramidFAST");
    } else if ( fd_name == "SIFT" ) {
        detector = new SiftFeatureDetector;
        detector2 = new SiftFeatureDetector;
    }

    extractor  = new SurfDescriptorExtractor(48, 12, true);
    extractor2 = new SurfDescriptorExtractor(48, 12, true);

    spd.detector  = detector2;
    spd.extractor = extractor2;
    spd.img = &img_2;
    spd.pts = &pts_2;
    spd.des = &descriptors_2;

    double t = getTickCount(), t1, t2;

    //void* spd_handle = run_parallel_func(&spd, SURF_parallel_func);

    detector->detect(img_1, pts_1);
    extractor->compute(img_1, pts_1, descriptors_1);

    detector->detect(img_2, pts_2);
    extractor->compute(img_2, pts_2, descriptors_2);

    //run_parallel_join(spd_handle);

    delete detector;
    delete detector2;
    delete extractor;
    delete extractor2;

    t1 = ((double)getTickCount() - t)/getTickFrequency();

#if 1
    BFMatcher matcher(NORM_L2);
    matcher.match(descriptors_1, descriptors_2, matches);
#else
    FlannBasedMatcher matcher;
    matcher.match(descriptors_1, descriptors_2, matches);
#endif

    t2 = ((double)getTickCount() - t)/getTickFrequency() - t1;
    t = ((double)getTickCount() - t)/getTickFrequency();
    printf("Time = %f %f, totoal = %f\n", t1, t2, t);

    /////////////////////////////////////////////////
    /// perform min value filter
    /////////////////////////////////////////////////
    if( 0 ) {
        //-- Quick calculation of max and min distances between keypoints
        double max_dist = 0; double min_dist = 100;

        for( int i = 0; i < descriptors_1.rows; i++ )
        {
            double dist = matches[i].distance;
            if( dist < min_dist )
                min_dist = dist;
            if( dist > max_dist )
                max_dist = dist;
        }

        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );

        //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
        //-- PS.- radiusMatch can also be used here.
        vector< DMatch > good_matches;
        for( int i = 0; i < descriptors_1.rows; i++ )
        {
            if( matches[i].distance < 9*min_dist )
            {
                  good_matches.push_back( matches[i]);
            }
        }

        matches = good_matches;
    }

    /////////////////////////////////////////////////
    /// perform median filter
    /////////////////////////////////////////////////
    if( 0 ) {
        int             i;
        vector<double>  me;
        vector<int>     mf;
        double          m_m, m_s, m_t;

        me.resize(matches.size());

        for(i=0; i<matches.size(); i++)
            me[i] = matches[i].distance;

        m_t = 0.5;
        FilterData_Median(me, mf,
                          m_m, m_s,
                          1.0/8.0, 7.0/8.0,
                          m_t);

        vector< DMatch > good_matches;

        for(i=0; i<mf.size(); i++) {
            if( mf[i] == 1 )
                good_matches.push_back(matches[i]);
        }

        printf("m_m, m_s = %f %f\n", m_m, m_s);

        matches = good_matches;
    }


    /////////////////////////////////////////////////
    /// perform optical flow filter
    /////////////////////////////////////////////////
    if( 0 ) {
        vector<DMatch>  newMatch;

        int nMatch = matches.size();
        if( nMatch > 20 ) {
            vector<Point2f> pts_p1, pts_p2;
            int             i;
            double          d;

            pts_p1.resize(nMatch);
            pts_p2.resize(nMatch);

            for(i=0; i<nMatch; i++) {
                pts_p1[i] = pts_1[matches[i].queryIdx].pt;
            }

            vector<uchar> vstatus(nMatch);
            vector<float> verror(nMatch);

            calcOpticalFlowPyrLK(img_1, img_2,
                                 pts_p1, pts_p2,
                                 vstatus, verror);

            for(i=0; i<nMatch; i++) {
                pts_p1[i] = pts_2[matches[i].trainIdx].pt;
            }

            for(i=0; i<nMatch; i++) {
                //if (vstatus[i] && verror[i] < 12.0) {
                if ( vstatus[i] ) {
                    d = sqrt( sqr(pts_p1[i].x-pts_p2[i].x) +
                              sqr(pts_p1[i].y-pts_p2[i].y) );
                    if( d < 10 ) {
                        newMatch.push_back(matches[i]);
                    }
                }
            }

            matches = newMatch;
        }
    }

    /////////////////////////////////////////////////
    /// filter wrong correspondences by histogram
    /////////////////////////////////////////////////
    if( 1 ) {
        int     i;
        Point2f p1, p2;
        double  d;
        FILE    *fp;
        char    fn[200];
        int     nMatch = matches.size();

        vector<double>  arr_d;
        vector<int>     arr_f;


        //sprintf(fn, "dist_%04d.txt", iFileNo++);
        //fp = fopen(fn, "wt");

        arr_d.resize(nMatch);
        for(i=0; i<nMatch; i++) {
            p1 = pts_1[matches[i].queryIdx].pt;
            p2 = pts_2[matches[i].trainIdx].pt;

            d = sqrt( sqr(p1.x-p2.x) + sqr(p1.y-p2.y) );
            arr_d[i] = d;

            //fprintf(fp, "%g\n", d);
        }
        //fclose(fp);

        // filter data
        vector<DMatch>  newMatch;
        double          fd_mean, fd_sig;

        FilterData_NormDist(arr_d, arr_f, 100, 0.7);
        //FilterData_Median(arr_d, arr_f, fd_mean, fd_sig, 1.0/8.0, 7.0/8.0, 0.7);


        for(i=0; i<nMatch; i++) {
            if( arr_f[i] == 1 ) {
                newMatch.push_back(matches[i]);
            }
        }

        //printf("fd_mean, sig = %9f %9f, %5d/%5d = %9f\n", fd_mean, fd_sig,
        //       newMatch.size(), matches.size(), 1.0*newMatch.size()/matches.size());

        matches = newMatch;
    }

    /////////////////////////////////////////////////
    /// filter wrong correspondences by fundamental matrix
    /////////////////////////////////////////////////
    if( 0 ) {
        fm = GetFundamentalMat(pts_1, pts_2,
                          pts_n1, pts_n2,
                          matches);
        cout << "F = " << fm << "\n\n";
    }

    return 0;
}


int MatchFeatures_OpenSURF(Mat &img_1, Mat &img_2,
                     vector<DMatch> &matches,
                     vector<KeyPoint> &pts_1, vector<KeyPoint> &pts_2,
                     vector<KeyPoint> &pts_n1, vector<KeyPoint> &pts_n2,
                     Mat &fm,
                     CParamArray *pa)
{
    IplImage    i1, i2;
    IpVec       ipts1, ipts2;


    double t = getTickCount(), t1, t2;

    // convert image
    i1 = IplImage(img_1);
    i2 = IplImage(img_2);

    // detect keypoints & descriptors
    surfDetDes(&i1, ipts1, false, 12, 48, 2, 0.0001f);
    surfDetDes(&i2, ipts2, false, 12, 48, 2, 0.0001f);

    t1 = ((double)getTickCount() - t)/getTickFrequency();

    // bruteforce match
    getMatches(ipts1, ipts2, matches);

    t2 = ((double)getTickCount() - t)/getTickFrequency() - t1;

    // convert keypoints
    convIpoint2KeyPoint(ipts1, pts_1);
    convIpoint2KeyPoint(ipts2, pts_2);

    t = ((double)getTickCount() - t)/getTickFrequency();
    printf("Time = %f %f, totoal = %f\n", t1, t2, t);


    /////////////////////////////////////////////////
    /// filter wrong correspondences by histogram
    /////////////////////////////////////////////////
    if( 1 ) {
        int     i;
        Point2f p1, p2;
        double  d;
        FILE    *fp;
        char    fn[200];
        int     nMatch = matches.size();

        vector<double>  arr_d;
        vector<int>     arr_f;


        //sprintf(fn, "dist_%04d.txt", iFileNo++);
        //fp = fopen(fn, "wt");

        arr_d.resize(nMatch);
        for(i=0; i<nMatch; i++) {
            p1 = pts_1[matches[i].queryIdx].pt;
            p2 = pts_2[matches[i].trainIdx].pt;

            d = sqrt( sqr(p1.x-p2.x) + sqr(p1.y-p2.y) );
            arr_d[i] = d;

            //fprintf(fp, "%g\n", d);
        }
        //fclose(fp);

        // filter data
        vector<DMatch>  newMatch;

        FilterData_NormDist(arr_d, arr_f, 100, 0.7);

        for(i=0; i<nMatch; i++) {
            if( arr_f[i] == 1 ) {
                newMatch.push_back(matches[i]);
            }
        }

        matches = newMatch;
    }

    return 0;
}


int MatchFeatures_SURF_CUDA(Mat &img_1, Mat &img_2,
                     vector<DMatch> &matches,
                     vector<KeyPoint> &pts_1, vector<KeyPoint> &pts_2,
                     vector<KeyPoint> &pts_n1, vector<KeyPoint> &pts_n2,
                     Mat &fm,
                     CParamArray *pa)
{
    string                  fd_name = "SURF";
    int                     SURF_minHessian = 400;

    // load parameters
    if( pa != NULL ) {
        pa->s("fd_name", fd_name);
        pa->i("SURF_minHessian", SURF_minHessian);
    }

    double t = getTickCount(), t1, t2;

    GpuMat  img_1c, img_2c;
    GpuMat  des1, des2;

    Mat     mask1_(img_1), mask2_(img_2);
    GpuMat  mask1, mask2;

    mask1_ = 1;
    mask2_ = 1;
    mask1.upload(mask1_);
    mask2.upload(mask2_);

    img_1c.upload(img_1);
    img_2c.upload(img_2);


    SURF_GPU    surf(SURF_minHessian);

    surf(img_1c, GpuMat(), pts_1, des1);
    surf(img_2c, GpuMat(), pts_2, des2);

    t1 = ((double)getTickCount() - t)/getTickFrequency();

    BruteForceMatcher_GPU_base matcher;
    matcher.match(des1, des2, matches);

    t2 = ((double)getTickCount() - t)/getTickFrequency() - t1;
    t = ((double)getTickCount() - t)/getTickFrequency();
    printf("Time = %f %f, totoal = %f\n", t1, t2, t);


    /////////////////////////////////////////////////
    /// filter wrong correspondences by histogram
    /////////////////////////////////////////////////
    if( 1 ) {
        int     i;
        Point2f p1, p2;
        double  d;
        FILE    *fp;
        char    fn[200];
        int     nMatch = matches.size();

        vector<double>  arr_d;
        vector<int>     arr_f;


        //sprintf(fn, "dist_%04d.txt", iFileNo++);
        //fp = fopen(fn, "wt");

        arr_d.resize(nMatch);
        for(i=0; i<nMatch; i++) {
            p1 = pts_1[matches[i].queryIdx].pt;
            p2 = pts_2[matches[i].trainIdx].pt;

            d = sqrt( sqr(p1.x-p2.x) + sqr(p1.y-p2.y) );
            arr_d[i] = d;

            //fprintf(fp, "%g\n", d);
        }
        //fclose(fp);

        // filter data
        vector<DMatch>  newMatch;

        FilterData_NormDist(arr_d, arr_f, 100, 0.7);

        for(i=0; i<nMatch; i++) {
            if( arr_f[i] == 1 ) {
                newMatch.push_back(matches[i]);
            }
        }

        matches = newMatch;
    }

    return 0;
}

int MatchFeatures_SIFT(Mat &img_1, Mat &img_2,
                     vector<DMatch> &matches,
                     vector<KeyPoint> &pts_1, vector<KeyPoint> &pts_2,
                     vector<KeyPoint> &pts_n1, vector<KeyPoint> &pts_n2,
                     Mat &fm,
                     CParamArray *pa)
{
    Ptr<FeatureDetector>    detector;
    SiftDescriptorExtractor extractor;

    Mat                     descriptors_1, descriptors_2;


    detector = new SiftFeatureDetector;

    detector->detect(img_1, pts_1);
    detector->detect(img_2, pts_2);

    extractor.compute(img_1, pts_1, descriptors_1);
    extractor.compute(img_2, pts_2, descriptors_2);


    BFMatcher matcher(NORM_L2);
    matcher.match(descriptors_1, descriptors_2, matches);

#if 0
    fm = GetFundamentalMat(pts_1, pts_2,
                      pts_n1, pts_n2,
                      matches);
    cout << "F = " << fm << "\n\n";
#endif

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int FindHomographyInliers2Views_t(std::vector<cv::KeyPoint> &kpt1,
                                  std::vector<cv::KeyPoint> &kpt2,
                                  std::vector<cv::DMatch> &matches)
{
    vector<cv::KeyPoint> ikpts,jkpts;
    vector<cv::Point2f>  ipts,jpts;

    GetAlignedPointsFromMatch(kpt1, kpt2, matches, ikpts,jkpts);
    KeyPointsToPoints(ikpts,ipts); KeyPointsToPoints(jkpts,jpts);

    //TODO flatten point2d?? or it takes max of width and height
    double minVal,maxVal;
    cv::minMaxIdx(ipts,&minVal,&maxVal);

    //threshold from Snavely07
    vector<uchar> status;
    cv::Mat H = cv::findHomography(ipts,jpts,status,CV_RANSAC, 0.004 * maxVal);
    return cv::countNonZero(status); //number of inliers
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int PointsPlaneFit(vector<CloudPoint> &cp, double *w, double &e)
{
    Matx33d         A;
    Matx31d         b, x;

    double          ptx, pty, ptz;
    double          c_xx, c_yy, c_xy, c_zx, c_zy,
                    c_x, c_y, c_z;

    vector<CloudPoint>::iterator        it;

    c_xx = 0;
    c_yy = 0;
    c_xy = 0;
    c_zx = 0;
    c_zy = 0;
    c_x  = 0;
    c_y  = 0;
    c_z  = 0;

    for(it=cp.begin(); it!=cp.end(); it++) {
        ptx = it->pt.x;
        pty = it->pt.y;
        ptz = it->pt.z;

        c_xx += ptx*ptx;
        c_yy += pty*pty;
        c_xy += ptx*pty;
        c_zx += ptz*ptx;
        c_zy += ptz*pty;
        c_x  += ptx;
        c_y  += pty;
        c_z  += ptz;
    }

    A(0, 0) = c_xx; A(0, 1) = c_xy; A(0, 2) = c_x;
    A(1, 0) = c_xy; A(1, 1) = c_yy; A(1, 2) = c_y;
    A(2, 0) = c_x;  A(2, 1) = c_y;  A(2, 2) = cp.size();
    b(0, 0) = c_zx;
    b(1, 0) = c_zy;
    b(2, 0) = c_z;

    cv::solve(A, b, x);

    w[0] = x(0, 0);
    w[1] = x(1, 0);
    w[2] = x(2, 0);

    e = 0;
    for(it=cp.begin(); it!=cp.end(); it++) {
        ptx = it->pt.x;
        pty = it->pt.y;
        ptz = it->pt.z;

        e += fabs(ptz - w[0]*ptx - w[1]*pty - w[2]);
    }

    return 0;
}


inline double FindSigmaSquared(std::vector<double> &vdErrorSquared)
{
    double dSigmaSquared;

    std::sort(vdErrorSquared.begin(), vdErrorSquared.end());

    double dMedianSquared = vdErrorSquared[vdErrorSquared.size() / 2];
    double dSigma = 1.4826 * (1 + 5.0 / (vdErrorSquared.size() * 2 - 6))
                    * sqrt(dMedianSquared);
    dSigma =  4.6851 * dSigma;
    dSigmaSquared = dSigma * dSigma;

    return dSigmaSquared;
}

int PointsPlaneFit_RANSAC(std::vector<CloudPoint> &cp,
                          cv::Vec4d &plane, cv::Matx34d &plane_se3,
                          std::vector<int> &cpFilter,
                          int nRansacs,
                          double kThreshold)
{
    using namespace Eigen;

    unsigned int nPoints = cp.size();
    std::vector<Vector3d>   pts;

    for(int i=0; i<nPoints; i++) {
        Vector3d p;
        p(0) = cp[i].pt.x;
        p(1) = cp[i].pt.y;
        p(2) = cp[i].pt.z;
        pts.push_back(p);
    }

    if(nPoints < 10) {
        printf("ERR: updateRansac: too few points (%d) to calc plane.\n", nPoints);
        return -1;
    }

    cpFilter.resize(cp.size(), 0);

    Vector3d v3BestMean;
    Vector3d v3BestNormal;
    double dBestDistSquared = 9e99;
    double dSigmaSquared_best;

    for(int i=0; i<nRansacs; i++) {
        int nA = rand() % nPoints;
        int nB = nA;
        int nC = nA;

        while(nB == nA)
            nB = rand() % nPoints;
        while(nC == nA || nC==nB)
            nC = rand() % nPoints;

        Vector3d v3Mean = ( pts[nA] + pts[nB] + pts[nC] ) / 3.0;

        Vector3d v3CA = pts[nC] - pts[nA];
        Vector3d v3BA = pts[nB] - pts[nA];
        Vector3d v3Normal = v3CA.cross(v3BA);

        if( fabs(v3Normal.dot(v3Normal)) < 1e-9 ) continue;
        v3Normal = v3Normal.normalized();


        double dSumError = 0.0;
        vector<double> err;

        err.reserve(nPoints);
        for(unsigned int i=0; i<nPoints; i++) {
            Vector3d v3Diff = pts[i] - v3Mean;

            double dNormDist = fabs(v3Diff.dot(v3Normal));

            err.push_back(dNormDist);
        }

        double dSigmaSquared = FindSigmaSquared(err)/20.0;

        for(unsigned int i=0; i<nPoints; i++) {
            double dNormDist;

            if( err[i] > dSigmaSquared )
                dNormDist = dSigmaSquared;
            else
                dNormDist = err[i];

            dSumError += dNormDist;
        }

        if( dSumError < dBestDistSquared ) {
            dBestDistSquared = dSumError;
            v3BestMean   = v3Mean;
            v3BestNormal = v3Normal;
            dSigmaSquared_best = dSigmaSquared;
        }
    }

    //dInliner_T = dBestDistSquared / nPoints;
    double dInliner_T = dSigmaSquared_best;
    double dFlt_T = dSigmaSquared_best*kThreshold;

    // Done the ransacs, now collect the supposed inlier set
    vector<Vector3d> vv3Inliers;
    for(unsigned int i=0; i<nPoints; i++) {
        Vector3d v3Diff = pts[i] - v3BestMean;

        //double dDistSq = v3Diff * v3Diff;
        //if( dDistSq == 0.0 ) continue;

        double dNormDist = fabs(v3Diff.dot(v3BestNormal));

        if(dNormDist < dInliner_T)
            vv3Inliers.push_back(pts[i]);

        if( dNormDist < dFlt_T )
            cpFilter[i] = 1;
    }

    printf("Inliners = %d / %d\n", vv3Inliers.size(), nPoints);

    // With these inliers, calculate mean and cov
    Vector3d v3MeanOfInliers = Vector3d::Constant(0.0);

    for(unsigned int i=0; i<vv3Inliers.size(); i++)
        v3MeanOfInliers += vv3Inliers[i];
    v3MeanOfInliers = v3MeanOfInliers / vv3Inliers.size();

    Matrix3d m3Cov = Matrix3d::Constant(0.0);
    for(unsigned int i=0; i<vv3Inliers.size(); i++) {
        Vector3d v3Diff = vv3Inliers[i] - v3MeanOfInliers;
        m3Cov += v3Diff * v3Diff.transpose();
    }

    // Find the principal component with the minimal variance: this is the plane normal
    MatrixXf m3Covf(3, 3);

    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++)
            m3Covf(i, j) = m3Cov(i, j);
    }

    JacobiSVD<MatrixXf> svd(m3Covf, ComputeThinU | ComputeThinV);

    Vector3f    v1, v2, v3;
    Vector3d    ax, ay, az;
    MatrixXf    u, v;

    u = svd.matrixU();
    v = svd.matrixV();

    v1 =  u.col(0).normalized();
    v2 =  u.col(1).normalized();
    v3 =  u.col(2).normalized();

    ax << v1(0), v1(1), v1(2);
    az << v3(0), v3(1), v3(2);

    ax(2) = (-az(0)*ax(0) - az(1)*ax(1)) / az(2);
    ax = ax.normalized();

    ay = az.cross(ax);
    ay = ay.normalized();

    // Use the version of the normal which points towards the cam center
    //if(v3Normal[2] < 0) v3Normal *= -1.0;
    Vector3d v3Normal, v3Mean;
    v3Normal << az(0), az(1), az(2);
    double Length2Origin = v3MeanOfInliers.dot(v3Normal);

    plane[0] = v3Normal[0];
    plane[1] = v3Normal[1];
    plane[2] = v3Normal[2];
    plane[3] = -Length2Origin;

    v3Mean << v3MeanOfInliers(0), v3MeanOfInliers(1), v3MeanOfInliers(2);

    MatrixXd    R(3, 3), t(3, 1), t1(3, 1);

    R <<    ax(0), ax(1), ax(2),
            ay(0), ay(1), ay(2),
            az(0), az(1), az(2);
    t1 << v3Mean(0), v3Mean(1), v3Mean(2);

    t = -R * t1;

    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++)
            plane_se3(j, i) = R(j, i);

        plane_se3(i, 3) = t(i);
    }

    return 0;
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

cv::Vec3d LocalCoord2G(cv::Matx34d &mse3)
{
    Matx33d R;
    Vec3d   t, tg;

    int     i, j;

    for(j=0; j<3; j++) {
        for(i=0; i<3; i++) {
            R(j, i) = mse3(j, i);
        }

        t(j) = mse3(j, 3);
    }

    tg = -R.t() * t;
    return tg;
}

cv::Matx34d GlobalCoord2L(cv::Vec3d &p)
{
    Matx34d mse3;
    Matx33d R(1, 0, 0, 0, 1, 0, 0, 0, 1);
    Vec3d   t;
    int     i, j;

    t = -p;

    for(j=0; j<3; j++) {
        for(i=0; i<3; i++)
            mse3(j, i) = R(j, i);

        mse3(j, 3) = t(j);
    }

    return mse3;
}

double CalcCameraDistance(cv::Matx34d &cam1, cv::Matx34d &cam2)
{
    Vec3d   c1, c2, vd;

    c1 = LocalCoord2G(cam1);
    c2 = LocalCoord2G(cam2);
    vd = c1 - c2;

    return sqrt(vd(0)*vd(0) + vd(1)*vd(1) + vd(2)*vd(2));
}

int CalcGround2CameraDis(cv::Matx34d &gp, vector<cv::Matx34d> &cam, vector<double> &d)
{
    Vec3d   gc, gn, cc, v;
    double  _d;

    // gound plane center & norm
    gc = LocalCoord2G(gp);
    gn(0) = gp(2, 0); gn(1) = gp(2, 1); gn(2) = gp(2, 2);

    // for each camera
    d.reserve(cam.size());

    for(int i=0; i<cam.size(); i++) {
        cc = LocalCoord2G(cam[i]);
        v = gc - cc;
        _d = fabs(v.dot(gn));
        d.push_back(_d);
    }

    return 0;
}

double pointInPlane(cv::Vec3d &pt, cv::Vec4d &p)
{
    return fabs(p(0)*pt(0) + p(1)*pt(1) + p(2)*pt(2) + p(3));
}

Vec3d point2Plane(cv::Vec4d &p, cv::Vec3d &pt)
{
    Vec3d   p_norm, p_int;
    double  d, r1, r2;

    p_norm(0) = p(0);
    p_norm(1) = p(1);
    p_norm(2) = p(2);

    // camera to ground distance
    d = (p(0)*pt(0) + p(1)*pt(1) + p(2)*pt(2) + p(3)) /
            sqrt(sqr(p(0)) + sqr(p(1)) + sqr(p(2)));

    // camera project to ground
    p_int = pt - d*p_norm;
    r1 = pointInPlane(p_int, p);
    p_int = pt + d*p_norm;
    r2 = pointInPlane(p_int, p);

    if( r1 < r2 ) {
        p_int = pt - d*p_norm;
    }

    p_int(1) = (-p(0)*p_int(0) - p(2)*p_int(2) - p(3))/p(1);

    return p_int;
}

int CalcCameraGround_axes(cv::Matx34d &cam, cv::Vec4d &p, cv::Matx34d &p_se3,
                          cv::Matx34d &axes_se3)
{
    Vec3d   cam_p, p_mean, p_norm, cam_ax, cam_pax;
    Vec3d   p_int, p_ax;

    Vec3d   n_ax, n_ay, n_az;

    cam_p  = LocalCoord2G(cam);
    p_mean = LocalCoord2G(p_se3);
    p_norm(0) = p(0);
    p_norm(1) = p(1);
    p_norm(2) = p(2);

    // camera project to ground
    p_int = point2Plane(p, cam_p);

    // camera axis-x
    cam_ax(0) = cam(0, 0);
    cam_ax(1) = cam(0, 1);
    cam_ax(2) = cam(0, 2);
    cam_pax = cam_p + cam_ax;

    p_ax = point2Plane(p, cam_pax) - p_int;

    // new axis
    n_ax = p_ax/sqrt(p_ax.dot(p_ax));
    n_az = p_norm/sqrt(p_norm.dot(p_norm));
    n_ay = n_az.cross(n_ax);

    // calc t = -R*t1
    Matx33d     R;
    Vec3d       t;

    R = Matx33d(n_ax(0), n_ax(1), n_ax(2),
                n_ay(0), n_ay(1), n_ay(2),
                n_az(0), n_az(1), n_az(2));

    t = -R * p_int;

    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++)
            axes_se3(j, i) = R(j, i);

        axes_se3(i, 3) = t(i);
    }

    return 0;
}

int TransCloudPoint(cv::Matx34d &t_se3, vector<CloudPoint> &cp)
{
    using namespace Eigen;

    int         i, j;
    Vector4d    p1, p2;
    Matrix4d    t;

    for(j=0; j<3; j++) {
        for(i=0; i<4; i++)
            t(j, i) = t_se3(j, i);
    }
    t(3, 0) = 0; t(3, 1) = 0; t(3, 2) = 0; t(3, 3) = 1;

    p1(3) = 1.0;

    for(int i=0; i<cp.size(); i++) {
        CloudPoint &p = cp[i];

        p1(0) = p.pt.x;
        p1(1) = p.pt.y;
        p1(2) = p.pt.z;

        p2 = t * p1;

        p.pt.x = p2(0);
        p.pt.y = p2(1);
        p.pt.z = p2(2);
    }

    return 0;
}

int TransCamera(cv::Matx34d &t_se3, cv::Matx34d &cam_se3)
{
    using namespace Eigen;

    Matrix3d    Rc, RtI, Rc_new;
    Vector3d    tc, tt, ttI, tc_new;

    int         i, j;

    for(j=0; j<3; j++) {
        for(i=0; i<3; i++) {
            RtI(i, j) = t_se3(j, i);
            Rc(j, i)  = cam_se3(j, i);
        }

        tt(j) = t_se3(j, 3);
        tc(j) = cam_se3(j, 3);
    }
    ttI = -RtI*tt;

    Rc_new = Rc * RtI;
    tc_new = Rc * ttI + tc;

    for(j=0; j<3; j++) {
        for(i=0; i<3; i++) {
            cam_se3(j, i) = Rc_new(j, i);
        }

        cam_se3(j, 3) = tc_new(j);
    }

    return 0;
}

int TransCamera(cv::Matx34d &t_se3, map<int, cv::Matx34d> &cams)
{
    using namespace Eigen;

    Matrix3d    Rc, RtI, Rc_new;
    Vector3d    tc, tt, ttI, tc_new;
    int         i, j;

    map<int, cv::Matx34d>::iterator itc;

    // get inv(Ht)
    for(j=0; j<3; j++) {
        for(i=0; i<3; i++) {
            RtI(i, j) = t_se3(j, i);
        }

        tt(j) = t_se3(j, 3);
    }
    ttI = -RtI*tt;

    // for each camera matrix
    for(itc=cams.begin(); itc!=cams.end(); itc++) {
        Matx34d &c = itc->second;

        for(j=0; j<3; j++) {
            for(i=0; i<3; i++)
                Rc(j, i) = c(j, i);

            tc(j) = c(j, 3);
        }

        Rc_new = Rc * RtI;
        tc_new = Rc * ttI + tc;

        for(j=0; j<3; j++) {
            for(i=0; i<3; i++) {
                c(j, i) = Rc_new(j, i);
            }

            c(j, 3) = tc_new(j);
        }
    }

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int CameraGPS_Fitting(std::vector<cv::Matx34d> &cams, std::vector<cv::Vec3d> &gps,
                      cv::Matx34d &trans)
{
    using namespace Eigen;

    vector<Vec3d>   cam_p;
    int             i, N;

    N = cams.size();

    // convert camera's position
    for(i=0; i<cams.size(); i++) {
        Vec3d p = LocalCoord2G(cams[i]);
        cam_p.push_back(p);
    }

    // gnerate matrix A
    MatrixXf A, b, x;
    double   cx, cy;

    A.resize(2*N, 6);
    b.resize(2*N, 1);
    x.resize(6, 1);

    for(i=0; i<N; i++) {
        cx = cam_p[i](0);
        cy = cam_p[i](1);

        A(i*2+0, 0) = cx; A(i*2+0, 1) = cy; A(i*2+0, 2) = 1;
        A(i*2+0, 3) = 0;  A(i*2+0, 4) = 0;  A(i*2+0, 5) = 0;

        A(i*2+1, 0) = 0;  A(i*2+1, 1) = 0;  A(i*2+1, 2) = 0;
        A(i*2+1, 3) = cx; A(i*2+1, 4) = cy; A(i*2+1, 5) = 1;

        b(i*2+0) = gps[i](0);
        b(i*2+1) = gps[i](1);
    }

    // SVD based least-squares
    x = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

    // determin scale
    double  s1, s2, s;

    s1 = sqrt(sqr(x(0)) + sqr(x(1)));
    s2 = sqrt(sqr(x(3)) + sqr(x(4)));
    s  = (s1 + s2)/2.0;
    printf("CameraGPS_Fitting:: s1 = %f, s2 = %f, s = %f\n", s1, s2, s);

    // convert to Mat se(3)
    trans = Matx34d(x(0), x(1), 0,  x(2),
                    x(3), x(4), 0,  x(5),
                    0,    0,    s,  0);

    return 0;
}

int CameraGPS_Fitting_lm(std::vector<cv::Matx34d> &cams, std::vector<cv::Vec3d> &gps,
                      cv::Matx34d &trans)
{
    using namespace Eigen;

    int             i, j, it;
    int             Ndata, Nparams;

    MatrixXd        cam_p, gps_p;
    MatrixXd        x_est, x_lm;
    MatrixXd        y_est, y_est_lm;
    MatrixXd        d, d_lm, dp;
    MatrixXd        J, H, H_lm, H_I;

    double          e, e_lm, e_lm_old;

    double          s, t, tx, ty, xc, yc;

    int             n_iters = 100;
    double          e_delta = 1e-3;
    double          lambda  = 0.01;
    int             updateJ = 1;


    // data & parameter size
    Ndata   = cams.size();
    Nparams = 4;

    // alloc matrix
    cam_p.resize(2, Ndata);
    gps_p.resize(2, Ndata);

    d.resize(2*Ndata, 1);
    d_lm.resize(2*Ndata, 1);
    dp.resize(Nparams, 1);

    J.resize(2*Ndata, Nparams);
    H.resize(Nparams, Nparams);
    H_lm.resize(Nparams, Nparams);
    H_I.resize(Nparams, Nparams);
    for(i=0; i<Nparams; i++) {
        for(j=0; j<Nparams; j++) {
            if( i==j ) H_I(i, j) = 1.0;
            else       H_I(i, j) = 0.0;
        }
    }

    x_est.resize(Nparams, 1);
    x_lm.resize(Nparams, 1);

    y_est.resize(2*Ndata, 1);
    y_est_lm.resize(2*Ndata, 1);


    // set initial gauss
    x_est << 1.0, 0, 0, 0;

    // convert camera & GPS positions
    for(i=0; i<cams.size(); i++) {
        Vec3d p = LocalCoord2G(cams[i]);

        cam_p(0, i) = p(0);
        cam_p(1, i) = p(1);

        gps_p(0, i) = gps[i](0);
        gps_p(1, i) = gps[i](1);
    }

    // begin iteration
    for(it=0, e_lm_old = 9e99; it<n_iters; it++) {
        if( updateJ == 1 ) {
            for(i=0; i<Ndata; i++) {
                s  = x_est(0);
                t  = x_est(1);
                xc = cam_p(0, i);
                yc = cam_p(1, i);

                J(i*2+0, 0) = xc*cos(t) - yc*sin(t);
                J(i*2+0, 1) = -s*xc*sin(t) - s*yc*cos(t);
                J(i*2+0, 2) = 1.0;
                J(i*2+0, 3) = 0.0;

                J(i*2+1, 0) = xc*sin(t) + yc*cos(t);
                J(i*2+1, 1) = s*xc*cos(t) - s*yc*sin(t);
                J(i*2+1, 2) = 0.0;
                J(i*2+1, 3) = 1.0;
            }

            // evaluate y by current x_est
            s  = x_est(0);
            t  = x_est(1);
            tx = x_est(2);
            ty = x_est(3);
            for(i=0; i<Ndata; i++) {
                xc = cam_p(0, i);
                yc = cam_p(1, i);

                y_est(i*2+0) = s*xc*cos(t) - s*yc*sin(t) + tx;
                y_est(i*2+1) = s*xc*sin(t) + s*yc*cos(t) + ty;

                d(i*2+0) = gps_p(0, i) - y_est(i*2+0);
                d(i*2+1) = gps_p(1, i) - y_est(i*2+1);
            }

            // calculate Hessian matrix
            H = J.transpose() * J;

            // calcualte error
            if( it == 0 ) {
                for(i=0, e = 0.0; i<Ndata; i++)
                    e += sqr(d(i*2+0)) + sqr(d(i*2+1));
            }
        }

        // calculate H_lm
        H_lm = H + lambda*H_I;

        // calculate step size
        dp = H_lm.inverse() * (J.transpose() * d);
        x_lm = x_est + dp;

        // calculate new error and resdiual
        s  = x_lm(0);
        t  = x_lm(1);
        tx = x_lm(2);
        ty = x_lm(3);

        for(i=0, e_lm=0.0; i<Ndata; i++) {
            xc = cam_p(0, i);
            yc = cam_p(1, i);

            y_est_lm(i*2+0) = s*xc*cos(t) - s*yc*sin(t) + tx;
            y_est_lm(i*2+1) = s*xc*sin(t) + s*yc*cos(t) + ty;

            d_lm(i*2+0) = gps_p(0, i) - y_est_lm(i*2+0);
            d_lm(i*2+1) = gps_p(1, i) - y_est_lm(i*2+1);

            e_lm += sqr(d_lm(i*2+0)) + sqr(d_lm(i*2+1));
        }

        printf("[%4d] e, e_lm = %12g, %12g, lambda = %12g\n",
               it, e, e_lm, lambda);

        // decide update by error
        if( e_lm < e ) {
            lambda = lambda/10.0;
            x_est = x_lm;
            e = e_lm;
            updateJ = 1;
        } else {
            updateJ = 0;
            lambda = lambda*10;
        }

        // check exit iteration
        if( fabs(e_lm - e_lm_old) < e_delta )
            break;

        e_lm_old = e_lm;
    }

    // output transformation matrix
    s  = x_est(0);
    t  = x_est(1);
    tx = x_est(2);
    ty = x_est(3);

    printf("s = %12g, t = %12g, tx, ty = %12g, %12g\n", s, t, tx, ty);

    // convert to Mat se(3)
    trans = Matx34d(s*cos(t), -s*sin(t), 0, tx,
                    s*sin(t),  s*cos(t), 0, ty,
                    0,         0,        s, 0);

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int FilterData_NormDist(vector<double> &d, vector<int> &d_out,
                        int hist_n, double n_sig,
                        double *d_mean, double *d_sig)
{
    int     n, nf;
    int     i;
    double  x, sd, x_sum, t;

    double  d_min, d_max, v, v_d;
    int     *hist, hi, hi_max;

    n = d.size();

    // find d_min, d_max
    d_min =  1e99;
    d_max = -1e99;
    for(i=0; i<n; i++) {
        v = d[i];
        if( v > d_max ) d_max = v;
        if( v < d_min ) d_min = v;
    }
    v_d = (d_max - d_min) / (hist_n-1);

    // generate histogram
    hist = new int[hist_n];
    for(i=0; i<hist_n; i++) hist[i] = 0;

    for(i=0; i<n; i++) {
        v = d[i];
        hi = (int)( (v-d_min) / v_d );

        hist[hi] ++;
    }

    // find center x
    hi = 0;
    hi_max = 0;
    for(i=0; i<hist_n; i++) {
        if( hist[i] > hi_max ) {
            hi_max = hist[i];
            hi = i;
        }
    }

    // get mean & std
    x = d_min + (hi+0.5)*v_d;
    x_sum = 0;

    for(i=0; i<n; i++) {
        v = d[i];
        x_sum += sqr(v - x);
    }
    sd = sqrt(x_sum / n);

    // threshold
    t = x + n_sig*sd;
    //t = x + n_sig * x;

    // filter data
    d_out.resize(n, 0);
    nf = 0;

    for(i=0; i<n; i++) {
        v = d[i];
        if( v < t ) {
            d_out[i] = 1;
            nf ++;
        }
    }

    printf("x = %9f, sd = %9f, t = %9f, filter = %5d/%5d (%9f)\n",
           x, sd, t,
           nf, n, nf*1.0/n);

    if( d_mean != NULL ) *d_mean = x;
    if( d_sig != NULL )  *d_sig = sd;

    return 0;
}

int FilterData_Median(vector<double> &d, vector<int> &d_out,
                      double &mean, double &sig,
                      double r1, double r2,
                      double n_sig)
{
    int         n, i, j, i1, i2;
    double      sum, t;

    vector<double>  ds;

    d_out.resize(d.size(), 1);

    // sort data
    n = d.size();
    ds = d;
    std::sort(ds.begin(), ds.end());

    // mean value
    i = n/2;
    i1 = n/4;
    i2 = 3.0*n/4.0;
    mean = 0;
    for(i=i1; i<i2; i++) {
        mean += ds[i];
    }
    mean /= 1.0*(i2-i1+1);

    // calc standard deviation
    i1 = r1*n;
    i2 = r2*n;
    sum = 0.0;
    for(i=i1; i<i2; i++) {
        sum += sqr(fabs(ds[i]) - mean);
    }
    sig = sqrt(sum/(i2-i1+1));

    // filter data
    t = mean + n_sig*sig;
    j = 0;

    for(i=0; i<n; i++) {
        if( fabs(d[i]) > t ) {
            d_out[i] = 0;
            j ++;
        } else
            d_out[i] = 1;
    }

    //printf("filter: %5d / %5d, t = %f\n", j, n, t);

    return 0;
}

int FilterData_FMe(vector<double> &d, vector<int> &d_out,
                   double &mean, double &sig,
                   double t)
{
    int         n, i, j;

    vector<double>::iterator it;

    n = d.size();
    d_out.resize(d.size(), 1);

    // calculate mean/sig
    mean = 0;
    sig = 0;
    for(it=d.begin(); it!=d.end(); it++) {
        mean += fabs(*it);
    }
    mean /= n;

    for(it=d.begin(); it!=d.end(); it++) {
        sig += sqr(*it);
    }
    sig /= n;
    sig = sqrt(sig);

    // filter data
    j = 0;
    for(i=0; i<n; i++) {
        if( fabs(d[i]) > t ) {
            d_out[i] = 0;
            j ++;
        } else
            d_out[i] = 1;
    }

    //printf("filter: %5d / %5d, t = %f\n", j, n, t);

    return 0;
}


int CloudPoint_FilterZ(vector<DMatch> &matches,   vector<DMatch> &matches_out,
                       vector<CloudPoint> &cloud, vector<CloudPoint> &cloud_out,
                       double &r_sigmean)
{
    vector<DMatch>          match_0;
    vector<double>          arr_z;
    vector<int>             filt_z;
    double                  mean_z, sig_z;

    int                             k, kk;
    vector<CloudPoint>::iterator    it;

    double                  tracker_baselineZFilter_r1 = 1.0/8.0;
    double                  tracker_baselineZFilter_r2 = 7.0/8.0;
    double                  tracker_baselineZFilter_nsig = 8.0;

    CParamArray *pa = pa_get();
    if( pa != NULL ) {
        pa->d("tracker_baselineZFilter_r1", tracker_baselineZFilter_r1);
        pa->d("tracker_baselineZFilter_r2", tracker_baselineZFilter_r2);
        pa->d("tracker_baselineZFilter_nsig", tracker_baselineZFilter_nsig);
    }

    matches_out.clear();
    cloud_out.clear();

    // check cloud
    if( cloud.size() < 10 ) {
        r_sigmean = 1.0;
        return 0;
    }

    // generate filtered match
    match_0.clear();
    for(k=0, it=cloud.begin(); it!=cloud.end(); it++, k++) {
        kk = it->match_id;
        match_0.push_back(matches[kk]);
    }

    // generate arr_z
    arr_z.resize(matches.size(), 1e9);

    for(k=0, it=cloud.begin(); it!=cloud.end(); it++, k++) {
        arr_z[k] = it->pt.z;
    }

    // filter by z
    FilterData_Median(arr_z, filt_z,
                      mean_z, sig_z,
                      tracker_baselineZFilter_r1, tracker_baselineZFilter_r2,
                      tracker_baselineZFilter_nsig);

    // output data
    for(k=0; k<match_0.size(); k++) {
        if( filt_z[k] == 1 ) {
            matches_out.push_back(match_0[k]);
            cloud_out.push_back(cloud[k]);
        }
    }

    r_sigmean = sig_z/mean_z;

    printf("mean_z, sig_z = %9f %9f - %9f (%5d/%5d = %9f)\n",
           mean_z, sig_z, r_sigmean,
           cloud_out.size(), cloud.size(), 1.0*cloud_out.size()/cloud.size());

    return 0;
}
