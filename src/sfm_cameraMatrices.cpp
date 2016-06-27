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

#include <vector>
#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>

#ifdef USE_EIGEN
#include <Eigen/Eigen>
#endif

#include "sfm_utils.h"
#include "sfm_cameraMatrices.h"
#include "sfm_triangulation.h"


using namespace cv;
using namespace std;


//
// from : http://people.csail.mit.edu/bkph/articles/Essential.pdf
//
void DecomposeEssentialUsingHorn90(double _E[9],
double _R1[9], double _R2[9],
double _t1[3], double _t2[3])
{
    using namespace Eigen;

    Matrix3d E = Map<Matrix<double,3,3,RowMajor> >(_E);
    Matrix3d EEt = E * E.transpose();
    //Vector3d e0e1 = E.col(0).cross(E.col(1)),e1e2 = E.col(1).cross(E.col(2)),e2e0 = E.col(2).cross(E.col(2));
    Vector3d e0e1 = E.col(0).cross(E.col(1)),e1e2 = E.col(1).cross(E.col(2)),e2e0 = E.col(2).cross(E.col(0));
    Vector3d b1,b2;


    if (e0e1.norm() > e1e2.norm() && e0e1.norm() > e2e0.norm()) {
        b1 = e0e1.normalized() * sqrt(0.5 * EEt.trace()); //Horn90 (18)
        b2 = -b1;
    } else if (e1e2.norm() > e0e1.norm() && e1e2.norm() > e2e0.norm()) {
        b1 = e1e2.normalized() * sqrt(0.5 * EEt.trace()); //Horn90 (18)
        b2 = -b1;
    } else {
        b1 = e2e0.normalized() * sqrt(0.5 * EEt.trace()); //Horn90 (18)
        b2 = -b1;
    }

    //Horn90 (19)
    Matrix3d cofactors;
    cofactors.col(0) = e1e2; cofactors.col(1) = e2e0; cofactors.col(2) = e0e1;
    cofactors.transposeInPlace();

    // B = [b]_x ,
    // see Horn90 (6) and http://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication
    Matrix3d B1; B1 <<	0,-b1(2),b1(1),
            b1(2),0,-b1(0),
            -b1(1),b1(0),0;
    Matrix3d B2; B2 <<	0,-b2(2),b2(1),
            b2(2),0,-b2(0),
            -b2(1),b2(0),0;

    Map<Matrix<double,3,3,RowMajor> > R1(_R1),R2(_R2);

    //Horn90 (24)
    R1 = (cofactors.transpose() - B1*E) / b1.dot(b1);
    R2 = (cofactors.transpose() - B2*E) / b2.dot(b2);
    Map<Vector3d> t1(_t1),t2(_t2);
    t1 = b1; t2 = b2;

    //cout << "Horn90 provided " << endl << R1 << endl << "and" << endl << R2 << endl;
}

bool CheckCoherentRotation(cv::Mat_<double>& R)
{
    if(fabsf(determinant(R))-1.0 > 1e-07) {
        cerr << "det(R) != +-1.0, this is not a rotation matrix" << endl;
        return false;
    }

    return true;
}

Mat GetFundamentalMat(const vector<KeyPoint>& imgpts1,
                      const vector<KeyPoint>& imgpts2,
                      vector<KeyPoint>& imgpts1_good,
                      vector<KeyPoint>& imgpts2_good,
                      vector<DMatch>& matches
                      )
{
    //Try to eliminate keypoints based on the fundamental matrix
    //(although this is not the proper way to do this)
    vector<uchar> status(imgpts1.size());

    // undistortPoints(imgpts1, imgpts1, cam_matrix, distortion_coeff);
    // undistortPoints(imgpts2, imgpts2, cam_matrix, distortion_coeff);

    imgpts1_good.clear(); imgpts2_good.clear();

    vector<KeyPoint> imgpts1_tmp;
    vector<KeyPoint> imgpts2_tmp;
    if (matches.size() <= 0) {
        imgpts1_tmp = imgpts1;
        imgpts2_tmp = imgpts2;
    } else {
        GetAlignedPointsFromMatch(imgpts1, imgpts2, matches, imgpts1_tmp, imgpts2_tmp);
    }

    Mat F;
    {
        vector<Point2f> pts1,pts2;
        KeyPointsToPoints(imgpts1_tmp, pts1);
        KeyPointsToPoints(imgpts2_tmp, pts2);

        double  minVal,maxVal;
        double  param1;

        cv::minMaxIdx(pts1,&minVal,&maxVal);

        param1 = 0.006*maxVal;
        //if( param1 > 3.0 ) param1 = 3.0;
        //printf("  cv::minMaxIdx = %f, %f\n", minVal, maxVal);

        //threshold from [Snavely07 4.1]
        F = findFundamentalMat(pts1, pts2, FM_RANSAC, param1, 0.99, status);
    }

    vector<DMatch> new_matches;

    printf("F keeping %5d/%5d\n", countNonZero(status), status.size());

    for (unsigned int i=0; i<status.size(); i++) {
        if (status[i]) {
            imgpts1_good.push_back(imgpts1_tmp[i]);
            imgpts2_good.push_back(imgpts2_tmp[i]);

            //new_matches.push_back(DMatch(matches[i].queryIdx,matches[i].trainIdx,matches[i].distance));
            new_matches.push_back(matches[i]);
        }
    }

    //keep only those points who survived the fundamental matrix
    matches = new_matches;

    return F;
}



void TakeSVDOfE(Mat_<double>& E, Mat& svd_u, Mat& svd_vt, Mat& svd_w)
{
    //Using Eigen's SVD
    cout << "Eigen3 SVD..\n";
    Eigen::Matrix3f  e = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> >((double*)E.data).cast<float>();
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(e, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf Esvd_u = svd.matrixU();
    Eigen::MatrixXf Esvd_v = svd.matrixV();
    svd_u = (Mat_<double>(3,3) << Esvd_u(0,0), Esvd_u(0,1), Esvd_u(0,2),
             Esvd_u(1,0), Esvd_u(1,1), Esvd_u(1,2),
             Esvd_u(2,0), Esvd_u(2,1), Esvd_u(2,2));
    Mat_<double> svd_v = (Mat_<double>(3,3) << Esvd_v(0,0), Esvd_v(0,1), Esvd_v(0,2),
                          Esvd_v(1,0), Esvd_v(1,1), Esvd_v(1,2),
                          Esvd_v(2,0), Esvd_v(2,1), Esvd_v(2,2));
    svd_vt = svd_v.t();
    svd_w = (Mat_<double>(1,3) << svd.singularValues()[0] , svd.singularValues()[1] , svd.singularValues()[2]);

    cout << "----------------------- SVD ------------------------\n";
    cout << "U:\n"<<svd_u<<"\nW:\n"<<svd_w<<"\nVt:\n"<<svd_vt<<endl;
    cout << "----------------------------------------------------\n";
}

bool TestTriangulation(const vector<CloudPoint>& pcloud, const Matx34d& P, vector<uchar>& status)
{
    vector<Point3d> pcloud_pt3d = CloudPointsToPoints(pcloud);
    vector<Point3d> pcloud_pt3d_projected(pcloud_pt3d.size());

    Matx44d P4x4 = Matx44d::eye();
    for(int i=0;i<12;i++) P4x4.val[i] = P.val[i];

    perspectiveTransform(pcloud_pt3d, pcloud_pt3d_projected, P4x4);

    status.resize(pcloud.size(),0);
    for (int i=0; i<pcloud.size(); i++) {
        status[i] = (pcloud_pt3d_projected[i].z > 0) ? 1 : 0;
    }
    int count = countNonZero(status);

    double percentage = ((double)count / (double)pcloud.size());

    printf("Triangulation: %5d/%5d = %9f are in front of camera\n", count, pcloud.size(), percentage*100.0);

    if(percentage < 0.75)
        return false; //less than 75% of the points are in front of the camera

    return true;
}

bool DecomposeEtoRandT(
        Mat_<double>& E,
        Mat_<double>& R1,
        Mat_<double>& R2,
        Mat_<double>& t1,
        Mat_<double>& t2)
{
    //Using Horn E decomposition
    DecomposeEssentialUsingHorn90(E[0],R1[0],R2[0],t1[0],t2[0]);

    return true;
}

bool FindCameraMatrices(const Mat& K,
                        const Mat& Kinv,
                        const Mat& distcoeff,
                        const vector<KeyPoint>& imgpts1,
                        const vector<KeyPoint>& imgpts2,
                        vector<KeyPoint>& imgpts1_good,
                        vector<KeyPoint>& imgpts2_good,
                        Matx34d& P,
                        Matx34d& P1,
                        vector<DMatch>& matches,
                        vector<CloudPoint>& outCloud
                        )
{
    cout << "Find camera matrices...";

    double t = getTickCount();

    Mat F = GetFundamentalMat(imgpts1,imgpts2,
                              imgpts1_good,imgpts2_good,
                              matches
                              );
    if(matches.size() < 100) { // || ((double)imgpts1_good.size() / (double)imgpts1.size()) < 0.25
        cerr << "not enough inliers after F matrix" << endl;
        return false;
    }

    // Essential matrix: compute then extract cameras [R|t]
    Mat_<double> E = K.t() * F * K; //according to HZ (9.12)

    // according to http://en.wikipedia.org/wiki/Essential_matrix#Properties_of_the_essential_matrix
    if(fabsf(determinant(E)) > 1e-07) {
        cout << "det(E) != 0 : " << determinant(E) << "\n";
        P1 = 0;
        return false;
    }

    Mat_<double> R1(3,3);
    Mat_<double> R2(3,3);
    Mat_<double> t1(1,3);
    Mat_<double> t2(1,3);


    //decompose E to P' , HZ (9.19)
    if ( !DecomposeEtoRandT(E,R1,R2,t1,t2) ) return false;

    if(determinant(R1)+1.0 < 1e-09) {
        //according to http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid
        cout << "det(R) == -1 ["<<determinant(R1)<<"]: flip E's sign" << endl;
        E = -E;
        DecomposeEtoRandT(E,R1,R2,t1,t2);
    }

    if (!CheckCoherentRotation(R1)) {
        cout << "resulting rotation is not coherent\n";
        P1 = 0;
        return false;
    }


    // triangulation
    vector<CloudPoint> pcloud,pcloud1;
    vector<KeyPoint> corresp;
    double reproj_error1, reproj_error2;
    vector<uchar> tmp_status1, tmp_status2;


    P1 = Matx34d(R1(0,0),	R1(0,1),	R1(0,2),	t1(0),
                 R1(1,0),	R1(1,1),	R1(1,2),	t1(1),
                 R1(2,0),	R1(2,1),	R1(2,2),	t1(2));

    reproj_error1 = TriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, distcoeff, P, P1, pcloud,  corresp);
    reproj_error2 = TriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, distcoeff, P1, P, pcloud1, corresp);

    //check if pointa are triangulated --in front-- of cameras for all 4 ambiguations
    if (!TestTriangulation(pcloud,P1,tmp_status1) || !TestTriangulation(pcloud1,P,tmp_status2) ||
            reproj_error1 > 100.0 || reproj_error2 > 100.0) {
        P1 = Matx34d(R1(0,0),	R1(0,1),	R1(0,2),	t2(0),
                     R1(1,0),	R1(1,1),	R1(1,2),	t2(1),
                     R1(2,0),	R1(2,1),	R1(2,2),	t2(2));

        pcloud.clear(); pcloud1.clear(); corresp.clear();
        reproj_error1 = TriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, distcoeff, P, P1, pcloud,  corresp);
        reproj_error2 = TriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, distcoeff, P1, P, pcloud1, corresp);

        if (!TestTriangulation(pcloud,P1,tmp_status1) || !TestTriangulation(pcloud1,P,tmp_status2) ||
                reproj_error1 > 100.0 || reproj_error2 > 100.0) {

            if (!CheckCoherentRotation(R2)) {
                cout << "resulting rotation is not coherent\n";
                std::cout << "R2: " << R2 << "\n\n";

                P1 = 0;
                return false;
            }

            P1 = Matx34d(R2(0,0),	R2(0,1),	R2(0,2),	t1(0),
                         R2(1,0),	R2(1,1),	R2(1,2),	t1(1),
                         R2(2,0),	R2(2,1),	R2(2,2),	t1(2));

            pcloud.clear(); pcloud1.clear(); corresp.clear();
            reproj_error1 = TriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, distcoeff, P, P1, pcloud,  corresp);
            reproj_error2 = TriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, distcoeff, P1, P, pcloud1, corresp);

            if (!TestTriangulation(pcloud,P1,tmp_status1) || !TestTriangulation(pcloud1,P,tmp_status2) ||
                    reproj_error1 > 100.0 || reproj_error2 > 100.0) {
                P1 = Matx34d(R2(0,0),	R2(0,1),	R2(0,2),	t2(0),
                             R2(1,0),	R2(1,1),	R2(1,2),	t2(1),
                             R2(2,0),	R2(2,1),	R2(2,2),	t2(2));

                pcloud.clear(); pcloud1.clear(); corresp.clear();
                reproj_error1 = TriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, distcoeff, P, P1, pcloud,  corresp);
                reproj_error2 = TriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, distcoeff, P1, P, pcloud1, corresp);

                if (!TestTriangulation(pcloud,P1,tmp_status1) || !TestTriangulation(pcloud1,P,tmp_status2) ||
                        reproj_error1 > 100.0 || reproj_error2 > 100.0) {
                    cout << "ERR: No one triangulation can success!" << endl;
                    return false;
                }
            }
        }
    }

    for (unsigned int i=0; i<pcloud.size(); i++) {
        if( tmp_status1[i] == 1 )
            outCloud.push_back(pcloud[i]);
    }


    t = ((double)getTickCount() - t)/getTickFrequency();
    //cout << "\nDone. (" << t <<"s)"<< "\n\n";

    return true;
}
