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
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>

#include "sfm_triangulation.h"
#include "sfm_utils.h"

using namespace std;
using namespace cv;


/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
Mat_<double> LinearLSTriangulation(Point3d u,		//homogenous image point (u,v,1)
                                   Matx34d P,		//camera 1 matrix
                                   Point3d u1,		//homogenous image point in 2nd camera
                                   Matx34d P1		//camera 2 matrix
                                   )
{
    Matx43d A(u.x*P(2,0)-P(0,0),	u.x*P(2,1)-P(0,1),		u.x*P(2,2)-P(0,2),
              u.y*P(2,0)-P(1,0),	u.y*P(2,1)-P(1,1),		u.y*P(2,2)-P(1,2),
              u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),	u1.x*P1(2,2)-P1(0,2),
              u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),	u1.y*P1(2,2)-P1(1,2)
              );
    Matx41d B(-(u.x*P(2,3)	-P(0,3)),
              -(u.y*P(2,3)	-P(1,3)),
              -(u1.x*P1(2,3)	-P1(0,3)),
              -(u1.y*P1(2,3)	-P1(1,3)));

    Mat_<double> X;
    solve(A, B, X, DECOMP_SVD);

    return X;
}



/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
Mat_<double> IterativeLinearLSTriangulation(Point3d u,          //homogenous image point (u,v,1)
                                            Matx34d P,			//camera 1 matrix
                                            Point3d u1,			//homogenous image point in 2nd camera
                                            Matx34d P1			//camera 2 matrix
                                            )
{
    double wi = 1, wi1 = 1;
    Mat_<double> X(4,1);

    // Hartley suggests 10 iterations at most
    for (int i=0; i<10; i++) {
        Mat_<double> X_;

        //reweight equations and solve
        Matx43d A((u.x*P(2,0)-P(0,0))/wi,		(u.x*P(2,1)-P(0,1))/wi,			(u.x*P(2,2)-P(0,2))/wi,
                  (u.y*P(2,0)-P(1,0))/wi,		(u.y*P(2,1)-P(1,1))/wi,			(u.y*P(2,2)-P(1,2))/wi,
                  (u1.x*P1(2,0)-P1(0,0))/wi1,	(u1.x*P1(2,1)-P1(0,1))/wi1,		(u1.x*P1(2,2)-P1(0,2))/wi1,
                  (u1.y*P1(2,0)-P1(1,0))/wi1,	(u1.y*P1(2,1)-P1(1,1))/wi1,		(u1.y*P1(2,2)-P1(1,2))/wi1 );
        Mat_<double> B = (Mat_<double>(4,1) <<	  -(u.x*P(2,3)   -P(0,3))/wi,
                                                  -(u.y*P(2,3)   -P(1,3))/wi,
                                                  -(u1.x*P1(2,3) -P1(0,3))/wi1,
                                                  -(u1.y*P1(2,3) -P1(1,3))/wi1 );

        solve(A,B,X_,DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;

        //recalculate weights
        double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
        double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);

        // quit iteration
        if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;

        wi = p2x;
        wi1 = p2x1;
    }

    return X;
}

//Triagulate points
double TriangulatePoints(const vector<KeyPoint>& pt_set1,
                        const vector<KeyPoint>& pt_set2,
                        const Mat& K,
                        const Mat& Kinv,
                        const Mat& distcoeff,
                        const Matx34d& P,
                        const Matx34d& P1,
                        vector<CloudPoint>& pointcloud,
                        vector<KeyPoint>& correspImg1Pt)
{
    //	pointcloud.clear();
    correspImg1Pt.clear();

    //cout << "Triangulating... ";

    double t = getTickCount();
    vector<double> reproj_error;
    unsigned int pts_size = pt_set1.size();


    Mat_<double> KP1 = K * Mat(P1);
    vector<KeyPoint> pt_s1, pt_s2;

    // undistort points
    if( 1 ) {
        vector<Point2f> _pt_set1_pt,_pt_set2_pt;
        KeyPointsToPoints(pt_set1, _pt_set1_pt);
        KeyPointsToPoints(pt_set2, _pt_set2_pt);

        //undistort
        Mat pt_set1_pt, pt_set2_pt;
        undistortPoints(_pt_set1_pt, pt_set1_pt, K, distcoeff);
        undistortPoints(_pt_set2_pt, pt_set2_pt, K, distcoeff);

        float  *p1, *p2;
        double *pk, fx, cx, fy, cy;
        double nx1, ny1, nx2, ny2;

        pk = (double *) K.data;
        fx = pk[0]; cx = pk[2];
        fy = pk[4]; cy = pk[5];

        p1 = (float *) pt_set1_pt.data;
        p2 = (float *) pt_set2_pt.data;

        pt_s1 = pt_set1;
        pt_s2 = pt_set2;

        for(int i=0; i<pts_size; i++) {
            nx1 = p1[i*2+0]*fx + cx;
            ny1 = p1[i*2+1]*fy + cy;

            nx2 = p2[i*2+0]*fx + cx;
            ny2 = p2[i*2+1]*fy + cy;

            pt_s1[i].pt.x = nx1;
            pt_s1[i].pt.y = ny1;

            pt_s2[i].pt.x = nx2;
            pt_s2[i].pt.y = ny2;
        }
    } else {
        pt_s1 = pt_set1;
        pt_s2 = pt_set2;
    }

    //#pragma omp parallel for num_threads(1)
    for (int i=0; i<pts_size; i++) {
        Point2f kp = pt_s1[i].pt;
        Point3d u(kp.x,kp.y,1.0);
        Mat_<double> um = Kinv * Mat_<double>(u);
        u.x = um(0); u.y = um(1); u.z = um(2);

        Point2f kp1 = pt_s2[i].pt;
        Point3d u1(kp1.x,kp1.y,1.0);
        Mat_<double> um1 = Kinv * Mat_<double>(u1);
        u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

        // calculate 3D point
        Mat_<double> X = IterativeLinearLSTriangulation(u,P,u1,P1);

        //reproject
        Mat_<double> xPt_img = KP1 * X;
        Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));

        //#pragma omp critical
        {
            double reprj_err = norm(xPt_img_-kp1);
            reproj_error.push_back(reprj_err);

            CloudPoint cp;
            cp.pt = Point3d(X(0),X(1),X(2));
            cp.reprojection_error = reprj_err;
            cp.match_id = i;

            pointcloud.push_back(cp);
            correspImg1Pt.push_back(pt_s1[i]);
        }
    }

    Scalar mse = mean(reproj_error);
    t = ((double)getTickCount() - t)/getTickFrequency();

    //cout << " Done. (" << pointcloud.size() << " points, " << t
    //     << "s, mean reproj err = " << mse[0] << ")" << endl;

    return mse[0];
}
