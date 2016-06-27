/**********************************************************************
Finds the 150 best features in an image and tracks them through the 
next two images.  The sequential mode is set in order to speed
processing.  The features are stored in a feature table, which is then
saved to a text file; each feature list is also written to a PPM file.
**********************************************************************/

#include <stdlib.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/video/tracking.hpp"

#include "klt.h"

using namespace cv_klt;
using namespace cv;

int drawFeatures(Mat &img, KLT_FeatureList &fl)
{
    int     i;
    int     x, y;

    for(i=0; i<fl->nFeatures; i++) {
        if( fl->feature[i]->val >= 0 ) {
            x = fl->feature[i]->x;
            y = fl->feature[i]->y;

            //printf("[%5d] %d %d\n", i, x, y);

            circle(img, Point(x, y), 4, Scalar(0,255,0), 1, 8);
        }
    }

    return 0;
}

int main(int argc, char *argv[])
{
    char            fnamein[100];

    VideoCapture    vc;
    Mat             img1, img2, img_t, img_d;
    char            key=0;
    int             timeDelay = 20;

    int             nFeatures = 150;
    int             ncols, nrows;
    int             i;

    KLT_TrackingContext tc;
    KLT_FeatureList     fl;

    // disable info print
    KLTSetVerbosity(0);

    // create KLT context
    tc = KLTCreateTrackingContext();
    tc->sequentialMode = TRUE;
    tc->writeInternalImages = FALSE;
    tc->affineConsistencyCheck = 2;  /* set this to 2 to turn on affine consistency check */

    // create feature list
    fl = KLTCreateFeatureList(nFeatures);

    // load video file
    strcpy(fnamein, "test_1.mp4");
    if( argc > 1 )  strcpy(fnamein, argv[1]);

    vc.open(fnamein);
    if( !vc.isOpened() ) {
        printf("Failed to open video file: %s\n", fnamein);
        return -1;
    }

    // get first image
    vc >> img_t;
    if( img_t.channels() > 1 ) cvtColor(img_t, img1, CV_BGR2GRAY);
    else img1 = img_t;

    ncols = img1.cols;
    nrows = img1.rows;

    // detect featrues
    KLTSelectGoodFeatures(tc, img1.data, ncols, nrows, fl);

    img_t.copyTo(img_d);
    drawFeatures(img_d, fl);
    imshow("klt_demo", img_d);

    key = waitKey(timeDelay);

    // for each frame
    i = 1;
    while ( key != 27 ) {
        // get next image
        vc >> img_t;
        if ( img_t.empty() ) break;

        if( img_t.channels() > 1 ) cvtColor(img_t, img2, CV_BGR2GRAY);
        else img2 = img_t;

        KLTTrackFeatures(tc, img1.data, img2.data, ncols, nrows, fl);
        KLTReplaceLostFeatures(tc, img2.data, ncols, nrows, fl);

        // show features
        img_t.copyTo(img_d);
        drawFeatures(img_d, fl);
        imshow("klt_demo", img_d);

        // copy img2 -> img1
        img2.copyTo(img1);

        i++;

        key = waitKey(timeDelay);
    }

    // free objs
    KLTFreeFeatureList(fl);
    KLTFreeTrackingContext(tc);

    return 0;
}

