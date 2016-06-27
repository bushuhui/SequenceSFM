/*******************************************************************************
 *
 * Sequence SfM for UAV images
 *
 * Author:
 *    Shuhui Bu (bushuhui@nwpu.edu.cn)
 *
 ******************************************************************************/


#ifndef __UTILS_GUI_H__
#define __UTILS_GUI_H__

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <map>

#include <QtGui>
#include <QMenu>
#include <QKeyEvent>
#include <QMouseEvent>
#include <qcursor.h>
#include <qmap.h>

#include <opencv2/core/core.hpp>

#include <QGLViewer/qglviewer.h>

#include "sfm_data.h"
#include "utils_event.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class PCD_Plane
{
public:
    PCD_Plane() {
        m_gridNum = 10;
        m_gridSize = 100.0;
    }

    PCD_Plane(cv::Matx34d &p_se3, int gridNum, double gridSize) {
        m_plane    = p_se3;
        m_gridNum  = gridNum;
        m_gridSize = gridSize;
    }

    ~PCD_Plane() {}

    int set(cv::Matx34d &p_se3, int grid_num = 10, double grid_size = 100.0) {
        m_plane    = p_se3;
        m_gridNum  = grid_num;
        m_gridSize = grid_size;
    }

public:
    cv::Matx34d     m_plane;
    int             m_gridNum;
    double          m_gridSize;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class PCD_Viewer : public QGLViewer
{
    Q_OBJECT

public:
    PCD_Viewer();
    ~PCD_Viewer();

    int setData(std::vector<CloudPoint> &dat);
    int setCamera(std::vector<cv::Matx34d> &cam);
    int setCamera(std::map<int, cv::Matx34d> &cam);
    int addCamera(cv::Matx34d &cam);

    int addPlane(PCD_Plane &plane);
    int setPlane(PCD_Plane &plane);
    int clearPlane(void);

    int redraw(void);
    int redraw_(void);

    void setCamSize(double cs) {
        m_camSize = cs;
    }

    double getCamSize(void) {
        return m_camSize;
    }

    void setPointSize(double ps) {
        m_pointSize = ps;
    }

    double getPointSize(void) {
        return m_pointSize;
    }

    double getSceneRange(void) {
        return m_sceneRange;
    }

    int setCamShow(int show=1) {
        m_bCamShow = show;
        return 0;
    }


protected :
    virtual void draw();
    virtual void init();

    virtual void keyPressEvent(QKeyEvent *e);
    //virtual void mousePressEvent(QMouseEvent* e);

    virtual QString helpString() const;

private :    
    QMutex                      *m_muxData;

    std::vector<CloudPoint>     m_arrPoints;
    std::vector<cv::Matx34d>    m_arrCam;
    std::vector<PCD_Plane>      m_arrPlane;

    double                      m_camSize;
    double                      m_pointSize;

    double                      m_sceneRange;

    int                         m_bCamShow;
};


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

typedef void* (*QT_WORKTHREAD)(void *args);


int qt_start(int argc, char *argv[],
             QT_WORKTHREAD workThread_func,
             QT_WORKTHREAD user_func = NULL);
int qt_exit(int code=0);

int imshow_qt(const char *name, cv::Mat &img);

int pcv_qt(const char *name, PCD_Viewer **viewer);
int pcv_redraw_qt(const char *name);
int pcv_redraw_qt(PCD_Viewer *viewer);

int waitKey_qt(int delay=0);


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void* run_parallel_func(void *args, QT_WORKTHREAD func);
int  run_parallel_join(void *dat);

#endif // end of __UTILS_GUI_H__
