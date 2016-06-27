/*******************************************************************************
 *
 * Sequence SfM for UAV images
 *
 * Author:
 *    Shuhui Bu (bushuhui@nwpu.edu.cn)
 *
 ******************************************************************************/


// Reference:
//  http://www.libqglviewer.com/refManual/classQGLViewer.html

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include <map>
#include <iostream>

#include <QtGui>
#include <QMenu>
#include <QKeyEvent>
#include <QMouseEvent>
#include <qcursor.h>
#include <qmap.h>

#include <Eigen/Eigen>

#include "utils.h"
#include "utils_gui.h"
#include "utils_event.h"


using namespace std;
using namespace cv;
using namespace qglviewer;



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
class EventQueue;
class WidgetMap;
class BackgroundWidget;
class CommandPackage;


EventQueue *g_eventQueue = NULL;
WidgetMap  *g_widgetMap = NULL;
BackgroundWidget *g_backgroundWidget = NULL;


int _send_command(CommandPackage *cmd, int wait);


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class EventQueue
{
public:
    EventQueue() {
        m_mux = new QMutex(QMutex::NonRecursive);
    }

    ~EventQueue() {
        delete m_mux;
    }

    void push(InputEvent &evt) {
        m_mux->lock();
        m_queue.push_back(evt);
        m_mux->unlock();
    }

    void pop(InputEvent &ev) {
        m_mux->lock();
        ev = m_queue.front();
        m_queue.pop_front();
        m_mux->unlock();
    }

    int size(void) {
        return m_queue.size();
    }

    void clear(void) {
        m_mux->lock();
        m_queue.clear();
        m_mux->unlock();
    }

    void clear(ru64 ct) {
        ru64    dt_lim = 1000;
        ru64    dt;

        InputEvent ev;

        m_mux->lock();

        while(1) {
            if( m_queue.size() == 0 ) break;

            ev = m_queue.front();
            dt = ct - ev.timestamp;
            if( dt > dt_lim ) {
                m_queue.pop_front();
            } else {
                break;
            }
        }

        m_mux->unlock();
    }

protected:
    QMutex              *m_mux;
    deque<InputEvent>   m_queue;
};


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class CommandPackage
{
public:
    CommandPackage() {
        cmdID = 0;
        title = "";
        state = 0;

        mux = new QMutex;
    }

    CommandPackage(int cid, std::string &t) {
        cmdID = cid;
        title = t;

        mux = new QMutex;
    }

    ~CommandPackage() {
        delete mux;
    }

    CommandPackage& operator =(const CommandPackage &cmd) {
        if( this != &cmd ) {
            cmdID = cmd.cmdID;
            title = cmd.title;
            dat   = cmd.dat;
        }

        return *this;
    }

    int setState(int st) {
        //mux->lock();
        state = st;
        //mux->unlock();
    }

    int getState(void) {
        //mux->lock();
        return state;
        //mux->unlock();
    }

public:
    int             cmdID;
    std::string     title;
    void            *dat;
    int             state;

    QMutex          *mux;
};


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

PCD_Viewer::PCD_Viewer()
{
    m_muxData = new QMutex(QMutex::NonRecursive);

    m_camSize = 3.0;
    m_pointSize = 3.0;

    m_bCamShow = 1;
}

PCD_Viewer::~PCD_Viewer()
{
    delete m_muxData;
}

int PCD_Viewer::setData(std::vector<CloudPoint> &dat)
{
    std::vector<CloudPoint>::iterator    it;

    double x_min, x_max, y_min, y_max, z_min, z_max;

    x_min = 1e30; x_max = -1e30;
    y_min = 1e30; y_max = -1e30;
    z_min = 1e30; z_max = -1e30;

    m_muxData->lock();

    // coopy data
    m_arrPoints.clear();
    for(it=dat.begin(); it!=dat.end(); it++) {
        m_arrPoints.push_back(*it);

        if( it->pt.x < x_min ) x_min = it->pt.x;
        if( it->pt.x > x_max ) x_max = it->pt.x;

        if( it->pt.y < y_min ) y_min = it->pt.y;
        if( it->pt.y > y_max ) y_max = it->pt.y;

        if( it->pt.z < z_min ) z_min = it->pt.z;
        if( it->pt.z > z_max ) z_max = it->pt.z;
    }

    m_muxData->unlock();

    // set scene range
    if( dat.size() > 0 ) {
        double x_rang, y_rang, z_rang;

        x_rang = max(fabs(x_min), fabs(x_max));
        y_rang = max(fabs(y_min), fabs(y_max));
        z_rang = max(fabs(z_min), fabs(z_max));
        m_sceneRange = sqrt(sqr(x_rang) + sqr(y_rang) + sqr(z_rang));

        #if 0
        printf("x: %f ~ %f\n", x_min, x_max);
        printf("y: %f ~ %f\n", y_min, y_max);
        printf("z: %f ~ %f\n", z_min, z_max);
        printf("range = %f\n", m_sceneRange);
        #endif

        setSceneRadius(m_sceneRange);
        setCamSize(m_sceneRange/20.0);
        //setPointSize(m_sceneRange/200.0);
    }

    return 0;
}

int PCD_Viewer::setCamera(std::vector<cv::Matx34d> &cam)
{
    std::vector<Matx34d>::iterator it;

    m_muxData->lock();

    m_arrCam.clear();
    m_arrCam.reserve(cam.size());

    for(it=cam.begin(); it!=cam.end(); it++) {
        m_arrCam.push_back(*it);
    }

    m_muxData->unlock();

    return 0;
}

int PCD_Viewer::setCamera(std::map<int, cv::Matx34d> &cam)
{
    std::map<int, cv::Matx34d>::iterator it;

    m_muxData->lock();

    m_arrCam.clear();
    m_arrCam.reserve(cam.size());

    for(it=cam.begin(); it!=cam.end(); it++) {
        m_arrCam.push_back(it->second);
    }

    m_muxData->unlock();

    return 0;
}

int PCD_Viewer::addCamera(cv::Matx34d &cam)
{
    m_muxData->lock();

    m_arrCam.push_back(cam);

    m_muxData->unlock();

    return 0;
}

int PCD_Viewer::addPlane(PCD_Plane &plane)
{
    m_muxData->lock();

    m_arrPlane.push_back(plane);

    m_muxData->unlock();

    return 0;
}

int PCD_Viewer::setPlane(PCD_Plane &plane)
{
    m_muxData->lock();

    m_arrPlane.clear();
    m_arrPlane.push_back(plane);

    m_muxData->unlock();

    return 0;
}

int PCD_Viewer::clearPlane(void)
{
    m_muxData->lock();

    m_arrPlane.clear();

    m_muxData->unlock();

    return 0;
}


int PCD_Viewer::redraw(void)
{
    CommandPackage  cmd;

    cmd.cmdID = 3;
    cmd.title = "";
    cmd.dat   = this;

    _send_command(&cmd, 1);

    return 0;
}

int PCD_Viewer::redraw_(void)
{
    updateGL();

    return 0;
}

void PCD_Viewer::draw()
{
    std::vector<CloudPoint>::iterator   it;
    std::vector<Matx34d>::iterator      itc;
    std::vector<PCD_Plane>::iterator    itp;

    int     i, j;

    m_muxData->lock();

    //////////////////////////////////////
    /// draw points
    //////////////////////////////////////
    glPointSize(m_pointSize);

    glBegin(GL_POINTS);

    for(it = m_arrPoints.begin(); it!=m_arrPoints.end(); it++) {
        glColor3ub(it->rgb[0], it->rgb[1], it->rgb[2]);
        glVertex3f(it->pt.x, it->pt.y, it->pt.z);
    }

    glEnd();


    //////////////////////////////////////
    /// draw camera
    //////////////////////////////////////
    if( m_bCamShow ) {

        glLineWidth(2.0);
        glBegin(GL_LINES);

        float _R[9], _t[3];

        for(itc = m_arrCam.begin(); itc != m_arrCam.end(); itc++) {
            cv::Matx34d &d = *itc;

            // NOTE: Eigen Matrix3f using row-first order
            //   therefore, _R = inv(R)
            _R[0] = d(0, 0); _R[1] = d(0, 1); _R[2] = d(0, 2);
            _R[3] = d(1, 0); _R[4] = d(1, 1); _R[5] = d(1, 2);
            _R[6] = d(2, 0); _R[7] = d(2, 1); _R[8] = d(2, 2);

            _t[0] = d(0, 3); _t[1] = d(1, 3); _t[2] = d(2, 3);

            Eigen::Matrix3f R(_R);
            Eigen::Vector3f t1(_t), t, pxyz;

            t = -R * t1;

            Eigen::Vector3f vright   =  R.col(0).normalized() * m_camSize;
            Eigen::Vector3f vup      =  R.col(1).normalized() * m_camSize;
            Eigen::Vector3f vforward =  R.col(2).normalized() * m_camSize;

            // x-axis
            pxyz = t + vright;

            glColor3ub(255, 0, 0);
            glVertex3f(t(0), t(1), t(2));
            glVertex3f(pxyz(0), pxyz(1), pxyz(2));

            // y-axis
            pxyz = t + vup;

            glColor3ub(0, 255, 0);
            glVertex3f(t(0), t(1), t(2));
            glVertex3f(pxyz(0), pxyz(1), pxyz(2));

            // z-axis
            pxyz = t + vforward;

            glColor3ub(0, 0, 255);
            glVertex3f(t(0), t(1), t(2));
            glVertex3f(pxyz(0), pxyz(1), pxyz(2));
        }

        glEnd();
    }

    //////////////////////////////////////
    /// draw plane
    //////////////////////////////////////
    if( 1 ) {
        glLineWidth(1.0);
        glBegin(GL_LINES);

        Eigen::Vector3d ax, ay, az;
        Eigen::MatrixXd R(3, 3), t(3, 1), t1(3, 1);
        Eigen::Vector3d p1, p2, p3, p4, pp1, pp2;
        int             gn;
        double          gs;

        for(itp=m_arrPlane.begin(); itp!=m_arrPlane.end(); itp++) {
            PCD_Plane &p = *itp;

            gn = p.m_gridNum;
            gs = p.m_gridSize;

            // get rotation & translation
            for(j=0; j<3; j++) {
                for(i=0; i<3; i++) {
                    R(j, i) = p.m_plane(j, i);
                }

                t(j) = p.m_plane(j, 3);

                ax(j) = p.m_plane(0, j);
                ay(j) = p.m_plane(1, j);
                az(j) = p.m_plane(2, j);
            }

            t1 = -R.transpose() * t;

            // draw plane axes
            p1 = t1 + ax*gn*gs/2;
            p2 = t1 + ay*gn*gs/2;
            p3 = t1 + az*gn*gs/2;

            glColor3ub(255, 0, 0);
            glVertex3f(t1(0), t1(1), t1(2));
            glVertex3f(p1(0), p1(1), p1(2));

            glColor3ub(0, 255, 0);
            glVertex3f(t1(0), t1(1), t1(2));
            glVertex3f(p2(0), p2(1), p2(2));

            glColor3ub(0, 0, 255);
            glVertex3f(t1(0), t1(1), t1(2));
            glVertex3f(p3(0), p3(1), p3(2));

            // draw grids
            p1 = t1 - ax*gn*gs/2 - ay*gn*gs/2;
            p2 = t1 - ax*gn*gs/2 + ay*gn*gs/2;
            p3 = t1 + ax*gn*gs/2 + ay*gn*gs/2;
            p4 = t1 + ax*gn*gs/2 - ay*gn*gs/2;

            glColor3ub(255, 255, 255);

            glVertex3f(p1(0), p1(1), p1(2));
            glVertex3f(p2(0), p2(1), p2(2));

            glVertex3f(p2(0), p2(1), p2(2));
            glVertex3f(p3(0), p3(1), p3(2));

            glVertex3f(p3(0), p3(1), p3(2));
            glVertex3f(p4(0), p4(1), p4(2));

            glVertex3f(p4(0), p4(1), p4(2));
            glVertex3f(p1(0), p1(1), p1(2));

            for(i=1; i<gn-1; i++) {
                pp1 = p1 + (p4-p1)/(gn-1)*i;
                pp2 = p2 + (p3-p2)/(gn-1)*i;

                glVertex3f(pp1(0), pp1(1), pp1(2));
                glVertex3f(pp2(0), pp2(1), pp2(2));
            }

            for(i=1; i<gn-1; i++) {
                pp1 = p1 + (p2-p1)/(gn-1)*i;
                pp2 = p4 + (p3-p4)/(gn-1)*i;

                glVertex3f(pp1(0), pp1(1), pp1(2));
                glVertex3f(pp2(0), pp2(1), pp2(2));
            }
        }

        glEnd();
    }

    m_muxData->unlock();
}

void PCD_Viewer::init()
{
    // Restore previous viewer state.
    restoreStateFromFile();

    glDisable(GL_LIGHTING);

    //setGridIsDrawn();
    //setAxisIsDrawn();
}

void PCD_Viewer::keyPressEvent(QKeyEvent *e)
{
    bool handled = false;

    // Get event modifiers key
    const Qt::KeyboardModifiers modifiers = e->modifiers();

    // add key press event to queue
    {
        ru32        key_raw, key;
        InputEvent  ev;

        // convert Qt key code to universial key code
        key_raw = e->key();
        input_event_qt_code_trans(key_raw, key);

        ev.type        = OSA_KEY_PRESS;
        ev.code_raw    = key_raw;
        ev.code        = key;
        ev.timestamp   = tm_get_millis();

        g_eventQueue->push(ev);
    }

    // A simple switch on e->key() is not sufficient if we want to take state key into account.
    // With a switch, it would have been impossible to separate 'F' from 'CTRL+F'.
    // That's why we use imbricated if...else and a "handled" boolean.

    if ((e->key()==Qt::Key_W) && (modifiers==Qt::NoButton))
    {
        handled = true;
        updateGL();
    }
    else if ((e->key()==Qt::Key_F) && (modifiers==Qt::NoButton))
    {
        handled = true;
        updateGL();
    }
    else if ( (e->key() == Qt::Key_C) && (modifiers == Qt::NoButton) ) {
        handled = true;
        if( m_bCamShow ) m_bCamShow = 0;
        else             m_bCamShow = 1;

        updateGL();
    }
    else if ( (e->key() == Qt::Key_R) && (modifiers==Qt::NoButton) )
    {
        handled = true;
        showEntireScene();
    } else {
        QGLViewer::keyPressEvent(e);
    }
}


QString PCD_Viewer::helpString() const
{
    QString text("<h2>Simple Pointcloud viewer</h2>");
    text += "This viewer show point clouds<br><br>";
    text += "Use <code>setShortcut()</code> to change standard action key bindings (display of axis, grid or fps, exit shortcut...).<br><br>";
    text += "Use <code>setMouseBinding()</code> and <code>setWheelBinding()</code> to change standard action mouse bindings ";
    text += "(camera rotation, translation, object selection...).<br><br>";
    text += "If you want to define <b>new</b> key or mouse actions, overload <code>keyPressEvent()</code> and/or ";
    text += "<code>mouse(Press|Move|Release)Event()</code> to define and bind your own new actions. ";
    text += "Use <code>setKeyDescription()</code> and <code>setMouseBindingDescription()</code> to add a description of your bindings in the help window.<br><br>";
    text += "In this example, we defined the <b>F</b> and <b>W</b> keys and the right mouse button opens a popup menu. ";
    text += "See the keyboard and mouse tabs in this help window for the complete bindings description.<br><br>";
    text += "By the way, exit shortcut has been binded to <b>Ctrl+Q</b>.";
    return text;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

typedef std::map<string, void *> WidgetMapType;

class WidgetMap
{
public:
    WidgetMap() {
        m_mux = new QMutex(QMutex::NonRecursive);
    }
    ~WidgetMap() {
        delete m_mux;
    }

    void* getWidget(const string &wn) {
        void *ret = NULL;
        WidgetMapType::const_iterator it;

        m_mux->lock();
        it = m_widgetMap.find(wn);
        if( it != m_widgetMap.end() ) {
            ret = it->second;
        }
        m_mux->unlock();

        return ret;
    }

    int existWidget(const string &wn) {
        int ret = 0;
        WidgetMapType::const_iterator it;

        m_mux->lock();
        it = m_widgetMap.find(wn);
        if( it != m_widgetMap.end() ) {
            ret = 1;
        }
        m_mux->unlock();

        return ret;
    }

    int addWidget(const string &wn, void *dat) {
        m_mux->lock();
        m_widgetMap.insert(make_pair(wn, dat));
        m_mux->unlock();
    }

    int delWidget(const string &wn) {
        int ret = 1;
        WidgetMapType::iterator it;

        m_mux->lock();
        it = m_widgetMap.find(wn);
        if( it != m_widgetMap.end() ) {
            m_widgetMap.erase(it);
            ret = 0;
        }
        m_mux->unlock();

        return ret;
    }

protected:
    WidgetMapType   m_widgetMap;
    QMutex          *m_mux;
};


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class Mat_Viewer : public QWidget
{
public:
    Mat_Viewer(QWidget *parent = 0);
    ~Mat_Viewer();

    int imshow(string &name, cv::Mat &img);

protected:
    void keyPressEvent(QKeyEvent *e);

    void paintEvent(QPaintEvent *);
    void closeEvent(QCloseEvent *);

protected:
    QImage      *m_img;
    QMutex      *m_muxData;
    string      m_name;
};


Mat_Viewer::Mat_Viewer(QWidget *parent) : QWidget(parent)
{
    m_muxData = new QMutex(QMutex::NonRecursive);

    m_img = NULL;
}

Mat_Viewer::~Mat_Viewer()
{
    if( m_img != NULL ) {
        delete m_img;
        m_img = NULL;
    }

    delete m_muxData;
}

int Mat_Viewer::imshow(string &name, cv::Mat &img)
{
    int     w, h, c;
    int     i, j;

    unsigned char *p1, *p2;

    m_name = name;

    // get mat image properties
    w = img.cols;
    h = img.rows;
    c = img.channels();
    p1 = img.data;

    // resize window
    resize(w, h);

    // create background image
    m_muxData->lock();

    if( m_img != NULL ) delete m_img;
    m_img = new QImage(w, h, QImage::Format_RGB888);

    // copy pixel buffer
    if( c == 1 ) {
        for(j=0; j<h; j++) {
            p2 = m_img->scanLine(j);

            for(i=0; i<w; i++) {
                p2[i*3+0] = p1[i];
                p2[i*3+1] = p1[i];
                p2[i*3+2] = p1[i];
            }

            p1 += w;
        }
    } else {
        for(j=0; j<h; j++) {
            p2 = m_img->scanLine(j);

            for(i=0; i<w; i++) {
                p2[i*3+0] = p1[i*3+2];
                p2[i*3+1] = p1[i*3+1];
                p2[i*3+2] = p1[i*3+0];
            }

            p1 += w*3;
        }
    }

    m_muxData->unlock();

    // redraw
    update();

    return 0;
}

void Mat_Viewer::keyPressEvent(QKeyEvent *e)
{
    // add key press event to queue
    {
        ru32        key_raw, key;
        InputEvent  ev;

        // convert Qt key code to universial key code
        key_raw = e->key();
        input_event_qt_code_trans(key_raw, key);

        ev.type        = OSA_KEY_PRESS;
        ev.code_raw    = key_raw;
        ev.code        = key;
        ev.timestamp   = tm_get_millis();

        g_eventQueue->push(ev);
    }
}


void Mat_Viewer::paintEvent(QPaintEvent *event)
{
    if( m_img != NULL ) {
        QPainter    painter(this);

        // draw image images
        painter.drawImage(QPoint(0, 0), *m_img);
    }
}

void Mat_Viewer::closeEvent(QCloseEvent *event)
{
#if 1
    printf("Mat_Viewer close (%s)\n", m_name.c_str());

    g_widgetMap->delWidget(m_name);
#else
    event->ignore();
#endif
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class WorkThread : public QThread
{
public:
    WorkThread(QObject *parent = 0);
    ~WorkThread();

    int setArg(void *arg) {
        m_arg = arg;
        return 0;
    }

protected:
    void run();

protected:
    void *m_arg;
};


WorkThread::WorkThread(QObject *parent) : QThread(parent)
{
    return;
}

WorkThread::~WorkThread()
{
    wait();
}

void WorkThread::run()
{
    QT_WORKTHREAD work_thread = (QT_WORKTHREAD) m_arg;

    work_thread(NULL);

    printf("!!! work thread ended !!!\n");
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class BackgroundWidget : public QWidget
{
    //Q_OBJECT

public:
    BackgroundWidget(QWidget *parent = 0);
    ~BackgroundWidget();

    int doCommand(CommandPackage *cmd, int wait=0);

protected:
    void timerEvent(QTimerEvent *event);

protected:
    std::deque<CommandPackage*>     m_cmdQueue;
    int                             m_timerID;

    QMutex                          *m_mux;
};


BackgroundWidget::BackgroundWidget(QWidget *parent) : QWidget(parent)
{
    m_timerID = startTimer(10);

    m_mux = new QMutex(QMutex::NonRecursive);
}


BackgroundWidget::~BackgroundWidget()
{
    killTimer(m_timerID);
    delete m_mux;
}

int BackgroundWidget::doCommand(CommandPackage *cmd, int wait)
{
    cmd->setState(0);

    m_mux->lock();
    m_cmdQueue.push_back(cmd);
    m_mux->unlock();

    if( wait ) {
        while( !cmd->getState() ) {
            tm_sleep(3);
        }
    }

    return 0;
}

void BackgroundWidget::timerEvent(QTimerEvent *event)
{
    CommandPackage  *cmd;

    while( m_cmdQueue.size() > 0 ) {
        // get first command
        m_mux->lock();
        cmd = m_cmdQueue.front();
        m_cmdQueue.pop_front();
        m_mux->unlock();

        //printf("cmd %2d, title = %s, dat = %x\n", cmd->cmdID, cmd->title.c_str(), cmd->dat);

        if( cmd->cmdID == 1 ) {
            Mat_Viewer      *mv;
            Mat             *img;

            img = (Mat*) cmd->dat;
            mv = (Mat_Viewer*) g_widgetMap->getWidget(cmd->title);

            if( mv == NULL ) {
                mv = new Mat_Viewer();
                mv->setAttribute(Qt::WA_DeleteOnClose);
                g_widgetMap->addWidget(cmd->title, mv);

                mv->imshow(cmd->title, *img);
                mv->setWindowTitle(QString(cmd->title.c_str()));
                mv->show();
            } else {
                mv->imshow(cmd->title, *img);
                mv->show();
            }

            cmd->setState(1);            
        } else if( cmd->cmdID == 2 ) {
            PCD_Viewer      *pcv;

            pcv = (PCD_Viewer*) g_widgetMap->getWidget(cmd->title);

            if( pcv == NULL ) {
                pcv = new PCD_Viewer();
                pcv->setAttribute(Qt::WA_DeleteOnClose);
                g_widgetMap->addWidget(cmd->title, pcv);

                pcv->setWindowTitle(QString(cmd->title.c_str()));
                pcv->show();
            }

            cmd->dat = pcv;
            cmd->setState(1);
        } else if ( cmd->cmdID == 3 ) {
            PCD_Viewer      *pcv;

            if( cmd->dat != NULL ) {
                pcv = (PCD_Viewer *) cmd->dat;

                pcv->redraw_();
            } else {
                pcv = (PCD_Viewer*) g_widgetMap->getWidget(cmd->title);

                if( pcv != NULL )
                    pcv->redraw_();
            }

            cmd->setState(1);
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int _send_command(CommandPackage *cmd, int wait)
{
    return g_backgroundWidget->doCommand(cmd, wait);
}

void *qt_workthread(void *arg)
{
    QT_WORKTHREAD   work_thread = (QT_WORKTHREAD) arg;

    return work_thread(NULL);
}

int qt_start(int argc, char *argv[],
             QT_WORKTHREAD workThread_func,
             QT_WORKTHREAD user_func)
{
    int ret;

    // create app obj
    QApplication app(argc, argv);

    app.setQuitOnLastWindowClosed(1);

    // create event queue
    g_eventQueue = new EventQueue();

    // create background widget
    BackgroundWidget bw;
    g_backgroundWidget = &bw;
    g_backgroundWidget->showMinimized();
    //g_backgroundWidget->resize(1, 1);
    g_backgroundWidget->hide();

    // create widget map
    g_widgetMap = new WidgetMap;

    // call user setup function (in GUI thread)
    if( user_func != NULL ) {
        user_func(NULL);
    }

    // create work thread
    WorkThread wt;
    wt.setArg((void*) workThread_func);
    wt.start();

    // begin main loop
    ret = app.exec();

    printf("\n!!! GUI thread stopped !!!\n");

    // terminate work thread
    wt.terminate();

    // free resources
    delete g_widgetMap;
    delete g_eventQueue;

    return 0;
}

int qt_exit(int code)
{
    qApp->exit(code);
    return 0;
}



int imshow_qt(const char *name, cv::Mat &img)
{
    CommandPackage  cmd;

    cmd.cmdID = 1;
    cmd.title = name;
    cmd.dat   = &img;

    g_backgroundWidget->doCommand(&cmd, 1);

    return 0;
}

int pcv_qt(const char *name, PCD_Viewer **viewer)
{
    CommandPackage  cmd;

    cmd.cmdID = 2;
    cmd.title = name;

    g_backgroundWidget->doCommand(&cmd, 1);

    *viewer = (PCD_Viewer*) cmd.dat;

    return 0;
}

int pcv_redraw_qt(const char *name)
{
    CommandPackage  cmd;

    cmd.cmdID = 3;
    cmd.title = name;
    cmd.dat   = NULL;

    g_backgroundWidget->doCommand(&cmd, 1);

    return 0;
}

int pcv_redraw_qt(PCD_Viewer *viewer)
{
    CommandPackage  cmd;

    cmd.cmdID = 3;
    cmd.title = "";
    cmd.dat   = viewer;

    g_backgroundWidget->doCommand(&cmd, 1);
}

int waitKey_qt(int delay)
{
    InputEvent  ev;
    ru64        t;

    t = tm_get_millis();

    if( delay == 0 ) {
        g_eventQueue->clear(t);

        while(g_eventQueue->size() == 0 ) {
            tm_sleep(5);
        }

        g_eventQueue->pop(ev);
        return ev.code;
    } else {
        ru64        t1, t2, dt, te;

        te = delay;
        g_eventQueue->clear(t);

        t1 = tm_get_millis();

        while(g_eventQueue->size() == 0 ) {
            tm_sleep(5);

            t2 = tm_get_millis();
            dt = t2 - t1;
            if( dt > te ) return -1;
        }

        g_eventQueue->pop(ev);
        return ev.code;
    }
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct run_parallel_data
{
    QT_WORKTHREAD   func;
    void            *args;
};

void* run_parallel_func(void *arg)
{
    run_parallel_data   *dat = (run_parallel_data*) arg;

    dat->func(dat->args);

    return NULL;
}

void* run_parallel_func(void *args, QT_WORKTHREAD func)
{
    pthread_t           *t_id;
    run_parallel_data   dat;

    t_id = new pthread_t;

    dat.args = args;
    dat.func = func;

    pthread_create(t_id, NULL, func, &dat);

    return t_id;
}

int run_parallel_join(void *dat)
{
    void            *ret;
    pthread_t       *t_id = (pthread_t*) dat;

    pthread_join(*t_id, &ret);

    return 0;
}
