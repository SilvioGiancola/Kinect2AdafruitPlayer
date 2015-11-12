#ifndef KINECT_H
#define KINECT_H

#include <QWidget>

#include <Eigen/Dense>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>  // transfromation

#include <iostream>
#include <QTime>
#include <QDateTime>
#include <QFile>
#include <QTextStream>
#include <QThread>


#include <adafruit.h>;
using namespace std;

#define SUCCESS 0
#define ERR_KIN_OPEN 1

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Kinect : public QThread
{
    Q_OBJECT
public:
    // Constructor
    explicit Kinect(QWidget *parent = 0);
    ~Kinect();

    void run();

    // Opening function
    int Open(int i = 0);

    // Closing function
    int Close();

    // Grabing function
    int GrabPointCloud();

    void Play();

    void Stop();


    // Opening verification function
    bool isOpen() {return _open;}
    bool _play;
    bool _save;

    bool UseAdaFruit = false;

    Adafruit * ada;

signals:

    void GrabbedPC(PointCloudT::Ptr PC);

private:
    // freenect2 stuff
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::Registration *registration;
    libfreenect2::BasePacketPipeline *pipeline;

    // check opening
    bool _open;

    // current serial of Kinect
    std::string _serial;
    Eigen::Matrix4f _KinectPose;


};

#endif // KINECT_H
