#include "kinect.h"


Kinect::Kinect(QWidget *parent) : QThread(parent)
{
    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

    _KinectPose = Eigen::Matrix4f::Identity();
    _open = false;
    _play = false;

    ada = new Adafruit(parent);

    //listener->onNewFrame();
}

Kinect::~Kinect()
{
    _play = false;
}


// Opening function
int Kinect::Open(int i)
{

    if (_open)
    {
        std::cout << "Device is already open!" << std::endl;
        return ERR_KIN_OPEN ;
    }

    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return ERR_KIN_OPEN;
    }

    _serial = freenect2.getDeviceSerialNumber(i);
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
    pipeline = new libfreenect2::CpuPacketPipeline();
#endif
#endif


    _serial = freenect2.getDeviceSerialNumber(i);

    dev = freenect2.openDevice(_serial,pipeline);

    if(dev == 0)
    {
        std::cout << "no device connected or failure opening the default one!" << std::endl;
        return ERR_KIN_OPEN;
    }

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
    dev->start();


    cout << dev->getColorCameraParams().cx << endl;
    cout << dev->getColorCameraParams().cy << endl;
    cout << dev->getColorCameraParams().fx << endl;
    cout << dev->getColorCameraParams().fy << endl;

    cout << dev->getIrCameraParams().cx << endl;
    cout << dev->getIrCameraParams().cy << endl;
    cout << dev->getIrCameraParams().fx << endl;
    cout << dev->getIrCameraParams().fy << endl;
    cout << dev->getIrCameraParams().k1 << endl;
    cout << dev->getIrCameraParams().k2 << endl;
    cout << dev->getIrCameraParams().k3 << endl;
    cout << dev->getIrCameraParams().p1 << endl;
    cout << dev->getIrCameraParams().p2 << endl;


    // Color
    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

    _open = true;

    return SUCCESS;
}

// Closing function
int Kinect::Close()
{
    if (!_open)
    {
        std::cout << "already closed" << std::endl;
        return ERR_KIN_OPEN;
    }
    // TODO: restarting ir stream doesn't work!
    // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
    dev->stop();
    dev->close();
    _open = false;
    return SUCCESS;
}

// Grabing function
int Kinect::GrabPointCloud()
{

    if (!_open)
    {
        std::cout << "stream not opened" << std::endl;
        return ERR_KIN_OPEN;
    }






     QDateTime timestamp = QDateTime::currentDateTime();

    // Acquire Frames
    libfreenect2::FrameMap frames;
    listener->waitForNewFrame(frames);

   //QDateTime timestamp = QDateTime::currentDateTime();


    if (UseAdaFruit)
    {
        if (!ada->isOpen())
            if (ada->Open() != SUCCESS)
                return ERR_OPEN;

        ada->Question(QString("*quat?"));
    }





    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];



    // Undistort and register frames
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
    registration->apply(rgb,depth,&undistorted,&registered);

    const float *undistorted_data = (float*)undistorted.data;
    const unsigned int *registered_data = (unsigned int*)registered.data;

    listener->release(frames);



    // Initialize my Point Cloud
    //PC.reset(new PointCloudT());
    PointCloudT::Ptr PC(new PointCloudT);
    PC->resize(undistorted.width * undistorted.height); // set the memory size to allocate
    PC->height = undistorted.height;        // set the height
    PC->width = undistorted.width;          // set the width
    PC->is_dense = false;                   // Kinect V2 returns organized and not dense point clouds




    int real_point = 0;
    // Set data into my Point cloud
    for (unsigned int i = 0; i < undistorted.height ;i++)
    {
        for (unsigned int j = 0; j < undistorted.width ;j++)
        {
            int index = i * undistorted.width + j;

            float depth = 0;

            //if (ui->radioButton_Meter->isChecked())
            depth = undistorted_data[index] / 1000.0f;
            // else if (ui->radioButton_MilliMeter->isChecked())
            // depth = undistorted_data[index];

            unsigned int rgba = registered_data[index];

            PointT P;

            if ( depth != 0 && rgba != 0)
            {
                P.x = -depth * (dev->getIrCameraParams().cx - j) / dev->getIrCameraParams().fx;
                P.y =  depth * (dev->getIrCameraParams().cy - i) / dev->getIrCameraParams().fy;
                P.z =  depth;


                P.a = (rgba >> 24) & 0xFF;
                P.r = (rgba >> 16) & 0xFF;
                P.g = (rgba >> 8)  & 0xFF;
                P.b =  rgba        & 0xFF;

                real_point++;
            }
            else
            {
                P.x = P.y = P.z = std::numeric_limits<float>::quiet_NaN();
            }

            PC->at(j,i) = P;
        }
    }







    PC->header.frame_id = QString("/home/silvio/PointClouds/Kinect%1_%2.pcd").arg(_serial.c_str()).arg(timestamp.toString("yyyy-MM-dd-HH:mm:ss:zzz")).toStdString();
    PC->header.stamp = timestamp.toMSecsSinceEpoch();                               // the stamp correspond to the acquisition time

    PC->sensor_origin_ = _KinectPose.block<4,1>(0,3);                               // set the translation of the Kinect
    PC->sensor_orientation_ = Eigen::Quaternionf(_KinectPose.block<3,3>(0,0));      // Set the rotation of the Kinect

    // Check data from AdaFruit
    if (UseAdaFruit)
    {
        Eigen::Matrix4f _AdaPose = Eigen::Matrix4f::Identity();
        QString answer = ada->Answer();
        std::cout << answer.toStdString() << std::endl;
        _AdaPose.block(0,0,3,3) = ada->ConvertToQuat(answer).matrix();

        // Set other parameters to PC (Ref Syst, stamp and ID)
        Eigen::Matrix4f off;
        off << -1, 0, 0, 0,   0, 0, 1, 0,     0, 1, 0, 0,   0, 0, 0, 1;
        Eigen::Matrix4f Real_Pose = _AdaPose * off;


        PC->sensor_origin_ = Real_Pose.block<4,1>(0,3);                               // set the translation of the Kinect
        PC->sensor_orientation_ = Eigen::Quaternionf(Real_Pose.block<3,3>(0,0));      // Set the rotation of the Kinect
    }



    std::cout << PC->sensor_orientation_.matrix() << std::endl << std::endl;




    //  std::cout << QDateTime::currentDateTime().toMSecsSinceEpoch() - PC->header.stamp << std::endl;




    //pcl::transformPointCloud (*PC, *PC, _KinectPose);


    emit GrabbedPC(PC);

    return SUCCESS;
}

void Kinect::Play()
{
    std::cout << "Play" << std::endl;
    _play = true;
    this->start();
}

void Kinect::Stop()
{
    std::cout << "Stop" << std::endl;
    _play = false;
}

void Kinect::run()
{
    while (_play)
    {
        GrabPointCloud();
        this->usleep(1000);


        /* times.push_back(t.elapsed());
        if (times.size()>100)
        {
            std::cout << " Moving Mean Freq is :" << 60.0 / ( ( times.at(59) - times.at(0) ) / 1000.0) << std::endl;
            times.pop_front();
        }*/
    }
}
