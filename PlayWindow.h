#ifndef PLAYWINDOW_H
#define PLAYWINDOW_H

#include <QMainWindow>
#include <pcl/visualization/pcl_visualizer.h> // viewer
#include <vtkRenderWindow.h>        // qvtk
#include <pcl/common/transforms.h>  // transfromation
#include <pcl/filters/filter.h> // remove nan
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/elch.h>//elch

#include <adafruit.h>
#include <kinect.h>

namespace Ui {
class PlayWindow;
}

#include <pcl/registration/lum.h>
#include <pcl/registration/correspondence_estimation.h>

class PlayWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit PlayWindow(QWidget *parent = 0);
    ~PlayWindow();

    // pcl 3D Viewer
    pcl::visualization::PCLVisualizer::Ptr viewer;

private slots:

    void ShowPC(PointCloudT::Ptr PC);

    // AdaFruit
    void on_pushButton_Cal_clicked();
    void on_pushButton_Quat_clicked();
    void on_pushButton_Open_clicked();
    void on_pushButton_Close_clicked();
    void on_std_send_clicked();



    // Kinect
    void on_pushButton_Kin_Close_clicked();
    void on_pushButton_Kin_Open_clicked();
    void on_pushButton_Kin_Grab_clicked();
    void on_pushButton_Kin_Play_clicked();
    void on_pushButton_Kin_Stop_clicked();




    void on_pushButton_CleanViewer_clicked();


    void on_checkBox_Ada_clicked(bool b);


private:
    Ui::PlayWindow *ui;
    Adafruit * ada;
    Kinect * kin;
    QThread * t;
};

#endif // PLAYWINDOW_H
