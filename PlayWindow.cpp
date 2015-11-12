#include "PlayWindow.h"
#include "ui_PlayWindow.h"

PlayWindow::PlayWindow(QWidget *parent) :    QMainWindow(parent),    ui(new Ui::PlayWindow)
{
    ui->setupUi(this);
    kin = new Kinect(this);
    ada = kin->ada;

    // Create 3D Viewer
    viewer.reset(new pcl::visualization::PCLVisualizer("Viewer",false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());

    viewer->addCoordinateSystem(1.0);
    // on_pushButton_CleanViewer_clicked();
    viewer->setCameraPosition(-3.5,1,1, // mi posiziono dietro ad un Kinect
                              0.5,0.5,0.5, // guardo un punto centrale
                              0,0,1);   // orientato con la z verso l'alto
    viewer->setCameraClipDistances(-10,10);
    viewer->setBackgroundColor (0.5, 0.5, 0.5);
    ui->qvtkWidget->update ();


    // this->moveToThread(t);

    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(),ui->qvtkWidget->GetRenderWindow());
    viewer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);


    connect(kin,SIGNAL(GrabbedPC(PointCloudT::Ptr)), this, SLOT(ShowPC(PointCloudT::Ptr)), Qt::DirectConnection);

}

PlayWindow::~PlayWindow()
{
    delete ui;
}




/**********
 * Viewer *
 * *********/
boost::mutex updateModelMutex;
std::vector<int> times;
void PlayWindow::ShowPC(PointCloudT::Ptr PC)
{
    updateModelMutex.lock();


    if (ui->checkBox_ShowPC->isChecked())
    {

        if (!viewer->updatePointCloud(PC))
            viewer->addPointCloud(PC);

        Eigen::Matrix4f currentPose = Eigen::Matrix4f::Identity();
        currentPose.block(0,0,3,3) = PC->sensor_orientation_.matrix();
        currentPose.block(0,3,3,1) = PC->sensor_origin_.head(3);

        viewer->updatePointCloudPose("cloud",Eigen::Affine3f(currentPose) );

    }



    ui->qvtkWidget->update ();


    updateModelMutex.unlock();



    // Timing
    times.push_back(QDateTime::currentDateTime().toMSecsSinceEpoch() - PC->header.stamp);

    double mean;
    if (times.size() > 60)
    {
        mean = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
        times.erase(times.begin());
    }

    if (ui->checkBox_SavePC->isChecked())
        pcl::io::savePCDFileASCII(PC->header.frame_id, *PC);


    std::vector<int> ind;
    pcl::removeNaNFromPointCloud(*PC, *PC, ind);
    ui->label_nPoint->setText(QString("Nb Point : %1").arg(PC->size()));
    ui->label_time->setText(QString("Time (ms) : %1").arg(mean));
    ui->label_fps->setText(QString("FPS (Hz) : %1").arg(1000/mean));



    return;
}

/****************
 * KINECT *
 ***************/
void PlayWindow::on_pushButton_Kin_Open_clicked()
{
    kin->Open();
    return;
}

void PlayWindow::on_pushButton_Kin_Close_clicked()
{
    kin->Close();
    return;
}

void PlayWindow::on_pushButton_Kin_Play_clicked()
{
    ui->qvtkWidget->setEnabled(false);
    ui->pushButton_Kin_Grab->setEnabled(false);
    kin->Play();
    return;
}

void PlayWindow::on_pushButton_Kin_Stop_clicked()
{
    ui->qvtkWidget->setEnabled(true);
    ui->pushButton_Kin_Grab->setEnabled(true);
    kin->Stop();
    return;
}




void PlayWindow::on_pushButton_Kin_Grab_clicked()
{
    kin->GrabPointCloud();
    //  PointCloudT::Ptr PC(new PointCloudT);
    //PC = kin->GrabPointCloud();

    /*  pcl::visualization::PointCloudColorHandlerRGBField<PointT> single_color(PC);
    viewer->removePointCloud("Grab");

    viewer->addPointCloud<PointT>(PC, single_color, "Grab");

    ui->qvtkWidget->update ();

    std::vector<int> ind;
    pcl::removeNaNFromPointCloud(*PC, *PC, ind);
    ui->label_nPoint->setText(QString::number(PC->size()));*/
}

void PlayWindow::on_pushButton_CleanViewer_clicked()
{
    viewer->removeAllPointClouds();
    // viewer->removeAllCoordinateSystems();
    // viewer->addCoordinateSystem(1.0);
    ui->qvtkWidget->update ();


    return;
}





/****************
 * ADAFRUIT *
****************/
void PlayWindow::on_pushButton_Open_clicked()
{
    int res = ada->Open();

    if ( res == SUCCESS )               std::cout << "Adafruit is connected and opened!" << std::endl;
    else if ( res == ERR_BAUDRATE )     std::cout << "Error in Baudrate!" << std::endl;
    else if ( res == ERR_DATABITS )     std::cout << "Error in DataBits!" << std::endl;
    else if ( res == ERR_PARITY )       std::cout << "Error in Parity!" << std::endl;
    else if ( res == ERR_STOPBITS )     std::cout << "Error in StopBits!" << std::endl;
    else if ( res == ERR_FLOWCONTROL )  std::cout << "Error in FlowControl!" << std::endl;
    else if ( res == ERR_PLUG )         std::cout << "Adafruit has not been found !" << std::endl;
    else if ( res == ERR_OPEN )         std::cout << "Opening error : " << ada->errorString().toStdString() << " ( try : \"sudo adduser myuser dialout\" and relog) " << std::endl;

    return;
}

void PlayWindow::on_pushButton_Close_clicked()
{
    ada->close();
    std::cout << "Adafruit has been closed" << std::endl;

    return;
}

void PlayWindow::on_pushButton_Cal_clicked()
{
    QString cal = ada->GetCal();
    ui->label_sys->setText(cal);

    return;
}

void PlayWindow::on_pushButton_Quat_clicked()
{
    Eigen::Quaternionf cal = ada->GetQuat();
    ui->label_W->setText(QString::number(cal.w()));
    ui->label_X->setText(QString::number(cal.x()));
    ui->label_Y->setText(QString::number(cal.y()));
    ui->label_Z->setText(QString::number(cal.z()));

    if (ui->checkBox_Save->isChecked())
    {
        QFile file(ui->lineEdit_Path->text().append("Adafruit Results ").append(QDateTime::currentDateTime().toString()));
        file.open(QFile::ReadWrite);
        QString result = QString("%1 %2 %3 %4").arg(cal.w()).arg(cal.x()).arg(cal.y()).arg(cal.z());
        file.write(result.toUtf8());
        file.close();
    }

    return;
}

void PlayWindow::on_std_send_clicked()
{
    ui->std_answer->setText(ada->Ask(ui->std_question->text()));
    return;
}



/****************
 * Main Function *
 * ******************/
#include <QApplication>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PlayWindow w;
    w.showMaximized();

    return a.exec();
}

void PlayWindow::on_checkBox_Ada_clicked(bool b)
{
    kin->UseAdaFruit = b;
}
/*
void PlayWindow::on_std_quat_clicked()
{
     ui->std_answer->setText(ada->Ask(QString("*quat?")));
}*/
