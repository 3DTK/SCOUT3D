#ifndef SCANNER_CONTROL_WIDGET_H
#define SCANNER_CONTROL_WIDGET_H

#include <QWidget>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace Ui {
class ScannerControlWidget;
}

class ScannerControlWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ScannerControlWidget(QWidget *parent = nullptr);
    ~ScannerControlWidget();

private slots:
    void handle_checkBoxLaser();
    void handle_sliderLaser0();
    void handle_sliderLaser1();
    void handle_sliderMotorSetpoint();
    void handle_groupBoxImageBright();
    void handle_groupBoxImageDark();
    void handle_buttonMotorZero();
    void updateCameraParameters();

private:
    void motorPositionCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void sendLaserCommand();
    void setFramerate(double frameRate);
    double getFramerate();
    void setCameraParameters(double frameRate, double shutter, double gain);

private:
    Ui::ScannerControlWidget* ui;
    ros::NodeHandle nh_;
    ros::Subscriber motorMessageSubscriber_;
    bool motorPositionReceived_;
};

#endif // SCANNER_CONTROL_WIDGET_H
