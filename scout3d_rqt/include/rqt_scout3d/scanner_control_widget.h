#ifndef SCANNER_CONTROL_WIDGET_H
#define SCANNER_CONTROL_WIDGET_H

#include <QWidget>
#include <QTimer>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>

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
    void handle_lightBrightness();
    void handle_sliderMotorSetpoint();
    void handle_groupBoxImageBright();
    void handle_groupBoxImageDark();
    void handle_buttonMotorZero();
    void handle_buttonImageCapture();
    void handle_buttonImageCaptureCalibration();
    void handle_buttonScan();
    void updateCameraParameters();

private:
    void motorPositionCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void imageColorCallback(const sensor_msgs::Image::ConstPtr& msg);
    void imageCalibrationCallback(const sensor_msgs::Image::ConstPtr& msg);
    void sendLaserCommand();
    void setFramerate(double frameRate);
    double getFramerate();
    void setCameraParameters(double frameRate, double shutter, double gain);

private:
    Ui::ScannerControlWidget* ui;
    ros::NodeHandle nh_;
    ros::Subscriber motorMessageSubscriber_;
    ros::Subscriber imageSubscriber_;
    ros::Subscriber imageCalibrationSubscriber_;
    bool motorPositionReceived_;
    long imageColorIndex_;
    long imageCalibrationIndex_;
    enum CalibrationState { bright, bright_capture, dark, dark_capture, laser0, laser0_capture, laser1, laser1_capture, done } calibrationState_;
    QTimer calibrationTimer_;
};

#endif // SCANNER_CONTROL_WIDGET_H
