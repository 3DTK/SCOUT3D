#include "rqt_scout3d/scanner_control_widget.h"
#include "ui_scanner_control_widget.h"
#include<scout3d_scanner/ScanCommand.h>
#include <scout3d_laser/LaserPowerCommand.h>
#include <scout3d_motor/MotorPositionCommand.h>
#include <scout3d_motor/LightCommand.h>
#include <QDebug>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <iomanip>
#include <QProcess>

ScannerControlWidget::ScannerControlWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ScannerControlWidget),
    motorPositionReceived_(false),
    imageColorIndex_(0),
    imageCalibrationIndex_(0),
    calibrationState_(CalibrationState::bright)
{
    ui->setupUi(this);

    connect(ui->checkBoxLaserOn, SIGNAL(stateChanged(int)), this, SLOT(handle_checkBoxLaser()));
    connect(ui->checkBoxLaserContinous, SIGNAL(stateChanged(int)), this, SLOT(handle_checkBoxLaser()));
    connect(ui->checkBoxLaserEqualPower, SIGNAL(stateChanged(int)), this, SLOT(handle_checkBoxLaser()));

    connect(ui->groupBoxImageBright, SIGNAL(clicked()), this, SLOT(handle_groupBoxImageBright()));
    connect(ui->groupBoxImageDark, SIGNAL(clicked()), this, SLOT(handle_groupBoxImageDark()));

    connect(ui->sliderLaser0, SIGNAL(valueChanged(int)), this, SLOT(handle_sliderLaser0()));
    connect(ui->sliderLaser1, SIGNAL(valueChanged(int)), this, SLOT(handle_sliderLaser1()));

    connect(ui->sliderLightBrightness, SIGNAL(valueChanged(int)), this, SLOT(handle_lightBrightness()));
    connect(ui->checkBoxLightOn, SIGNAL(stateChanged(int)), this, SLOT(handle_lightBrightness()));

    connect(ui->buttonMotorZero, SIGNAL(clicked()), this, SLOT(handle_buttonMotorZero()));

    ui->labelLaser0->setText(QString::number(ui->sliderLaser0->value() / 100.0f, 'f', 2));
    ui->labelLaser1->setText(QString::number(ui->sliderLaser1->value() / 100.0f, 'f', 2));

    ui->sliderMotorSetpoint->setDisabled(true);
    motorMessageSubscriber_ = nh_.subscribe("/motor/motorPosition", 1, &ScannerControlWidget::motorPositionCallback, this);

    ui->checkBoxLaserOn->setCheckState(Qt::CheckState::Unchecked);
    sendLaserCommand();

    ui->SpinBoxImageFramerate->setValue(getFramerate());
    updateCameraParameters();
    connect(ui->SpinBoxImageFramerate, SIGNAL(valueChanged(double)), this, SLOT(updateCameraParameters()));
    connect(ui->SpinBoxImageBrightShutter, SIGNAL(valueChanged(double)), this, SLOT(updateCameraParameters()));
    connect(ui->SpinBoxImageBrightGain, SIGNAL(valueChanged(double)), this, SLOT(updateCameraParameters()));
    connect(ui->SpinBoxImageDarkShutter, SIGNAL(valueChanged(double)), this, SLOT(updateCameraParameters()));
    connect(ui->SpinBoxImageDarkGain, SIGNAL(valueChanged(double)), this, SLOT(updateCameraParameters()));
    connect(ui->buttonImageCapture, SIGNAL(clicked()), this, SLOT(handle_buttonImageCapture()));
    connect(ui->buttonImageCaptureCalibration, SIGNAL(clicked()), this, SLOT(handle_buttonImageCaptureCalibration()));

    connect(ui->buttonScan, SIGNAL(clicked()), this, SLOT(handle_buttonScan()));
    connect(ui->buttonDownload, SIGNAL(clicked()), this, SLOT(handle_buttonDownload()));
}

ScannerControlWidget::~ScannerControlWidget()
{
    motorMessageSubscriber_.shutdown();

    ui->checkBoxLaserOn->setCheckState(Qt::CheckState::Unchecked);
    sendLaserCommand();

    delete ui;
}

void ScannerControlWidget::handle_checkBoxLaser()
{
    if (ui->checkBoxLaserEqualPower->checkState() == Qt::CheckState::Checked) {
        if (ui->sliderLaser0->value() < ui->sliderLaser1->value()) {
            ui->sliderLaser1->setValue(ui->sliderLaser0->value());
        } else if (ui->sliderLaser1->value() < ui->sliderLaser0->value()) {
            ui->sliderLaser0->setValue(ui->sliderLaser1->value());
        }
    }

    sendLaserCommand();
}

void ScannerControlWidget::handle_sliderLaser0()
{
    if (ui->checkBoxLaserEqualPower->checkState() == Qt::CheckState::Checked) {
        ui->sliderLaser1->setValue(ui->sliderLaser0->value());
    }

    ui->labelLaser0->setText(QString::number(ui->sliderLaser0->value() / 100.0f, 'f', 2));
    ui->labelLaser1->setText(QString::number(ui->sliderLaser1->value() / 100.0f, 'f', 2));

    sendLaserCommand();
}

void ScannerControlWidget::handle_sliderLaser1()
{
    if (ui->checkBoxLaserEqualPower->checkState() == Qt::CheckState::Checked) {
        ui->sliderLaser0->setValue(ui->sliderLaser1->value());
    }

    ui->labelLaser0->setText(QString::number(ui->sliderLaser0->value() / 100.0f, 'f', 2));
    ui->labelLaser1->setText(QString::number(ui->sliderLaser1->value() / 100.0f, 'f', 2));

    sendLaserCommand();
}

void ScannerControlWidget::handle_lightBrightness()
{
    float value = 0;
    if (ui->checkBoxLightOn->checkState() == Qt::CheckState::Checked) {
        value =ui->sliderLightBrightness->value() / 100.0f;
    }

    scout3d_motor::LightCommand command;
    command.request.brightness = value;
    ros::service::call("/motor/setLight", command);
}

void ScannerControlWidget::handle_sliderMotorSetpoint()
{
    double value = ui->sliderMotorSetpoint->value();
    ui->labelMotorSetpoint->setText(QString::number(value, 'f', 2));

    scout3d_motor::MotorPositionCommand command;
    command.request.position = value * M_PI / 180.0;
    ros::service::call("/motor/setMotorPosition", command);
}

void ScannerControlWidget::handle_groupBoxImageBright()
{
    ui->groupBoxImageDark->setChecked(!ui->groupBoxImageBright->isChecked());

    updateCameraParameters();
}

void ScannerControlWidget::handle_groupBoxImageDark()
{
    ui->groupBoxImageBright->setChecked(!ui->groupBoxImageDark->isChecked());

    updateCameraParameters();
}

void ScannerControlWidget::handle_buttonMotorZero()
{
    double value = 0;
    ui->sliderMotorSetpoint->setValue(value);
    ui->labelMotorSetpoint->setText(QString::number(value, 'f', 2));

    scout3d_motor::MotorPositionCommand command;
    command.request.position = value * M_PI / 180.0;
    ros::service::call("/motor/setMotorZero", command);
}

void ScannerControlWidget::handle_buttonScan()
{
    ui->groupBoxImageDark->setChecked(true);
    ui->groupBoxImageBright->setChecked(false);
    ui->checkBoxLaserOn->setCheckState(Qt::CheckState::Unchecked);
    ui->checkBoxLaserContinous->setCheckState(Qt::CheckState::Unchecked);

    scout3d_scanner::ScanCommand command;
    command.request.start_angle = ui->spinBoxScanStartAngle->value() * M_PI / 180.0;
    command.request.stop_angle = ui->spinBoxScanStopAngle->value() * M_PI / 180.0;
    command.request.laser0_power = ui->sliderLaser0->value() / 100.0f;
    command.request.laser1_power = ui->sliderLaser1->value() / 100.0f;
    command.request.camera_shutter = ui->SpinBoxImageDarkShutter->value();
    command.request.camera_gain = ui->SpinBoxImageDarkGain->value();

    ros::service::call("/startScan", command);
}

void ScannerControlWidget::handle_buttonDownload()
{
    QProcess* process = new QProcess(this);
    process->start("xdg-open", QStringList() << "http://scout3d-camera0.local/bag/");
}

void ScannerControlWidget::handle_buttonImageCapture()
{
    ui->buttonImageCapture->setDisabled(true);
    ui->buttonImageCaptureCalibration->setDisabled(true);

    imageSubscriber_ = nh_.subscribe("/camera/image_raw", 1, &ScannerControlWidget::imageColorCallback, this);
}

void ScannerControlWidget::handle_buttonImageCaptureCalibration()
{
    switch (calibrationState_) {

    case CalibrationState::bright:
    {
        ui->buttonImageCapture->setDisabled(true);
        ui->buttonImageCaptureCalibration->setDisabled(true);

        scout3d_laser::LaserPowerCommand command;
        command.request.laser_mode = 1;
        command.request.green_power = 0;
        command.request.blue_power = 0;
        ros::service::call("/laser/setLaserPower", command);

        ui->groupBoxImageBright->setChecked(true);
        ui->groupBoxImageDark->setChecked(false);
        updateCameraParameters();

        calibrationState_ = CalibrationState::bright_capture;
        calibrationTimer_.singleShot(0, this, SLOT(handle_buttonImageCaptureCalibration()));
        break;
    }

    case CalibrationState::bright_capture:
    {
        sleep(1);
        imageCalibrationSubscriber_ = nh_.subscribe("/camera/image_raw", 1, &ScannerControlWidget::imageCalibrationCallback, this);
        break;
    }

    case CalibrationState::dark:
    {
        ui->groupBoxImageBright->setChecked(false);
        ui->groupBoxImageDark->setChecked(true);
        updateCameraParameters();

        calibrationState_ = CalibrationState::dark_capture;
        calibrationTimer_.singleShot(0, this, SLOT(handle_buttonImageCaptureCalibration()));
        break;
    }

    case CalibrationState::dark_capture:
    {
        sleep(1);
        imageCalibrationSubscriber_ = nh_.subscribe("/camera/image_raw", 1, &ScannerControlWidget::imageCalibrationCallback, this);
        break;
    }

    case CalibrationState::laser0:
    {
        scout3d_laser::LaserPowerCommand command;
        command.request.laser_mode = 1;
        command.request.green_power = ui->sliderLaser0->value() / 100.0f;
        command.request.blue_power = 0;
        ros::service::call("/laser/setLaserPower", command);

        calibrationState_ = CalibrationState::laser0_capture;
        calibrationTimer_.singleShot(0, this, SLOT(handle_buttonImageCaptureCalibration()));
        break;
    }

    case CalibrationState::laser0_capture:
    {
        sleep(1);
        imageCalibrationSubscriber_ = nh_.subscribe("/camera/image_raw", 1, &ScannerControlWidget::imageCalibrationCallback, this);
        break;
    }

    case CalibrationState::laser1:
    {
        scout3d_laser::LaserPowerCommand command;
        command.request.laser_mode = 1;
        command.request.green_power = 0;
        command.request.blue_power = ui->sliderLaser1->value() / 100.0f;
        ros::service::call("/laser/setLaserPower", command);

        calibrationState_ = CalibrationState::laser1_capture;
        calibrationTimer_.singleShot(0, this, SLOT(handle_buttonImageCaptureCalibration()));
        break;
    }

    case CalibrationState::laser1_capture:
    {
        sleep(1);
        imageCalibrationSubscriber_ = nh_.subscribe("/camera/image_raw", 1, &ScannerControlWidget::imageCalibrationCallback, this);
        break;
    }

    case CalibrationState::done:
    {
        ui->groupBoxImageBright->setChecked(true);
        ui->groupBoxImageDark->setChecked(false);
        updateCameraParameters();

        sendLaserCommand();

        ui->buttonImageCapture->setDisabled(false);
        ui->buttonImageCaptureCalibration->setDisabled(false);
        calibrationState_ = CalibrationState::bright;
        break;
    }
    }
}

void ScannerControlWidget::motorPositionCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    double value = msg->position.at(0) * 180.0 / M_PI;

    if (!motorPositionReceived_) {
        ui->sliderMotorSetpoint->setDisabled(false);

        motorPositionReceived_ = true;
        ui->sliderMotorSetpoint->setValue(value);
        ui->labelMotorSetpoint->setText(QString::number(value, 'f', 2));
        connect(ui->sliderMotorSetpoint, SIGNAL(valueChanged(double)), this, SLOT(handle_sliderMotorSetpoint()));
    }

    ui->labelMotorValue->setText(QString::number(value, 'f', 2));
}

void ScannerControlWidget::imageColorCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    boost::filesystem::create_directories(boost::filesystem::path("/tmp/scout3d/capture"));

    std::stringstream ss;
    ss << "/tmp/scout3d/capture/image" << std::setfill('0') << std::setw(3) << imageColorIndex_++ << ".png";

    cv::imwrite(ss.str(), cv_ptr->image);

    imageSubscriber_.shutdown();
    ui->buttonImageCapture->setDisabled(false);
    ui->buttonImageCaptureCalibration->setDisabled(false);
}

void ScannerControlWidget::imageCalibrationCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    boost::filesystem::create_directories(boost::filesystem::path("/tmp/scout3d/calibration"));

    std::stringstream ss;
    if (calibrationState_ == CalibrationState::bright_capture) {
        ss << "/tmp/scout3d/calibration/image" << std::setfill('0') << std::setw(3) << imageCalibrationIndex_ << "_bright.png";
    } else if (calibrationState_ == CalibrationState::dark_capture) {
        ss << "/tmp/scout3d/calibration/image" << std::setfill('0') << std::setw(3) << imageCalibrationIndex_ << "_dark.png";
    } else if (calibrationState_ == CalibrationState::laser0_capture) {
        ss << "/tmp/scout3d/calibration/image" << std::setfill('0') << std::setw(3) << imageCalibrationIndex_ << "_laser0.png";
    } else if (calibrationState_ == CalibrationState::laser1_capture) {
        ss << "/tmp/scout3d/calibration/image" << std::setfill('0') << std::setw(3) << imageCalibrationIndex_ << "_laser1.png";
    }

    cv::imwrite(ss.str(), cv_ptr->image);

    imageCalibrationSubscriber_.shutdown();

    if (calibrationState_ == CalibrationState::bright_capture) {
        calibrationState_ = CalibrationState::dark;
    } else if (calibrationState_ == CalibrationState::dark_capture) {
        calibrationState_ = CalibrationState::laser0;
    } else if (calibrationState_ == CalibrationState::laser0_capture) {
        calibrationState_ = CalibrationState::laser1;
    } else if (calibrationState_ == CalibrationState::laser1_capture) {
        imageCalibrationIndex_++;
        calibrationState_ = CalibrationState::done;
    }

    calibrationTimer_.singleShot(0, this, SLOT(handle_buttonImageCaptureCalibration()));
}

void ScannerControlWidget::sendLaserCommand()
{
    scout3d_laser::LaserPowerCommand command;
    command.request.laser_mode = ui->checkBoxLaserContinous->checkState() == Qt::CheckState::Checked;;
    if (ui->checkBoxLaserOn->checkState() == Qt::CheckState::Checked) {
        command.request.green_power = ui->sliderLaser0->value() / 100.0f;
        command.request.blue_power = ui->sliderLaser1->value() / 100.0f;
    } else {
        command.request.green_power = 0;
        command.request.blue_power = 0;
    }

    ros::service::call("/laser/setLaserPower", command);
}

double ScannerControlWidget::getFramerate()
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;

    ros::service::call("/camera/spinnaker_camera_nodelet/set_parameters", srv_req, srv_resp);

    double frameRate = 0;
    for (const dynamic_reconfigure::DoubleParameter& param : srv_resp.config.doubles) {
        if (param.name.compare("acquisition_frame_rate") == 0) {
            frameRate = param.value;
        }
    }

    return frameRate;
}

void ScannerControlWidget::setCameraParameters(double frameRate, double shutter, double gain)
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "acquisition_frame_rate";
    double_param.value = frameRate;
    conf.doubles.push_back(double_param);

    double_param.name = "exposure_time";
    double_param.value = shutter * 1e6;
    conf.doubles.push_back(double_param);

    double_param.name = "gain";
    double_param.value = gain;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    ros::service::call("/camera/spinnaker_camera_nodelet/set_parameters", srv_req, srv_resp);
}

void ScannerControlWidget::updateCameraParameters()
{
    double shutter = 0;
    double gain = 0;

    if (ui->groupBoxImageBright->isChecked()) {
        shutter = ui->SpinBoxImageBrightShutter->value();
        gain = ui->SpinBoxImageBrightGain->value();
    } else {
        shutter = ui->SpinBoxImageDarkShutter->value();
        gain = ui->SpinBoxImageDarkGain->value();
    }

    setCameraParameters(ui->SpinBoxImageFramerate->value(), shutter, gain);
}
