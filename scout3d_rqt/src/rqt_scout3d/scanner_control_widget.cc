#include "rqt_scout3d/scanner_control_widget.h"
#include "ui_scanner_control_widget.h"
#include <scout3d_laser/LaserPowerCommand.h>
#include <scout3d_motor/MotorPositionCommand.h>
#include <QDebug>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

ScannerControlWidget::ScannerControlWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ScannerControlWidget)
{
    ui->setupUi(this);

    connect(ui->checkBoxLaserOn, SIGNAL(stateChanged(int)), this, SLOT(handle_checkBoxLaser()));
    connect(ui->checkBoxLaserContinous, SIGNAL(stateChanged(int)), this, SLOT(handle_checkBoxLaser()));
    connect(ui->checkBoxLaserEqualPower, SIGNAL(stateChanged(int)), this, SLOT(handle_checkBoxLaser()));

    connect(ui->groupBoxImageBright, SIGNAL(clicked()), this, SLOT(handle_groupBoxImageBright()));
    connect(ui->groupBoxImageDark, SIGNAL(clicked()), this, SLOT(handle_groupBoxImageDark()));

    connect(ui->sliderLaser0, SIGNAL(valueChanged(int)), this, SLOT(handle_sliderLaser0()));
    connect(ui->sliderLaser1, SIGNAL(valueChanged(int)), this, SLOT(handle_sliderLaser1()));

    connect(ui->buttonMotorZero, SIGNAL(clicked()), this, SLOT(handle_buttonMotorZero()));

    ui->labelLaser0->setText(QString::number(ui->sliderLaser0->value() / 100.0f, 'f', 2));
    ui->labelLaser1->setText(QString::number(ui->sliderLaser1->value() / 100.0f, 'f', 2));

    motorPositionReceived_ = false;
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

    ros::service::call("/camera/camera_nodelet/set_parameters", srv_req, srv_resp);

    double frameRate = 0;
    for (const dynamic_reconfigure::DoubleParameter& param : srv_resp.config.doubles) {
        if (param.name.compare("frame_rate") == 0) {
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

    double_param.name = "frame_rate";
    double_param.value = frameRate;
    conf.doubles.push_back(double_param);

    double_param.name = "shutter_speed";
    double_param.value = shutter;
    conf.doubles.push_back(double_param);

    double_param.name = "gain";
    double_param.value = gain;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    ros::service::call("/camera/camera_nodelet/set_parameters", srv_req, srv_resp);
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
