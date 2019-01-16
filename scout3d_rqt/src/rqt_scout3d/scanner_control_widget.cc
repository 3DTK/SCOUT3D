#include "rqt_scout3d/scanner_control_widget.h"
#include "ui_scanner_control_widget.h"
#include <scout3d_laser/LaserPowerCommand.h>
#include <scout3d_motor/MotorPositionCommand.h>
#include <QDebug>

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
    motorMessageSubscriber_ = nh_.subscribe("/motor/motorPosition", 1, &ScannerControlWidget::motorPositionCallback, this);

    ui->checkBoxLaserOn->setCheckState(Qt::CheckState::Unchecked);
    sendLaserCommand();
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
    command.request.position = value;
    ros::service::call("/motor/setMotorPosition", command);
}

void ScannerControlWidget::handle_groupBoxImageBright()
{
    ui->groupBoxImageDark->setChecked(!ui->groupBoxImageBright->isChecked());
}

void ScannerControlWidget::handle_groupBoxImageDark()
{
    ui->groupBoxImageBright->setChecked(!ui->groupBoxImageDark->isChecked());
}

void ScannerControlWidget::handle_buttonMotorZero()
{
    double value = 0;
    ui->sliderMotorSetpoint->setValue(value);
    ui->labelMotorSetpoint->setText(QString::number(value, 'f', 2));

    scout3d_motor::MotorPositionCommand command;
    command.request.position = value;
    ros::service::call("/motor/setMotorZero", command);
}

void ScannerControlWidget::motorPositionCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    double value = msg->position.at(0) * 180.0 / M_PI;

    if (!motorPositionReceived_) {
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
