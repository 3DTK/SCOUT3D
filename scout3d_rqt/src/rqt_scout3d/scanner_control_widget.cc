#include "rqt_scout3d/scanner_control_widget.h"
#include "ui_scanner_control_widget.h"
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
    connect(ui->sliderMotorSetpoint, SIGNAL(valueChanged(double)), this, SLOT(handle_sliderMotorSetpoint()));

    laserState_.on = ui->checkBoxLaserOn->checkState() == Qt::CheckState::Checked;
    laserState_.continous = ui->checkBoxLaserContinous->checkState() == Qt::CheckState::Checked;
    laserState_.power[0] = ui->sliderLaser0->value() / 100.0f;
    laserState_.power[1] = ui->sliderLaser1->value() / 100.0f;

    ui->labelLaser0->setText(QString::number(laserState_.power[0], 'f', 2));
    ui->labelLaser1->setText(QString::number(laserState_.power[1], 'f', 2));

    double value = 0;
    ui->labelMotorValue->setText(QString::number(value, 'f', 2));

    double setpoint = ui->sliderMotorSetpoint->value();
    ui->labelMotorSetpoint->setText(QString::number(setpoint, 'f', 2));
}

ScannerControlWidget::~ScannerControlWidget()
{
    laserState_.on = false;
    emit laserStateChanged(laserState_);

    delete ui;
}

void ScannerControlWidget::handle_checkBoxLaser()
{
    laserState_.on = ui->checkBoxLaserOn->checkState() == Qt::CheckState::Checked;
    laserState_.continous = ui->checkBoxLaserContinous->checkState() == Qt::CheckState::Checked;
    if (ui->checkBoxLaserEqualPower->checkState() == Qt::CheckState::Checked) {
        if (ui->sliderLaser0->value() < ui->sliderLaser1->value()) {
            ui->sliderLaser1->setValue(ui->sliderLaser0->value());
        } else if (ui->sliderLaser1->value() < ui->sliderLaser0->value()) {
            ui->sliderLaser0->setValue(ui->sliderLaser1->value());
        }
    }

    laserState_.power[0] = ui->sliderLaser0->value() / 100.0f;
    laserState_.power[1] = ui->sliderLaser1->value() / 100.0f;

    emit laserStateChanged(laserState_);
}

void ScannerControlWidget::handle_sliderLaser0()
{
    laserState_.power[0] = ui->sliderLaser0->value() / 100.0f;

    if (ui->checkBoxLaserEqualPower->checkState() == Qt::CheckState::Checked) {
        ui->sliderLaser1->setValue(laserState_.power[0] * 100.0f);
    }

    ui->labelLaser0->setText(QString::number(laserState_.power[0], 'f', 2));

    emit laserStateChanged(laserState_);
}

void ScannerControlWidget::handle_sliderLaser1()
{
    laserState_.power[1] = ui->sliderLaser1->value() / 100.0f;

    if (ui->checkBoxLaserEqualPower->checkState() == Qt::CheckState::Checked) {
        ui->sliderLaser0->setValue(laserState_.power[1] * 100.0f);
    }

    ui->labelLaser1->setText(QString::number(laserState_.power[1], 'f', 2));

    emit laserStateChanged(laserState_);
}

void ScannerControlWidget::handle_sliderMotorSetpoint()
{
    double setpoint = ui->sliderMotorSetpoint->value();
    ui->labelMotorSetpoint->setText(QString::number(setpoint, 'f', 2));
}

void ScannerControlWidget::handle_groupBoxImageBright()
{
    ui->groupBoxImageDark->setChecked(!ui->groupBoxImageBright->isChecked());
}

void ScannerControlWidget::handle_groupBoxImageDark()
{
    ui->groupBoxImageBright->setChecked(!ui->groupBoxImageDark->isChecked());
}
