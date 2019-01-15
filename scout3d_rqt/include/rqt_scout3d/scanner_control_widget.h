#ifndef SCANNER_CONTROL_WIDGET_H
#define SCANNER_CONTROL_WIDGET_H

#include <QWidget>

namespace Ui {
class ScannerControlWidget;
}

class ScannerControlWidget : public QWidget
{
    Q_OBJECT

public:
    struct LaserState {
        bool on;
        bool continous;
        float power[2];
    };

public:
    explicit ScannerControlWidget(QWidget *parent = nullptr);
    ~ScannerControlWidget();

signals:
    void laserStateChanged(ScannerControlWidget::LaserState state);

public slots:
    void handle_checkBoxLaser();
    void handle_sliderLaser0();
    void handle_sliderLaser1();
    void handle_sliderMotorSetpoint();
    void handle_groupBoxImageBright();
    void handle_groupBoxImageDark();

private:
    Ui::ScannerControlWidget* ui;
    LaserState laserState_;
};

#endif // SCANNER_CONTROL_WIDGET_H
