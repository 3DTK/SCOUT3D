#include "rqt_scout3d/scanner_control.h"
#include <scout3d_laser/LaserPowerCommand.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <ros/ros.h>

namespace rqt_scout3d {

ScannerControl::ScannerControl()
    : rqt_gui_cpp::Plugin()
    , widget_(nullptr)
{
    // Constructor is called first before initPlugin function, needless to say.

    // give QObjects reasonable names
    setObjectName("ScannerControl");

    ScannerControlWidget::LaserState state;
    state.on = false;
    updateLaserState(state);
}

void ScannerControl::initPlugin(qt_gui_cpp::PluginContext& context)
{
    QStringList argv = context.argv();
    widget_ = new ScannerControlWidget();
    context.addWidget(widget_);

    connect(widget_, SIGNAL(laserStateChanged(ScannerControlWidget::LaserState)), this, SLOT(updateLaserState(ScannerControlWidget::LaserState)));
}

void ScannerControl::shutdownPlugin()
{
    // TODO unregister all publishers here
}

void ScannerControl::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    // TODO save intrinsic configuration, usually using:
    // instance_settings.setValue(k, v)
}

void ScannerControl::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    // TODO restore intrinsic configuration, usually using:
    // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

void ScannerControl::updateLaserState(ScannerControlWidget::LaserState state)
{
    scout3d_laser::LaserPowerCommand command;
    command.request.laser_mode = state.continous;
    if (state.on) {
        command.request.green_power = state.power[0];
        command.request.blue_power = state.power[1];
    } else {
        command.request.green_power = 0;
        command.request.blue_power = 0;
    }

    ros::service::call("/laser/setLaserPower", command);
}

} // namespace
PLUGINLIB_EXPORT_CLASS(rqt_scout3d::ScannerControl, rqt_gui_cpp::Plugin)
