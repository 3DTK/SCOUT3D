#include "rqt_scout3d/scanner_control.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace rqt_scout3d {

ScannerControl::ScannerControl()
    : rqt_gui_cpp::Plugin()
    , widget_(nullptr)
{
    setObjectName("ScannerControl");
}

void ScannerControl::initPlugin(qt_gui_cpp::PluginContext& context)
{
    QStringList argv = context.argv();
    widget_ = new ScannerControlWidget();
    context.addWidget(widget_);
}

void ScannerControl::shutdownPlugin()
{
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

} // namespace
PLUGINLIB_EXPORT_CLASS(rqt_scout3d::ScannerControl, rqt_gui_cpp::Plugin)
