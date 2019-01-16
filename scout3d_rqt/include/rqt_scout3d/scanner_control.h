#ifndef RQT_SCOUT3D_SCANNER_CONTROL_H
#define RQT_SCOUT3D_SCANNER_CONTROL_H

#include "scanner_control_widget.h"
#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <QTimer>

namespace rqt_scout3d {

class ScannerControl : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
    ScannerControl();

public:
    void initPlugin(qt_gui_cpp::PluginContext& context);
    void shutdownPlugin();
    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private:
    ScannerControlWidget* widget_;
};

} // namespace

#endif // RQT_SCOUT3D_SCANNER_CONTROL_H
