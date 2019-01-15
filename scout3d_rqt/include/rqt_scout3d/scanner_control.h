#ifndef RQT_SCOUT3D_SCANNER_CONTROL_H
#define RQT_SCOUT3D_SCANNER_CONTROL_H

#include <rqt_gui_cpp/plugin.h>
#include "scanner_control_widget.h"
#include <QWidget>

namespace rqt_scout3d {

class ScannerControl
        : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
    ScannerControl();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

    // Comment in to signal that the plugin has a way to configure it
    //bool hasConfiguration() const;
    //void triggerConfiguration();

public slots:
    void updateLaserState(ScannerControlWidget::LaserState);

private:
    ScannerControlWidget* widget_;
};
} // namespace

#endif // RQT_SCOUT3D_SCANNER_CONTROL_H
