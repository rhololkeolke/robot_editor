#ifndef ROBOT_EDITOR_ROBOT_EDITOR_H_
#define ROBOT_EDITOR_ROBOT_EDITOR_H_

#include <QObject>
#include <ui_main_window.h>

#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>


class QMainWindow;
class RobotPreview;
namespace robot_state_publisher { class RobotStatePublisher; }
namespace KDL { class Tree; }

class RobotEditor : public QObject
{
Q_OBJECT
public:
	RobotEditor();
	~RobotEditor();

    void show();

public Q_SLOTS:
	void openTrigger();
	void saveTrigger();
	void saveAsTrigger();
	void exitTrigger();

private:
	void updateParams(const std::string& urdf);
	void publishJointStates(const std::string& urdf);
	
private:
    QMainWindow main_window_;
	Ui::MainWindow main_window_ui_;
	RobotPreview* robot_preview_;

	QString file_name_;

	ros::NodeHandle nh_;

	KDL::Tree* robot_tree_ = NULL;
	robot_state_publisher::RobotStatePublisher* robot_state_pub_ = NULL;
};

#endif
