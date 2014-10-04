#include <QApplication>
#include <ros/ros.h>
#include "robot_editor.h"
#include <ui_main_window.h>

int main(int argc, char** argv)
{
	if(!ros::isInitialized())
	{
		ros::init(argc, argv, "robot_editor", ros::init_options::AnonymousName);
	}

	QApplication app(argc, argv);

    RobotEditor* robot_editor;
    ros::NodeHandle nh("~");
    std::string urdf_file = "";
    if(nh.getParam("urdf_file", urdf_file)) {
        robot_editor = new RobotEditor(urdf_file);
    }
    else
        robot_editor = new RobotEditor();

	robot_editor->show();

	int return_code = app.exec();

    delete robot_editor;

    return return_code;
}
