#include <QApplication>
#include <ros/ros.h>
#include "robot_editor.h"

int main(int argc, char** argv)
{
	if(!ros::isInitialized())
	{
		ros::init(argc, argv, "robot_editor", ros::init_options::AnonymousName);
	}

	QApplication app(argc, argv);

	RobotEditor* editor = new RobotEditor();
	editor->show();

	app.exec();

	delete editor;
	
}
