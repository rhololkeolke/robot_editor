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

	QMainWindow *main_window = new QMainWindow;
	RobotEditor* robot_editor = new RobotEditor(main_window);

	main_window->show();
	
	int result = app.exec();

	delete robot_editor;
	delete main_window;

	return result;
}
