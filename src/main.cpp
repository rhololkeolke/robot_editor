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
	Ui::MainWindow ui;
	ui.setupUi(main_window);

	RobotEditor* editor = new RobotEditor(ui.rvizFrame);//ui.rvizDisplay);
	main_window->show();
	
	return app.exec();
}
