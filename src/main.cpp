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

	RobotEditor* editor = new RobotEditor(ui.rvizFrame);

	QObject::connect(ui.actionExit, SIGNAL(triggered()), editor, SLOT(exitTrigger()));
	QObject::connect(ui.actionOpen, SIGNAL(triggered()), editor, SLOT(openTrigger()));
	QObject::connect(ui.actionSave, SIGNAL(triggered()), editor, SLOT(saveTrigger()));
	QObject::connect(ui.actionSave_As, SIGNAL(triggered()), editor, SLOT(saveAsTrigger()));
	
	main_window->show();
	
	return app.exec();
}
