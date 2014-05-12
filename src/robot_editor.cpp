#include "robot_editor.h"
#include "robot_preview.h"


#include <cstdlib>

RobotEditor::RobotEditor(QMainWindow *main_window)
{
	main_window_ui_.setupUi(main_window);
	robot_preview_ = new RobotPreview(main_window_ui_.rvizFrame);

	QObject::connect(main_window_ui_.actionExit, SIGNAL(triggered()), this, SLOT(exitTrigger()));
	QObject::connect(main_window_ui_.actionOpen, SIGNAL(triggered()), this, SLOT(openTrigger()));
	QObject::connect(main_window_ui_.actionSave, SIGNAL(triggered()), this, SLOT(saveTrigger()));
	QObject::connect(main_window_ui_.actionSave_As, SIGNAL(triggered()), this, SLOT(saveAsTrigger()));
}

RobotEditor::~RobotEditor()
{
	delete robot_preview_;
}

void RobotEditor::openTrigger() {
	printf("Open selected\n");
}

void RobotEditor::saveTrigger() {
	printf("Save selected\n");
}

void RobotEditor::saveAsTrigger() {
	printf("Save as selected\n");
}

void RobotEditor::exitTrigger() {
	printf("Exit selected\n");
}
