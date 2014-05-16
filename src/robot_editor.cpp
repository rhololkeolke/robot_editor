#include "robot_editor.h"
#include "robot_preview.h"

#include <QFileDialog>
#include <QString>

#include <cstdlib>
#include <fstream>

RobotEditor::RobotEditor()
{
	main_window_ui_.setupUi(&main_window_);
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

void RobotEditor::show()
{
	main_window_.show();
}

void RobotEditor::openTrigger() {
	QString file_name = QFileDialog::getOpenFileName(0, tr("Open URDF File"), "~", tr("XML Files (*.xml)"));

	printf("file selected: %s\n", qPrintable(file_name));

	std::ifstream selected_file(file_name.toStdString().c_str());
	std::string file_contents((std::istreambuf_iterator<char>(selected_file)), std::istreambuf_iterator<char>());

	main_window_ui_.xmlEdit->setText(QString::fromStdString(file_contents));
}

void RobotEditor::saveTrigger() {
	printf("Save selected\n");
}

void RobotEditor::saveAsTrigger() {
	printf("Save as selected\n");
}

void RobotEditor::exitTrigger() {
	// quit the application
	exit(0);
}
