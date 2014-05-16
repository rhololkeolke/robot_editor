#include "robot_editor.h"
#include "robot_preview.h"

#include <QFileDialog>
#include <QString>

#include <cstdlib>
#include <fstream>

#include <XmlRpcValue.h>

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
	file_name_ = QFileDialog::getOpenFileName(0, tr("Open URDF File"), ".", tr("XML Files (*.xml)"));

	// this will be true if the user cancels
	if(file_name_.isEmpty())
		return;

	// debugging print for now
	printf("file selected: %s\n", qPrintable(file_name_));

	// convert the file to a string
	std::ifstream selected_file(file_name_.toStdString().c_str());
	std::string file_contents((std::istreambuf_iterator<char>(selected_file)), std::istreambuf_iterator<char>());

	// fill the text editor with this string
	main_window_ui_.xmlEdit->setText(QString::fromStdString(file_contents));

	// also set the rosparam
	this->updateParams(file_contents);
}

void RobotEditor::saveTrigger() {
	if(file_name_.isEmpty())
	{
		printf("Calling save as\n");
		this->saveAsTrigger();
		return;
	}

	std::ofstream output_file(file_name_.toStdString().c_str());
	std::string file_contents = main_window_ui_.xmlEdit->toPlainText().toStdString();
	output_file << file_contents;

	// update the rosparam
	this->updateParams(file_contents);
}

void RobotEditor::saveAsTrigger() {
	printf("Save as selected\n");
	file_name_ = QFileDialog::getSaveFileName(0, tr("Save As..."),
											  ".", tr("XML (*.xml)"));

	if(file_name_.isEmpty())
		return; // user canceled

	std::ofstream output_file(file_name_.toStdString().c_str());
	std::string file_contents = main_window_ui_.xmlEdit->toPlainText().toStdString();
	output_file << file_contents;

	// update the rosparam
	this->updateParams(file_contents);
}

void RobotEditor::exitTrigger() {
	// quit the application
	exit(0);
}

void RobotEditor::updateParams(const std::string& urdf)
{
	XmlRpc::XmlRpcValue robot_description(urdf);
	nh_.setParam("robot_editor/robot_description", robot_description);
}
