#include <QVBoxLayout>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>

#include "robot_editor.h"

RobotEditor::RobotEditor(QWidget* parent) :
	QWidget(parent)
{
	// construct and lay out render panel
	render_panel_ = new rviz::RenderPanel();
	QVBoxLayout* main_layout = new QVBoxLayout;
	main_layout->addWidget(render_panel_);

	// set the top-level layout for this widget
	setLayout(main_layout);

	// initialize the main RViz Classes
	manager_ = new rviz::VisualizationManager(render_panel_);
	render_panel_->initialize(manager_->getSceneManager(), manager_);
	manager_->initialize();
	manager_->startUpdate();

	// Create a grid display
	grid_ = manager_->createDisplay("rviz/Grid", "adjustable grid", true);
	ROS_ASSERT(grid_ != NULL);

	// configure the GridDisplay the way we like
	grid_->subProp("Line Style")->setValue("Billboards");
	grid_->subProp("Color")->setValue(Qt::yellow);
}

RobotEditor::~RobotEditor()
{
	delete manager_;
}
