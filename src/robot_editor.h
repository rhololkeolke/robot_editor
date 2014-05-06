#ifndef ROBOT_EDITOR_ROBOT_EDITOR_H_
#define ROBOT_EDITOR_ROBOT_EDITOR_H_

#include <QWidget>

namespace rviz
{
	class Display;
	class RenderPanel;
	class VisualizationManager;
}

class RobotEditor: public QWidget
{
Q_OBJECT
public:
  RobotEditor(QWidget* parent = 0);
  virtual ~RobotEditor();

private:
	rviz::VisualizationManager* manager_;
	rviz::RenderPanel* render_panel_;
	rviz::Display* grid_;
	rviz::Display* robot_model_;
};

#endif
