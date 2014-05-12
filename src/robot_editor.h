#ifndef ROBOT_EDITOR_ROBOT_EDITOR_H_
#define ROBOT_EDITOR_ROBOT_EDITOR_H_

#include <QObject>
#include <ui_main_window.h>

class QMainWindow;
class RobotPreview;

class RobotEditor : public QObject
{
Q_OBJECT
public:
	RobotEditor(QMainWindow* main_window);
	~RobotEditor();

public Q_SLOTS:
	void openTrigger();
	void saveTrigger();
	void saveAsTrigger();
	void exitTrigger();
	  
  
private:
	Ui::MainWindow main_window_ui_;
	RobotPreview* robot_preview_;
};

#endif
