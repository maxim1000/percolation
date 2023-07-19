#include <QApplication>
#include <QMainWindow>
#include <QOpenGLWidget>
#include <QVTKOpenGLNativeWidget.h>
class TMainWindow:public QMainWindow
{
public:
	TMainWindow();
private:
	QVTKOpenGLNativeWidget *Scene;
};
TMainWindow::TMainWindow()
	:QMainWindow(nullptr)
{
	setWindowTitle("Percolation");
	resize(800,600);
	Scene=new QVTKOpenGLNativeWidget(this);
	setCentralWidget(Scene);
}
int main(int argc,char *argv[])
{
	QApplication app(argc,argv);
	TMainWindow window;
	window.show();
	return app.exec();
}
