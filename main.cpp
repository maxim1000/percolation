#include <QApplication>
#include <QMainWindow>
int main(int argc,char *argv[])
{
	QApplication app(argc,argv);
	QMainWindow window(nullptr);
	window.show();
	return app.exec();
}
