#include <QApplication>
#include <QMainWindow>
#include <QOpenGLWidget>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkCamera.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
class TMainWindow:public QMainWindow
{
public:
	TMainWindow();
private:
	QVTKOpenGLNativeWidget *Scene;
	vtkNew<vtkRenderer> Renderer;
};
TMainWindow::TMainWindow()
	:QMainWindow(nullptr)
{
	setWindowTitle("Percolation");
	resize(800,600);
	Scene=new QVTKOpenGLNativeWidget(this);
	setCentralWidget(Scene);
	{//set the camera
		vtkNew<vtkCamera> camera;
		camera->SetViewUp(0,1,0);
		camera->SetFocalPoint(0,0,0);
		camera->SetPosition(-100,100,-100);
		Renderer->SetActiveCamera(camera);
	}
	{//add a cube
		vtkNew<vtkPolyData> mesh;
		{//set the vertices
			vtkNew<vtkPoints> vertices;
			vertices->SetDataTypeToDouble();
			vertices->SetNumberOfPoints(8);
			for(int v=0;v<8;++v)
			{
				double coordinates[3];
				for(int c=0;c<3;++c)
					coordinates[c]=2*double((v>>c)&1)-1;
				vertices->SetPoint(v,coordinates);
			}
			mesh->SetPoints(vertices);
		}
		{//set faces
			const vtkIdType faceVertexIndices[6][4]=
			{
				{1,0,2,3},
				{1,3,7,5},
				{0,1,5,4},
				{0,4,6,2},
				{3,2,6,7},
				{4,5,7,6}
			};
			vtkNew<vtkCellArray> faces;
			for(int f=0;f<6;++f)
				faces->InsertNextCell(4,faceVertexIndices[f]);
			mesh->SetPolys(faces);
		}
		{//add to the renderer
			vtkNew<vtkPolyDataMapper> mapper;
			mapper->SetInputData(mesh);
			vtkNew<vtkActor> actor;
			actor->SetMapper(mapper);
			Renderer->AddActor(actor);
		}
	}
	Scene->renderWindow()->AddRenderer(Renderer);
}
int main(int argc,char *argv[])
{
	QApplication app(argc,argv);
	TMainWindow window;
	window.show();
	return app.exec();
}
