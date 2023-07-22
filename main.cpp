#include <QApplication>
#include <QDockWidget>
#include <QLineEdit>
#include <QMainWindow>
#include <QOpenGLWidget>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkCamera.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <array>
#include <map>
#include <queue>
#include <random>
#include <set>
vtkSmartPointer<vtkPolyData> MakePercolationModel(const std::uint32_t seed)
{
	using TVoxel=std::array<int,3>;
	std::set<TVoxel> filledVoxels;
	{//fill percolation voxels
		using TEdge=std::array<TVoxel,2>;
		const double threshold=0.25;
		const int maxCoordinate=50;
		std::mt19937 randomSource(seed);
		std::bernoulli_distribution edgeIsEnabled(threshold);
		std::map<TEdge,bool> cached;
		std::queue<TVoxel> todo;
		todo.push(TVoxel{0,0,0});
		filledVoxels.insert(TVoxel{0,0,0});
		while(!todo.empty())
		{
			const auto current=todo.front();
			todo.pop();
			for(int axis=0;axis<3;++axis)
			{
				for(int direction:{-1,1})
				{
					auto neighbour=current;
					neighbour[axis]+=direction;
					if(std::abs(neighbour[axis])>maxCoordinate)
						continue;
					if(filledVoxels.count(neighbour)!=0)
						continue;
					auto edge=TEdge{current,neighbour};
					if(edge[0]>edge[1])
						std::swap(edge[0],edge[1]);
					if(cached.count(edge)==0)
						cached[edge]=edgeIsEnabled(randomSource);
					if(cached.at(edge))
					{
						filledVoxels.insert(neighbour);
						todo.push(neighbour);
					}
				}
			}
		}
	}
	vtkNew<vtkPolyData> mesh;
	{//fill the mesh wiith the voxels
		using TVertex=std::array<int,3>;//min-min-min corner of a voxel
		mesh->SetPoints(vtkNew<vtkPoints>());
		mesh->SetPolys(vtkNew<vtkCellArray>());
		std::map<TVertex,vtkIdType> addedVertices;
		const auto addFace=[&](const std::array<TVertex,4> &vertices)
		{
			vtkIdType vtkVertices[4];
			for(int v=0;v<4;++v)
			{
				const auto vertex=vertices[v];
				if(addedVertices.count(vertex)==0)
				{
					addedVertices[vertex]=mesh->GetPoints()->InsertNextPoint(
						vertex[0],
						vertex[1],
						vertex[2]);
				}
				vtkVertices[v]=addedVertices.at(vertex);
			}
			mesh->GetPolys()->InsertNextCell(4,vtkVertices);
		};
		const std::array<int,4> faceCornerIndices[3][2]=
		{
			{
				{0,4,6,2},
				{1,3,7,5}
			},
			{
				{0,1,5,4},
				{3,2,6,7},
			},
			{
				{1,0,2,3},
				{4,5,7,6}
			}
		};
		for(const auto &voxel:filledVoxels)
		{
			for(int axis=0;axis<3;++axis)
			{
				for(int direction:{-1,1})
				{
					auto neighbour=voxel;
					neighbour[axis]+=direction;
					if(filledVoxels.count(neighbour)!=0)
						continue;//no wall
					const auto &cornerIndices=faceCornerIndices[axis][(direction+1)/2];
					std::array<TVertex,4> vertices;
					for(int v=0;v<4;++v)
					{
						auto vertex=voxel;
						for(int axis=0;axis<3;++axis)
							vertex[axis]+=(cornerIndices[v]>>axis)&1;
						vertices[v]=vertex;
					}
					addFace(vertices);
				}
			}
		}
	}
	return mesh;
}
///////////////////////////////////////////////////////////////////////////////
class TMainWindow:public QMainWindow
{
public:
	TMainWindow();
private:
	QMainWindow Window;
	QVTKOpenGLNativeWidget *Scene;
	QDockWidget *DockWidget;
	QLineEdit *SeedEditor;
	vtkNew<vtkRenderer> Renderer;
	vtkSmartPointer<vtkActor> Actor;
	void UpdateModel();
};
TMainWindow::TMainWindow()
	:Window(nullptr)
{
	Window.setWindowTitle("Percolation");
	Window.resize(800,600);
	Scene=new QVTKOpenGLNativeWidget(&Window);
	Window.setCentralWidget(Scene);
	{//set the camera
		vtkNew<vtkCamera> camera;
		camera->SetViewUp(0,1,0);
		camera->SetFocalPoint(0,0,0);
		camera->SetPosition(-300,100,-50);
		Renderer->SetActiveCamera(camera);
	}
	UpdateModel();
	Scene->renderWindow()->AddRenderer(Renderer);
	DockWidget=new QDockWidget(&Window);
	Window.addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea,DockWidget);
	SeedEditor=new QLineEdit(DockWidget);
	DockWidget->setWidget(SeedEditor);
	Window.show();
}
void TMainWindow::UpdateModel()
{
	const std::uint32_t seed=4;
	if(Actor!=nullptr)
		Renderer->RemoveActor(Actor);
	Actor=vtkNew<vtkActor>();
	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(MakePercolationModel(seed));
	Actor->SetMapper(mapper);
	Renderer->AddActor(Actor);
}
int main(int argc,char *argv[])
{
	QApplication app(argc,argv);
	TMainWindow window;
	return app.exec();
}
