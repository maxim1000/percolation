#include <QApplication>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QOpenGLWidget>
#include <QPushButton>
#include <QValidator>
#include <QVBoxLayout>
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
#include <sstream>
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
	QWidget *PanelWidget;
	QLineEdit *SeedEditor;
	vtkNew<vtkRenderer> Renderer;
	vtkSmartPointer<vtkActor> Actor;
	void UpdateModel();
	void ChangeSeed(int change);
	std::uint32_t GetCurrentSeed() const;
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
	Scene->renderWindow()->AddRenderer(Renderer);
	DockWidget=new QDockWidget(&Window);
	Window.addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea,DockWidget);
	PanelWidget=new QWidget(DockWidget);
	{//fill the panel
		auto *label=new QLabel(PanelWidget);
		label->setText("seed");
		SeedEditor=new QLineEdit(PanelWidget);
		SeedEditor->setText("4");
		SeedEditor->setValidator(
			new QIntValidator(0,std::numeric_limits<std::int32_t>::max()));//doesn't work with uint32 (apparently something signed 32-bit is inside)
		connect(SeedEditor,&QLineEdit::editingFinished,[this](){UpdateModel();});
		auto *layout=new QVBoxLayout();
		auto *seedLayout=new QHBoxLayout();
		seedLayout->addWidget(label);
		seedLayout->addWidget(SeedEditor,1);
		layout->addLayout(seedLayout);
		auto *buttonsLayout=new QHBoxLayout();
		auto *previousSeedButton=new QPushButton("previous",PanelWidget);
		connect(previousSeedButton,&QPushButton::clicked,[this](){ChangeSeed(-1);});
		buttonsLayout->addWidget(previousSeedButton);
		auto *nextSeedButton=new QPushButton("next",PanelWidget);
		connect(nextSeedButton,&QPushButton::clicked,[this](){ChangeSeed(1);});
		buttonsLayout->addWidget(nextSeedButton);
		layout->addLayout(buttonsLayout);
		layout->addStretch(1);
		PanelWidget->setLayout(layout);
	}
	DockWidget->setWidget(PanelWidget);
	UpdateModel();
	Window.show();
}
void TMainWindow::UpdateModel()
{
	if(Actor!=nullptr)
		Renderer->RemoveActor(Actor);
	Actor=vtkNew<vtkActor>();
	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(MakePercolationModel(GetCurrentSeed()));
	Actor->SetMapper(mapper);
	Renderer->AddActor(Actor);
	Renderer->GetRenderWindow()->Render();
}
void TMainWindow::ChangeSeed(int change)
{
	auto seed=GetCurrentSeed();
	if(change>0)
	{
		const auto max=std::numeric_limits<std::uint32_t>::max();
		if(seed>=max-change)
			seed=max;
		else
			seed=seed+change;
	}
	else
	{
		const auto min=0;
		if(seed<=min-change)
			seed=min;
		else
			seed=seed+change;
	}
	SeedEditor->setText(std::to_string(seed).c_str());
	UpdateModel();
}
std::uint32_t TMainWindow::GetCurrentSeed() const
{
	const auto text=SeedEditor->text().toStdString();
	if(text.empty())
		return 0;
	std::uint32_t result;
	std::istringstream(text)>>result;
	return result;
}
int main(int argc,char *argv[])
{
	QApplication app(argc,argv);
	TMainWindow window;
	return app.exec();
}
