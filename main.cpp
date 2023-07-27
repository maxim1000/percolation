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
#include <vtkImageData.h>
#include <vtkMarchingCubes.h>
#include <vtkPointLocator.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <array>
#include <cmath>
#include <map>
#include <queue>
#include <random>
#include <set>
#include <sstream>
class TFasterImageAccessor
{
public:
	TFasterImageAccessor(const vtkSmartPointer<vtkImageData>&);
	double Get(int x,int y,int z) const;
	void Set(int x,int y,int z,double value);
private:
	vtkSmartPointer<vtkImageData> Image;
	double *Pointer;
	vtkIdType Increments[3];
	int Start[3];
	double *GetPointer(int x,int y,int z) const;
};
TFasterImageAccessor::TFasterImageAccessor(
	const vtkSmartPointer<vtkImageData> &image)
	:Image(image)
{
	if(Image->GetScalarType()!=VTK_DOUBLE)
		throw std::logic_error("TFasterImageAccessor: only double scalars are supported");
	if(Image->GetNumberOfScalarComponents()!=1)
		throw std::logic_error("TFasterImageAccessor: only single-component scalars are supported");
	Pointer=reinterpret_cast<double*>(Image->GetScalarPointer());
	Image->GetIncrements(Increments);
	for(int c=0;c<3;++c)
		Start[c]=Image->GetExtent()[2*c];
}
double TFasterImageAccessor::Get(int x,int y,int z) const
{
	return *GetPointer(x,y,z);
}
void TFasterImageAccessor::Set(int x,int y,int z,double value)
{
	*GetPointer(x,y,z)=value;
}
double *TFasterImageAccessor::GetPointer(int x,int y,int z) const
{
	double *result=Pointer;
	result+=(x-Start[0])*Increments[0];
	result+=(y-Start[1])*Increments[1];
	result+=(z-Start[2])*Increments[2];
	return result;
}
///////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkImageData> SmoothImage(
	vtkSmartPointer<vtkImageData> image,
	int count)
{
	while(count-->0)
	{
		vtkNew<vtkImageData> smoothed;
		int extent[6];
		image->GetExtent(extent);
		smoothed->SetExtent(extent);
		smoothed->AllocateScalars(VTK_DOUBLE,1);
		const TFasterImageAccessor imageReader(image);
		for(int z=extent[4];z<=extent[5];++z)
		{
			for(int y=extent[2];y<=extent[3];++y)
			{
				for(int x=extent[0];x<=extent[1];++x)
				{
					double sum=0;
					int count=0;
					for(int dz=-1;dz<=1;++dz)
					{
						if(z+dz<extent[4] || z+dz>extent[5])
							continue;
						for(int dy=-1;dy<=1;++dy)
						{
							if(y+dy<extent[2] || y+dy>extent[3])
								continue;
							for(int dx=-1;dx<=1;++dx)
							{
								if(x+dx<extent[0] || x+dx>extent[1])
									continue;
								sum+=imageReader.Get(x+dx,y+dy,z+dz);
								++count;
							}
						}
					}
					smoothed->SetScalarComponentFromDouble(x,y,z,0,sum/count);
				}
			}
		}
		image=smoothed;
	}
	return image;
}
vtkSmartPointer<vtkPolyData> MakePercolationModel(
	const std::uint32_t seed,
	bool makeCloud)
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
	if(makeCloud)
	{
		const double radius=10;
		const auto maxCoordinate=[&]()
		{
			int result=0;
			for(const auto &voxel:filledVoxels)
				for(int coordinate:voxel)
					result=std::max(result,std::abs(coordinate));
			return result;
		}();
		const auto locator=[&]()
		{
			vtkNew<vtkPoints> points;
			for(const auto &voxel:filledVoxels)
			{
				double coordinates[3];
				for(int c=0;c<3;++c)
					coordinates[c]=double(voxel[c]);
				points->InsertNextPoint(coordinates);
			}
			vtkNew<vtkPolyData> polyData;
			polyData->SetPoints(points);
			vtkNew<vtkPointLocator> result;
			result->SetDataSet(polyData);
			result->BuildLocator();
			return result;
		}();
		const auto image=[&]()
		{
			const int dimension=maxCoordinate+int(radius+1);
			vtkNew<vtkImageData> image;
			image->SetExtent(-dimension,dimension,-dimension,dimension,-dimension,dimension);
			image->AllocateScalars(VTK_DOUBLE,1);
			for(int z=-dimension;z<=dimension;++z)
			{
				for(int y=-dimension;y<=dimension;++y)
				{
					for(int x=-dimension;x<=dimension;++x)
					{
						const double point[]={double(x),double(y),double(z)};
						double squaredDistance=std::numeric_limits<double>::max();
						const double limit=radius+1;
						const auto closestPointId=
							locator->FindClosestPointWithinRadius(limit,point,squaredDistance);
						if(closestPointId==-1)
							squaredDistance=limit*limit;
						image->SetScalarComponentFromDouble(x,y,z,0,std::sqrt(squaredDistance));
					}
				}
			}
			return SmoothImage(image,3);
		}();
		const auto isosurface=[&]()
		{
			vtkNew<vtkMarchingCubes> builder;
			builder->SetInputData(image);
			builder->SetNumberOfContours(1);
			builder->SetValue(0,radius);
			builder->SetComputeGradients(false);
			builder->SetComputeNormals(false);
			builder->SetComputeScalars(false);
			builder->Update();
			return vtkSmartPointer<vtkPolyData>(builder->GetOutput());
		}();
		return isosurface;
	}
	else
	{//fill the mesh wiith the voxels
		using TVertex=std::array<int,3>;//min-min-min corner of a voxel
		vtkNew<vtkPolyData> mesh;
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
		return mesh;
	}
}
///////////////////////////////////////////////////////////////////////////////
class TMainWindow:public QMainWindow
{
public:
	TMainWindow();
private:
	QMainWindow Window;
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
	{//set up the 3D scene
		auto *scene=new QVTKOpenGLNativeWidget(&Window);
		Window.setCentralWidget(scene);
		vtkNew<vtkCamera> camera;
		camera->SetViewUp(0,1,0);
		camera->SetFocalPoint(0,0,0);
		camera->SetPosition(-300,100,-50);
		Renderer->SetActiveCamera(camera);
		scene->renderWindow()->AddRenderer(Renderer);
	}
	{//fill the panel
		auto *dockWidget=new QDockWidget(&Window);
		Window.addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea,dockWidget);
		auto *panelWidget=new QWidget(dockWidget);
		auto *layout=new QVBoxLayout();
		{//add seed editor with label
			auto *label=new QLabel(panelWidget);
			label->setText("seed");
			SeedEditor=new QLineEdit(panelWidget);
			SeedEditor->setText("4");
			SeedEditor->setValidator(
				new QIntValidator(0,std::numeric_limits<std::int32_t>::max()));//doesn't work with uint32 (apparently something signed 32-bit is inside)
			connect(SeedEditor,&QLineEdit::editingFinished,[this](){UpdateModel();});
			auto *seedLayout=new QHBoxLayout();
			seedLayout->addWidget(label);
			seedLayout->addWidget(SeedEditor,1);
			layout->addLayout(seedLayout);
		}
		{//add buttons
			auto *buttonsLayout=new QHBoxLayout();
			auto *previousSeedButton=new QPushButton("previous",panelWidget);
			connect(previousSeedButton,&QPushButton::clicked,[this](){ChangeSeed(-1);});
			buttonsLayout->addWidget(previousSeedButton);
			auto *nextSeedButton=new QPushButton("next",panelWidget);
			connect(nextSeedButton,&QPushButton::clicked,[this](){ChangeSeed(1);});
			buttonsLayout->addWidget(nextSeedButton);
			layout->addLayout(buttonsLayout);
		}
		layout->addStretch(1);
		panelWidget->setLayout(layout);
		dockWidget->setWidget(panelWidget);
	}
	UpdateModel();
	Window.show();
}
void TMainWindow::UpdateModel()
{
	if(Actor!=nullptr)
		Renderer->RemoveActor(Actor);
	Actor=vtkNew<vtkActor>();
	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(MakePercolationModel(GetCurrentSeed(),true));
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
