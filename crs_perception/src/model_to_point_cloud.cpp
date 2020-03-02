#include <crs_perception/model_to_point_cloud.hpp>

#include <pcl/filters/voxel_grid.h>
#include <vtkCellArray.h>
#include <vtkPolyDataMapper.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>

namespace crs_perception
{
bool ModelToPointCloud::convertToPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
  // todo(ayoungs): handle multiple file formats?
  // todo(ayoungs): handle error for bad file
  vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkSTLReader> readerQuery = vtkSmartPointer<vtkSTLReader>::New();
  readerQuery->SetFileName(file_path_.c_str());
  readerQuery->Update();
  polydata1 = readerQuery->GetOutput();

  // make sure that the polygons are triangles!
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
  triangleFilter->SetInputData(polydata1);
  triangleFilter->Update();

  vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
  triangleMapper->Update();
  polydata1 = triangleMapper->GetInput();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
  uniformSampling(polydata1, num_samples_, *cloud);

  // Voxelgrid
  pcl::VoxelGrid<pcl::PointXYZ> grid_;
  grid_.setInputCloud(cloud);
  grid_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);

  grid_.filter(*point_cloud);

  return true;
}

void ModelToPointCloud::uniformSampling(vtkSmartPointer<vtkPolyData> polydata,
                                        std::size_t n_samples,
                                        pcl::PointCloud<pcl::PointXYZ>& cloud_out)
{
  polydata->BuildCells();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();

  double p1[3], p2[3], p3[3], totalArea = 0;
  std::vector<double> cumulativeAreas(cells->GetNumberOfCells(), 0);
  vtkIdType npts = 0, *ptIds = nullptr;
  std::size_t cellId = 0;
  for (cells->InitTraversal(); cells->GetNextCell(npts, ptIds); cellId++)
  {
    polydata->GetPoint(ptIds[0], p1);
    polydata->GetPoint(ptIds[1], p2);
    polydata->GetPoint(ptIds[2], p3);
    totalArea += vtkTriangle::TriangleArea(p1, p2, p3);
    cumulativeAreas[cellId] = totalArea;
  }

  cloud_out.points.resize(n_samples);
  cloud_out.width = static_cast<std::uint32_t>(n_samples);
  cloud_out.height = 1;

  for (std::size_t i = 0; i < n_samples; i++)
  {
    Eigen::Vector3f p;
    Eigen::Vector3f n(0, 0, 0);
    Eigen::Vector3f c(0, 0, 0);
    randPSurface(polydata, &cumulativeAreas, totalArea, p);
    cloud_out.points[i].x = p[0];
    cloud_out.points[i].y = p[1];
    cloud_out.points[i].z = p[2];
  }
}

inline void ModelToPointCloud::randPSurface(vtkSmartPointer<vtkPolyData> polydata,
                                            std::vector<double>* cumulativeAreas,
                                            double totalArea,
                                            Eigen::Vector3f& p)
{
  float r = static_cast<float>(uniformDeviate(rand()) * totalArea);

  std::vector<double>::iterator low = std::lower_bound(cumulativeAreas->begin(), cumulativeAreas->end(), r);
  vtkIdType el = vtkIdType(low - cumulativeAreas->begin());

  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType* ptIds = nullptr;
  polydata->GetCellPoints(el, npts, ptIds);
  polydata->GetPoint(ptIds[0], A);
  polydata->GetPoint(ptIds[1], B);
  polydata->GetPoint(ptIds[2], C);
  float r1 = static_cast<float>(uniformDeviate(rand()));
  float r2 = static_cast<float>(uniformDeviate(rand()));
  randomPointTriangle(float(A[0]),
                      float(A[1]),
                      float(A[2]),
                      float(B[0]),
                      float(B[1]),
                      float(B[2]),
                      float(C[0]),
                      float(C[1]),
                      float(C[2]),
                      r1,
                      r2,
                      p);
}

inline void ModelToPointCloud::randomPointTriangle(float a1,
                                                   float a2,
                                                   float a3,
                                                   float b1,
                                                   float b2,
                                                   float b3,
                                                   float c1,
                                                   float c2,
                                                   float c3,
                                                   float r1,
                                                   float r2,
                                                   Eigen::Vector3f& p)
{
  float r1sqr = std::sqrt(r1);
  float OneMinR1Sqr = (1 - r1sqr);
  float OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
}

inline double ModelToPointCloud::uniformDeviate(int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

}  // namespace crs_perception
