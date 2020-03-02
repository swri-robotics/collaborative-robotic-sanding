#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace crs_perception
{
class ModelToPointCloud
{
public:
  ModelToPointCloud(std::string file_path, int num_samples, float leaf_size)
    : file_path_(file_path), num_samples_(num_samples), leaf_size_(leaf_size)
  {
  }

  bool convertToPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

private:
  std::string file_path_;
  int num_samples_;
  float leaf_size_;

  void uniformSampling(vtkSmartPointer<vtkPolyData> polydata,
                       std::size_t n_samples,
                       pcl::PointCloud<pcl::PointXYZ>& cloud_out);
  inline void randPSurface(vtkSmartPointer<vtkPolyData> polydata,
                           std::vector<double>* cumulativeAreas,
                           double totalArea,
                           Eigen::Vector3f& p);
  inline void randomPointTriangle(float a1,
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
                                  Eigen::Vector3f& p);
  inline double uniformDeviate(int seed);
};

}  // namespace crs_perception
