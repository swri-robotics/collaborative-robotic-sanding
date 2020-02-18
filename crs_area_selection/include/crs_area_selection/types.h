#ifndef CRS_AREA_SELECTION_TYPES_H
#define CRS_AREA_SELECTION_TYPES_H

#include <Eigen/StdVector>

namespace crs_area_selection
{
using PointVector = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;

}

#endif
