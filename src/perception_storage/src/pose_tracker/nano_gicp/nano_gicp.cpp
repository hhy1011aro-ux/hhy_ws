#include "perception_storage/pose_tracker/nano_gicp/lsq_registration.hpp"
#include "perception_storage/pose_tracker/nano_gicp/impl/lsq_registration_impl.hpp"
#include "perception_storage/pose_tracker/nano_gicp/nano_gicp.hpp"
#include "perception_storage/pose_tracker/nano_gicp/impl/nano_gicp_impl.hpp"

template class nano_gicp::LsqRegistration<pcl::PointXYZI, pcl::PointXYZI>;
template class nano_gicp::NanoGICP<pcl::PointXYZI, pcl::PointXYZI>;