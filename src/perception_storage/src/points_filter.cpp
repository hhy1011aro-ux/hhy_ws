#include "perception_storage/points_filter.hpp"

points_filter::points_filter()
{

}

void points_filter::norm_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr & source_points,pcl::PointCloud<pcl::PointXYZI>::Ptr & target_points,
                                float threshold,Eigen::Vector3f reference_dir)
{
  pcl::NormalEstimation<pcl::PointXYZI,pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(source_points);
  normalEstimation.setRadiusSearch(0.05);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
  normalEstimation.setSearchMethod(kdtree);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  normalEstimation.compute(*normals);
  pcl::PointCloud<pcl::PointXYZI>::Ptr norm_filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  for(int i = 0; i < source_points->size(); i++) 
  {
    float dot = fabs(normals->at(i).getNormalVector3fMap().normalized().dot(reference_dir));
    if(dot > threshold)
    {
      norm_filtered_ptr->push_back(source_points->at(i));
    }
  }
  pcl::copyPointCloud<pcl::PointXYZI>(*norm_filtered_ptr,*target_points);
}

void points_filter::sampling_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr & source_points,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr & target_points,
                                    std::string method,float scale)
{
  if(method=="voxel")
  {
   pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
   voxel_filter.setInputCloud(source_points);
   voxel_filter.setLeafSize(scale,scale,scale);
   pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_points(new pcl::PointCloud<pcl::PointXYZI>);
   voxel_filter.filter(*voxel_points);
   pcl::copyPointCloud<pcl::PointXYZI>(*voxel_points,*target_points);
  }
  if(method=="uniform")
  {
   pcl::PointCloud<pcl::PointXYZI>::Ptr down_points(new pcl::PointCloud<pcl::PointXYZI>);
   pcl::UniformSampling<pcl::PointXYZI> us_filter;
   us_filter.setInputCloud(source_points);
   us_filter.setRadiusSearch(scale);
   pcl::PointCloud<int> down_indices;
   us_filter.compute(down_indices);
   pcl::copyPointCloud(*source_points,down_indices.points,*down_points);
   pcl::copyPointCloud<pcl::PointXYZI>(*down_points,*target_points);
  }
}

std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr,
           pcl::PointIndices::Ptr,
           pcl::ModelCoefficients::Ptr> points_filter::naive_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr & raw_points,
                                                                   float threshold)
{
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(threshold);
  seg.setInputCloud(raw_points);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.segment(*inliers, *coefficients);
  pcl::PointCloud<pcl::PointXYZI>::Ptr fit_plane_points(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud<pcl::PointXYZI>(*raw_points, *inliers, *fit_plane_points);

  return std::make_tuple(fit_plane_points,inliers,coefficients);
}

std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr,
            std::vector<int>,
            Eigen::VectorXf> points_filter::naive_line(pcl::PointCloud<pcl::PointXYZI>::Ptr &raw_points,
                                                       float threshold,bool optimize_coefficients)
{
  pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr _projected_model_(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(raw_points));	
  pcl::RandomSampleConsensus<pcl::PointXYZI> _projected_fit_(_projected_model_);	
  _projected_fit_.setDistanceThreshold(threshold);
  _projected_fit_.setMaxIterations(100);
  _projected_fit_.computeModel();
  std::vector<int> _inliers_;
  _projected_fit_.getInliers(_inliers_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr _projected_line_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud<pcl::PointXYZI>(*raw_points,_inliers_,*_projected_line_);
  Eigen::VectorXf _projected_coef_;
  _projected_fit_.getModelCoefficients(_projected_coef_);
  // RANSAC原始系数来自随机样本；按需使用全部内点重估，降低有厚度点带上的方向抖动。
  if(optimize_coefficients && _inliers_.size()>2 && _projected_coef_.size()==6)
  {
    Eigen::VectorXf optimized_coefficients;
    _projected_model_->optimizeModelCoefficients(_inliers_,_projected_coef_,optimized_coefficients);
    if(optimized_coefficients.size()==6)
    {
      _projected_coef_=optimized_coefficients;
    }
  }

  return std::make_tuple(_projected_line_,_inliers_,_projected_coef_);
}

std::tuple<Eigen::Vector3f,Eigen::Vector3f> points_filter::target_orient(const TargetOrient orient_method,pcl::PointCloud<pcl::PointXYZI>::Ptr & target_points)
{ 
  Eigen::Vector3f sum_points = Eigen::Vector3f::Zero();
  for (int j = 0; j < target_points->size(); j++)
  {
    sum_points+=Eigen::Vector3f(target_points->points[j].x,target_points->points[j].y,target_points->points[j].z);
  }
  Eigen::Vector3f mean_points=sum_points/target_points->size();
  switch(orient_method)
  {
    case TargetOrient::NONE:
    {
      auto _plane_with_points_=naive_plane(target_points,0.015);
      auto _inliner_points_=std::get<0>(_plane_with_points_);
      Eigen::Vector4f _center_;
      pcl::compute3DCentroid(*_inliner_points_, _center_);
      return std::make_tuple(Eigen::Vector3f(std::get<2>(_plane_with_points_)->values[0],
                             std::get<2>(_plane_with_points_)->values[1],
                             std::get<2>(_plane_with_points_)->values[2]),
                             _center_.head(3));
    }
    case TargetOrient::QR:
    {
      int fit_num = target_points->size();
      Eigen::Matrix<float, Eigen::Dynamic, 3> matA0(fit_num, 3);
      Eigen::Matrix<float, Eigen::Dynamic, 1> matB0(fit_num, 1); 
      for (int j = 0; j < target_points->size(); j++)
      {
        matA0(j, 0) = target_points->points[j].x;
        matA0(j, 1) = target_points->points[j].y;
        matA0(j, 2) = target_points->points[j].z;
        matB0(j, 0) = -1.0;
      }
      Eigen::Vector3f norm = matA0.colPivHouseholderQr().solve(matB0);
      norm.normalize();
      return std::make_tuple(norm,mean_points);
    }
    case TargetOrient::SVD:
    { 
      Eigen::MatrixXf decenter= Eigen::MatrixXf::Zero(3,target_points->size());
      for(int ci=0;ci<target_points->size();ci++)
      {
        auto point_ci=target_points->at(ci);
        Eigen::Vector3f delta=Eigen::Vector3f(point_ci.x,point_ci.y,point_ci.z)-mean_points;
        decenter.col(ci)=delta;
      }
      Eigen::JacobiSVD<Eigen::MatrixXf> svd(decenter, Eigen::ComputeThinU | Eigen::ComputeThinV);
      Eigen::Vector3f svd_normal = (svd.matrixU().block(0, 2, 3, 1)).eval();
      svd_normal.normalize();
      return std::make_tuple(svd_normal,mean_points);
    }
    case TargetOrient::CERES:
    {
      double abcd[4] = {0,1,0,1};
      ceres::Problem problem;
      for (int i = 0; i < target_points->points.size(); i++)
      {
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<PLANE_FITTING_COST, 1, 4>(
                new PLANE_FITTING_COST(target_points->points[i].x,
                                       target_points->points[i].y,
                                       target_points->points[i].z)
            ),
            loss_function,
            abcd
        );
      }
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.minimizer_progress_to_stdout = false;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      Eigen::Vector3f ceres_param = Eigen::Vector3f(abcd[0], abcd[1], abcd[2]);
      ceres_param.normalize();
      return std::make_tuple(ceres_param,mean_points);
    }
    default:
      break;
  }
}
