#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Core>
#include "fcl/broadphase/broadphase.h"
#include "fcl/collision.h"
#include "fcl/config.h"
#include "fcl/math/transform.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace planner
{
class PathFinder
{
public:
  PathFinder(Eigen::MatrixXd far_plane, Eigen::Matrix3d roty, Eigen::Vector4d params);
  virtual ~PathFinder();
  bool plan(std::vector<Eigen::Vector3d> &path);
  bool poseToExplore(std::vector<Eigen::Vector4d> &path);

  //void setPose(const geometry_msgs::PoseConstPtr &msg);
  void setPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  void setOctomap(const octomap_msgs::Octomap::ConstPtr &msg);
  void goalSelector(Eigen::Vector3d &goal, bool goal_set);
  double evaluateGain(Eigen::Vector3d random_pose, double yaw);
  std::vector<Eigen::Vector3d> getVisPoints();

  octomap::point3d pointEigenToOctomap(const Eigen::Vector3d &point)
  {
    return octomap::point3d(point.x(), point.y(), point.z());
  }

  Eigen::Vector3d pointOctomapToEigen(const octomap::point3d &point)
  {
    return Eigen::Vector3d(point.x(), point.y(), point.z());
  }

  double randMToN(double m, double n)
  {
    return m + (rand() / (RAND_MAX / (n - m)));
  }

private:
  bool isValid(const ompl::base::State *state);

  std::shared_ptr<fcl::CollisionGeometry> drone_;

  std::shared_ptr<fcl::CollisionGeometry> tree_obj_;
  octomap::OcTree *oct_tree_;
  Eigen::Vector4d start_pose_;
  bool ready_tree_;
  bool ready_pose_;
  Eigen::MatrixXd far_plane_;
  Eigen::Matrix3d roty_;
  ob::ProblemDefinitionPtr pdef_;
  ob::StateSpacePtr space_;
  ob::SpaceInformationPtr si_;
  Eigen::Vector3d goal_position_;
  Eigen::Vector4d params_;
  std::vector<Eigen::Vector3d> points_visualization_;
  std::vector<Eigen::Vector3d> points_visualization_max_;

  enum CellType
  {
    free = 0,
    occupied = 1,
    uknown = 2
  };

  double getVisibility(const Eigen::Vector3d &view_point, const Eigen::Vector3d &voxel_to_test,
                       bool stop_at_unknown_cell);

  double getCellProbabilityPoint(const Eigen::Vector3d &point, double *probability);

  bool isPathFeasible(const Eigen::Vector3d &test_voxel, const Eigen::Vector3d &origin_voxel);

  void sphereCoords();

  void visualizeFrustum(Eigen::Vector3d point, double yaw);

  void setFree();

  void planAtBottom();

  // FRUSTUM

  Eigen::Vector3d planeFromPoints(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, double &dist);
  double distP(Eigen::Vector3d normal, Eigen::Vector3d point);
  bool isInFrustum(Eigen::MatrixXd near_plane, Eigen::MatrixXd far_plane_, Eigen::Vector3d test_point,
                   Eigen::Vector3d trans);
};

class myMotionValidator : public ompl::base::MotionValidator
{
public:
  // implement checkMotion()

  myMotionValidator(ompl::base::SpaceInformation *si, octomap::OcTree *oct) : ompl::base::MotionValidator(si)
  {
    oct_ = oct;
  }

  myMotionValidator(const ompl::base::SpaceInformationPtr &si, octomap::OcTree *oct) : ompl::base::MotionValidator(si)
  {
    oct_ = oct;
  }

  virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
  {
    double hx, hy, hz;
    hx = 0.25;
    hy = 0.25;
    hz = 0.1;
    double resolution = 0.03;

    const ob::SE3StateSpace::StateType *se3state1 = s1->as<ob::SE3StateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType *pos1 = se3state1->as<ob::RealVectorStateSpace::StateType>(0);
    const ob::SE3StateSpace::StateType *se3state2 = s2->as<ob::SE3StateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType *pos2 = se3state2->as<ob::RealVectorStateSpace::StateType>(0);
    octomap::KeyRay ray;

    for (double x = -hx; x <= hx; x += 0.08)
    {
      for (double y = -hy; y <= hy; y += 0.08)
      {
        for (double z = -hz; z <= hz; z += 0.05)
        {
          octomap::point3d start(pos1->values[0] + x, pos1->values[1] + y, pos1->values[2] + z);
          octomap::point3d end(pos2->values[0] + x, pos2->values[1] + y, pos2->values[2] + z);
          oct_->computeRayKeys(start, end, ray);

          for (octomap::OcTreeKey key : ray)
          {
            octomap::OcTreeNode *node = oct_->search(key);
            if (node != NULL)
            {
              if (oct_->isNodeOccupied(node))
              {
                std::cout << "this node is occupied " << std::endl;
                return false;
              }
            }
            /*if (node == NULL)
            {
              return false;
            }
            else if (oct_->isNodeOccupied(node))
            {
              return false;
            }*/
          }
        }
      }
    }

    return true;
  }

  virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                           std::pair<ompl::base::State *, double> &last_valid) const
  {
    return true;
  }

  octomap::OcTree *oct_;
};

}  // namespace planner