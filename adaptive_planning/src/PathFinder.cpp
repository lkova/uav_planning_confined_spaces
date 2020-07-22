#include "adaptive_planning/PathFinder.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

///// TO DO LIST
/*
 - put subsribers and publishers to ros params
 - split global and local planning
 - global not needed in basement environment, but useful overall
 - meh exploration and use nbvplanner as much as possible
*/
namespace planner
{
PathFinder::PathFinder(Eigen::MatrixXd far_plane, Eigen::Matrix3d roty, Eigen::Vector4d params)
{
  far_plane_ = far_plane;
  roty_ = roty;
  ros::NodeHandle nh_;
  params_ = params;
  // manager_ = new volumetric_mapping::OctomapManager(nh_, nh_);
  drone_ = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.5, 0.5, 0.2));
  fcl::OcTree *tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.1)));
  tree_obj_ = std::shared_ptr<fcl::CollisionGeometry>(tree);

  space_ = ob::StateSpacePtr(new ob::SE3StateSpace());

  // set them to parameters
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, -1);
  bounds.setHigh(0, 15);
  bounds.setLow(1, -1.4);
  bounds.setHigh(1, 1.4);
  bounds.setLow(2, 0);
  bounds.setHigh(2, 13);

  space_->as<ob::SE3StateSpace>()->setBounds(bounds);
  si_ = ob::SpaceInformationPtr(new ob::SpaceInformation(space_));
  si_->setStateValidityChecker(std::bind(&planner::PathFinder::isValid, this, std::placeholders::_1));

  // robot size x, y, z
}

PathFinder::~PathFinder()
{
}

// check if path is valid - no collisions inside of the global planner
bool PathFinder::isValid(const ob::State *state)
{
  /*const ob::SE3StateSpace::StateType *se3state =
  state->as<ob::SE3StateSpace::StateType>();

  // extract the first component of the state and cast it to what we expect
  const ob::RealVectorStateSpace::StateType *pos =
  se3state->as<ob::RealVectorStateSpace::StateType>(0);

  // extract the second component of the state and cast it to what we expect
  const ob::SO3StateSpace::StateType *rot =
  se3state->as<ob::SO3StateSpace::StateType>(1);

  fcl::CollisionObject treeObj((tree_obj_));
  fcl::CollisionObject aircraftObject(drone_);

  // check validity of state defined by pos & rot
  fcl::Vec3f translation(pos->values[0], pos->values[1], pos->values[2]);
  fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
  aircraftObject.setTransform(rotation, translation);
  fcl::CollisionRequest requestType(1, false, 1, false);
  fcl::CollisionResult collisionResult;
  fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

  return (!collisionResult.isCollision());*/

  // size of drone, set to parameters
  double hx, hy, hz;
  hx = 0.25;
  hy = 0.25;
  hz = 0.1;

  const ob::SE3StateSpace::StateType *se3state1 = state->as<ob::SE3StateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType *pos1 = se3state1->as<ob::RealVectorStateSpace::StateType>(0);

  octomap::KeyRay ray;

  for (double x = -hx; x <= hx; x += 0.05)
  {
    for (double y = -hy; y <= hy; y += 0.05)
    {
      for (double z = -hz; z <= hz; z += 0.05)
      {
        octomap::point3d start(pos1->values[0] - x, pos1->values[1] - y, pos1->values[2] - z);
        octomap::point3d end(pos1->values[0] + x, pos1->values[1] + y, pos1->values[2] + z);
        oct_tree_->computeRayKeys(start, end, ray);

        for (octomap::OcTreeKey key : ray)
        {
          octomap::OcTreeNode *node = oct_tree_->search(key);
          if (node != NULL)
          {
            if (oct_tree_->isNodeOccupied(node))
            {
              std::cout << "this node is occupied " << std::endl;
              return false;
            }
          }
        }
      }
    }
  }

  return true;
}

bool PathFinder::plan(std::vector<Eigen::Vector3d> &path)
{
  if (!ready_pose_)
  {
    return false;
  }
  // create a problem instance
  og::PathGeometric *path_smooth = NULL;

  // setup start
  ob::ScopedState<ob::SE3StateSpace> start(space_);
  start->setXYZ(start_pose_.x(), start_pose_.y(), start_pose_.z());
  start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

  // setup goal
  ob::ScopedState<ob::SE3StateSpace> goal(space_);

  goal->setXYZ(goal_position_.x(), goal_position_.y(), goal_position_.z());
  goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

  // setup motion validator with new oct_tree
  si_->setMotionValidator(std::make_shared<myMotionValidator>(si_, oct_tree_));

  // set Optimizattion objective
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si_));
  obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);

  // Setup problem definiton
  ob::ProblemDefinitionPtr pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si_));
  pdef->setStartAndGoalStates(start, goal);
  pdef->setOptimizationObjective(obj);

  ob::PlannerPtr plan(new og::BiTRRT(si_));
  plan->setProblemDefinition(pdef);
  plan->setup();

  // si_->printSettings(std::cout);

  // print the problem settings
  // pdef->print(std::cout);

  ob::PlannerStatus solved = plan->solve(3);
  ROS_INFO("is straight path valid %d ", pdef->isStraightLinePathValid());

  if (solved && ready_pose_ && pdef->hasSolution())
  {
    // ob::PathPtr path = pdef->getSolutionPath();
    og::PathGeometric *path_smooth = pdef->getSolutionPath()->as<og::PathGeometric>();

    // path_smooth->printAsMatrix(std::cout);
    og::PathSimplifier *pathBSpline = new og::PathSimplifier(si_);
    // path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric
    // &>(*pdef->getSolutionPath()));
    pathBSpline->smoothBSpline(*path_smooth, 3);

    ROS_INFO("count number of states %d", path_smooth->getStateCount());
    for (std::size_t idx = 0; idx < path_smooth->getStateCount(); idx++)
    {
      // cast the abstract state type to the type we expect
      const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(idx)->as<ob::SE3StateSpace::StateType>();

      // extract the first component of the state and cast it to what we expect
      const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

      // extract the second component of the state and cast it to what we expect
      const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

      Eigen::Vector3d pos_value(pos->values[0], pos->values[1], pos->values[2]);
      path.push_back(pos_value);

      // ROS_INFO("path %lf %lf %lf", pos->values[0], pos->values[1],
      // pos->values[2]);
    }
    ROS_INFO("path solved");
    pdef->clearSolutionPaths();
    return true;
  }
  return false;
}

void PathFinder::goalSelector(Eigen::Vector3d &goal, bool goal_set)
{
  if (goal_set)
  {
    goal_position_ = goal;
    // ready_pose_ = true;
  }

  else
  {
    // ready_pose_ = false;
  }
}

// calculate gain inside of camera frustum
double PathFinder::evaluateGain(Eigen::Vector3d random_pose, double yaw)
{
  Eigen::Matrix3d rotz;
  rotz = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  double gain = 0.0;
  // double max_distance = params_[0];
  // double min_distance = params_[1];
  double min_distance = 0.001;
  double max_distance = 1.5;
  double half_horizontal_fov = params_[2] / 2;
  double half_vertical_fov = params_[2] / 2;

  double resolution = 0.05 * 4;
  double step_size = atan(resolution / 1);
  Eigen::Vector3d origin(start_pose_[0], start_pose_[1], start_pose_[2]);
  // std::cout << "hv vv " << tan(-half_horizontal_fov) * min_distance << " " <<
  // tan(half_horizontal_fov) * min_distance
  //          << std::endl;
  points_visualization_.clear();
  for (double hv = -half_horizontal_fov; hv <= half_horizontal_fov; hv += step_size)
  {
    for (double vv = -half_vertical_fov; vv <= half_vertical_fov; vv += step_size)
    {
      Eigen::Vector3d start_point(min_distance, tan(hv) * min_distance, tan(vv) * min_distance);
      Eigen::Vector3d end_point(max_distance, tan(hv) * max_distance, tan(vv) * max_distance);

      start_point = rotz * roty_ * start_point + random_pose + origin;
      end_point = rotz * roty_ * end_point + random_pose + origin;

      gain += getVisibility(start_point, end_point, true);
    }
  }

  return gain;
}

// void PathFinder::setPose(const geometry_msgs::PoseConstPtr &msg)
// {
//   double yaw = tf::getYaw(msg->orientation);

//   start_pose_[0] = msg->position.x;
//   start_pose_[1] = msg->position.y;
//   start_pose_[2] = msg->position.z;
//   start_pose_[3] = tf::getYaw(msg->orientation);
//   // std::cout << "this is pose " << start_pose_ << std::endl;
//   ready_pose_ = true;
// }

// Set pose from covariance message, but with real drone we need Odometry
void PathFinder::setPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  double yaw = tf::getYaw(msg->pose.pose.orientation);

  start_pose_[0] = msg->pose.pose.position.x;
  start_pose_[1] = msg->pose.pose.position.y;
  start_pose_[2] = msg->pose.pose.position.z;
  start_pose_[3] = yaw;
  // std::cout << "this is pose " << start_pose_ << std::endl;
  ready_pose_ = true;
}

void PathFinder::setOctomap(const octomap_msgs::Octomap::ConstPtr &msg)
{
  oct_tree_ = dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg));
  // ROS_INFO("octomap loaded");
  if (oct_tree_->size() > 0)
  {
    fcl::OcTree *tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(oct_tree_));

    tree_obj_ = std::shared_ptr<fcl::CollisionGeometry>(tree);
    ready_tree_ = true;
    // ROS_INFO("resolution %d", oct_tree_->getResolution());
  }
}

double PathFinder::getVisibility(const Eigen::Vector3d &view_point, const Eigen::Vector3d &voxel_to_test,
                                 bool stop_at_unknown_cell)
{
  octomap::KeyRay key_ray;

  oct_tree_->computeRayKeys(pointEigenToOctomap(view_point), pointEigenToOctomap(voxel_to_test), key_ray);

  double gain = 0.0;
  octomap::point3d point_octomap;
  Eigen::Vector3d point_eigen;

  for (octomap::OcTreeKey key : key_ray)
  {
    octomap::OcTreeNode *node = oct_tree_->search(key);

    if (node == NULL)
    {
      point_octomap = oct_tree_->keyToCoord(key);
      point_eigen = pointOctomapToEigen(point_octomap);

      Eigen::Vector3d du = point_eigen - view_point;
      double dist = du.norm();

      // it can be locked to certain place because tank size is known.
      // due to imperfect octomap it can count pixels that it shouldn't :)
      if (point_eigen.z() > 2.0)
      {
        break;
      }
      if (stop_at_unknown_cell && dist > 1.0)
      {
        // std::cout << "it is here" << std::endl;
        gain = 1.0;
        points_visualization_.push_back(point_eigen);
        break;
      }
      // added check with height
    }
    else if (oct_tree_->isNodeOccupied(node))
    {
      gain = 0;
      break;
    }
  }
  // std::cout << "gain from visibility " << gain << std::endl;
  return gain;
}

double PathFinder::getCellProbabilityPoint(const Eigen::Vector3d &point, double *probability)
{
  octomap::OcTreeNode *node = oct_tree_->search(point.x(), point.y(), point.z());
  if (node == NULL)
  {
    if (probability)
    {
      *probability = -1.0;
    }
    return 2.0;
  }
  else
  {
    if (probability)
    {
      *probability = node->getOccupancy();
    }
    if (oct_tree_->isNodeOccupied(node))
    {
      return 1.0;
    }
    else
    {
      return 0.0;
    }
  }
}

bool PathFinder::isPathFeasible(const Eigen::Vector3d &test_voxel, const Eigen::Vector3d &origin_voxel)
{
  // SWITCH TO PARAMETERS ASAP
  double hx, hy, hz;
  hx = 0.3;
  hy = 0.3;
  hz = 0.1;

  // hx = 0.5;
  // hy = 0.5;
  // hz = 0.3;

  octomap::KeyRay ray;
  int free_count = 0;
  int total = 0;
  int unknown_count = 0;
  // std::cout << "origin " << origin_voxel << std::endl;
  // std::cout << "testi " << test_voxel << std::endl;
  int occupied_count = 0;
  Eigen::Vector3d addi(hx + 0.1, hy + 0.1, hz);

  for (double x = -hx; x <= hx; x += 0.03)
  {
    for (double y = -hy; y <= hy; y += 0.03)
    {
      for (double z = -hz; z <= hz; z += 0.03)
      {
        octomap::point3d start(origin_voxel[0] + x, origin_voxel[1] + y, origin_voxel[2] + z);
        octomap::point3d end(test_voxel[0] + origin_voxel[0] + x, test_voxel[1] + origin_voxel[1] + y,
                             test_voxel[2] + origin_voxel[2] + z);

        // octomap::point3d start(3 + x, 0 + y, 6 + z);
        // octomap::point3d end(4 + x, 0 + y, 6 + z);
        oct_tree_->computeRayKeys(start, end, ray);

        for (octomap::OcTreeKey key : ray)
        {
          octomap::point3d pp = oct_tree_->keyToCoord(key);
          Eigen::Vector3d dd(pp.x(), pp.y(), pp.z());

          // points_visualization_max_.push_back(dd);

          octomap::OcTreeNode *node = oct_tree_->search(key);
          if ((dd.array() < (origin_voxel - addi).array()).all() || (dd.array() > (origin_voxel + addi).array()).all())
          {
            if (node != NULL)
            {
              if (oct_tree_->isNodeOccupied(node))
              {
                std::cout << "  OCCUPIED" << std::endl;
                // points_visualization_max_.clear();
                // if (dd.x() < -0.235 || dd.x() > -0.215)
                // {
                //   std::cout << "last standing point " << dd << std::endl;
                //   occu++;
                // }
                occupied_count++;
                if (occupied_count > 3)
                  return false;
              }
              else
              {
                free_count++;
              }
            }
            else if (node == NULL)
            {
              unknown_count++;
              if (unknown_count > 50)
              {
                // std::cout << origin_voxel + test_voxel << std::endl;
                // std::cout << origin_voxel - addi << std::endl;
                // std::cout << origin_voxel + addi << std::endl;
                return false;
              }
            }
          }
        }
      }
    }
  }

  if (free == 0)
  {
    ROS_INFO("path not feasible");
    return false;
  }
  ROS_INFO("path is feasible, free voxel count %d", free_count);

  return true;
}

bool PathFinder::poseToExplore(std::vector<Eigen::Vector4d> &path)
{
  // find goal by exploring

  // future params= {max_radius, number_of_poses}
  // maybe stop searching for poses if the max gain is bigger than threshold
  // add exponential decaying based on distance
  if (ready_pose_ == false)
  {
    return false;
  }

  // setFree();
  double max_radius = 0.5;
  int number_of_poses = 3;
  int generate_poses = 20;
  // std::vector<Eigen::Vector4d> random_poses;

  double max_gain = 0;

  Eigen::Vector3d origin_position(start_pose_[0], start_pose_[1], start_pose_[2]);
  Eigen::Vector4d best_pose;
  // ROS_INFO("started pose generation");
  std::vector<Eigen::Vector4d> random_poses;
  int counter = 0;
  std::vector<std::vector<Eigen::Vector3d>> aaa;
  for (int i = 0; i < generate_poses; i++)
  {
    // i = 0;
    Eigen::Vector3d random_pose;
    double phi = randMToN(0, M_PI * 2.0);
    double theta = randMToN(-M_PI / 2.0, M_PI / 2.0);
    // double theta = randMToN(-M_PI / 4.0, M_PI / 4.0);
    double r = randMToN(0.0, max_radius);

    // x
    random_pose[0] = r * sin(theta) * cos(phi);
    // y
    random_pose[1] = r * sin(theta) * sin(phi);
    // z
    random_pose[2] = r * cos(theta);
    // yaw
    double yaw = randMToN(-M_PI, M_PI);

    points_visualization_max_.clear();
    /*random_pose[0] = 0.6;
    random_pose[1] = 0;
    random_pose[2] = 0.0;*/
    // std::cout << "What in the world is happening " << std::endl;
    if (isPathFeasible(random_pose, origin_position))
    {
      counter++;
      random_poses.push_back(Eigen::Vector4d(random_pose[0], random_pose[1], random_pose[2], yaw));
    }
    if (counter == number_of_poses)
      break;
  }

  // ROS_INFO("done with pose generation");
  // std::cout << "number of feasible " << counter << std::endl;
  points_visualization_max_.clear();
  int i = 0;
  for (Eigen::Vector4d pose : random_poses)
  {
    Eigen::Vector3d random_pose(pose[0], pose[1], pose[2]);
    double yaw = pose[3];
    double gain = evaluateGain(random_pose, yaw);

    if (gain > max_gain)
    {
      max_gain = gain;
      best_pose[0] = random_pose[0] + origin_position[0];
      best_pose[1] = random_pose[1] + origin_position[1];
      best_pose[2] = random_pose[2] + origin_position[2];
      best_pose[3] = yaw;
      points_visualization_max_ = points_visualization_;
    }
    i++;
    // std::cout << "gain " << gain << std::endl;
  }

  // ROS_INFO("ended");
  if (max_gain == 0)
  {
    return false;
  }

  // std::cout << "best pose " << best_pose << std::endl;
  // std::cout << "max gain " << max_gain << std::endl;
  path.push_back(best_pose);
  return true;
}

void PathFinder::sphereCoords()
{
  double theta = randMToN(0, M_PI * 2.0);
  double phi = randMToN(-M_PI / 2.0, M_PI / 2.0);
  double r = randMToN(0.0, 1);

  double angle_step = 0.1;
  double radius_step = 0.2;
  double min_radius = 0.0;
  double max_radius = 1.0;
  Eigen::Vector3d origin(start_pose_[0], start_pose_[1], start_pose_[2]);

  for (double r = max_radius; r >= min_radius; r -= radius_step)
  {
    for (double phi = M_PI / 4.0; phi >= 0.0; phi -= angle_step)
    {
      for (double theta = 0; theta < M_PI * 2.0; theta += angle_step)
      {
        double dx = r * sin(phi) * cos(theta);
        double dy = r * sin(phi) * sin(theta);
        double dz = r * cos(theta);

        // is in frustum
        origin[0] += dx;
        origin[1] += dy;
        origin[2] += dz;
        double yaw = atan2(dy, dx);
      }
    }
  }
}

void PathFinder::visualizeFrustum(Eigen::Vector3d point, double yaw)
{
  Eigen::Matrix3d rotz;
  rotz = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  double gain = 0.0;
  double max_distance = params_[0];
  double min_distance = params_[1];
  double half_horizontal_fov = params_[2] / 2;
  double half_vertical_fov = params_[2] / 2;

  double resolution = 0.05 * 4;
  double step_size = atan(resolution / min_distance);
  Eigen::Vector3d origin(start_pose_[0], start_pose_[1], start_pose_[2]);
  // std::cout << "hv vv " << tan(-half_horizontal_fov) * min_distance << " " <<
  // tan(half_horizontal_fov) * min_distance
  //          << std::endl;
  for (double hv = -half_horizontal_fov; hv <= half_horizontal_fov; hv += step_size)
  {
    for (double vv = -half_vertical_fov; vv <= half_vertical_fov; vv += step_size)
    {
      Eigen::Vector3d start_point(min_distance, tan(hv) * min_distance, tan(vv) * min_distance);
      Eigen::Vector3d end_point(max_distance, tan(hv) * max_distance, tan(vv) * max_distance);

      start_point = roty_ * rotz * start_point + point + origin;
      end_point = roty_ * rotz * end_point + point + origin;
    }
  }
}

void PathFinder::setFree()
{
  const Eigen::Vector3d position(start_pose_[0], start_pose_[1], start_pose_[2]);
  const Eigen::Vector3d bounding_box_size(0.65, 0.65, 0.2);
  double log_odds_value = oct_tree_->getClampingThresMinLog();
  const bool lazy_eval = true;
  const double resolution = 0.03;
  const double epsilon = 0.001;  // Small offset to not hit boundary of nodes.
  Eigen::Vector3d epsilon_3d;
  epsilon_3d.setConstant(epsilon);

  Eigen::Vector3d bbx_min = position - bounding_box_size / 2 - epsilon_3d;
  Eigen::Vector3d bbx_max = position + bounding_box_size / 2 + epsilon_3d;
  // std::cout << position << std::endl;
  // std::cout << bbx_min << std::endl;
  std::cout << start_pose_ << std::endl;
  for (double x_position = bbx_min.x(); x_position <= bbx_max.x(); x_position += resolution)
  {
    for (double y_position = bbx_min.y(); y_position <= bbx_max.y(); y_position += resolution)
    {
      for (double z_position = bbx_min.z(); z_position <= bbx_max.z(); z_position += resolution)
      {
        octomap::point3d point = octomap::point3d(x_position, y_position, z_position);
        octomap::OcTreeNode *node = oct_tree_->search(point);
        if (node == NULL)

          oct_tree_->setNodeValue(point, log_odds_value, lazy_eval);
      }
    }
  }
  // This is necessary since lazy_eval is set to true.
  // oct_tree_->updateInnerOccupancy();
}

std::vector<Eigen::Vector3d> PathFinder::getVisPoints()
{
  return points_visualization_max_;
}

}  // namespace planner