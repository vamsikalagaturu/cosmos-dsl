#include "arm_actions/utils.hpp"

Utils::Utils(std::shared_ptr<Logger> logger)
{
  _logger = logger;
}

Utils::~Utils() {}

void Utils::printLinkNames(KDL::Tree& tree)
{
  std::vector<std::string> link_names;

  // get all segments
  KDL::SegmentMap segments = tree.getSegments();

  // iterate through all segments
  for (auto segment : segments)
  {
    // get the name of the segment
    std::string name = segment.first;

    // add the name to the vector
    link_names.push_back(name);
  }

  // print the link names
  std::cout << "Link names: " << std::endl;
  for (int i = 0; i < link_names.size(); i++)
  {
    std::cout << link_names[i] << std::endl;
  }
  std::cout << std::endl;
}

void Utils::printLinkNamesFromChain(KDL::Chain& chain)
{
  std::vector<std::string> link_names;
  for (int i = 0; i < chain.getNrOfSegments(); i++)
  {
    link_names.push_back(chain.getSegment(i).getName());
  }

  std::cout << "Link names: " << std::endl;
  for (int i = 0; i < link_names.size(); i++)
  {
    std::cout << link_names[i] << std::endl;
  }
  std::cout << std::endl;
}

void Utils::printJointNames(KDL::Chain& chain)
{
  std::vector<std::string> joint_names;
  for (int i = 0; i < chain.getNrOfSegments(); i++)
  {
    joint_names.push_back(chain.getSegment(i).getJoint().getName());
  }

  std::cout << "Joint names: " << std::endl;
  for (int i = 0; i < joint_names.size(); i++)
  {
    std::cout << joint_names[i] << std::endl;
  }
  std::cout << std::endl;
}

template <typename T>
void Utils::printVec(const T& vec)
{
  std::cout << "[";
  for (int i = 0; i < vec.size(); ++i)
  {
    std::cout << vec[i] << ", ";
    if (i != vec.size() - 1)
    {
      std::cout << ", ";
    }
  }
  std::cout << "]" << std::endl;
}

template <typename T>
void Utils::printJntArr(const T& jntArr)
{
  std::cout << "[";
  for (int i = 0; i < jntArr.rows(); ++i)
  {
    std::cout << jntArr(i);
    if (i != jntArr.rows() - 1)
    {
      std::cout << ", ";
    }
  }
  std::cout << "]" << std::endl;
}

int Utils::initialize_robot(const std::string& urdf_path, KDL::Chain& robot_chain,
                            const std::string& base_link, const std::string& tool_link,
                            const std::vector<double>& initial_joint_angles, KDL::JntArray& q)
{
  KDL::Tree robot_tree;

  // load the robot URDF into the KDL tree
  if (!kdl_parser::treeFromFile(urdf_path, robot_tree))
  {
    _logger->logError("Failed to construct KDL tree");
    return -1;
  }

  // create the KDL chain
  if (!robot_tree.getChain(base_link, tool_link, robot_chain))
  {
    _logger->logError("Failed to get KDL chain");
    return -1;
  }

  // set the initial joint angles
  q.resize(robot_chain.getNrOfJoints());
  for (int i = 0; i < robot_chain.getNrOfJoints(); i++)
  {
    q(i) = initial_joint_angles[i];
  }

  _logger->logInfo("Successfully initialized robot");
  return 0;
}
