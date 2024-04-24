#include <string>
#include <sstream>
#include <memory>
#include <fstream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cstdlib>  // For getenv()

#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

// Gazebo
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>


#include "modelcreator.h"

namespace gazebo {


  class TilePluginPrivate
  {

  public:

    gazebo_ros::Node::SharedPtr rosnode;

    /// \brief Pointer to the parent actor.
    gazebo::physics::ModelPtr robotModel;

    /// \brief Pointer to the world, for convenience.
    gazebo::physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
    sdf::ElementPtr sdf;

    /// \brief connection to world iteration
    gazebo::event::ConnectionPtr connection;

    /// \brief Time of the last update.
    gazebo::common::Time lastUpdate;

    rclcpp::Time rostime;
  };

  class TilePlugin: public WorldPlugin {
    public:
      TilePlugin();
      virtual ~TilePlugin();
      void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) override;

    private:
      physics::WorldPtr parent_;
      std::unique_ptr<TilePluginPrivate> pTilePlugin_;
  };
}

