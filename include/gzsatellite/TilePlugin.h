#include <string>
#include <sstream>
#include <memory>
#include <fstream>
#include <vector>
#include <algorithm>

#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include "modelcreator.h"

namespace gazebo {

  class TilePlugin: public rclcpp::Node, public WorldPlugin {
    public:
      TilePlugin();
      virtual ~TilePlugin();
      void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    private:
      physics::WorldPtr parent_;
  };
}

