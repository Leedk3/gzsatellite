#include "gzsatellite/TilePlugin.h"

namespace fs = boost::filesystem;

namespace gazebo {

static const std::string root = "./gzsatellite/";

TilePlugin::TilePlugin() : Node("tile_plugin_node") {
}

TilePlugin::~TilePlugin() {
  // Do not call shutdown if ROS 2 is needed elsewhere
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}
// ----------------------------------------------------------------------------

void TilePlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent_ = _parent;

  std::string service, name;
  double lat, lon, zoom;
  double quality;
  double width, height;
  double shift_x, shift_y;

  // // Geographic paramters
  this->declare_parameter<std::string>("tileserver", std::string("https://xdworld.vworld.kr/2d/Satellite/service/{z}/{x}/{y}.jpeg"));
  service = this->get_parameter("tileserver").as_string();
  this->declare_parameter<double>("latitude", double(36.381365));
  lat = this->get_parameter("latitude").as_double();
  this->declare_parameter<double>("longitude", double(127.364937));
  lon = this->get_parameter("longitude").as_double();
  this->declare_parameter<double>("zoom", double(15));
  zoom = this->get_parameter("zoom").as_double();
  // Geographic size parameters
  this->declare_parameter<double>("width", double(25));
  width = this->get_parameter("width").as_double();
  this->declare_parameter<double>("height", double(25));
  height = this->get_parameter("height").as_double();
  this->declare_parameter<double>("shift_ew", double(0));
  shift_x = this->get_parameter("shift_ew").as_double();
  this->declare_parameter<double>("shift_ns", double(0));
  shift_y = this->get_parameter("shift_ns").as_double();
  // Model parameters
  this->declare_parameter<std::string>("name", std::string("ETRI"));
  name = this->get_parameter("name").as_string();
  this->declare_parameter<double>("jpg_quality", double(255));
  quality = this->get_parameter("jpg_quality").as_double();

  //
  // Create the model creator with parameters
  //

  gzsatellite::GeoParams params;
  params.tileserver   = service;
  params.lat          = lat;
  params.lon          = lon;
  params.zoom         = zoom;
  params.width        = width;
  params.height       = height;
  params.shift_x      = shift_x;
  params.shift_y      = shift_y;

  gzsatellite::ModelCreator m(params, root);

  //
  // Create a world model and add it to the Gazebo World
  //

  auto modelSDF = m.createModel(name, quality);
  this->parent_->InsertModelSDF(*modelSDF);

  gzmsg << "World model '" << name << "' (" << std::setprecision(10) << lat << "," << lon << ") created." << std::endl;

  double originLat, originLon;
  m.getOriginLatLon(originLat, originLon);
  gzdbg << std::setprecision(10) << originLat << "," << originLon << std::endl;

  // std::cout << modelSDF->ToString() << std::endl;
}

// ----------------------------------------------------------------------------

GZ_REGISTER_WORLD_PLUGIN(TilePlugin)
}