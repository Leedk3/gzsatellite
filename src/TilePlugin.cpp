#include "gzsatellite/TilePlugin.h"

namespace fs = boost::filesystem;

namespace gazebo {

using namespace std::chrono_literals;

static const std::string root = "./gzsatellite/";

TilePlugin::TilePlugin() : pTilePlugin_(std::make_unique<TilePluginPrivate>()) {
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
  pTilePlugin_->world = _parent;

  pTilePlugin_->rosnode = gazebo_ros::Node::Get(_sdf);
  RCLCPP_INFO(pTilePlugin_->rosnode->get_logger(), "Loading Tile plugin...");
  
  this->parent_ = _parent;

  // std::string service, name;
  // double lat, lon, zoom;
  // double quality;
  // double width, height;
  // double shift_x, shift_y;

  const char* service = std::getenv("GZSATELLITE_SERVICE");
  const char* lat_str = std::getenv("GZSATELLITE_LAT");
  const char* lon_str = std::getenv("GZSATELLITE_LON");
  const char* zoom_str = std::getenv("GZSATELLITE_ZOOM");
  const char* width_str = std::getenv("GZSATELLITE_WIDTH");
  const char* height_str = std::getenv("GZSATELLITE_HEIGHT");
  const char* shift_x_str = std::getenv("GZSATELLITE_SHIFT_EW");
  const char* shift_y_str = std::getenv("GZSATELLITE_SHIFT_NS");
  const char* name = std::getenv("GZSATELLITE_NAME");
  const char* quality_str = std::getenv("GZSATELLITE_QUALITY");

  if (!service || !lat_str || !lon_str || !zoom_str || !width_str || !height_str || !shift_x_str || !shift_y_str || !name || !quality_str) {
      std::cerr << "One or more environment variables are not set." << std::endl;
      return;
  }

  double lat = std::atof(lat_str);
  double lon = std::atof(lon_str);
  double zoom = std::atof(zoom_str);
  double width = std::atof(width_str);
  double height = std::atof(height_str);
  double shift_x = std::atof(shift_x_str);
  double shift_y = std::atof(shift_y_str);
  double quality = std::atof(quality_str);

  gzmsg << "GzSatellite Plugin loaded - service: " << service << std::endl;
  gzmsg << "GzSatellite Plugin loaded - lat: " << lat << std::endl;
  gzmsg << "GzSatellite Plugin loaded - lon: " << lon << std::endl;
  gzmsg << "GzSatellite Plugin loaded - zoom: " << zoom << std::endl;
  gzmsg << "GzSatellite Plugin loaded - width: " << width << std::endl;
  gzmsg << "GzSatellite Plugin loaded - height: " << height << std::endl;
  gzmsg << "GzSatellite Plugin loaded - shift_x: " << shift_x << std::endl;
  gzmsg << "GzSatellite Plugin loaded - shift_y: " << shift_y << std::endl;
  gzmsg << "GzSatellite Plugin loaded - name: " << name << std::endl;
  gzmsg << "GzSatellite Plugin loaded - quality: " << quality << std::endl;

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