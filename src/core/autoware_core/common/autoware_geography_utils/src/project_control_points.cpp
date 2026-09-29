#include <autoware/geography_utils/projection.hpp>

#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <geographic_msgs/msg/geo_point.hpp>

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

struct ControlPoint
{
  std::string name;
  double latitude;
  double longitude;
};

int main()
{
  using MapProjectorInfo = autoware_map_msgs::msg::MapProjectorInfo;
  using GeoPoint = geographic_msgs::msg::GeoPoint;

  // ------------------------------------------------------------
  // Map projector configuration
  // Origin = control point 8598
  // ------------------------------------------------------------
  MapProjectorInfo projector_info;

  projector_info.projector_type =
    MapProjectorInfo::TRANSVERSE_MERCATOR;

  projector_info.vertical_datum =
    MapProjectorInfo::WGS84;

  projector_info.map_origin.latitude =
    40.44374890693742;

  projector_info.map_origin.longitude =
    -79.94547730185674;

  // Z is not used for our 2D registration.
  projector_info.map_origin.altitude = 0.0;
  projector_info.scale_factor = 0.9996;

  // ------------------------------------------------------------
  // GNSS control points
  // ------------------------------------------------------------
  const std::vector<ControlPoint> points = {
    {"8594", 40.44366871753468,  -79.94547289855265},
    {"8596", 40.443691818735964, -79.94554896573243},
    {"8598", 40.44374890693742,  -79.94547730185674},
    {"8600", 40.44382173703159,  -79.94553250064807},
    {"8602", 40.443731670909926, -79.94563042514910},
    {"8604", 40.443676733591445, -79.94535882223288},
    {"8608", 40.443741528890165, -79.94527667063920}
  };

  std::cout << std::fixed << std::setprecision(9);

  std::cout << "Point"
            << "\tX [m]"
            << "\tY [m]"
            << "\tZ [m]"
            << std::endl;

  for (const auto & p : points) {
    GeoPoint geo_point;

    geo_point.latitude = p.latitude;
    geo_point.longitude = p.longitude;

    // We only care about XY here.
    geo_point.altitude = 0.0;

    const auto local =
      autoware::geography_utils::project_forward(
        geo_point, projector_info);

    std::cout
      << p.name << "\t"
      << local.x << "\t"
      << local.y << "\t"
      << local.z << std::endl;
  }

  return 0;
}