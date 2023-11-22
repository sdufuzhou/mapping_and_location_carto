#ifndef PLUGINS_MAPPING_LIB_INCLUDE_KARTO_MAPPING_VISUALIZATION_H_
#define PLUGINS_MAPPING_LIB_INCLUDE_KARTO_MAPPING_VISUALIZATION_H_

#include <ros/ros.h>
#include <include/karto_mapping/sba.h>
#include <visualization_msgs/Marker.h>

// namespace gomros {
// namespace data_process {
// namespace mapping_and_location {
namespace sba {

// draw the graph on rviz
void drawGraph(const SysSBA &sba, const ros::Publisher &camera_pub,
               const ros::Publisher &point_pub, int decimation = 1, int bicolor = 0);

}  // namespace sba
// }  // namespace mapping_and_location
// }  // namespace data_process
// }  // namespace gomros

#endif
