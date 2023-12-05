/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-10-01 16:20:55
 * @LastEditTime: 2022-10-25 16:07:51
 */
/*
 * slam_karto
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

/**

@mainpage karto_gmapping

@htmlinclude manifest.html

*/
#ifndef PLUGINS_MAPPING_LIB_INCLUDE_KARTO_MAPPING_SLAM_KAR_H_
#define PLUGINS_MAPPING_LIB_INCLUDE_KARTO_MAPPING_SLAM_KAR_H_

#include <jsoncpp/json/json.h>
#include <sys/stat.h>

#include <ctime>
#include <map>
#include <string>
#include <vector>

#include "include/config_struct.h"
#include "include/mapping_lib/karto_mapping/Mapper.h"
#include "include/mapping_lib/karto_mapping/spa_solver.h"
#include "include/mapping_lib/karto_mapping/Karto.h"
#include "include/mapping_lib/karto_mapping/Math.h"
#include "message_lib/grid_map.h"
#include "message_lib/simple_grid_map.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace gomros {
namespace data_process {
namespace mapping_and_location {
class SlamKarto {
  using SimpleGridMap = gomros::message::SimpleGridMap;
  using MapInfo = gomros::message::MapInfo;
  // using LaserRangeFinder = karto::LaserRangeFinder;

 public:
  explicit SlamKarto(const KartoMappingConfig& config);
  ~SlamKarto();

  void SetConfiguration(const KartoMappingConfig& config);

  void begin_slam(std::vector<char>* output, std::string given_map_name);

  void SetLaserParam(float min_angle, float max_angle, float min_range,
                     float max_range, float resolution);

  bool map_successed;

 private:
  bool addScan(karto::LaserRangeFinder* laser, std::vector<double> scan,
               karto::Pose2& karto_pose);

  bool updateMap(std::vector<char>* output, std::string map_name);

  std::vector<std::string> SplitCString(std::string& strSource, std::string ch);

  karto::LaserRangeFinder* getLaser();

  void json_save(const char* file_name, Json::Value v);

  void json_read(const char* file_name, std::string* read_data);
  double resolution_;
  int throttle_scans_;
  // Karto bookkeeping
  karto::Mapper* mapper_;
  karto::Dataset* dataset_;
  SpaSolver* solver_;
  std::map<std::string, karto::LaserRangeFinder*> lasers_;
  std::map<std::string, bool> lasers_inverted_;
  // Internal state
  bool got_map_;
  int laser_count_;
  unsigned marker_count_;
  bool inverted_laser_;
  double laser_x_offset;
  double laser_y_offset;
  double laser_th_offset;
  KartoMappingConfig m_MappingConfig;
  karto::LaserRangeFinder* laser_;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros

#endif  // PLUGINS_MAPPING_LIB_INCLUDE_KARTO_MAPPING_SLAM_KAR_H_
