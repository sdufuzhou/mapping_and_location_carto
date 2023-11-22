/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-07-26 19:26:31
 * @LastEditTime: 2022-09-28 21:24:34
 * @Author: lcfc-desktop
 */
#ifndef PLUGINS_LOCATION_LIB_INCLUDE_SCAN_MATCHING_SCAN_MATCHING_H_
#define PLUGINS_LOCATION_LIB_INCLUDE_SCAN_MATCHING_SCAN_MATCHING_H_

#include <Eigen/Core>
#include <vector>

#include "message_lib/grid_map.h"
#include "message_lib/position_message.h"
#include "message_lib/radar_message.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

using gomros::message::MapInfo;
using gomros::message::GlobalCoordinateMap;
using gomros::message::Position;
using gomros::message::RadarSensoryMessage;

class GridArrayAdapter {
 public:
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(GlobalCoordinateMap * grid) : grid_(grid) {
        info = grid_->get_info();
    }

    void GetValue(int row, int column, double *value) const {
      if (row < 0 || column < 0 || row >= info.miMapWidth
          || column >= info.miMapHeight) {
        *value = 1.;
      } else {
        double d = grid_->get((row * info.mdResolution +
                               info.mdOriginXInWorld),
                              (column * info.mdResolution +
                               info.mdOriginYInWorld));
        if (d < 0) {
          d = 0;
        }
        *value = 1 - d;
      }
    }

    int NumRows() const {
    return info.miMapWidth;
    }

    int NumCols() const {
    return info.miMapHeight;
    }

 private:
    MapInfo info;
    GlobalCoordinateMap * grid_;
};

class ScanMatchingOptions {
 public:
    double occupied_space_cost_factor = 1.;
    double translation_delta_cost_factor = 10.;
    double rotation_delta_cost_factor = 20.;

    int max_num_iterations = 10;
    int num_threads = 2;

    bool use_nonmonotonic_steps = false;
};

Position ScanMatching(const Position &init,
                      GlobalCoordinateMap *map,
                      const RadarSensoryMessage &range_data,
                      const ScanMatchingOptions &option);

}  // namespace location
}  // namespace data_process
}  // namespace gomros

#endif  // PLUGINS_LOCATION_LIB_INCLUDE_SCAN_MATCHING_SCAN_MATCHING_H_


