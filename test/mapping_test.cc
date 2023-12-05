/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Autor: fuzhou
 * @LastEditors: fuzhou
 */
#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "agv_config_lib/ConfigUtils.h"
#include "common_lib/gomros.h"
#include "include/mapping_and_location/mapping_and_location.h"


TEST(mapping_location_test, mapping_test) {
  gomros::common::InitMaster(1);
  using namespace gomros::data_process::mapping_and_location;
  std::string config_dir = "./data";
  MappingAndLocation* p = new MappingAndLocation(config_dir);

  while (true) {
    usleep(1);
  }
  delete p;
}
