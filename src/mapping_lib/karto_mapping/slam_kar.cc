/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-08-06 11:11:30
 * @LastEditTime: 2022-11-09 15:12:40
 * @Author: lcfc-desktop
 */

#include "include/mapping_lib/karto_mapping/slam_kar.h"

#include <fstream>
#include <iostream>
#include <string>

namespace gomros {
namespace data_process {
namespace mapping_and_location {

SlamKarto::SlamKarto(const KartoMappingConfig& config)
    : got_map_(false), laser_count_(0), marker_count_(0) {
  SetConfiguration(config);
  map_successed = false;
  mapper_ = new karto::Mapper();
  dataset_ = new karto::Dataset();

  // Setting General Parameters from the Parameter Server
  // 是否使用 ScanMatching 算法， 设置为true则会算法上纠正里程计的误差
  bool use_scan_matching = true;
  mapper_->setParamUseScanMatching(use_scan_matching);
  // 是否使用每个 scan 的质心来查看两个 scan 的距离
  bool use_scan_barycenter = true;
  mapper_->setParamUseScanBarycenter(use_scan_barycenter);
  // 设置建立节点最小时间段， 如果超过这个时间里程计小于最小移动距离，则建立一个节点
  // double minimum_time_interval = 3600;
  double minimum_time_interval = 3600;
  mapper_->setParamMinimumTimeInterval(minimum_time_interval);
  // 设置最小距离，里程计移动长度超过此距离，则建立一个节点
  double minimum_travel_distance = config.travel_distance;
  mapper_->setParamMinimumTravelDistance(minimum_travel_distance);
  // double minimum_travel_heading = 0.5;
  double minimum_travel_heading = config.travel_angle;
  // 设置最小偏转角，如果里程计转向超过此值，则建立一个节点
  //  mapper_->setParamMinimumTravelHeading(karto::math::DegreesToRadians(10.0));
  mapper_->setParamMinimumTravelHeading(minimum_travel_heading);
  // 设置 ScanBuffer 的长度， 其应该设置为约 ScanBufferMaximumScanDistance / MinimumTravelDistance
  int scan_buffer_size = 100;
  mapper_->setParamScanBufferSize(scan_buffer_size);
  // 设置 ScanBuffer 的最大长度和 Size作用类似
  double scan_buffer_maximum_scan_distance = 20.0;
  mapper_->setParamScanBufferMaximumScanDistance(
      scan_buffer_maximum_scan_distance);
  // 设置 最小scans 连接 的最小响应阈值
  double link_match_minimum_response_fine = 0.8;
  mapper_->setParamLinkMatchMinimumResponseFine(
      link_match_minimum_response_fine);
  // 设置两个连接的 scans 最大距离， 大于此值则不考虑两者的响应阈值	
  double link_scan_maximum_distance = 10.0;
  mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);
  // 搜寻 回环匹配的最大距离
  double loop_search_maximum_distance = config.loop_search_distance;
  mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);
  // mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);
  // 是否做回环匹配优化
  bool do_loop_closing = true;
  mapper_->setParamDoLoopClosing(do_loop_closing);
  // 找到的回环匹配的node，必须位于大于此值的 ScanBuffer上	
  // int loop_match_minimum_chain_size = 10;
   int loop_match_minimum_chain_size = 20;
  mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);
  // 回环匹配时粗匹配的最大协方差值， 小于此值才认为是一个可行解
  double loop_match_maximum_variance_coarse = 0.9;
  // mapper_->setParamLoopMatchMaximumVarianceCoarse(
  //     karto::math::Square(0.4));
  mapper_->setParamLoopMatchMaximumVarianceCoarse(
      loop_match_maximum_variance_coarse);
  // 回环匹配时粗匹配的最小响应， 响应值大于此值将会开始粗精度的回环优化
  double loop_match_minimum_response_coarse = 0.5;
  // mapper_->setParamLoopMatchMinimumResponseCoarse(
  //     0.6);
  mapper_->setParamLoopMatchMinimumResponseCoarse(
      loop_match_minimum_response_coarse);
  // 回环匹配最小响应阈值，大于此值才开始进行高精度匹配	
  double loop_match_minimum_response_fine = 0.6;
  // mapper_->setParamLoopMatchMinimumResponseFine(
  //     0.8);
  mapper_->setParamLoopMatchMinimumResponseFine(
      loop_match_minimum_response_fine);
  // 纠正位姿时使用的匹配器的大小
  double correlation_search_space_dimension = 0.3;
  mapper_->setParamCorrelationSearchSpaceDimension(
      correlation_search_space_dimension);
  // 纠正位姿时使用的解析度
  double correlation_search_space_resolution = 0.01;
  mapper_->setParamCorrelationSearchSpaceResolution(
      correlation_search_space_resolution);
  // 纠正位姿时将会被此值平滑
  double correlation_search_space_smear_deviation = 0.03;
  mapper_->setParamCorrelationSearchSpaceSmearDeviation(
      correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters from the Parameter
  // Server
  // 回环检测时 匹配器的大小
  double loop_search_space_dimension = 9.0;
  mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);
  // 回环检测时 匹配器的大小
  double loop_search_space_resolution = 0.05;
  mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);
  // 回环检测时的 平滑系数
  double loop_search_space_smear_deviation = 0.03;
  mapper_->setParamLoopSearchSpaceSmearDeviation(
      loop_search_space_smear_deviation);
  // Setting Scan Matcher Parameters from the Parameter Server
  // scan-matching时 对里程计的补偿系数
  // double distance_variance_penalty = 0.3;
   double distance_variance_penalty = 0.09;
  // mapper_->setParamDistanceVariancePenalty(0.2);
  mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);
  // scan-matching时 对角度的补偿系数
  // double angle_variance_penalty = 0.174;
   double angle_variance_penalty = 0.349055 * 0.349055;
  // mapper_->setParamAngleVariancePenalty(karto::math::Square(karto::math::DegreesToRadians(20)));
  mapper_->setParamAngleVariancePenalty(angle_variance_penalty);
  // 精匹配时搜索的角度范围
  // double fine_search_angle_offset = 0.00174;
   double fine_search_angle_offset = 0.00349;
  // mapper_->setParamFineSearchAngleOffset(karto::math::DegreesToRadians(0.2));
  mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);
  // 粗匹配时搜索的角度范围	
  // double coarse_search_angle_offset = 0.174;
  double coarse_search_angle_offset = 0.349;
  // mapper_->setParamCoarseSearchAngleOffset(karto::math::DegreesToRadians(20));
  mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);
  // 粗匹配时的角度解析度
  // double coarse_angle_resolution = 0.0174;
  double coarse_angle_resolution = 0.0349;
  // mapper_->setParamCoarseAngleResolution(karto::math::DegreesToRadians(2));
  mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);
  // 最小角度补偿，防止评分过小	
  double minimum_angle_penalty = 0.9;
  mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);
  // 最小距离补偿，防止评分过小
  double minimum_distance_penalty = 0.5;
  mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);
  // 在没有发现好的匹配的情况下，是否增加搜索范围	
  bool use_response_expansion = false;
  mapper_->setParamUseResponseExpansion(use_response_expansion);

  // Set solver to be used in loop closure
  solver_ = new SpaSolver();
  mapper_->SetScanSolver(solver_);
  // getLaser();
}

SlamKarto::~SlamKarto() {
  if (solver_) delete solver_;
  if (mapper_) delete mapper_;
  if (dataset_) delete dataset_;
  // TODO: delete the pointers in the lasers_ map; not sure whether or not
  // I'm supposed to do that.
}

void SlamKarto::SetConfiguration(const KartoMappingConfig& config) {
  m_MappingConfig = config;
}

std::vector<std::string> SlamKarto::SplitCString(std::string& str,
                                                 std::string delimit) {
  std::vector<std::string> result;
  size_t pos = str.find(delimit);
  str += delimit;  //将分隔符加入到最后一个位置，方便分割最后一位
  while (pos != std::string::npos) {
    result.push_back(str.substr(0, pos));
    str = str.substr(
        pos +
        1);  // substr的第一个参数为起始位置，第二个参数为复制长度，默认为string::npos到最后一个位置
    pos = str.find(delimit);
  }
  return result;
}
karto::LaserRangeFinder* SlamKarto::getLaser() {
  // Check whether we know about this laser yet

  // Create a laser range finder device and copy in data from the first
  // scan
  // karto::Name name("laser0");
  std::string name("laser0");
  karto::LaserRangeFinder* laser =
      karto::LaserRangeFinder::CreateLaserRangeFinder(
          karto::LaserRangeFinder_Custom, karto::Name(name));
  laser->SetOffsetPose(
      karto::Pose2(laser_x_offset, laser_y_offset, laser_th_offset));
  laser->SetMinimumRange(0.1);
  laser->SetMaximumRange(19);
  laser->SetMinimumAngle(karto::math::DegreesToRadians(-95));
  laser->SetMaximumAngle(karto::math::DegreesToRadians(95));
  laser->SetAngularResolution(karto::math::DegreesToRadians(0.5));
  // TODO: expose this, and many other parameters
  // laser_->SetRangeThreshold(12.0);

  // Store this laser device for later
  lasers_[name] = laser;

  // Add it to the dataset, which seems to be necessary
  dataset_->Add(laser);

  return lasers_[name];
}

void SlamKarto::SetLaserParam(float min_angle, float max_angle, float min_range,
                              float max_range, float resolution) {
  if (min_range < 0) min_range = 0.5;
  if (max_range < 0) max_range = 20;
  std::string name("laser0");
  laser_x_offset = m_MappingConfig.radar_position_x;
  laser_y_offset = m_MappingConfig.radar_position_y;
  laser_th_offset = m_MappingConfig.radar_position_theta;
  laser_ = karto::LaserRangeFinder::CreateLaserRangeFinder(
      karto::LaserRangeFinder_Custom, karto::Name(name));
  laser_->SetOffsetPose(
      karto::Pose2(laser_x_offset, laser_y_offset, laser_th_offset));
  laser_->SetMinimumRange(min_range);
  laser_->SetMaximumRange(max_range);
  laser_->SetMinimumAngle(min_angle);
  laser_->SetMaximumAngle(max_angle);
  laser_->SetAngularResolution(resolution);
  dataset_->Add(laser_);
}

//这段代码是用于执行基于 Karto 算法的离线建图的核心函数，这段代码的目的是读取离线记录的激光数据和里程计位置
void SlamKarto::begin_slam(std::vector<char>* output, std::string map_name) {
  /// <summary>
  ///初始化 laser参数
  resolution_ = m_MappingConfig.mapping_resolution;
  if (resolution_ < 0 || resolution_ > 1) resolution_ = 0.05;
    // 初始化变量，包括激光扫描数据、里程计位置和计数器等。
  karto::Pose2 odom_pose;
  map_successed = false;
  usleep(10000);
  std::vector<double> laserScan;//激光扫描数据
  //里程计位置
  double odom_x = 0.0;
  double odom_y = 0.0;
  double odom_th = 0.0;
  bool got_first_scan = false;

  int processedScancount = 0;//计数器
  std::string record_str;
  std::string raw_map_data_path =
      m_MappingConfig.map_data_file_path + "/map.rawmap";
  std::ifstream rawMapFileName(
      raw_map_data_path.c_str());  ////未处理地图数据文件yxh
  usleep(100000);
  // 确保地图数据文件成功打开
  assert(rawMapFileName);
  // 从文件逐行读取记录
  while (getline(rawMapFileName, record_str)) {
    laserScan.clear();
    std::vector<std::string> odom_str = SplitCString(record_str, "\t");
    odom_x = atof(odom_str[2].c_str());
    odom_y = atof(odom_str[3].c_str());
    odom_th = atof(odom_str[4].c_str());

    std::vector<std::string> laser_str = SplitCString(odom_str[6], ",");
    for (int i = 0; i < laser_str.size() - 1; i++) {
      laserScan.push_back(atof(laser_str[i].c_str()));
    }

    // karto::Pose2 odom_poset(
    //     odom_x + laser_x_offset * cos(odom_th) - laser_y_offset * sin(odom_th),
    //     odom_y + laser_x_offset * sin(odom_th) + laser_y_offset * cos(odom_th),
    //     odom_th);

    // 构造激光数据对应的里程计位姿
    karto::Pose2 odom_poset(
        odom_x,
        odom_y,
        odom_th);
    // 调用 addScan 函数将激光数据和里程计位姿添加到 SLAM 中
    if (addScan(laser_, laserScan, odom_poset)) {
      processedScancount++;
    }
    usleep(50000);
  }
  // 更新地图
  updateMap(output, map_name);
  rawMapFileName.close();
}

bool SlamKarto::updateMap(std::vector<char>* output,
                          std::string given_map_name) {
  karto::OccupancyGrid* occ_grid = karto::OccupancyGrid::CreateFromScans(
      mapper_->GetAllProcessedScans(), resolution_);
  if (!occ_grid) return false;
  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset =
      occ_grid->GetCoordinateConverter()->GetOffset();
  MapInfo info;
  info.miMapHeight = height;
  info.miMapWidth = width;
  info.mdOriginXInWorld = offset.GetX();
  info.mdOriginYInWorld = offset.GetY();
  info.mdResolution = resolution_;
  given_map_name.erase(
      std::remove(given_map_name.begin(), given_map_name.end(), ' '),
      given_map_name.end());
  time_t now_time = time(NULL);
  tm* time = gmtime(&now_time);
  std::string create_time =
      std::to_string(time->tm_year + 1900) + "-" +
      std::to_string(time->tm_mon + 1) + "-" + std::to_string(time->tm_mday) +
      "-" + std::to_string(time->tm_hour + 8) + "-" +
      std::to_string(time->tm_min) + "-" + std::to_string(time->tm_sec);
  std::string map_name;
  if (given_map_name.empty())
    map_name = create_time + ".smap";
  else
    map_name = given_map_name + ".smap";
  printf("要保存的地图名为%s\n", map_name.c_str());

  std::cout << "width = " << width << ", height = " << height
            << ", scale = " << occ_grid->GetCoordinateConverter()->GetScale()
            << ", offset: " << offset.GetX() << ", " << offset.GetY()
            << std::endl;
  SimpleGridMap* grid_map = new SimpleGridMap(info);
  grid_map->datas.reserve(info.miMapWidth * info.miMapHeight);
  Json::Value map_json;
  Json::Value map_header;
  Json::Value point_append;
  Json::Value atring_add;
  map_header["mapType"] = "2D-Map";
  map_header["mapName"] = map_name;
  atring_add["x"] = info.mdOriginXInWorld;
  atring_add["y"] = info.mdOriginYInWorld;
  map_header["minPos"] = atring_add;
  atring_add["x"] = info.mdOriginXInWorld + resolution_ * width;
  atring_add["y"] = info.mdOriginYInWorld + resolution_ * height;
  map_header["maxPos"] = atring_add;
  map_header["resolution"] = resolution_;
  map_header["version"] = "1.0.6";
  map_json["header"] = map_header;
  int index_cnt = 0;
  for (kt_int32s y = 0; y < height; y++) {
    for (kt_int32s x = 0; x < width; x++) {
      Json::Value point_xy;
      // Getting the value at position x,y
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));
      switch (value) {
        case karto::GridStates_Unknown:
          grid_map->datas.push_back(255);
          break;
        case karto::GridStates_Occupied:
          grid_map->datas.push_back(100);
          point_xy["x"] = info.mdOriginXInWorld + x * resolution_;
          point_xy["y"] = info.mdOriginYInWorld + y * resolution_;
          point_append[index_cnt] = point_xy;
          index_cnt++;
          break;
        case karto::GridStates_Free:
          grid_map->datas.push_back(0);
          break;
        default:
          printf("Encountered unknown cell value at %d, %d", x, y);
          break;
      }
    }
  }
  map_json["normalPosList"] = point_append;
  grid_map->to_json_char_array(output);
  std::string full_map_data_path =
      m_MappingConfig.map_data_file_path + "/" + map_name;
  json_save(full_map_data_path.c_str(), map_json);
  std::string data_string;
  std::string full_map_config_path =
      m_MappingConfig.map_config_file_path + "/map_config.json";
  json_read(full_map_config_path.c_str(), &data_string);
  Json::Value data_json;
  Json::Reader reader;
  if (reader.parse(data_string, data_json)) {
    Json::Value data_json_copy = data_json;
    Json::Value json_map_info;
    json_map_info["width"] = width;
    json_map_info["height"] = height;
    json_map_info["resolution"] = resolution_;
    struct stat statbuf;
    stat(full_map_data_path.c_str(), &statbuf);
    int size = statbuf.st_size;
    json_map_info["size"] = size;
    json_map_info["createTime"] = create_time;
    json_map_info["mapName"] = map_name;
    json_map_info["isUseful"] = false;
    data_json_copy["AllMapList"].append(json_map_info);
    data_json["AllMapList"] = data_json_copy["AllMapList"];
    json_save(full_map_config_path.c_str(), data_json);
  }
  delete grid_map;
  delete occ_grid;
  usleep(200000);
  map_successed = true;
  return true;
}
void SlamKarto::json_save(const char* file_name, Json::Value v) {
  Json::StyledWriter style_writer;
  std::string str = style_writer.write(v);
  str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
  str.erase(std::remove(str.begin(), str.end(), '\r'), str.end());
  str.erase(std::remove(str.begin(), str.end(), ' '), str.end());
  std::ofstream ofs(file_name, std::ios::trunc);
  ofs << str;
  ofs.close();
}

void SlamKarto::json_read(const char* file_name, std::string* read_data) {
  std::ifstream read_file;
  std::ostringstream oss;
  read_file.open(file_name, std::ios::binary);
  if (read_file.is_open()) {
    oss.str("");
    oss << read_file.rdbuf();
    *read_data = oss.str();
    read_file.close();
  } else {
  }
}

bool SlamKarto::addScan(karto::LaserRangeFinder* laser,
                        std::vector<double> scan, karto::Pose2& karto_pose) {
  // Create a vector of doubles for karto
  std::vector<kt_double> readings;

  for (unsigned int i = 0; i < scan.size(); i++)
    readings.push_back((double)scan[i]);

  // create localized range scan
  karto::LocalizedRangeScan* range_scan =
      new karto::LocalizedRangeScan(laser->GetName(), readings);
  range_scan->SetOdometricPose(karto_pose);
  range_scan->SetCorrectedPose(karto_pose);

  // Add the localized range scan to the mapper
  bool processed;
  if ((processed = mapper_->Process(range_scan))) {
    karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();
    // Add the localized range scan to the dataset (for memory management)
    dataset_->Add(range_scan);
  } else
    delete range_scan;

  return processed;
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
