/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-09-28 09:42:40
 * @LastEditTime: 2023-04-01 17:09:16
 * @Author: lcfc-desktop
 */
#include "include/mapping_and_location_impl.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {
MappingAndLocationImpl::MappingAndLocationImpl(
    const KartoAndGridConfig &config) {
  karto_and_grid_config_ = config;
  radar_pose_base_link_.mfX =
      karto_and_grid_config_.location_config.radar_position_x;
  radar_pose_base_link_.mfY =
      karto_and_grid_config_.location_config.radar_position_y;
  radar_pose_base_link_.mfTheta =
      karto_and_grid_config_.location_config.radar_position_theta;
  gomros::common::SLAMLogger::SetLoggerConfig(
      (gomros::common::LOG_LEVEL)config.log_level, config.log_file_name,
      config.is_printf_to_terminal);
  LoadMapNameFromFile();
  device_state_ = std::make_shared<DeviceState>(DeviceState());
  device_state_->mcChassisState = SYSTEM_STATE_INIT_POSE;
  // mapping_module_ = new MappingManagerImpl(
  //     karto_and_grid_config_.mapping_config, device_state_, p_logger_);

  location_module = new LocationManagerImpl(
      karto_and_grid_config_.location_config, device_state_);
  mapping_module_->SetDeviceState(device_state_);
  location_module->SetDeviceState(device_state_);
  node_ = new Node(config.node_name);
  CallBackEvent odom_enevt{this, OdomCallBackFunc};
  node_->SubscribeTopic(config.sub_odom_topic, odom_enevt);
  CallBackEvent radar_event{this, RadarCallBackFunc};
  node_->SubscribeTopic(config.sub_radar_topic, radar_event);
  CallBackEvent start_location_event{this, StartLocateCallBack};
  node_->SubscribeTopic(config.sub_global_locate_cmd_topic,
                        start_location_event);
  CallBackEvent start_mapping_event{this, StartMappingCallBack};
  node_->SubscribeTopic(config.sub_start_mapping_cmd_topic,
                        start_mapping_event);
  CallBackEvent stop_mapping_event{this, StopMappingCallBack};
  node_->SubscribeTopic(config.sub_stop_mapping_cmd_topic, stop_mapping_event);
  state_publisher_ = node_->AdvertiseTopic(config.ad_state_topic);
  pose_publisher_ = node_->AdvertiseTopic(config.ad_pose_topic);
  node_->Debug();
  PublishRobotState();
  if (LoadMapDataFromFile(map_file_full_path_)) {
    LoadPoseFromFile();
    location_module->SetIniPose(initial_position_);
    SLAM_INFO("设置初始位姿完毕\n");
    location_module->SetTotalOdom(total_odom_);
    SLAM_INFO("设置总里程数完毕\n");
    location_module->SetMapData(*map_shared_pointer_);
    SLAM_INFO("设置地图完毕\n");
    SLAM_INFO("开始开机初始定位\n");
    location_module->StartGlobalLocating(
        config.location_config.is_initial_locate);
    SLAM_INFO("开始创建发送位姿线程\n");
    pthread_create(&send_pose_thread, NULL, SendPose, this);
  } else {
    SLAM_ERROR("加载地图错误\n");
  }
}

//------------------------新加，里面内容待改----------------------
MappingAndLocationImpl::MappingAndLocationImpl(
    const CartoAndGridConfig &config) {
  carto_and_grid_config_ = config;
  //------------------雷达初始位姿参数配置---------------------
  radar_pose_base_link_.mfX =
      carto_and_grid_config_.location_config_.radar_position_x;
  radar_pose_base_link_.mfY =
      carto_and_grid_config_.location_config_.radar_position_y;
  radar_pose_base_link_.mfTheta =
      carto_and_grid_config_.location_config_.radar_position_theta;

  device_state_ = std::make_shared<DeviceState>(DeviceState());
  device_state_->mcChassisState = SYSTEM_STATE_INIT_POSE;
  mapping_module_ = new MappingManagerImpl(carto_and_grid_config_.Carto_config,
                                           device_state_);

  // MappingManagerImpl类的对象调用自身的成员函数
  mapping_module_->SetDeviceState(device_state_);
  location_module =
      new LocationManagerImpl(config.location_config_, device_state_);
  mapping_module_->SetDeviceState(device_state_);
  location_module->SetDeviceState(device_state_);

  node_ = new Node(config.node_name);

  //------------------新加------------------------
  CallBackEvent imu_event{this, ImuCallBackFunc};
  node_->SubscribeTopic(config.sub_imu_topic, imu_event);

  CallBackEvent odom_enevt{this, OdomCallBackFunc};
  // config
  node_->SubscribeTopic(config.sub_odom_topic, odom_enevt);

  CallBackEvent radar_event{this, RadarCallBackFunc};
  node_->SubscribeTopic(config.sub_radar_topic, radar_event);

  CallBackEvent start_location_event{this, StartLocateCallBack};
  node_->SubscribeTopic(config.sub_global_locate_cmd_topic,
                        start_location_event);

  CallBackEvent start_mapping_event{this, StartMappingCallBack};
  node_->SubscribeTopic(config.sub_start_mapping_cmd_topic,
                        start_mapping_event);

  CallBackEvent stop_mapping_event{this, StopMappingCallBack};
  node_->SubscribeTopic(config.sub_stop_mapping_cmd_topic, stop_mapping_event);

  state_publisher_ = node_->AdvertiseTopic(config.ad_state_topic);
  pose_publisher_ = node_->AdvertiseTopic(config.ad_pose_topic);
  node_->Debug();
  PublishRobotState();
  //以下都是重定位使用的
  LoadMapNameFromFile();
  if (LoadMapDataFromFile(map_file_full_path_)) {
    LoadPoseFromFile();
    location_module->SetIniPose(initial_position_);
    SLAM_INFO("设置初始位姿完毕\n");

    location_module->SetTotalOdom(total_odom_);
    SLAM_INFO("设置总里程数完毕\n");  //机器人跑的路程距离

    location_module->SetMapData(*map_shared_pointer_);
    SLAM_INFO("设置地图完毕\n");

    SLAM_INFO("开始开机初始定位\n");
    location_module->StartGlobalLocating(
        config.location_config_.is_initial_locate);

    SLAM_INFO("开始创建发送位姿线程\n");
    pthread_create(&send_pose_thread, NULL, SendPose, this);
  } else {
    SLAM_ERROR("加载地图错误\n");
  }
}

MappingAndLocationImpl::~MappingAndLocationImpl() {
  pthread_join(send_pose_thread, NULL);
  delete node_;
  delete mapping_module_;  // MappingManagerImpl类的指针
  delete location_module;
}

//---------------新加------------
void MappingAndLocationImpl::SetConfiguration(
    const CartoAndGridConfig &config) {
  mapping_module_->SetConfiguration(config.Carto_config);
  location_module->SetConfiguration(config.location_config_);
}

KartoAndGridConfig MappingAndLocationImpl::GetKartoAndGridConfig() {
  return karto_and_grid_config_;
}

CartoAndGridConfig MappingAndLocationImpl::GetCartoAndGridConfig() {
  return carto_and_grid_config_;
}

void MappingAndLocationImpl::OdomCallBackFunc(void *object, char *buf,
                                              const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  gomros::common::Message<OdometerMessage> odometer_data;
  odometer_data.FromCharArray(buf, size);
  lp_this->location_module->SetOdomData(odometer_data.pack);

  if (lp_this->device_state_->mcChassisState == SYSTEM_STATE_MAPPING) {
    lp_this->mapping_module_->SetOdomData(odometer_data.pack);
  }
}

void MappingAndLocationImpl::RadarCallBackFunc(void *object, char *buf,
                                               const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  RadarSensoryMessage radar_data;
  radar_data.from_char_array(buf, size);
  if (lp_this->device_state_->mcChassisState != SYSTEM_STATE_MAPPING) {
    lp_this->location_module->SetRadarData(radar_data);
  } else {
    lp_this->mapping_module_->SetRadarData(radar_data);
  }
}
void MappingAndLocationImpl::ImuCallBackFunc(void *object, char *buf,
                                             const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  gomros::common::Message<ImuSensoryMessage> imu_data;
  imu_data.FromCharArray(buf, size);

  // lp_this->location_module->SetImuData(imu_data.pack);
  if (lp_this->device_state_->mcChassisState == SYSTEM_STATE_MAPPING) {
    lp_this->mapping_module_->SetImuData(imu_data.pack);
  }
}

void MappingAndLocationImpl::StartLocateCallBack(void *object, char *buf,
                                                 const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  gomros::common::Message<Position> data;
  data.FromCharArray(buf, size);
  data.pack.mlTimestamp = gomros::common::GetCurrentTime_us();
  lp_this->initial_position_ = data.pack;
  SLAM_INFO("回调函数调用初始化定位, 传入的初始位姿为x=%f, y=%f, theta=%f\n",
            lp_this->initial_position_.mfX, lp_this->initial_position_.mfY,
            lp_this->initial_position_.mfTheta);
  lp_this->device_state_->mcChassisState = SYSTEM_STATE_INIT_POSE;
  lp_this->PublishRobotState();
  lp_this->location_module->SetHaveMap(false);
  lp_this->location_module->SetHavePose(false);
  lp_this->LoadMapNameFromFile();
  if (lp_this->LoadMapDataFromFile(lp_this->map_file_full_path_)) {
    SLAM_INFO("地图加载成功\n");
    lp_this->location_module->SetMapData(*(lp_this->map_shared_pointer_));
    lp_this->location_module->SetIniPose(data.pack);
    lp_this->location_module->StartGlobalLocating(true);
  }
}

void MappingAndLocationImpl::StartMappingCallBack(void *object, char *buf,
                                                  const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  CustomTaskString task_string;
  task_string.from_char_array(buf, size);
  // Json::Reader reader;
  // Json::Value data_json;
  std::string data_string;
  data_string = task_string.TaskString;
  SLAM_INFO("###收到开始建图指令\n");
  lp_this->location_module->StopLocate();  //停止定位
  lp_this->StartMapping();                 //开始建图
}

void MappingAndLocationImpl::StopMappingCallBack(void *object, char *buf,
                                                 const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  CustomTaskString task_string;
  task_string.from_char_array(buf, size);
  Json::Reader reader;
  Json::Value data_json;
  std::string data_string;
  data_string = task_string.TaskString;
  SLAM_INFO("###收到停止建图指令\n");
  if (!reader.parse(data_string, data_json)) {
    SLAM_WARN("###停止建图指令解析错误\n");
  } else {
    std::string map_name = data_json["map_name"].asString();
    SLAM_INFO("下发的地图名为%s\n", map_name.c_str());
    lp_this->StopMapping(map_name);
  }
}

void MappingAndLocationImpl::StartChargeRelocate() {
  if (have_charge_position) {
    initial_position_ = charge_positipn_;
    device_state_->mcChassisState = SYSTEM_STATE_INIT_POSE;
    PublishRobotState();
    // location_module->SetHaveMap(false);
    location_module->SetHavePose(false);
    // LoadMapNameFromFile();
    // if (LoadMapDataFromFile(map_file_full_path_)) {
    // SLAM_INFO( "地图加载成功\n");
    // location_module->SetMapData(*(map_shared_pointer_));
    location_module->SetIniPose(charge_positipn_);
    location_module->StartGlobalLocating(false);
    // }
  }
}

void MappingAndLocationImpl::StartMapping() {
  if (mapping_module_->StartMapping()) {
    SLAM_INFO("####开始建图\n");
    PublishRobotState();
    location_module->ResetPose();
  }
}

void MappingAndLocationImpl::StopMapping(std::string map_name) {
  if (device_state_->mcChassisState == SYSTEM_STATE_MAPPING) {
    SLAM_INFO("####停止建图\n");  //假如在建图状态，就停止建图
    mapping_module_->StopMapping(map_name);
    sleep(2);
    PublishRobotState();
  } else {
    SLAM_WARN("####未处于建图状态\n");
    device_state_->mcChassisState = SYSTEM_STATE_MAPPING;
    PublishRobotState();
    sleep(1);
    mapping_module_->StopMapping(map_name);
    sleep(2);
    PublishRobotState();
  }
}

void MappingAndLocationImpl::SetInitPose(Position init_pose) {
  location_module->SetIniPose(init_pose);
}

// void MappingAndLocationImpl::LoadMapNameFromFile() {
//   std::string data_string;
//   std::string map_config_full_path =
//       karto_and_grid_config_.mapping_config.map_config_file_path + "/" +
//       "map_config.json";
//   if (JsonRead(map_config_full_path.c_str(), &data_string)) {
//     Json::Reader reader;
//     Json::Value data_json;
//     if (!reader.parse(data_string, data_json)) {
//       SLAM_ERROR( "###解析地图配置数据错误\n");
//       map_file_full_path_ =
//           karto_and_grid_config_.mapping_config.map_data_file_path + "/" +
//           "ale.smap";
//       return;
//     } else {
//       map_file_full_path_ =
//           karto_and_grid_config_.mapping_config.map_data_file_path + "/" +
//           data_json["defaultMap"].asString();
//     }
//   } else {
//     SLAM_ERROR( "###打开地图配置文件失败\n");
//     map_file_full_path_ =
//         karto_and_grid_config_.mapping_config.map_data_file_path + "/" +
//         "ale.smap";
//     return;
//   }
//   SLAM_INFO( "加载到的默认地图全路径为%s\n",
//            map_file_full_path_.c_str());
// }

//-----------------新加（map_config_full_path参数存在问题）-------------------
void MappingAndLocationImpl::LoadMapNameFromFile() {
  std::string data_string;
  std::string map_config_full_path =
      carto_and_grid_config_.Carto_config.map_config_file_path + "/" +
      "map_config.json";
  if (JsonRead(map_config_full_path.c_str(), &data_string)) {
    Json::Reader reader;
    Json::Value data_json;
    if (!reader.parse(data_string, data_json)) {
      SLAM_ERROR("###解析地图配置数据错误\n");
      map_file_full_path_ =
          carto_and_grid_config_.Carto_config.map_data_file_path + "/" +
          "ale.smap";
      return;
    } else {
      map_file_full_path_ =
          carto_and_grid_config_.Carto_config.map_data_file_path + "/" +
          data_json["defaultMap"].asString();
    }
  } else {
    SLAM_ERROR("###打开地图配置文件失败\n");
    map_file_full_path_ =
        carto_and_grid_config_.Carto_config.map_data_file_path + "/" +
        "ale.smap";
    return;
  }
  SLAM_INFO("加载到的默认地图全路径为%s\n", map_file_full_path_.c_str());
}

void *MappingAndLocationImpl::SendPose(void *ptr) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(ptr);
  int pose_cnt = 0, state_cnt = 0;
  while (1) {
    if (lp_this->location_module->FinishLocate())
      lp_this->current_position_ = lp_this->location_module->GetCurrentPose();
    else
      lp_this->current_position_.mclDeltaPosition = lp_this->initial_position_;
    if (!isnanl(lp_this->current_position_.mclDeltaPosition.mfX) &&
        !isnanl(lp_this->current_position_.mclDeltaPosition.mfY) &&
        !(lp_this->current_position_.mclDeltaPosition.mfY == 0.0 ||
          lp_this->current_position_.mclDeltaPosition.mfX == 0.0)) {
      gomros::common::Message<gomros::message::TotalOdomMessgae>
          total_odom_and_pose;
      total_odom_and_pose.pack.odom_oose = lp_this->current_position_;
      total_odom_and_pose.pack.total_odom =
          lp_this->location_module->GetTotalOdom();
      total_odom_and_pose.pack.radar_pose = total_odom_and_pose.pack.odom_oose;
      total_odom_and_pose.pack.radar_pose.mclDeltaPosition =
          total_odom_and_pose.pack.radar_pose.mclDeltaPosition *
          lp_this->radar_pose_base_link_;
      std::vector<char> out;
      total_odom_and_pose.ToCharArray(&out);
      lp_this->pose_publisher_->Publish(out.data(), out.size());
    }
    pose_cnt++;
    if (pose_cnt >= 50 * 3) {
      SLAM_INFO("当前位姿x=%f, y=%f, theta=%f\n",
                lp_this->current_position_.mclDeltaPosition.mfX,
                lp_this->current_position_.mclDeltaPosition.mfY,
                lp_this->current_position_.mclDeltaPosition.mfTheta);
      SLAM_DEBUG(
          "当前纯里程计位姿x=%f, y=%f, theta=%f\n",
          lp_this->location_module->GetRawOdomPose().mclDeltaPosition.mfX,
          lp_this->location_module->GetRawOdomPose().mclDeltaPosition.mfY,
          lp_this->location_module->GetRawOdomPose().mclDeltaPosition.mfTheta);
      pose_cnt = 0;
    }
    state_cnt++;
    if (state_cnt >= 50 * 3) {
      lp_this->PublishRobotState();
      state_cnt = 0;
    }
    usleep(20000);
  }
}

void MappingAndLocationImpl::PublishRobotState() {
  // 状态改变
  gomros::common::Message<DeviceState> data;
  SLAM_INFO("当前底盘状态为%d\n", device_state_->mcChassisState);
  data.pack.mcChassisState = device_state_->mcChassisState;
  std::vector<char> output;
  data.ToCharArray(&output);  //将this->pack中的数据序列后放入 output
  state_publisher_->Publish(output.data(), output.size());
}

//------------------已修改的（待验证）--------------------
void MappingAndLocationImpl::LoadPoseFromFile() {
  std::string data_string;
  char filename[128], filename1[128];
  SLAM_INFO("初始位姿文件存放路径%s\n",
            carto_and_grid_config_.location_config_.init_pose_file_path
                .c_str());  //------location_config_---------
  snprintf(filename, sizeof(filename), "%s/zero.json",
           carto_and_grid_config_.location_config_.init_pose_file_path.c_str());
  snprintf(filename1, sizeof(filename1), "%s/zero1.json",
           carto_and_grid_config_.location_config_.init_pose_file_path.c_str());
  if (!JsonRead(filename, &data_string) && !JsonRead(filename1, &data_string)) {
    initial_position_.mfX = 0;
    initial_position_.mfY = 0;
    initial_position_.mfTheta = 0;
    initial_position_.mlTimestamp = gomros::common::GetCurrentTime_us();
    total_odom_ = 0;
    SLAM_ERROR("打开初始位姿文件失败\n");
    return;
  }
  Json::Reader reader;
  Json::Value data_json;
  initial_position_.mlTimestamp = gomros::common::GetCurrentTime_us();
  if (!reader.parse(data_string, data_json)) {
    initial_position_.mfX = 0;
    initial_position_.mfY = 0;
    initial_position_.mfTheta = 0;
    initial_position_.mlTimestamp = gomros::common::GetCurrentTime_us();
    total_odom_ = 0;
    SLAM_ERROR("解析初始位姿数据失败\n");
    return;
  } else {
    SLAM_INFO(
        "###读取初始位姿成功\n");  //-------应该指的是AGV的初始位姿----------
    initial_position_.mfX = data_json["zero_pos"]["AgvX"].asFloat();
    initial_position_.mfY = data_json["zero_pos"]["AgvY"].asFloat();
    initial_position_.mfTheta = data_json["zero_pos"]["AgvTheta"].asFloat();
    initial_position_.mlTimestamp = gomros::common::GetCurrentTime_us();
    if (initial_position_.mfX < x_min_ || initial_position_.mfX > x_max_ ||
        initial_position_.mfY < y_min_ || initial_position_.mfY > y_max_) {
      initial_position_.mfX = 0;
      initial_position_.mfY = 0;
      initial_position_.mfTheta = 0;
    }
    total_odom_ = data_json["zero_pos"]["Total_odom"].asDouble();
    SLAM_INFO("###读取的初始位姿为x=%f,y=%f,theta=%f\n", initial_position_.mfX,
              initial_position_.mfY, initial_position_.mfTheta);
  }
}

bool MappingAndLocationImpl::LoadMapDataFromFile(std::string map_file_name) {
  char filename[128];
  SLAM_INFO("地图文件存放全路径%s\n", map_file_name.c_str());
  std::string map_data;
  if (JsonRead(map_file_name.c_str(), &map_data)) {
    LoadGridMap(map_data);
    return true;
  } else {
    return false;
  }
}

// 由字符串数据生成地图
void MappingAndLocationImpl::LoadGridMap(std::string map_data) {
  Json::Reader reader;
  Json::Value map_json;
  if (!reader.parse(map_data, map_json)) {
    SLAM_ERROR("###地图解析失败\n");
  } else {
    MapInfo info;
    float resolution = map_json["header"]["resolution"].asDouble();
    float origen_x = map_json["header"]["minPos"]["x"].asDouble();
    float origen_y = map_json["header"]["minPos"]["y"].asDouble();
    info.mdOriginXInWorld = origen_x;
    info.mdOriginYInWorld = origen_y;
    info.mdResolution = resolution;
    int width = info.RealXToMapX(map_json["header"]["maxPos"]["x"].asDouble());
    int height = info.RealYToMapY(map_json["header"]["maxPos"]["y"].asDouble());
    x_min_ = origen_x;
    y_min_ = origen_y;
    info.miMapWidth = width;
    info.miMapHeight = height;
    x_max_ = x_min_ + width;
    y_max_ = y_min_ + height;
    map_shared_pointer_ = std::make_shared<SimpleGridMap>(info);
    SLAM_INFO("###resolution=%f,width=%d,height=%d,origin_x=%f,origin_y=%f\n",
              resolution, map_shared_pointer_->map_info.miMapWidth,
              map_shared_pointer_->map_info.miMapHeight,
              map_shared_pointer_->map_info.mdOriginXInWorld,
              map_shared_pointer_->map_info.mdOriginYInWorld);
    map_shared_pointer_->datas = std::vector<char>(width * height, 0);
    for (int j = 0; j < map_json["normalPosList"].size(); j++) {
      int x = map_shared_pointer_->map_info.RealXToMapX(
          map_json["normalPosList"][j]["x"].asDouble());
      int y = map_shared_pointer_->map_info.RealYToMapY(
          map_json["normalPosList"][j]["y"].asDouble());
      int index = map_shared_pointer_->map_info.MapIndexToArraryIndex(x, y);
      if (index <= map_shared_pointer_->datas.size()) {
        map_shared_pointer_
            ->datas[map_shared_pointer_->map_info.MapIndexToArraryIndex(x, y)] =
            100;
      }
    }
    if (map_json.isMember("advancedPointList")) {
      SLAM_INFO("#########地图文件存在cap点\n");
      Json::Value cap_json = map_json["advancedPointList"];
      for (int j = 0; j < cap_json.size(); j++) {
        std::string classname = cap_json[j]["className"].asString();
        if (strcmp(classname.c_str(), "ChargePoint") == 0) {
          charge_positipn_.mfX = cap_json[j]["pos"]["x"].asFloat();
          charge_positipn_.mfY = cap_json[j]["pos"]["y"].asFloat();
          charge_positipn_.mfTheta = cap_json[j]["dir"].asFloat();
          SLAM_INFO("#########停车点为(%f,%f,%f)\n", charge_positipn_.mfX,
                    charge_positipn_.mfY, charge_positipn_.mfTheta);
          have_charge_position = true;
        }
      }
    } else {
      SLAM_ERROR("#########地图文件没有cap点\n");
    }
  }
}

void MappingAndLocationImpl::JsonSave(const char *file_name, Json::Value v) {
  Json::StyledWriter style_writer;
  std::string str = style_writer.write(v);
  str.erase(std::remove(str.begin(), str.end(), '\n'),
            str.end());  // 去除文件的换行
  str.erase(std::remove(str.begin(), str.end(), '\r'),
            str.end());  // 去除文件的空格
  str.erase(std::remove(str.begin(), str.end(), ' '),
            str.end());  // 去除文件的空格
  std::ofstream ofs(file_name, std::ios::trunc);
  ofs << str;
  ofs.close();
}
// 将JSON格式数据存入文件
bool MappingAndLocationImpl::JsonRead(const char *file_name,
                                      std::string *read_data) {
  std::ifstream read_file;
  std::ostringstream oss;
  read_file.open(file_name, std::ios::binary);
  if (read_file.is_open()) {
    oss.str("");
    oss << read_file.rdbuf();
    *read_data = oss.str();
    read_file.close();
    return true;
  } else {
    return false;
  }
}

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
