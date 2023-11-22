/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-10-27 20:42:35
 * @LastEditTime: 2023-03-10 12:08:46
 */

#include "include/common/logger.h"

namespace gomros {
namespace common {

SLAMLogger::SLAMLogger() {
  if (p_logger_ != nullptr) return;
  p_logger_ = new Logger(gomros::common::LOG_LEVEL::DEBUG,
                         "./MappingAndLocation", true);
}

SLAMLogger::~SLAMLogger() {
  if (p_logger_) delete p_logger_;
  if (logger_) delete logger_;
}

void SLAMLogger::SetLoggerConfig(const gomros::common::LOG_LEVEL& level,
                                 const std::string& logger_name,
                                 bool out_terminal) {
  // std::lock_guard<std::recursive_mutex> lk(mutex_logger_data_);
  if (p_logger_) delete p_logger_;
  p_logger_ = new Logger(level, logger_name, out_terminal);
}

SLAMLogger* SLAMLogger::logger_ = nullptr;
Logger* SLAMLogger::p_logger_ = nullptr;

}  // namespace common
}  // namespace gomros
