#!/bin/bash
###
 # @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 # @Version: 1.0
 # @Author: tong
### 
# test_file="valgrind -v --leak-check=full --show-leak-kinds=indirect,definite ./__build/bin/calibration_test"
# ${test_file} --gtest_filter=calibration_test.Single_Steer_Calibration_Test
# ${test_file} --gtest_filter=calibration_test.Diff_Wheel_Calibration_Test
${test_file} --gtest_filter=mapping_location_test.mapping_test
# ${test_file}
