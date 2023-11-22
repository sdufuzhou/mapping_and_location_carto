#!/bin/bash
###
 # @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 # @Version: 1.0
 # @Author: renjy
 # @Date: 2022-07-07 17:51:20
 # @LastEditTime: 2022-07-07 17:57:50
### 

cd __build
lcov -d ./ -c -o init.info 
lcov -a init.info -o total.info
lcov --remove total.info '*/usr/include/*' '*/usr/lib/*' '*/usr/lib64/*' '*/src/log/*' '*/tests/*' '*/usr/local/include/*' '*/usr/local/lib/*' '*/usr/local/lib64/*' '*/third/*' 'testa.cpp' -o final.info
genhtml -o cover_report --legend --title "lcov"  --prefix=./ final.info
browser cover_report/index.html &
