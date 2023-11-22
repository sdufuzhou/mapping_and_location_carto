
###
 # @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 # @Version: 1.0
 # @Author: tong
### 
#!/bin/bash

set -e

version_file="mapping_lib.h.in"
#set -x


echo "#ifndef __SCM_VERSION_H__" > $version_file
echo "#define __SCM_VERSION_H__" >> $version_file

svn log -l 1 -v >> mapping_lib.h.in


echo "#endif" >> $version_file

mv $version_file ./include/mapping_lib
