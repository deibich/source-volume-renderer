# ======================================================================== #
# Copyright 2019 Ingo Wald                                                 #
#                                                                          #
# Licensed under the Apache License, Version 2.0 (the "License");          #
# you may not use this file except in compliance with the License.         #
# You may obtain a copy of the License at                                  #
#                                                                          #
#     http://www.apache.org/licenses/LICENSE-2.0                           #
#                                                                          #
# Unless required by applicable law or agreed to in writing, software      #
# distributed under the License is distributed on an "AS IS" BASIS,        #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. #
# See the License for the specific language governing permissions and      #
# limitations under the License.                                           #
# ======================================================================== #

embed_ptx(
  OUTPUT_TARGET
    cubeTest-ptx
  PTX_LINK_LIBRARIES
    owl::owl
  SOURCES
    deviceCode.cu
)

add_executable(cubeTest hostCode.cpp)
target_compile_features(cubeTest PUBLIC cxx_std_20)
target_link_libraries(cubeTest
  PRIVATE
    cubeTest-ptx
    owl_viewer
    tinyxml2::tinyxml2
    cfitsio
)
