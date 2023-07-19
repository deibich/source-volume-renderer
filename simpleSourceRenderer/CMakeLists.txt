embed_ptx(
  OUTPUT_TARGET
    simpleSourceRenderer-ptx
  PTX_LINK_LIBRARIES
    owl::owl
  SOURCES
    deviceCode.cu
)

find_package(tinyxml2 CONFIG REQUIRED)
find_package(unofficial-cfitsio CONFIG REQUIRED)

add_executable(SimpleSourceRenderer
	"hostCode.cpp"
  "deviceCode.h"
)

target_link_libraries(SimpleSourceRenderer
  PRIVATE
    simpleSourceRenderer-ptx
    owl::owl
    owl_viewer
    tinyxml2::tinyxml2
    cfitsio
)