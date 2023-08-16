embed_ptx(
  OUTPUT_TARGET
    simpleSourceRenderer-ptx
  PTX_LINK_LIBRARIES
    owl::owl
  SOURCES
    deviceCode.cu
)


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