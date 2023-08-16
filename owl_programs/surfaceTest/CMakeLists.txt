embed_ptx(
  OUTPUT_TARGET
    surfaceTest-ptx
  PTX_LINK_LIBRARIES
    owl::owl
  SOURCES
    deviceCode.cu
)
find_package(GLEW REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
add_executable(surfaceTest
	"hostCode.cpp"
  "deviceCode.h"
)

target_link_libraries(surfaceTest
  PRIVATE
    surfaceTest-ptx
    owl::owl
    stb_image
    GLEW::GLEW
    glfw
)