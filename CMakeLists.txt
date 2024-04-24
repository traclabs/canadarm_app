project(CANADARM_APP C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
include_directories(${ros_app_MISSION_DIR}/fsw/platform_inc)

# Create the app module
add_cfe_app(canadarm_app fsw/src/canadarm_app.c)
target_link_libraries(canadarm_app m)

target_include_directories(canadarm_app PUBLIC
  fsw/mission_inc
  fsw/platform_inc
)
