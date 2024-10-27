cmake_minimum_required(VERSION 3.8)
project(stamp_dude_bringup)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# launchディレクトリをインストール
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# 自動インストール
ament_auto_package()
