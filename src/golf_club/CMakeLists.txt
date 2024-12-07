cmake_minimum_required(VERSION 3.8)
project(golf_club)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install resource directories (config, launch, meshes, urdf)
install(
  DIRECTORY config/ launch/ meshes/ urdf/
  DESTINATION share/${PROJECT_NAME}
)

# Mark the package
ament_package()
