cmake_minimum_required(VERSION 3.5.0)
project(surf-seg VERSION 0.1.0 LANGUAGES C CXX)
find_package(PCL 1.12.1 REQUIRED) # TODO specify components used
find_package(VTK COMPONENTS 
  CommonCore
  CommonDataModel
  InteractionStyle
  RenderingContextOpenGL2
  RenderingCore
  RenderingFreeType
  RenderingGL2PSOpenGL2
  RenderingOpenGL2
)

if (NOT VTK_FOUND)
  message(FATAL_ERROR "Unable to find the VTK build folder.")
endif()


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(surf-seg source/main.cpp)

target_link_libraries (surf-seg ${PCL_LIBRARIES} ${VTK_LIBRARIES})
# vtk_module_autoinit is needed
vtk_module_autoinit(
  TARGETS surf-seg 
  MODULES ${VTK_LIBRARIES}
)


