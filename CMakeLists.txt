# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
#
cmake_minimum_required (VERSION 3.8)

project ("ClothSimulation")

# 包含子项目。
add_subdirectory ("src")

find_package(OpenGL)

include_directories("Include")
link_directories("Libs")
# find_library ("glfw3")
add_library (glad "src/glad.c")

find_package(CGAL REQUIRED)

if(CGAL_FOUND)
	include( ${CGAL_USE_FILE} )
	add_executable (main "main.cpp")
	target_link_libraries (main CGAL::CGAL glad glfw3 ${OPENGL_glu_LIBRARY} ${OPENGL_LIBRARIES})
else()
	message("ERROR: this program requires CGAL will not be compiled.")
endif()