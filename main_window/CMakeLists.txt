cmake_minimum_required(VERSION 3.10)

project(dvr_pmeter VERSION 0.1.0 LANGUAGES CXX)#使用 project 命令可以定义项目的名称、版本号、语言、编译器等属性。通过设置 #LANGUAGES 参数，可以指定项目使用的编程语言，CMake 会相应地生成适合该语言的构建系统。在这里，指定使用 C++ 编程语言。这#将使 CMake 为项目生成相应的编译器指令，如编译选项、链接选项等，并生成可执行文件、动态库或静态库等目标文件。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(
    FATAL_ERROR
      "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there."
  )
endif()

if (MSVC)
    add_compile_options($<$<C_COMPILER_ID:MSVC>:/utf-8>)
    add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
    #add_compile_options(/MP)

    # -DWIN32_LEAN_AND_MEAN is for winsock.h has already been included error
    # -D_SILENCE_CXX17_ALLOCATOR_VOID_DEPRECATION_WARNING is for warning STL4009
    #   std::allocator<void> is deprecated in C++17
    add_definitions(
        -DNOMINMAX
        -DWIN32_LEAN_AND_MEAN
        -D_SILENCE_CXX17_ALLOCATOR_VOID_DEPRECATION_WARNING
        -D_CRT_SECURE_NO_WARNINGS)
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   /MP /wd4200")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /wd4200 /Zc:__cplusplus /bigobj")
endif()

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS Widgets LinguistTools REQUIRED)

find_package(PCL 1.12 REQUIRED)

set(PRJ_SOURCES
        main.cpp
        main_window.h
        main_window.cpp
        main_window.ui
        Algorithm.h
        Algorithm.cpp
        Algorithm.ui
        cloud_repair.cpp
        cloud_repair.h
        PCPoint.cpp
        PCPoint.h
        Guass.cpp
        Guass.h
)
set(PRJ_RESOURCES main_window.qrc)


set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(dvr_pmeter
    ${PRJ_SOURCES}
    ${PRJ_RESOURCES}
)

target_compile_definitions(dvr_pmeter PRIVATE ${PCL_DEFINITIONS})
target_include_directories(dvr_pmeter PRIVATE ${PCL_INCLUDE_DIRS})
target_link_directories(dvr_pmeter PRIVATE ${PCL_LIBRARY_DIRS})

target_link_libraries(dvr_pmeter PRIVATE
    Qt5::Widgets
    ${PCL_LIBRARIES})
