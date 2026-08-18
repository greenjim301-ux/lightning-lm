# 独立的、只给Emscripten用的CMake工程，构建ui/wasm_ui_client.cc + 它依赖的那部分UI代码
# (PangolinUiScene/UiCar/UiCloud/UiTrajectory/GlColoredShader——全都不依赖PCL/rclcpp，
# 详见各自源文件里的注释)。故意不作为主CMakeLists.txt的add_subdirectory：主工程的
# find_package(ament_cmake/PCL/rclcpp/...)在Emscripten交叉编译下根本找不到，
# 两边没法共用同一次cmake配置，只能各自独立配置。
#
# 产物是MODULARIZE+EXPORT_ES6的.mjs+.wasm(不是独立html)，供web/(React/Vite)动态import——
# 详见web/README或src/ui/wasm_ui_client.cc顶部注释。宿主页面需要提供<canvas id="canvas">
# (Pangolin的emscripten窗口后端写死找这个id，见pango_windowing/src/display_emscripten.cpp)
# 并在import前设置window.LIGHTNING_WS_URL指向UiWireServer地址。
#
# 用法（需要先按reference-lightning-lm-build-env memory里记的方式，编译出wasm版Pangolin）：
#   source <emsdk>/emsdk_env.sh
#   mkdir -p build-wasm && cd build-wasm
#   emcmake cmake ../src/ui/wasm -DPangolin_DIR=/path/to/pangolin-em-src/build-em \
#       -DEigen3_DIR=/usr/share/eigen3/cmake   # 交叉编译时Eigen3不会自动去host路径找，得手动指
#   emmake make
#   把生成的lightning_ui_wasm_client.mjs/.wasm拷到web/public/wasm/下(见web/README)。
cmake_minimum_required(VERSION 3.16)
project(lightning_ui_wasm_client)

if(NOT EMSCRIPTEN)
    message(FATAL_ERROR
        "This CMakeLists.txt only builds with an Emscripten toolchain (use emcmake). "
        "For the native build, use the top-level CMakeLists.txt + colcon instead.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LIGHTNING_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(LIGHTNING_THIRDPARTY "${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty")

find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        ${LIGHTNING_SRC}
        ${LIGHTNING_THIRDPARTY}
)

add_executable(lightning_ui_wasm_client
        ${CMAKE_CURRENT_SOURCE_DIR}/../wasm_ui_client.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../ui_car.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../ui_cloud.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../ui_trajectory.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../gl_colored_shader.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../pangolin_ui_scene.cc
)

target_link_libraries(lightning_ui_wasm_client ${Pangolin_LIBRARIES})

set_target_properties(lightning_ui_wasm_client PROPERTIES SUFFIX ".mjs")
target_compile_options(lightning_ui_wasm_client PRIVATE -fexceptions)
target_link_options(lightning_ui_wasm_client PRIVATE
        -sFULL_ES3=1 -sASYNCIFY=1 --bind -sALLOW_MEMORY_GROWTH=1
        -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=createLightningUiModule
        -sENVIRONMENT=web
        # emscripten_set_main_loop internally assigns Module['noExitRuntime'] = true to keep
        # the runtime alive across the requestAnimationFrame loop; without exporting it,
        # Emscripten replaces it with a getter-only guard stub and that assignment throws
        # "Cannot set property noExitRuntime of #<Object> which has only a getter".
        -sEXPORTED_RUNTIME_METHODS=noExitRuntime
        -fexceptions -lwebsocket.js)
