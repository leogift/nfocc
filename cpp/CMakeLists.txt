# NFOCC — 4 路环视鱼眼 + 运动双目近场 OCC 深度重建
#
# ## 快速开始
#   python3 tools/sync_third_party.py --link      # 可选: 三方库内置
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build -j && ctest --test-dir build --output-on-failure
#
# ## 依赖
#   -DNF_DEPS=managed   包管理器优先(vcpkg/Conan), 回退系统路径【默认】
#   -DNF_DEPS=vendored  优先用工程内 third_party/
#   -DNF_DEPS=system    只用系统路径
#
# ## 加速
#   -DNF_USE_OPENMP=ON  (默认) 按相机/按行并行
#   -DNF_USE_EIGEN=ON   (默认) 矩阵与 SVD 运算
#   -DNF_USE_BLAS=ON    (默认) Eigen 后端走 OpenBLAS
cmake_minimum_required(VERSION 3.16)

project(nfocc LANGUAGES CXX C VERSION 0.3.0)

# ---------------------------------------------------------------- 代码规范
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # 供 clangd / IDE

# ADR-001: 我们自己的代码禁异常/RTTI(三方库边界除外)
if(NOT MSVC)
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# ---------------------------------------------------------------- 选项
option(NF_BUILD_TESTS    "构建测试"                    ON)
option(NF_BUILD_TOOLS    "构建工具与主程序"             ON)
option(NF_WITH_OPENCV    "启用 OpenCV (SGBM 引擎)"      ON)
option(NF_WITH_ZMQ       "启用 ZeroMQ (IO)"            ON)
option(NF_WITH_ORT       "启用 onnxruntime (模型引擎)"  ON)
# vcam 后端：用 /Volumes/Workspace/virtualcamera/cpp 的相机模型做投影。
# 默认 OFF —— 它是可选后端，开启后 nf_vcam_* 系列才有实现；
# 关闭时桥接层编成返回 UNSUPPORTED 的桩，业务代码无需 #ifdef。
option(NF_WITH_VCAM      "启用 vcam 相机模型后端"       ON)
set(NF_VCAM_DIR "" CACHE PATH "virtualcamera/cpp 源码目录（NF_WITH_VCAM=ON 时必填）")
option(NF_USE_OPENMP     "启用 OpenMP 并行"             ON)
option(NF_USE_EIGEN      "启用 Eigen"                  ON)
option(NF_USE_BLAS       "Eigen 后端使用 OpenBLAS"      ON)
option(NF_STRICT_WERROR  "警告即错误 (CI 用)"           OFF)
if(NF_STRICT_WERROR AND NOT MSVC)
  add_compile_options(-Werror)
endif()

# ---------------------------------------------------------------- 依赖
include(cmake/nf_deps.cmake)

# ---------------------------------------------------------------- 核心库
# **零第三方依赖** —— V-BUILD-2: ldd 里不得出现 OpenCV/ZeroMQ/ORT。
# Eigen 是 header-only 且只用于纯计算, 不算"链接依赖"; OpenMP 是编译器运行时。
file(GLOB_RECURSE NF_CORE_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/camera/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lut/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/baseline/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/voxel/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cluster/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/util/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/match_radial.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/scene.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/scene_render.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lut/viewplan.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pipeline/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/io/io_protocol.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/io/hanqtech_loader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/io/ws_source.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/io/ws_png.cpp
)
# macOS 会在每个文件旁生成 ._xxx 资源分支, GLOB 会把它当源文件 -> 编译报错
list(FILTER NF_CORE_SRC EXCLUDE REGEX "/\\._[^/]*$")

add_library(nfocc_core STATIC ${NF_CORE_SRC})
target_include_directories(nfocc_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(nfocc_core PUBLIC NF_VERSION_STR="${NF_VERSION}")
if(NOT MSVC)
  target_compile_options(nfocc_core PRIVATE -O3 -fno-exceptions -fno-rtti)
endif()

# --- 线程: TaskExecutor / FramePool 需要 pthread ---
find_package(Threads REQUIRED)
target_link_libraries(nfocc_core PUBLIC Threads::Threads)

# ---------------------------------------------------------------- vcam 后端
#
# 用 virtualcamera/cpp 的相机模型（KB / radial_poly / e-ftheta / pinhole）
# 替代 NFOCC 自己的投影层。
#
# ## 为什么用 add_subdirectory 而不是拷贝源码
# vcam 是独立工程，自带 CMakeLists 与测试。拷贝进 third_party/ 会产生
# 两份会漂移的副本。add_subdirectory 让它作为子工程参与同一次构建，
# 版本由 NF_VCAM_DIR 指向的目录决定（可以是 git submodule）。
#
# ## 为什么默认 OFF
# 它是可选后端。关闭时 vcam_bridge.cpp 编成桩，`nf_vcam_backend()`
# 返回 NF_VCAM_BACKEND_NONE，业务代码按返回值判断即可。
set(NF_VCAM_FOUND OFF)
if(NF_WITH_VCAM)
  # NF_WITH_VCAM 默认 ON（vcam 是主路径），所以这里**不能 FATAL_ERROR**：
  # 换台机器 / CI 上没有 vcam 时，构建必须能继续（回退到原生实现），
  # 而不是直接失败。用户显式传了 NF_VCAM_DIR 却路径不对才报错。
  if(NOT NF_VCAM_DIR)
    foreach(_vcam_hint
            "${CMAKE_CURRENT_SOURCE_DIR}/../virtualcamera/cpp"
            "/Volumes/Workspace/virtualcamera/cpp"
            "/opt/virtualcamera/cpp")
      if(EXISTS "${_vcam_hint}/CMakeLists.txt")
        set(NF_VCAM_DIR "${_vcam_hint}" CACHE PATH
            "virtualcamera/cpp 源码目录" FORCE)
        break()
      endif()
    endforeach()
  endif()

  if(NF_VCAM_DIR AND NOT EXISTS "${NF_VCAM_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "NF_VCAM_DIR 下没有 CMakeLists.txt: ${NF_VCAM_DIR}")
  endif()
  if(NOT NF_VCAM_DIR)
    message(WARNING
      "NFOCC: NF_WITH_VCAM=ON 但没找到 vcam 源码，回退到原生相机模型。"
      "需要 vcam 时传 -DNF_VCAM_DIR=/path/to/virtualcamera/cpp")
    set(NF_WITH_VCAM OFF)
  endif()
endif()

if(NF_WITH_VCAM)

  # 优先用 third_party/vcam 下的**预编译库**（由 tools/sync_vcam.py 生成）；
  # 没有时才回退到 add_subdirectory 编源码。
  #
  # 为什么优先预编译：NFOCC 的其它三方件（OpenCV/ZeroMQ/ORT）也都是
  # third_party 下的预编译产物，统一起来交叉编译时只有一条路径要维护。
  set(_vcam_prebuilt "${CMAKE_CURRENT_SOURCE_DIR}/third_party/vcam/lib/libvcam_core.a")

  if(EXISTS "${_vcam_prebuilt}")
    set(NF_VCAM_FOUND ON)
    add_library(nf_vcam STATIC IMPORTED)
    set_target_properties(nf_vcam PROPERTIES
      IMPORTED_LOCATION "${_vcam_prebuilt}"
      INTERFACE_INCLUDE_DIRECTORIES
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/vcam/include")
    target_link_libraries(nfocc_core PUBLIC nf_vcam)
    target_compile_definitions(nfocc_core PUBLIC NF_WITH_VCAM=1)
    message(STATUS "NFOCC: vcam 后端 <- third_party/vcam (预编译)")
  else()
    message(STATUS "NFOCC: third_party/vcam 未找到，从源码编")
    set(VCAM_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(VCAM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory("${NF_VCAM_DIR}" "${CMAKE_BINARY_DIR}/vcam" EXCLUDE_FROM_ALL)

    if(TARGET vcam_core)
      set(NF_VCAM_FOUND ON)
      target_link_libraries(nfocc_core PUBLIC vcam_core)
      target_compile_definitions(nfocc_core PUBLIC NF_WITH_VCAM=1)
      message(STATUS "NFOCC: vcam 后端 <- ${NF_VCAM_DIR} (源码)")
    else()
      message(FATAL_ERROR "vcam 子工程没有产出 vcam_core 目标")
    endif()
  endif()
else()
  message(STATUS "NFOCC: vcam 后端关闭 (NF_WITH_VCAM=OFF)")
endif()


# --- OpenMP: 按相机/按行并行 ---
if(NF_USE_OPENMP AND NF_OMP_FOUND)
  target_compile_definitions(nfocc_core PUBLIC NF_HAS_OPENMP=1)
  if(NF_OMP_CMAKE_TARGET)
    target_link_libraries(nfocc_core PUBLIC OpenMP::OpenMP_CXX)
  else()
    # macOS: AppleClang 不带 libomp, 必须手工加头文件与运行库。
    # **不加 -Xpreprocessor -fopenmp 的话 `#pragma omp` 会被静默忽略**
    # (编译通过但完全串行), 这是 macOS 上最容易漏的一条。
    target_include_directories(nfocc_core PUBLIC ${NF_OMP_INC})
    target_link_libraries(nfocc_core PUBLIC ${NF_OMP_LIB})
    target_compile_options(nfocc_core PUBLIC -Xpreprocessor -fopenmp)
  endif()
elseif(NF_USE_OPENMP)
  message(STATUS "NFOCC: OpenMP 不可用, 串行编译")
endif()

# --- Eigen: 位姿/矩阵/最小二乘 ---
if(NF_USE_EIGEN AND NF_EIGEN_FOUND)
  # 用 imported target 而不是变量: Eigen 5.0 不导出 EIGEN3_INCLUDE_DIR
  target_link_libraries(nfocc_core PUBLIC ${NF_EIGEN_TARGET})
  target_compile_definitions(nfocc_core PUBLIC NF_HAS_EIGEN=1)
  if(NF_USE_BLAS AND NF_BLAS_FOUND)
    target_link_libraries(nfocc_core PUBLIC ${NF_BLAS_LIB})
    target_compile_definitions(nfocc_core PUBLIC EIGEN_USE_BLAS=1)
    message(STATUS "NFOCC: Eigen + OpenBLAS 后端")
  else()
    message(STATUS "NFOCC: Eigen (无 BLAS 后端)")
  endif()
endif()

# ---------------------------------------------------------------- 引擎: SGBM
set(NF_SGBM_ON OFF)
if(NF_WITH_OPENCV AND NF_OPENCV_FOUND)
  set(NF_SGBM_ON ON)
  add_library(nfocc_engine_sgbm STATIC
    src/engine/engine_sgbm.cpp
    src/engine/engine_bind_sgbm.cpp)
  target_include_directories(nfocc_engine_sgbm PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
  target_include_directories(nfocc_engine_sgbm PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/engine)
  target_link_libraries(nfocc_engine_sgbm PUBLIC nfocc_core ${OpenCV_LIBS})
  # OpenCV 会抛 cv::Exception。我们**不在库内 catch**(跨 C++ 标准边界异常
  # ABI 不匹配, 实测 SIGTRAP), 改为调用前自查所有 CV_Assert 条件。
  # 但仍需开异常支持, 否则包含 OpenCV 头文件的 TU 无法编译。
  target_compile_options(nfocc_engine_sgbm PRIVATE -fexceptions)
  message(STATUS "NFOCC: OpenCV ${OpenCV_VERSION} -> SGBM 引擎")
elseif(NF_WITH_OPENCV)
  message(WARNING "NFOCC: 未找到 OpenCV, SGBM 引擎跳过")
endif()

# ---------------------------------------------------------------- 引擎: Model
set(NF_ORT_ON OFF)
if(NF_WITH_ORT AND NF_ORT_FOUND)
  set(NF_ORT_ON ON)
  add_library(nfocc_engine_model STATIC
    src/engine/engine_model.cpp
    src/engine/engine_bind_model.cpp)
  target_link_libraries(nfocc_engine_model PUBLIC nfocc_core onnxruntime::onnxruntime)
  target_compile_definitions(nfocc_engine_model PRIVATE NF_WITH_ORT=1)
  # onnxruntime_c_api.h 的 NO_EXCEPTION 在 C++11 下展开为 noexcept,
  # clang 不允许 noexcept 出现在函数指针 typedef 中 -> 该文件单独用 C++17。
  set_source_files_properties(src/engine/engine_model.cpp
    PROPERTIES COMPILE_FLAGS "-std=c++17 -fexceptions")
  message(STATUS "NFOCC: onnxruntime -> 模型引擎")
else()
  add_library(nfocc_engine_model STATIC
    src/engine/engine_model_null.cpp
    src/engine/engine_bind_model.cpp)
  message(STATUS "NFOCC: 模型引擎用空实现")
endif()
target_include_directories(nfocc_engine_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(nfocc_engine_model PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/engine)
target_link_libraries(nfocc_engine_model PUBLIC nfocc_core)

# ---------------------------------------------------------------- IO
# ZLIB 探测**必须在两个分支之前**：PNG 解码在 ZMQ 开/关两种配置下都要用。
# 原实现只在 ZMQ 分支里 find_package，ZMQ=OFF（交叉编译）时
# `ZLIB::ZLIB` 引用到一个不存在的 target，Generate 阶段直接失败。
#
# 交叉编译时找不到 aarch64 的 zlib（宿主 dylib 不能用），此时降级为
# **不编 PNG 支持**（NF_HAS_ZLIB=0），核心库照常构建 —— 板端拿到的
# 是二进制帧输入，本来也不需要解码 PNG。
if(CMAKE_CROSSCOMPILING)
  find_package(ZLIB QUIET)
else()
  find_package(ZLIB QUIET)
endif()
if(ZLIB_FOUND AND NOT CMAKE_CROSSCOMPILING)
  set(NF_HAS_ZLIB 1)
  message(STATUS "NFOCC: ZLIB -> PNG 解码启用")
else()
  set(NF_HAS_ZLIB 0)
  message(STATUS "NFOCC: ZLIB 不可用 -> PNG 解码关闭(仅影响读图，核心库不受影响)")
endif()
# ws_png.cpp 编在 nfocc_core 里，所以宏必须同时给 core 目标，
# 否则交叉编译时它仍会 #include <zlib.h> 去找宿主头文件。
target_compile_definitions(nfocc_core PRIVATE NF_HAS_ZLIB=${NF_HAS_ZLIB})

if(NF_WITH_ZMQ AND NF_ZMQ_FOUND)
add_library(nfocc_io STATIC src/io/zmq_io.cpp src/io/file_io.cpp
                            src/io/input_source.cpp
                            src/io/hanqtech_loader.cpp
                            src/io/ws_source.cpp src/io/ws_png.cpp)
  target_include_directories(nfocc_io PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${ZMQ_INCLUDE_DIRS})
  if(DEFINED NF_ZMQ_LIB_DIR)
    target_include_directories(nfocc_io PUBLIC ${NF_THIRD_PARTY_DIR}/zeromq/include)
    target_link_directories(nfocc_io PUBLIC ${NF_ZMQ_LIB_DIR})
    target_link_libraries(nfocc_io PUBLIC zmq)
  else()
    target_link_libraries(nfocc_io PUBLIC ${ZMQ_LIBRARIES})
    target_link_directories(nfocc_io PUBLIC ${ZMQ_LIBRARY_DIRS})
  endif()
  if(NF_HAS_ZLIB)
    target_link_libraries(nfocc_io PUBLIC ZLIB::ZLIB)
  endif()
  target_compile_options(nfocc_io PRIVATE -fexceptions)
  target_compile_definitions(nfocc_io PUBLIC NF_HAS_ZMQ=1)
  target_compile_definitions(nfocc_io PRIVATE NF_HAS_ZLIB=${NF_HAS_ZLIB})
  message(STATUS "NFOCC: ZeroMQ -> IO")
else()
  add_library(nfocc_io STATIC src/io/file_io.cpp src/io/input_source.cpp
                            src/io/hanqtech_loader.cpp
                            src/io/ws_source.cpp src/io/ws_png.cpp)
  if(NF_HAS_ZLIB)
    target_link_libraries(nfocc_io PUBLIC ZLIB::ZLIB)
  endif()
  target_compile_options(nfocc_io PRIVATE -fexceptions)
  target_compile_definitions(nfocc_io PRIVATE NF_HAS_ZLIB=${NF_HAS_ZLIB})
  message(STATUS "NFOCC: IO 使用文件实现")
endif()
target_include_directories(nfocc_io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(nfocc_io PUBLIC nfocc_core)

# ---------------------------------------------------------------- 主程序
if(NF_BUILD_TOOLS)
  # 外参扰动序列生成器：模拟急刹急转的 yaw/pitch 偏差，供 D4/自校准验证
add_executable(nfocc_disturb_gen tools/nfocc_disturb_gen.cpp)
target_link_libraries(nfocc_disturb_gen PRIVATE nfocc_core)

add_executable(nfocc_run tools/nfocc_run.cpp)
  # **不链接 nfocc_engine_bind**: bind 符号已各自并入引擎库
  target_link_libraries(nfocc_run PRIVATE nfocc_core nfocc_engine_model nfocc_io)
  if(NF_SGBM_ON)
    target_link_libraries(nfocc_run PRIVATE nfocc_engine_sgbm)
    target_compile_definitions(nfocc_run PRIVATE NF_WITH_SGBM=1)
    # nfocc_viz 直接用 SGBM 引擎（可视化视差图），必须链接同一库。
    # 它是**调试工具**：交叉编译时 OpenCV=OFF，无从编起，
    # 故只在 SGBM 可用时构建，避免 aarch64 链接期报 undefined reference。
    # 视差级探针：分离匹配误差与几何/融合误差。
    # 需要 SGBM 引擎与场景渲染（合成真值），故同样只在 NF_SGBM_ON 下构建。
    add_executable(nfocc_disp_probe tools/nfocc_disp_probe.cpp)
    target_link_libraries(nfocc_disp_probe PRIVATE nfocc_core nfocc_engine_model
                                                   nfocc_engine_sgbm)
    target_compile_options(nfocc_disp_probe PRIVATE -fexceptions)

    add_executable(nfocc_viz tools/nfocc_viz.cpp)
    target_link_libraries(nfocc_viz PRIVATE nfocc_core nfocc_engine_model
                                          nfocc_io nfocc_engine_sgbm)
    target_compile_options(nfocc_viz PRIVATE -fexceptions)
  endif()
  if(NF_HAS_ZMQ)
    target_compile_definitions(nfocc_run PRIVATE NF_HAS_ZMQ=1)
  endif()

  # 合成上游数据生成器(端到端联调 / SIL 输入录制)
  add_executable(nfocc_gen tools/nfocc_gen.cpp)
  target_link_libraries(nfocc_gen PRIVATE nfocc_core nfocc_io)
  if(NF_HAS_ZMQ)
    target_compile_definitions(nfocc_gen PRIVATE NF_HAS_ZMQ=1)
  endif()

  # 下游接收端(端到端联调)
  add_executable(nfocc_sink tools/nfocc_sink.cpp)
  target_link_libraries(nfocc_sink PRIVATE nfocc_core nfocc_io)
  if(NF_HAS_ZMQ)
    target_compile_definitions(nfocc_sink PRIVATE NF_HAS_ZMQ=1)
  endif()
endif()

# ---------------------------------------------------------------- 测试
if(NF_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()


# ---------------------------------------------------------------- 开发探针
# 零位移视差探针：同一张图送进 SGBM，视差必须为 0。
# 这是"匹配链路是否自洽"的最小判据，比端到端精度更早发现问题。
# 逐像素视差精度探针（需要 SGBM + vcam）
if(NF_SGBM_ON)
  add_executable(probe_disp_exact tools/probe/probe_disp_exact.cpp)
  target_link_libraries(probe_disp_exact PRIVATE nfocc_core nfocc_engine_sgbm)
  set_target_properties(probe_disp_exact PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

# REV-003 精简器性能探针（不依赖 SGBM）
add_executable(probe_dvf_perf tools/probe/probe_dvf_perf.cpp)
target_link_libraries(probe_dvf_perf PRIVATE nfocc_core)
set_target_properties(probe_dvf_perf PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

if(NF_SGBM_ON)
    add_executable(probe_zero_disp tools/probe/probe_zero_disp.cpp)
    target_link_libraries(probe_zero_disp PRIVATE nfocc_core nfocc_engine_sgbm)
    set_target_properties(probe_zero_disp PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

# ---------------------------------------------------------------- 安装
# 布局:  bin/nfocc_run  lib/libnfocc_*.a  include/nfocc/*.h
# RPATH 用 $ORIGIN/../lib (见 nf_deps.cmake), 整个目录拷走即可运行。
install(TARGETS nfocc_core nfocc_engine_model nfocc_io ARCHIVE DESTINATION lib)
if(TARGET nfocc_engine_sgbm)
  install(TARGETS nfocc_engine_sgbm ARCHIVE DESTINATION lib)
endif()
if(TARGET nfocc_run)
  install(TARGETS nfocc_run RUNTIME DESTINATION bin)
endif()
install(DIRECTORY include/nfocc DESTINATION include
        FILES_MATCHING PATTERN "*.h")

# ---------------------------------------------------------------- 汇总
message(STATUS "---------------------------------------------")
message(STATUS "NFOCC ${NF_VERSION}  (${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR})")
message(STATUS "  构建类型 : ${CMAKE_BUILD_TYPE}")
message(STATUS "  依赖模式 : ${NF_DEPS}")
message(STATUS "  SGBM     : ${NF_SGBM_ON}   Model: ${NF_ORT_ON}")
message(STATUS "  ZeroMQ   : ${NF_ZMQ_FOUND}")
message(STATUS "  OpenMP   : ${NF_OMP_FOUND}   Eigen: ${NF_EIGEN_FOUND}   BLAS: ${NF_BLAS_FOUND}")
message(STATUS "  测试/工具: ${NF_BUILD_TESTS}/${NF_BUILD_TOOLS}")
message(STATUS "---------------------------------------------")
