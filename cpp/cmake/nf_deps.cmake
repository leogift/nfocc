# 第三方依赖管理
#
# ## 策略(三层, 按优先级)
#
# 1. **首选 vcpkg / Conan**(现代包管理): `cmake/toolchain-vcpkg.cmake`
#    或 `-DCMAKE_TOOLCHAIN_FILE=.../conan_toolchain.cmake`
#    —— 由包管理器接管下载、版本锁定、ABI 一致性。
#
# 2. **次选: 工程内 `third_party/` 内置**(本目录)
#    把依赖 (header 或 dylib/so) 拷到 `third_party/<name>/`, CMake 从
#    **工程内路径**找, 并用 **RPATH**(`$ORIGIN/../lib`) 在运行时定位。
#    优点: 工程自包含, 换机器/上板不需重装系统库。
#
# 3. **兜底: 系统路径**(`find_package` 的默认行为)
#
# 三者的选择由 `-DNF_DEPS=managed|vendored|system` 控制, 默认 `managed`。
#
# ## 为什么必须管 RPATH
#   部署到 Aarch64 板端时, `libnform` 依赖的 `libopencv_core.so` / `libzmq.so`
#   通常不在系统默认搜索路径。不写 RPATH 就要用户手动设 `LD_LIBRARY_PATH`,
#   这在量产镜像里不可接受。RPATH 方案:
#     macOS:  @loader_path/../lib   (install_name_tool / -Wl,-rpath)
#     Linux:  $ORIGIN/../lib
#   相对路径跟着可执行文件走, 整个目录拷过去就能跑。

# ---------------------------------------------------------------- RPATH
# 一律使用**相对** RPATH(可重定位), 并把 build 树与 install 树都设好。
set(NF_RPATH_BUILD   "$ORIGIN/../lib;$ORIGIN/../../lib")
set(NF_RPATH_INSTALL "$ORIGIN/../lib")

if(APPLE)
  # macOS 用 @loader_path 表示"可执行文件所在目录"
  set(NF_RPATH_BUILD
      "@loader_path/../lib;@loader_path/../../lib;@loader_path/../../../lib")
  set(NF_RPATH_INSTALL "@loader_path/../lib")
endif()

set(CMAKE_SKIP_BUILD_RPATH            FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH    FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
set(CMAKE_INSTALL_RPATH               "${NF_RPATH_INSTALL}")
set(CMAKE_BUILD_RPATH                 "${NF_RPATH_BUILD}")

# ---------------------------------------------------------------- 依赖模式
set(NF_DEPS "managed" CACHE STRING "依赖来源: managed | vendored | system")
set_property(CACHE NF_DEPS PROPERTY STRINGS managed vendored system)

set(NF_THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")

# ==================================================================
# 交叉编译守卫
#
# 宿主(macOS)Homebrew 里的库是 arm64-**darwin**, 不能链进 linux 目标。
# 若不拦, 表现为: 配置成功、打印"Eigen 5.0.1 <- 系统", 直到链接期
# 才报 "building for macOS-arm64 but attempting to link with file built
# for GNU/Linux-arm64" —— 或者更糟, 侥幸通过但运行崩。
#
# 因此交叉编译时**跳过所有宿主依赖探测**, 只保留 header-only 的 Eigen
# (若 sysroot 里有), 其余一律关闭。调用方要用第三方库, 必须显式提供
# NF_SYSROOT 或 NF_VCPKG_ROOT。
# ==================================================================
set(NF_CROSS_COMPILING OFF)
if(CMAKE_CROSSCOMPILING)
  set(NF_CROSS_COMPILING ON)
  message(STATUS "NFOCC: 交叉编译模式 -> 跳过宿主依赖 (${CMAKE_SYSTEM_PROCESSOR})")
endif()


# ---------------------------------------------------------------- Eigen
# header-only, 无需 RPATH。
# **Eigen 5.0 的 Eigen3Config 只导出 `Eigen3::Eigen` 目标, 不设 EIGEN3_INCLUDE_DIR**
# —— 只读变量会得到空串, 表现为"找到 Eigen 了但编译时没 include 路径"
# (实测: flags.make 里只有 -Iinclude 和 libomp, 没有 eigen, 而日志却打印了
#  "Eigen 5.0.1 <- 系统", 是假象)。因此优先用 imported target。
set(NF_EIGEN_FOUND OFF)
set(NF_EIGEN_TARGET "")

if(NF_CROSS_COMPILING)
  # 交叉编译: 只用 toolchain 的 include(eigen 是 header-only, 找得到就能用)
  find_path(NF_EIGEN_CROSS_INC Eigen/Core PATHS ${CMAKE_FIND_ROOT_PATH})
  if(NF_EIGEN_CROSS_INC)
    add_library(nf_eigen INTERFACE)
    target_include_directories(nf_eigen INTERFACE ${NF_EIGEN_CROSS_INC})
    set(NF_EIGEN_TARGET nf_eigen)
    set(NF_EIGEN_FOUND ON)
    message(STATUS "NFOCC: Eigen <- sysroot ${NF_EIGEN_CROSS_INC}")
  else()
    message(STATUS "NFOCC: 交叉编译无 Eigen (可选, 不影响核心库)")
  endif()
elseif(NF_DEPS STREQUAL "vendored")
  if(EXISTS "${NF_THIRD_PARTY_DIR}/eigen/include/eigen3/Eigen/Core")
    set(EIGEN3_INCLUDE_DIR "${NF_THIRD_PARTY_DIR}/eigen/include/eigen3" CACHE PATH "" FORCE)
    set(NF_EIGEN_FOUND ON)
    message(STATUS "NFOCC: Eigen <- third_party (内置)")
  endif()
endif()

# **交叉编译时不得回退到宿主的 Eigen** —— 否则 include 路径指向 macOS 的
# /opt/homebrew/include/eigen3, 链接期才炸。守卫必须放在这里, 不能只放 vendored 分支。
if(NOT NF_EIGEN_FOUND AND NOT NF_CROSS_COMPILING)
  find_package(Eigen3 QUIET)
  if(TARGET Eigen3::Eigen)
    set(NF_EIGEN_FOUND ON)
    set(NF_EIGEN_TARGET Eigen3::Eigen)
    message(STATUS "NFOCC: Eigen ${Eigen3_VERSION} <- 系统 (imported target)")
  endif()
endif()

# 兜底: 只有变量没有目标时, 自己造一个接口目标, 保证 include 能传下去
if(NF_EIGEN_FOUND AND NOT NF_EIGEN_TARGET)
  if(NOT EIGEN3_INCLUDE_DIR)
    find_path(EIGEN3_INCLUDE_DIR signature_of_eigen3_matrix_library
              PATH_SUFFIXES eigen3)
  endif()
  if(EIGEN3_INCLUDE_DIR)
    add_library(nf_eigen INTERFACE)
    target_include_directories(nf_eigen INTERFACE ${EIGEN3_INCLUDE_DIR})
    set(NF_EIGEN_TARGET nf_eigen)
    message(STATUS "NFOCC: Eigen <- ${EIGEN3_INCLUDE_DIR}")
  else()
    set(NF_EIGEN_FOUND OFF)
    message(STATUS "NFOCC: Eigen 未找到 (无 target 也无头文件路径)")
  endif()
endif()

# ---------------------------------------------------------------- OpenBLAS
set(NF_BLAS_FOUND OFF)
set(NF_BLAS_LIB     "")
set(NF_BLAS_INCLUDE "")
# **交叉编译时绝不取宿主 BLAS** —— /opt/homebrew 下是 mach-o aarch64-darwin,
# 链进 linux 目标会在链接期报格式不兼容(或侥幸通过但运行崩)。
if(NF_DEPS STREQUAL "vendored" AND NOT NF_CROSS_COMPILING)
  file(GLOB NF_BLAS_CAND "${NF_THIRD_PARTY_DIR}/openblas/lib/libopenblas*")
  if(NF_BLAS_CAND)
    list(GET NF_BLAS_CAND 0 NF_BLAS_LIB)
    set(NF_BLAS_INCLUDE "${NF_THIRD_PARTY_DIR}/openblas/include")
    set(NF_BLAS_FOUND ON)
    message(STATUS "NFOCC: OpenBLAS <- third_party (内置)")
  endif()
endif()
if(NOT NF_BLAS_FOUND AND NOT NF_CROSS_COMPILING)
  # 注意: Homebrew 的 libopenblas.dylib 是**符号链接**, find_library 默认
  # 不跟随符号链接 -> 直接失败。改用 glob 取实体文件(带版本号的那个)。
  file(GLOB NF_BLAS_ENT "${NF_THIRD_PARTY_DIR}/openblas/lib/libopenblas*"
                         "/opt/homebrew/opt/openblas/lib/libopenblas*"
                         "/usr/local/opt/openblas/lib/libopenblas*"
                         "/usr/lib/*/libopenblas*")
  set(NF_BLAS_LIB "")
  foreach(c ${NF_BLAS_ENT})
    get_filename_component(d "${c}" DIRECTORY)
    get_filename_component(e "${c}" NAME)
    if(e MATCHES "\\.(dylib|so)(\\.[0-9]+)*$")
      # 排除软链(指向带版本号的实体文件), 优先取实体
      execute_process(COMMAND test -L "${c}" RESULT_VARIABLE _islink)
      if(NOT _islink EQUAL 0)
        set(NF_BLAS_LIB "${c}")
        break()
      endif()
    endif()
  endforeach()
  if(NF_BLAS_LIB)
    set(NF_BLAS_FOUND ON)
    get_filename_component(NF_BLAS_DIR "${NF_BLAS_LIB}" DIRECTORY)
    message(STATUS "NFOCC: OpenBLAS <- ${NF_BLAS_LIB}")
  endif()
endif()

# ---------------------------------------------------------------- OpenMP
# macOS 的 AppleClang **不带** libomp, 必须显式链接 Homebrew 的 libomp,
# 且要在编译期加 -Xpreprocessor -fopenmp。这是 macOS 上最常见的坑:
# 不加这两条, `#pragma omp` 会被静默忽略(编译通过, 但是串行)。
set(NF_OMP_FOUND OFF)
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  set(NF_OMP_FOUND ON)
  set(NF_OMP_CMAKE_TARGET ON)      # 有 OpenMP::OpenMP_CXX 目标
  message(STATUS "NFOCC: OpenMP ${OpenMP_CXX_VERSION} <- CMake 默认")
elseif(APPLE)
  # 手工定位 Homebrew libomp
  find_path(NF_OMP_INC NAMES omp.h PATHS /opt/homebrew/opt/libomp/include
            /usr/local/opt/libomp/include)
  find_library(NF_OMP_LIB NAMES omp PATHS /opt/homebrew/opt/libomp/lib
               /usr/local/opt/libomp/lib)
  if(NF_OMP_INC AND NF_OMP_LIB)
    set(NF_OMP_FOUND ON)
    add_compile_options(-Xpreprocessor -fopenmp)
    set(NF_OMP_HAND_LINKED ON)
    message(STATUS "NFOCC: OpenMP <- libomp ${NF_OMP_LIB}")
  endif()
endif()
if(NOT NF_OMP_FOUND)
  message(STATUS "NFOCC: 未找到 OpenMP, 全部编译为串行")
endif()

# ---------------------------------------------------------------- OpenCV
set(NF_OPENCV_FOUND OFF)
if(NF_DEPS STREQUAL "vendored")
  if(EXISTS "${NF_THIRD_PARTY_DIR}/opencv/lib")
    list(APPEND CMAKE_PREFIX_PATH "${NF_THIRD_PARTY_DIR}/opencv")
  endif()
endif()
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
  set(NF_OPENCV_FOUND ON)
endif()

# ---------------------------------------------------------------- ZeroMQ
set(NF_ZMQ_FOUND OFF)
if(NF_DEPS STREQUAL "vendored")
  file(GLOB NF_ZMQ_CAND "${NF_THIRD_PARTY_DIR}/zeromq/lib/libzmq*")
  if(NF_ZMQ_CAND)
    list(GET NF_ZMQ_CAND 0 NF_ZMQ_LIB_FILE)
    get_filename_component(NF_ZMQ_LIB_DIR "${NF_ZMQ_LIB_FILE}" DIRECTORY)
    set(NF_ZMQ_FOUND ON)
    message(STATUS "NFOCC: ZeroMQ <- third_party (内置)")
  endif()
endif()
if(NOT NF_ZMQ_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(ZMQ QUIET libzmq)
    if(ZMQ_FOUND)
      set(NF_ZMQ_FOUND ON)
    endif()
  endif()
endif()

# ---------------------------------------------------------------- onnxruntime
set(NF_ORT_FOUND OFF)
if(NF_DEPS STREQUAL "vendored")
  if(EXISTS "${NF_THIRD_PARTY_DIR}/onnxruntime/lib")
    list(APPEND CMAKE_PREFIX_PATH "${NF_THIRD_PARTY_DIR}/onnxruntime")
  endif()
endif()
find_package(onnxruntime QUIET)
if(onnxruntime_FOUND)
  set(NF_ORT_FOUND ON)
endif()
