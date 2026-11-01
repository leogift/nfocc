# NFOCC 测试。分层: unit(单元) / integration(集成) / system(系统)
#
# 运行: ctest --test-dir build --output-on-failure
#       ctest --test-dir build -L unit      # 只跑单元

set(NF_TEST_INC ${CMAKE_CURRENT_SOURCE_DIR})

# ---------------------------------------------------------------- 单元测试
add_executable(test_camera unit/test_camera.cpp)
target_include_directories(test_camera PRIVATE ${NF_TEST_INC})
target_link_libraries(test_camera PRIVATE nfocc_core)
target_compile_options(test_camera PRIVATE -fexceptions)
add_test(NAME unit_camera COMMAND test_camera)
set_tests_properties(unit_camera PROPERTIES LABELS unit)

add_executable(test_geometry unit/test_geometry.cpp)
target_include_directories(test_geometry PRIVATE ${NF_TEST_INC})
target_link_libraries(test_geometry PRIVATE nfocc_core)
target_compile_options(test_geometry PRIVATE -fexceptions)
add_test(NAME unit_geometry COMMAND test_geometry)
set_tests_properties(unit_geometry PROPERTIES LABELS unit)

add_executable(test_voxel unit/test_voxel.cpp)
target_include_directories(test_voxel PRIVATE ${NF_TEST_INC})
target_link_libraries(test_voxel PRIVATE nfocc_core)
target_compile_options(test_voxel PRIVATE -fexceptions)
add_test(NAME unit_voxel COMMAND test_voxel)
set_tests_properties(unit_voxel PROPERTIES LABELS unit)

# 引擎绑定: 防同名符号覆盖的静默错误。需要**同时**链 SGBM 与模型两个库 ——
# 这正是当初暴露问题的链接组合, 所以只有 SGBM 可用时才构建。
if(TARGET nfocc_engine_sgbm)
  add_executable(test_engine_bind unit/test_engine_bind.cpp)
  target_include_directories(test_engine_bind PRIVATE ${NF_TEST_INC})
  target_link_libraries(test_engine_bind PRIVATE nfocc_core nfocc_engine_model
                                                 nfocc_engine_sgbm)
  add_test(NAME unit_engine_bind COMMAND test_engine_bind)
  set_tests_properties(unit_engine_bind PROPERTIES LABELS unit)
endif()

if(TARGET nfocc_engine_sgbm)
  add_executable(test_engine unit/test_engine.cpp)
  target_include_directories(test_engine PRIVATE ${NF_TEST_INC})
  target_link_libraries(test_engine PRIVATE nfocc_core nfocc_engine_sgbm)
  target_compile_options(test_engine PRIVATE -fexceptions)
  add_test(NAME unit_engine COMMAND test_engine)
  set_tests_properties(unit_engine PROPERTIES LABELS unit)
endif()

add_executable(test_concurrency unit/test_concurrency.cpp)
target_include_directories(test_concurrency PRIVATE ${NF_TEST_INC})
target_link_libraries(test_concurrency PRIVATE nfocc_core)
target_compile_options(test_concurrency PRIVATE -fexceptions -pthread)
target_link_libraries(test_concurrency PRIVATE pthread)
add_test(NAME unit_concurrency COMMAND test_concurrency)
set_tests_properties(unit_concurrency PROPERTIES LABELS unit)

add_executable(test_io_protocol unit/test_io_protocol.cpp)
target_include_directories(test_io_protocol PRIVATE ${NF_TEST_INC})
target_link_libraries(test_io_protocol PRIVATE nfocc_core)
target_compile_options(test_io_protocol PRIVATE -fexceptions)
add_test(NAME unit_io_protocol COMMAND test_io_protocol)
set_tests_properties(unit_io_protocol PROPERTIES LABELS unit)

add_executable(test_match unit/test_match.cpp)
target_include_directories(test_match PRIVATE ${NF_TEST_INC})
target_link_libraries(test_match PRIVATE nfocc_core)
target_compile_options(test_match PRIVATE -fexceptions)
add_test(NAME unit_match COMMAND test_match)
set_tests_properties(unit_match PROPERTIES LABELS unit)

add_executable(test_hanqtech unit/test_hanqtech.cpp)
target_include_directories(test_hanqtech PRIVATE ${NF_TEST_INC})
target_link_libraries(test_hanqtech PRIVATE nfocc_core)
target_compile_options(test_hanqtech PRIVATE -fexceptions)
add_test(NAME unit_hanqtech COMMAND test_hanqtech)
set_tests_properties(unit_hanqtech PROPERTIES LABELS unit)

add_executable(test_scene unit/test_scene.cpp)
target_include_directories(test_scene PRIVATE ${NF_TEST_INC})
target_link_libraries(test_scene PRIVATE nfocc_core)
target_compile_options(test_scene PRIVATE -fexceptions)
add_test(NAME unit_scene COMMAND test_scene)
set_tests_properties(unit_scene PROPERTIES LABELS unit)

# ---------------------------------------------------------------- 集成测试
add_executable(test_pipeline integration/test_pipeline.cpp)
target_include_directories(test_pipeline PRIVATE ${NF_TEST_INC})
target_link_libraries(test_pipeline PRIVATE nfocc_core nfocc_engine_model)
if(TARGET nfocc_engine_sgbm)
  target_link_libraries(test_pipeline PRIVATE nfocc_engine_sgbm)
  target_compile_definitions(test_pipeline PRIVATE NF_WITH_SGBM=1)
endif()
target_compile_options(test_pipeline PRIVATE -fexceptions)
add_test(NAME integration_pipeline COMMAND test_pipeline)
set_tests_properties(integration_pipeline PROPERTIES LABELS integration)

if(TARGET nfocc_engine_sgbm)
  add_executable(test_depth_accuracy integration/test_depth_accuracy.cpp)
  target_include_directories(test_depth_accuracy PRIVATE ${NF_TEST_INC})
  target_link_libraries(test_depth_accuracy PRIVATE nfocc_core nfocc_engine_sgbm)
  target_compile_options(test_depth_accuracy PRIVATE -fexceptions)
  add_test(NAME depth_accuracy COMMAND test_depth_accuracy)
  set_tests_properties(depth_accuracy PROPERTIES LABELS integration)
endif()

# ---------------------------------------------------------------- 系统测试
if(TARGET nfocc_run)
  add_test(NAME system_run_synthetic
           COMMAND nfocc_run --synthetic --frames 3 --stats)
  set_tests_properties(system_run_synthetic PROPERTIES
                       LABELS system
                       PASS_REGULAR_EXPRESSION "稳态 RSS")
endif()
