cmake_minimum_required(VERSION 4.2)


# >>> Project
set(NAME "VehicleFrameworkVR")
set(FRIENDLY_NAME "Vehicle Framework VR")
set(VERSION 0.1.0)

message(">>> Start build for '${NAME}' v${VERSION}")

# >>> Folders
set(ROOT_DIR  "${CMAKE_CURRENT_SOURCE_DIR}")
set(SOURCE_DIR "${ROOT_DIR}/src")
set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# >>> Include guards
if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(FATAL_ERROR ">> In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there.")
endif()

# >>> Set variable from environment by same name
macro(set_from_environment VARIABLE)
  if(NOT DEFINED ${VARIABLE} AND DEFINED ENV{${VARIABLE}})
    set(${VARIABLE} $ENV{${VARIABLE}})
  endif()
endmacro()


# >>> Setup vcpkg
message(">>> Setup vcpkg")
set_from_environment(VCPKG_ROOT)
if(DEFINED VCPKG_ROOT)
  set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
  set(VCPKG_TARGET_TRIPLET "x64-windows-static" CACHE STRING "")
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>" CACHE STRING "")
else()
  message(FATAL_ERROR ">> Variable VCPKG_ROOT is not set!")
endif()


# >>> Locate F4VR-CommonFramework (submodule or external path)
if(DEFINED F4VR_COMMON_FRAMEWORK_PATH AND NOT F4VR_COMMON_FRAMEWORK_PATH STREQUAL "")
  set(F4VR_COMMON_FRAMEWORK_PATH_SELECTED ${F4VR_COMMON_FRAMEWORK_PATH} CACHE STRING "" FORCE)
  message(">>> Using F4VR-CommonFramework from custom variable: '${F4VR_COMMON_FRAMEWORK_PATH_SELECTED}'")
else()
  set(F4VR_COMMON_FRAMEWORK_PATH_SELECTED "${ROOT_DIR}/external/F4VR-CommonFramework" CACHE STRING "" FORCE)
  message(">>> Using F4VR-CommonFramework in external: '${F4VR_COMMON_FRAMEWORK_PATH_SELECTED}'")
endif()
# Verify path is correct
if(NOT EXISTS "${F4VR_COMMON_FRAMEWORK_PATH_SELECTED}/src/ModBase.h")
  message(FATAL_ERROR ">> F4VR-CommonFramework path not found in '${F4VR_COMMON_FRAMEWORK_PATH_SELECTED}'!\n >> Did you init submodules? (git submodule update --init --recursive)\n >> or set F4VR_COMMON_FRAMEWORK_PATH?")
endif()

# >>> Create Project
project(${NAME} VERSION ${VERSION} LANGUAGES CXX)
message(">>> Building project '${PROJECT_NAME}' v:${PROJECT_VERSION}")
message(">>> Using found F4VR-CommonFramework in: '${F4VR_COMMON_FRAMEWORK_PATH_SELECTED}'")

# >>> Build flags
add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
set(Boost_USE_STATIC_RUNTIME OFF CACHE BOOL "")
set(Boost_USE_STATIC_LIBS ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF)

# >>> Bring in the framework (it brings CommonLibF4 transitively)
add_subdirectory(${F4VR_COMMON_FRAMEWORK_PATH_SELECTED} "F4VR-CommonFramework" EXCLUDE_FROM_ALL)
# >>> IMPORTANT:
# For CommonLibF4 to build successfully you must satisfy it's vcpkg dependencies
# AND specify <"builtin-baseline": "b4a3d89125e45bc8f80fb94bef9761d4f4e14fb9"> as it will fail to build with latests"


# >>> Version & resource files
configure_file(${ROOT_DIR}/cmake/version.rc.in  ${BUILD_DIR}/version.rc  @ONLY)
configure_file(${ROOT_DIR}/cmake/Version.h.in   ${BUILD_DIR}/include/Version.h @ONLY)
configure_file(${ROOT_DIR}/cmake/resources.rc.in ${BUILD_DIR}/resources.rc @ONLY)
source_group(TREE ${BUILD_DIR} FILES ${BUILD_DIR}/include/Version.h ${BUILD_DIR}/resources.rc)

# >>> Sources
file(GLOB_RECURSE headers CONFIGURE_DEPENDS "${SOURCE_DIR}/*.h")
file(GLOB_RECURSE sources CONFIGURE_DEPENDS "${SOURCE_DIR}/*.cpp")
file(GLOB_RECURSE inls    CONFIGURE_DEPENDS "${SOURCE_DIR}/*.inl")
source_group(TREE ${ROOT_DIR} FILES ${headers} ${sources} ${inls})

# >>> Resource files
list(APPEND resource_files
  "${ROOT_DIR}/data/config/${NAME}.ini"
)
# NOTE: Additional resources added here

# >>> Target (DLL)
add_library(${PROJECT_NAME} SHARED
  ${headers}
  ${sources}
  ${inls}
  ${resource_files}
  ${BUILD_DIR}/include/Version.h
  ${BUILD_DIR}/version.rc
  ${BUILD_DIR}/resources.rc
  "${ROOT_DIR}/.clang-format"
  "${ROOT_DIR}/.editorconfig"
)

# >>> C++ standard & defs
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_definitions(${PROJECT_NAME} PRIVATE _UNICODE)

# >>> Includes
target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${BUILD_DIR}/include
    ${SOURCE_DIR}
)

# >>> Dependencies
# Link only the framework; it re-exports CommonLibF4 and other usage requirements.
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    F4VRCommon::framework
)

# cpptrace via vcpkg
find_package(cpptrace CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE cpptrace::cpptrace)

# nlohmann-json via vcpkg (vehicle profiles)
find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)

# >>> PCH
target_precompile_headers(${PROJECT_NAME} PRIVATE src/PCH.h)

# >>> Compile / Link options
target_compile_options(${PROJECT_NAME}
  PRIVATE
    /sdl # Enable Additional Security Checks
	/utf-8 # Set Source and Executable character sets to UTF-8
	/Zi # Debug Information Format

	/permissive- # Standards conformance

	/Zc:alignedNew # C++17 over-aligned allocation
	/Zc:auto # Deduce Variable Type
	/Zc:char8_t
	/Zc:__cplusplus # Enable updated __cplusplus macro
	/Zc:externC
	/Zc:externConstexpr # Enable extern constexpr variables
	/Zc:forScope # Force Conformance in for Loop Scope
	/Zc:hiddenFriend
	/Zc:implicitNoexcept # Implicit Exception Specifiers
	/Zc:lambda
	/Zc:noexceptTypes # C++17 noexcept rules
	/Zc:preprocessor # Enable preprocessor conformance mode
	/Zc:referenceBinding # Enforce reference binding rules
	/Zc:rvalueCast # Enforce type conversion rules
	/Zc:sizedDealloc # Enable Global Sized Deallocation Functions
	/Zc:strictStrings # Disable string literal type conversion
	/Zc:ternary # Enforce conditional operator rules
	/Zc:threadSafeInit # Thread-safe Local Static Initialization
	/Zc:tlsGuards
	/Zc:trigraphs # Trigraphs Substitution
	/Zc:wchar_t # wchar_t Is Native Type

	/external:anglebrackets
	/external:W0

    /W4 # Warning level
    # Tracy: the framework's profiling configuration, a Release build with the Tracy client
    "$<$<CONFIG:Release,Tracy>:/Zc:inline;/JMC-;/Ob3>"
)

target_link_options(${PROJECT_NAME}
  PRIVATE
    /WX # Treat Linker Warnings as Errors
    "$<$<CONFIG:DEBUG>:/INCREMENTAL;/OPT:NOREF;/OPT:NOICF>"
    "$<$<CONFIG:Release,Tracy>:/INCREMENTAL:NO;/OPT:REF;/OPT:ICF;/DEBUG:FULL>"
)

# >>> fix a warning with /Ob2 being overridden with /Ob3
string(REPLACE "/Ob2" "/Ob3" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
string(REPLACE "/Ob2" "/Ob3" CMAKE_CXX_FLAGS_TRACY "${CMAKE_CXX_FLAGS_TRACY}")

# >>> Offline tests of the vehicle model (pure C++, no game, no PCH): suspension, tyre, drivetrain and
# a flat-ground drive simulation that checks the reference profile against the design targets.
option(VFVR_BUILD_TESTS "Build the offline vehicle model tests" ON)
if(VFVR_BUILD_TESTS)
  file(GLOB model_sources CONFIGURE_DEPENDS "${SOURCE_DIR}/model/*.cpp")
  add_executable(VehicleModelTests "${ROOT_DIR}/tests/VehicleModelTests.cpp" ${model_sources})
  target_compile_features(VehicleModelTests PRIVATE cxx_std_23)
  target_include_directories(VehicleModelTests PRIVATE ${SOURCE_DIR})
  target_link_libraries(VehicleModelTests PRIVATE nlohmann_json::nlohmann_json)
  target_compile_options(VehicleModelTests PRIVATE /utf-8 /permissive- /W4)
  set_target_properties(VehicleModelTests PROPERTIES INTERPROCEDURAL_OPTIMIZATION OFF)
  enable_testing()
  add_test(NAME VehicleModelTests COMMAND VehicleModelTests "${ROOT_DIR}/data/mod/F4SE/Plugins/VehicleFrameworkVR/Vehicles/sedan.json")

  # The motorcycle: the roll stabiliser (stands up, leans into corners, lets go of a fallen bike) and its performance.
  add_executable(MotoTests "${ROOT_DIR}/tests/MotoTests.cpp" ${model_sources})
  target_compile_features(MotoTests PRIVATE cxx_std_23)
  target_include_directories(MotoTests PRIVATE ${SOURCE_DIR} "${ROOT_DIR}/tests")
  target_link_libraries(MotoTests PRIVATE nlohmann_json::nlohmann_json)
  target_compile_options(MotoTests PRIVATE /utf-8 /permissive- /W4)
  set_target_properties(MotoTests PROPERTIES INTERPROCEDURAL_OPTIMIZATION OFF)
  add_test(NAME MotoTests COMMAND MotoTests "${ROOT_DIR}/data/mod/F4SE/Plugins/VehicleFrameworkVR/Vehicles/rusianmother.json")

  # Vehicle health and radiation protection: the damage formulas, the collision damage, the conditions, the repair and the profile defaults.
  add_executable(HealthTests "${ROOT_DIR}/tests/HealthTests.cpp" ${model_sources})
  target_compile_features(HealthTests PRIVATE cxx_std_23)
  target_include_directories(HealthTests PRIVATE ${SOURCE_DIR} "${ROOT_DIR}/tests")
  target_link_libraries(HealthTests PRIVATE nlohmann_json::nlohmann_json)
  target_compile_options(HealthTests PRIVATE /utf-8 /permissive- /W4)
  set_target_properties(HealthTests PROPERTIES INTERPROCEDURAL_OPTIMIZATION OFF)
  add_test(NAME HealthTests COMMAND HealthTests "${ROOT_DIR}/data/mod/F4SE/Plugins/VehicleFrameworkVR/Vehicles/sedan.json")

  # Battery: the drain between idle and full throttle, the charging, the dead engine and the profile's battery block.
  add_executable(BatteryTests "${ROOT_DIR}/tests/BatteryTests.cpp" ${model_sources})
  target_compile_features(BatteryTests PRIVATE cxx_std_23)
  target_include_directories(BatteryTests PRIVATE ${SOURCE_DIR} "${ROOT_DIR}/tests")
  target_link_libraries(BatteryTests PRIVATE nlohmann_json::nlohmann_json)
  target_compile_options(BatteryTests PRIVATE /utf-8 /permissive- /W4)
  set_target_properties(BatteryTests PROPERTIES INTERPROCEDURAL_OPTIMIZATION OFF)
  add_test(NAME BatteryTests COMMAND BatteryTests)

  add_executable(TurretTests "${ROOT_DIR}/tests/TurretTests.cpp" ${model_sources})
  target_compile_features(TurretTests PRIVATE cxx_std_23)
  target_include_directories(TurretTests PRIVATE ${SOURCE_DIR} "${ROOT_DIR}/tests")
  target_link_libraries(TurretTests PRIVATE nlohmann_json::nlohmann_json)
  target_compile_options(TurretTests PRIVATE /utf-8 /permissive- /W4)
  set_target_properties(TurretTests PROPERTIES INTERPROCEDURAL_OPTIMIZATION OFF)
  add_test(NAME TurretTests COMMAND TurretTests)
  # Every turret type shipped in data/mod (the JSON files the plugin reads).
  file(GLOB turret_files "${ROOT_DIR}/data/mod/F4SE/Plugins/VehicleFrameworkVR/Turrets/*.json")
  add_test(NAME TurretFilesTests COMMAND TurretTests ${turret_files})
endif()

# >>> Post build command to copy plugins (supports multiple paths separated by ';')
# COPY_PLUGIN_CONFIGURATIONS limits the copy to some build configurations, e.g. "Release" or "Debug;RelWithDebInfo" (case-insensitive)
set(COPY_PLUGIN_CONFIGURATIONS "all" CACHE STRING "Build configurations the post build plugin copy runs for: 'all' or a ';' separated list")
if(POST_BUILD_COPY_PLUGIN)
  if(DEFINED COPY_PLUGIN_BASE_PATH)
    # Treat semicolon-separated COPY_PLUGIN_BASE_PATH as a CMake list
    set(COPY_PLUGIN_PATH_LIST ${COPY_PLUGIN_BASE_PATH})

    # Resolved per configuration at generate time: $<CONFIG:a,b> is 1 when building any of them
    string(TOLOWER "${COPY_PLUGIN_CONFIGURATIONS}" copy_plugin_configs)
    if(copy_plugin_configs STREQUAL "all")
      set(copy_plugin_enabled 1)
    else()
      if(CMAKE_CONFIGURATION_TYPES)
        string(TOLOWER "${CMAKE_CONFIGURATION_TYPES}" known_configs)
        foreach(config IN LISTS copy_plugin_configs)
          if(NOT config IN_LIST known_configs)
            message(FATAL_ERROR "COPY_PLUGIN_CONFIGURATIONS has unknown configuration '${config}', expected 'all' or any of: ${CMAKE_CONFIGURATION_TYPES}")
          endif()
        endforeach()
      endif()
      string(REPLACE ";" "," copy_plugin_configs "${copy_plugin_configs}")
      set(copy_plugin_enabled "$<CONFIG:${copy_plugin_configs}>")
    endif()

    message(">>> Create post build command: copy targets on '${COPY_PLUGIN_CONFIGURATIONS}' configurations to:")
    foreach(copy_path IN LISTS COPY_PLUGIN_PATH_LIST)
      set(copy_dir "${copy_path}/F4SE/Plugins")
      message("    ${copy_dir}/")
      # Each command is wrapped in $<cond:...> so it expands to nothing, and is dropped, for the configurations it does not run on
      add_custom_command(
        TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMENT "Post build copy of ${PROJECT_NAME}.dll and .pdb to plugins folder: '${copy_dir}/'"
        COMMAND "$<${copy_plugin_enabled}:${CMAKE_COMMAND};-E;make_directory;${copy_dir}>"
        COMMAND "$<${copy_plugin_enabled}:${CMAKE_COMMAND};-E;copy;$<TARGET_FILE:${PROJECT_NAME}>;$<TARGET_PDB_FILE:${PROJECT_NAME}>;${copy_dir}>"
        COMMAND "$<$<NOT:${copy_plugin_enabled}>:${CMAKE_COMMAND};-E;echo;Skipping: $<CONFIG> is not in COPY_PLUGIN_CONFIGURATIONS>"
        COMMAND_EXPAND_LISTS
      )
    endforeach()
  else()
    message(FATAL_ERROR "Variable 'COPY_PLUGIN_BASE_PATH' is not defined but 'POST_BUILD_COPY_PLUGIN' is true!")
  endif()
else()
  message(">>> No post build copy")
endif()

# >>> Target to package the mod into a versioned .7z file in build/package/, see cmake/package.cmake
# Not part of a normal build, build it explicitly: `cmake --build build --config Release --target package_mod`
# or a build preset with "targets": ["package_mod"]. It builds the plugin first when it is out of date.
add_custom_target(package_mod
  COMMAND ${CMAKE_COMMAND}
          "-DPROJECT_NAME=${PROJECT_NAME}"
          "-DPROJECT_FRIENDLY_NAME=${FRIENDLY_NAME}"
          "-DPROJECT_VERSION=${PROJECT_VERSION}"
          "-DROOT_DIR=${ROOT_DIR}"
          "-DBUILD_DIR=${BUILD_DIR}"
          "-DTARGET_FILE=$<TARGET_FILE:${PROJECT_NAME}>"
          "-DTARGET_PDB_FILE=$<TARGET_PDB_FILE:${PROJECT_NAME}>"
          "-DCONFIG=$<CONFIG>"
          -P "${ROOT_DIR}/cmake/package.cmake"
  COMMENT "Packaging ${PROJECT_NAME} into '${BUILD_DIR}/package/'"
  VERBATIM
)
add_dependencies(package_mod ${PROJECT_NAME})
