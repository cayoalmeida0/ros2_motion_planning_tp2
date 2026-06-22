cmake_minimum_required(VERSION 3.16)
project(libaria VERSION 3.0.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

option(LIBARIA_BUILD_LEGACY_TEST_PROGRAMS "Build curated legacy test executables" ON)
option(LIBARIA_BUILD_LEGACY_EXAMPLE_PROGRAMS "Build curated legacy example executables" ON)
option(LIBARIA_BUILD_SAFE_TEST_RUNNER "Build the safe non-motion test runner" ON)
option(LIBARIA_BUILD_HARDWARE_TEST_RUNNER "Build the opt-in hardware test runner" ON)
option(LIBARIA_REGISTER_HARDWARE_CTEST "Register the hardware test runner with CTest" OFF)

set(ARIA_SRC
  src/ArAction.cpp
  src/ArActionAvoidFront.cpp
  src/ArActionAvoidSide.cpp
  src/ArActionBumpers.cpp
  src/ArActionColorFollow.cpp
  src/ArActionConstantVelocity.cpp
  src/ArActionDeceleratingLimiter.cpp
  src/ArActionDesired.cpp
  src/ArActionDriveDistance.cpp
  src/ArActionGoto.cpp
  src/ArActionGotoStraight.cpp
  src/ArActionGroup.cpp
  src/ArActionGroups.cpp
  src/ArActionInput.cpp
  src/ArActionIRs.cpp
  src/ArActionJoydrive.cpp
  src/ArActionKeydrive.cpp
  src/ArActionLimiterBackwards.cpp
  src/ArActionLimiterForwards.cpp
  src/ArActionLimiterRot.cpp
  src/ArActionLimiterTableSensor.cpp
  src/ArActionMovementParameters.cpp
  src/ArActionMovementParametersDebugging.cpp
  src/ArActionRatioInput.cpp
  src/ArActionRobotJoydrive.cpp
  src/ArActionStallRecover.cpp
  src/ArActionStop.cpp
  src/ArActionTriangleDriveTo.cpp
  src/ArActionTurn.cpp
  src/ArACTS.cpp
  src/ArAMPTU.cpp
  src/ArAnalogGyro.cpp
  src/ArArg.cpp
  src/ArArgumentBuilder.cpp
  src/ArArgumentParser.cpp
  src/ArASyncTask.cpp
  src/ArBasePacket.cpp
  src/ArBatteryConnector.cpp
  src/ArBatteryMTX.cpp
  src/ArBumpers.cpp
  src/ArCameraCommands.cpp
  src/ArCameraCollection.cpp
  src/ArCondition_LIN.cpp
  src/ArConfig.cpp
  src/ArConfigArg.cpp
  src/ArConfigGroup.cpp
  src/ArDataLogger.cpp
  src/ArDeviceConnection.cpp
  src/ArDPPTU.cpp
  src/ArFileDeviceConnection.cpp
  src/ArFileParser.cpp
  src/ArForbiddenRangeDevice.cpp
  src/ArFunctorASyncTask.cpp
  src/ArGPS.cpp
  src/ArGPSConnector.cpp
  src/ArGPSCoords.cpp
  src/ArGripper.cpp
  src/ArInterpolation.cpp
  src/ArIrrfDevice.cpp
  src/ArIRs.cpp
  src/ArJoyHandler.cpp
  src/ArKeyHandler.cpp
  src/ArLaser.cpp
  src/ArLaserConnector.cpp
  src/ArLaserFilter.cpp
  src/ArLaserLogger.cpp
  src/ArLaserReflectorDevice.cpp
  src/ArLCDConnector.cpp
  src/ArLCDMTX.cpp
  src/ArLineFinder.cpp
  src/ArLMS1XX.cpp
  src/ArLMS2xx.cpp
  src/ArLMS2xxPacket.cpp
  src/ArLMS2xxPacketReceiver.cpp
  src/ArLog.cpp
  src/ArLogFileConnection.cpp
  src/ArMap.cpp
  src/ArMapComponents.cpp
  src/ArMapInterface.cpp
  src/ArMapObject.cpp
  src/ArMapUtils.cpp
  src/ArMD5Calculator.cpp
  src/ArMode.cpp
  src/ArModes.cpp
  src/ArModule.cpp
  src/ArModuleLoader.cpp
  src/ArMutex.cpp
  src/ArMutex_LIN.cpp
  src/ArNetServer.cpp
  src/ArNMEAParser.cpp
  src/ArNovatelGPS.cpp
  src/ArP2Arm.cpp
  src/ArPriorityResolver.cpp
  src/ArPTZ.cpp
  src/ArPTZConnector.cpp
  src/ArRangeBuffer.cpp
  src/ArRangeDevice.cpp
  src/ArRangeDeviceThreaded.cpp
  src/ArRatioInputKeydrive.cpp
  src/ArRatioInputJoydrive.cpp
  src/ArRatioInputRobotJoydrive.cpp
  src/ArRecurrentTask.cpp
  src/ArRobot.cpp
  src/ArRobotBatteryPacketReader.cpp
  src/ArRobotConfig.cpp
  src/ArRobotConfigPacketReader.cpp
  src/ArRobotConnector.cpp
  src/ArRobotJoyHandler.cpp
  src/ArRobotPacket.cpp
  src/ArRobotPacketReceiver.cpp
  src/ArRobotPacketReaderThread.cpp
  src/ArRobotPacketSender.cpp
  src/ArRobotParams.cpp
  src/ArRobotTypes.cpp
  src/ArRVisionPTZ.cpp
  src/ArS3Series.cpp
  src/ArSZSeries.cpp
  src/ArSick.cpp
  src/ArSimpleConnector.cpp
  src/ArSimulatedLaser.cpp
  src/ArSocket.cpp
  src/ArSonarConnector.cpp
  src/ArSonarDevice.cpp
  src/ArSonarMTX.cpp
  src/ArSensorReading.cpp
  src/ArSonyPTZ.cpp
  src/ArSoundsQueue.cpp
  src/ArSoundPlayer.cpp
  src/ArStringInfoGroup.cpp
  src/ArSyncLoop.cpp
  src/ArSyncTask.cpp
  src/ArSystemStatus.cpp
  src/ArSonarAutoDisabler.cpp
  src/ArSpeech.cpp
  src/ArTCM2.cpp
  src/ArTCMCompassDirect.cpp
  src/ArTCMCompassRobot.cpp
  src/ArTcpConnection.cpp
  src/ArThread.cpp
  src/ArThread_LIN.cpp
  src/ArTransform.cpp
  src/ArTrimbleGPS.cpp
  src/ArUrg.cpp
  src/ArUrg_2_0.cpp
  src/ArVCC4.cpp
  src/Aria.cpp
  src/ariaUtil.cpp
  src/md5.cpp
  src/ArSocket_LIN.cpp
  src/ArJoyHandler_LIN.cpp
  src/ArSerialConnection_LIN.cpp
  src/ArSignalHandler_LIN.cpp
  src/ArVersalogicIO.cpp
  src/ArMTXIO.cpp
)


set(AR_NETWORKING_SRC
  ArNetworking/src/ArCentralForwarder.cpp
  ArNetworking/src/ArCentralManager.cpp
  ArNetworking/src/ArClientArgUtils.cpp
  ArNetworking/src/ArClientBase.cpp
  ArNetworking/src/ArClientData.cpp
  ArNetworking/src/ArClientFileUtils.cpp
  ArNetworking/src/ArClientHandlerConfig.cpp
  ArNetworking/src/ArClientHandlerRobotUpdate.cpp
  ArNetworking/src/ArClientRatioDrive.cpp
  ArNetworking/src/ArClientSimpleConnector.cpp
  ArNetworking/src/ArClientSwitchManager.cpp
  ArNetworking/src/ArHybridForwarderVideo.cpp
  ArNetworking/src/ArMapChanger.cpp
  ArNetworking/src/ArNetPacket.cpp
  ArNetworking/src/ArNetPacketReceiverTcp.cpp
  ArNetworking/src/ArNetPacketReceiverUdp.cpp
  ArNetworking/src/ArNetPacketSenderTcp.cpp
  ArNetworking/src/ArServerBase.cpp
  ArNetworking/src/ArServerClient.cpp
  ArNetworking/src/ArServerData.cpp
  ArNetworking/src/ArServerFileUtils.cpp
  ArNetworking/src/ArServerHandlerCameraCollection.cpp
  ArNetworking/src/ArServerHandlerCamera.cpp
  ArNetworking/src/ArServerHandlerCommands.cpp
  ArNetworking/src/ArServerHandlerCommMonitor.cpp
  ArNetworking/src/ArServerHandlerConfig.cpp
  ArNetworking/src/ArServerHandlerMap.cpp
  ArNetworking/src/ArServerHandlerMapping.cpp
  ArNetworking/src/ArServerHandlerPopup.cpp
  ArNetworking/src/ArServerInfoDrawings.cpp
  ArNetworking/src/ArServerInfoRobot.cpp
  ArNetworking/src/ArServerInfoSensor.cpp
  ArNetworking/src/ArServerInfoStrings.cpp
  ArNetworking/src/ArServerMode.cpp
  ArNetworking/src/ArServerModeDrive.cpp
  ArNetworking/src/ArServerModeIdle.cpp
  ArNetworking/src/ArServerModeRatioDrive.cpp
  ArNetworking/src/ArServerModeStop.cpp
  ArNetworking/src/ArServerModeWander.cpp
  ArNetworking/src/ArServerSimpleCommands.cpp
  ArNetworking/src/ArServerSimpleOpener.cpp
  ArNetworking/src/ArServerUserInfo.cpp
  ArNetworking/src/ArTempDirectoryHelper.cpp
  ArNetworking/src/md5.cpp
)

add_library(Aria SHARED ${ARIA_SRC})
add_library(ArNetworking SHARED ${AR_NETWORKING_SRC})

add_executable(omron_robot_cli
  tools/omron_robot_cli.cpp
)

function(libaria_add_aria_program target source)
  add_executable(${target} ${source})
  target_include_directories(${target}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include/Aria
      ${CMAKE_CURRENT_SOURCE_DIR}/ArNetworking/include
  )
  target_link_libraries(${target}
    PRIVATE
      Aria
  )
  set_target_properties(${target} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )
endfunction()

target_compile_options(Aria PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-format -Wno-format-security>
)

target_compile_options(ArNetworking PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wno-format -Wno-format-security>
)

target_include_directories(Aria
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_include_directories(ArNetworking
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ArNetworking/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(Aria
  PRIVATE
    Threads::Threads
    ${CMAKE_DL_LIBS}
    rt
    m
)

target_link_libraries(ArNetworking
  PUBLIC
    Aria
)

target_link_libraries(omron_robot_cli
  PRIVATE
    ArNetworking
)

set_target_properties(omron_robot_cli PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)

set_target_properties(Aria ArNetworking PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)

set(LIBARIA_SAFE_TEST_SOURCES
  tests/actionArgumentTest.cpp
  tests/actionAverageTest.cpp
  tests/angleBetweenTest.cpp
  tests/angleFixTest.cpp
  tests/angleTest.cpp
  tests/lineTest.cpp
  tests/mathTests.cpp
  tests/robotListTest.cpp
)

set(LIBARIA_MANUAL_TEST_SOURCES
  tests/absoluteHeadingActionTest.cpp
)

set(LIBARIA_SAFE_EXAMPLE_SOURCES
  examples/functorExample.cpp
  examples/threadExample.cpp
)

set(LIBARIA_MANUAL_EXAMPLE_SOURCES
  examples/simpleConnect.cpp
  examples/simpleMotionCommands.cpp
  examples/teleopActionsExample.cpp
)

set(LIBARIA_SAFE_RUNNER_DEPENDENCIES)
set(LIBARIA_HARDWARE_RUNNER_DEPENDENCIES)

if(LIBARIA_BUILD_LEGACY_TEST_PROGRAMS)
  foreach(source IN LISTS LIBARIA_SAFE_TEST_SOURCES LIBARIA_MANUAL_TEST_SOURCES)
    get_filename_component(target ${source} NAME_WE)
    libaria_add_aria_program(${target} ${source})
  endforeach()

  foreach(source IN LISTS LIBARIA_SAFE_TEST_SOURCES)
    get_filename_component(target ${source} NAME_WE)
    list(APPEND LIBARIA_SAFE_RUNNER_DEPENDENCIES ${target})
  endforeach()

  foreach(source IN LISTS LIBARIA_MANUAL_TEST_SOURCES)
    get_filename_component(target ${source} NAME_WE)
    list(APPEND LIBARIA_HARDWARE_RUNNER_DEPENDENCIES ${target})
  endforeach()
endif()

if(LIBARIA_BUILD_LEGACY_EXAMPLE_PROGRAMS)
  foreach(source IN LISTS LIBARIA_SAFE_EXAMPLE_SOURCES LIBARIA_MANUAL_EXAMPLE_SOURCES)
    get_filename_component(target ${source} NAME_WE)
    libaria_add_aria_program(${target} ${source})
  endforeach()

  foreach(source IN LISTS LIBARIA_SAFE_EXAMPLE_SOURCES)
    get_filename_component(target ${source} NAME_WE)
    list(APPEND LIBARIA_SAFE_RUNNER_DEPENDENCIES ${target})
  endforeach()

  foreach(source IN LISTS LIBARIA_MANUAL_EXAMPLE_SOURCES)
    get_filename_component(target ${source} NAME_WE)
    list(APPEND LIBARIA_HARDWARE_RUNNER_DEPENDENCIES ${target})
  endforeach()
endif()

if(LIBARIA_BUILD_SAFE_TEST_RUNNER AND LIBARIA_SAFE_RUNNER_DEPENDENCIES)
  add_executable(libaria_safe_test_runner
    tests/libariaSafeTestRunner.cpp
  )
  target_link_libraries(libaria_safe_test_runner
    PRIVATE
      Aria
  )
  set_target_properties(libaria_safe_test_runner PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )
  add_dependencies(libaria_safe_test_runner ${LIBARIA_SAFE_RUNNER_DEPENDENCIES})

  add_custom_target(run_libaria_safe_tests
    COMMAND libaria_safe_test_runner
    DEPENDS libaria_safe_test_runner
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )

  if(BUILD_TESTING)
    add_test(
      NAME libaria_safe_test_runner
      COMMAND libaria_safe_test_runner
    )
    set_tests_properties(libaria_safe_test_runner PROPERTIES
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
  endif()
endif()

if(LIBARIA_BUILD_HARDWARE_TEST_RUNNER AND LIBARIA_HARDWARE_RUNNER_DEPENDENCIES)
  add_executable(libaria_hardware_test_runner
    tests/libariaHardwareTestRunner.cpp
  )
  target_link_libraries(libaria_hardware_test_runner
    PRIVATE
      Aria
  )
  set_target_properties(libaria_hardware_test_runner PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )
  add_dependencies(libaria_hardware_test_runner ${LIBARIA_HARDWARE_RUNNER_DEPENDENCIES})

  add_custom_target(run_libaria_hardware_tests
    COMMAND libaria_hardware_test_runner
    DEPENDS libaria_hardware_test_runner
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )

  if(BUILD_TESTING AND LIBARIA_REGISTER_HARDWARE_CTEST)
    add_test(
      NAME libaria_hardware_test_runner
      COMMAND libaria_hardware_test_runner
    )
    set_tests_properties(libaria_hardware_test_runner PROPERTIES
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      LABELS "hardware"
    )
  endif()
endif()

#############
## Install ##
#############

install(TARGETS Aria ArNetworking omron_robot_cli
  EXPORT libariaTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/Aria/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Aria
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY ArNetworking/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ArNetworking
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/libariaConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/libariaConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/libariaConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libaria
)

install(EXPORT libariaTargets
  FILE libariaTargets.cmake
  NAMESPACE libaria::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libaria
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/libariaConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/libariaConfigVersion.cmake"
    "${CMAKE_CURRENT_SOURCE_DIR}/package.xml"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libaria
)