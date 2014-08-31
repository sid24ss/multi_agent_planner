cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Coverage)

rosbuild_init()
SET(CMAKE_CXX_FLAGS "-std=c++0x")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(multi_agent_planner 
											# src/StateReps/RobotState.cpp
                                            # src/StateReps/ContArmState.cpp
                                            # src/StateReps/ContBaseState.cpp
                                            # src/StateReps/DiscBaseState.cpp
                                            # src/StateReps/DiscObjectState.cpp
                                            # src/StateReps/ContObjectState.cpp
                                            # src/StateReps/GraphState.cpp
                                            # src/StateReps/GoalState.cpp
                                            
                                            # src/MotionPrimitives/ArmMotionPrimitive.cpp
                                            # src/MotionPrimitives/BaseMotionPrimitive.cpp
                                            # src/MotionPrimitives/MotionPrimitive.cpp
                                            # src/MotionPrimitives/FileParser.cpp
                                            # src/MotionPrimitives/MotionPrimitivesMgr.cpp
                                            # src/MotionPrimitives/BaseAdaptiveMotionPrimitive.cpp
                                            # src/MotionPrimitives/ArmAdaptiveMotionPrimitive.cpp
                                            # src/MotionPrimitives/ArmTuckMotionPrimitive.cpp
                                            # src/MotionPrimitives/ArmUntuckMotionPrimitive.cpp
                                            # src/MotionPrimitives/TorsoMotionPrimitive.cpp

                                            # src/Heuristics/BFS3DHeuristic.cpp
                                            # src/Heuristics/BFS2DHeuristic.cpp
                                            # src/Heuristics/BFS2DRotFootprintHeuristic.cpp
                                            # src/Heuristics/EndEffectorHeuristic.cpp
                                            # src/Heuristics/EndEffLocalHeuristic.cpp
                                            # src/Heuristics/BaseWithRotationHeuristic.cpp
                                            # src/Heuristics/AbstractHeuristic.cpp
                                            # src/Heuristics/ArmAnglesHeuristic.cpp
                                            # src/Heuristics/BFS3DWithRotationHeuristic.cpp
                                            # src/Heuristics/EndEffOnlyRotationHeuristic.cpp
                                            # src/Heuristics/HeuristicMgr.cpp
                                            # src/Heuristics/2Dgridsearch.cpp

                                            # src/SearchRequest.cpp
                                            # src/Environment.cpp
                                            src/OccupancyGridUser.cpp
                                            # src/CollisionSpaceMgr.cpp
                                            src/ParameterCatalog.cpp
                                            # src/HashManager.cpp
                                            # src/TransitionData.cpp
                                            # src/Visualizer.cpp
                                            # src/PathPostProcessor.cpp

                                            # src/ExperimentFramework/randomStartGoalGenerator.cpp
                                            # src/StatsWriter.cpp
                                            )
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
