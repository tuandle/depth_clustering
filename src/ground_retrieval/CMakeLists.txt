add_library(ground_retrieve SHARED depth_ground_retriever.cpp)
target_link_libraries(ground_retrieve
                      identifiable
                      velodyne_utils
                      difference
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})