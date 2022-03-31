foreach(LIB_PATH ${LIB_PATHS})
    file(GLOB LIB_CPP ${LIB_PATH}/*)

    get_filename_component(LIB_NAME ${LIB_PATH} NAME)

    add_library(${LIB_NAME}_lib ${LIB_CPP})

    add_dependencies(${LIB_NAME}_lib
    #     billiards_game_generate_messages_cpp
    #     yolo_server_ros_generate_messages_cpp
    #     hiwin_ros_generate_messages_cpp
    #     zed_ros_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
    )
endforeach()
