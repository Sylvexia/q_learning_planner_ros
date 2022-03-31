foreach(DEPS_PATH ${DEPS_PATH})
    add_subdirectory(${DEPS_PATH})
endforeach()