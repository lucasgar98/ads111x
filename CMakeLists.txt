# include support for eil-cmake-utils
list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_LIST_DIR}/eil-cmake-utils/cmake
)

idf_component_register(
    SRCS ads111x.c
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers
)

include(eil_ci)

# include common cmake file for components
#set(ESP_IDF_LIB_CMAKE ${CMAKE_CURRENT_LIST_DIR}/common/cmake/esp-idf-lib.cmake)
#if(EXISTS ${ESP_IDF_LIB_CMAKE})
#    include(${ESP_IDF_LIB_CMAKE})
#else()
#    message(WARNING "${ESP_IDF_LIB_CMAKE} not found")
#endif()
