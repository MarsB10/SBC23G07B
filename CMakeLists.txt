# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS ./components/ssd1306)
set(EXTRA_COMPONENT_DIRS ./components/thingsboard_connection)
set(EXTRA_COMPONENT_DIRS C:/Espressif/frameworks/esp-idf-v4.4.1/examples/common_components/protocol_examples_common)


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(HWyTB1_proyecto)