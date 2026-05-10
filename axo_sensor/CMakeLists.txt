# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# set(SDKCONFIG_DEFAULTS "../sdkconfig.defaults")
set(SDKCONFIG_DEFAULTS "../sdkconfig.defaults;../sdkconfig.local")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(axo_sensor)
