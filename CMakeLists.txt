cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Use the custom partition table
set(PARTITION_TABLE_CSV_PATH "${CMAKE_SOURCE_DIR}/partitions.csv")

project(radar_wifi_csi)
