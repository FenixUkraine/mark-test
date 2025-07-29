# CMakeLists.txt для проекта CC113L Receiver
cmake_minimum_required(VERSION 3.5)

# Устанавливаем имя проекта
set(PROJECT_NAME "cc113l_receiver")

# Включаем ESP-IDF
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Создаем проект
project(${PROJECT_NAME})

# Отключаем строгие предупреждения для совместимости
target_compile_options(${PROJECT_NAME}.elf PRIVATE
    -Wno-format
    -Wno-unused-variable
    -Wno-unused-function
)