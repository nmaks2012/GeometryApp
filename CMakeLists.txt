cmake_minimum_required(VERSION 3.30)
project(GeometryApp VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Все предупреждения - это ошибки
    add_compile_options(-Wfatal-errors -Wall -Werror -Wno-deprecated-declarations)

    # Не больше одной ошибки за раз
    add_compile_options(-fmax-errors=1)
endif()

# Ищем необходимые библиотеки
find_package(GTest REQUIRED)

include(FetchContent)
FetchContent_Declare(
  CPM
  GIT_REPOSITORY https://github.com/cpm-cmake/CPM.cmake
  GIT_TAG origin/master
)
FetchContent_MakeAvailable(CPM)

CPMAddPackage(
    NAME matplotplusplus
    GITHUB_REPOSITORY alandefreitas/matplotplusplus
    GIT_TAG master # or use a tag like v1.1.0
)

file(GLOB HEADER_FILES "${CMAKE_SOURCE_DIR}/include/*.hpp")
file(GLOB SRC_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp")
list(REMOVE_ITEM SRC_FILES "${CMAKE_SOURCE_DIR}/src/main.cpp")

# Создаём статическую библиотеку
add_library(${PROJECT_NAME}_imp STATIC "${CMAKE_SOURCE_DIR}/src/main.cpp" ${SRC_FILES} ${HEADER_FILES})

# Добавляем в проект используемые сторонние библиотеки
target_include_directories(${PROJECT_NAME}_imp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OPENSSL_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${matplotplusplus_SOURCE_DIR}/source
)
target_link_libraries(${PROJECT_NAME}_imp PRIVATE ${OPENSSL_LIBRARIES} ${Boost_LIBRARIES} matplot)

# Создаём исполняемый таргет и линкуем к нему статическую библиотеку
add_executable(${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/src/main.cpp")
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_imp)

#
# Тесты
#

# Рекурсивно ищем все используемые в тестах .cpp файлы
file(GLOB TEST_SRC_FILES "${CMAKE_SOURCE_DIR}/tests/*.cpp")

add_executable(${PROJECT_NAME}_tests "${TEST_SRC_FILES}")
target_link_libraries(${PROJECT_NAME}_tests PRIVATE ${PROJECT_NAME}_imp GTest::GTest GTest::Main)
target_include_directories(${PROJECT_NAME}_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Включаем тестирование
enable_testing()
add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME}_tests)

# Автоматически находим и добавляем тесты в CTest
include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME}_tests)