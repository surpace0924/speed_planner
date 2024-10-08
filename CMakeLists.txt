cmake_minimum_required(VERSION 3.10)

# プロジェクト名
project(speed_planner)

# C++20
set(CMAKE_CXX_STANDARD 20)

# Eigenライブラリのパスを指定
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Pythonのパッケージを探す
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Python3 REQUIRED COMPONENTS NumPy)

# Pythonのインクルードディレクトリを追加
include_directories(${Python3_INCLUDE_DIRS})
include_directories(${Python3_NumPy_INCLUDE_DIRS})

# 実行ファイルの出力先を設定
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

# 実行ファイルを生成
add_executable(main_app src/main.cpp)

# Eigenライブラリをリンク
target_link_libraries(main_app Eigen3::Eigen ${Python3_LIBRARIES})
