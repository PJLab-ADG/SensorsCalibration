#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
source $SCRIPT_DIR/log.sh

build_path=$(pwd)
cd $build_path



function is_folder_exist(){
    # 使用 [ 命令判断文件夹是否存在
    if [ -d "$build_path/build" ]; then
        log_info "Enter the \033[34m[build]\033[32m folder"
        cd build
    else
        log_info "Create a \033[34m[build]\033[32m folder in $build_path"
        mkdir build
        cd build
    fi
}

function is_cmake_build(){
    # 提示用户输入 Y 或 n
    read -p "Do you want to cmake ..? (Y/n)" answer

    # 判断用户输入的值是否为 Y 或 y
    if [[ "${answer}" == "Y" || "${answer}" == "y" ]]; then
        log_info "Start executing \033[34m[cmake ..]\033[32m instruction"
        cmake .. 
        log_info "Start executing \033[34m[make .]\033[32m instruction"
        make  
    else
        log_info "Start executing \033[34m[make .]\033[32m instruction"
        make  
    fi

}


function main(){
    if [ -e "$build_path/CMakeLists.txt" ]; then
        log_info "\033[34m[$build_path]\033[32m"
    else
        log_error "\033[34m[$build_path]\033[32m/CMakeLists.txt 文件不存在"
        exit
    fi

    is_folder_exist
    is_cmake_build
}
main