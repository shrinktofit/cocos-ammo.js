#!/usr/bin/env bash

echo -e "\033[01;32m --------------- START -------------------- \033[0m"
now=`date +'%Y-%m-%d %H:%M:%S'`
start_time=$(date --date="$now" +%s)

if [ "$1" = "cleanup" ]; then
rm -rf BUILD_WASM
echo "WASM build directory removed"
mkdir BUILD_WASM
echo "WASM build directory created"
fi

cd BUILD_WASM

# make for debug/release and wasm/asm respectively
#check if imput parameter is debug
if [ "$2" = "debug" ]; then
    echo -e "\033[01;32m --------------- Build Debug --------------- \033[0m"
    cmake .. -G "Unix Makefiles" -B ./ -DBUILD_WASM=ON -DCMAKE_BUILD_TYPE=debug -DCMAKE_TOOLCHAIN_FILE=${EMSCRIPTEN}/cmake/Modules/Platform/Emscripten.cmake
    make -j 12
    echo -e "\033[01;32m ---------- Build wasm debug DONE ----------  \033[0m"
    
    cmake .. -G "Unix Makefiles" -B ./ -DBUILD_WASM=OFF -DCMAKE_BUILD_TYPE=debug -DCMAKE_TOOLCHAIN_FILE=${EMSCRIPTEN}/cmake/Modules/Platform/Emscripten.cmake
    make -j 12
    echo -e "\033[01;32m ---------- Build asm debug DONE ----------  \033[0m"

    cp -r ../../bullet-release-embind/debug/bullet.debug.asm.js ../../../../../cocos-engine/native/external/emscripten/bullet
    cp -r ../../bullet-release-embind/debug/bullet.debug.wasm.js ../../../../../cocos-engine/native/external/emscripten/bullet
    cp -r ../../bullet-release-embind/debug/bullet.debug.wasm.wasm ../../../../../cocos-engine/native/external/emscripten/bullet

    echo -e "\033[01;32m ------------ Copy Done --------------  \033[0m"
else
    echo -e "\033[01;32m --------------- Build Release --------------- \033[0m"
    cmake .. -G "Unix Makefiles" -B ./ -DBUILD_WASM=ON -DCMAKE_BUILD_TYPE=release -DCMAKE_TOOLCHAIN_FILE=${EMSCRIPTEN}/cmake/Modules/Platform/Emscripten.cmake
    make -j 12
    
    echo -e "\033[01;32m ---------- Build wasm release DONE ----------  \033[0m"
    cmake .. -G "Unix Makefiles" -B ./ -DBUILD_WASM=OFF -DCMAKE_BUILD_TYPE=release -DCMAKE_TOOLCHAIN_FILE=${EMSCRIPTEN}/cmake/Modules/Platform/Emscripten.cmake
    make -j 12
    echo -e "\033[01;32m ---------- Build  asm release DONE ----------  \033[0m"

    cp -r ../../bullet-release-embind/release/bullet.release.asm.js ../../../../../cocos-engine/native/external/emscripten/bullet
    cp -r ../../bullet-release-embind/release/bullet.release.wasm.js ../../../../../cocos-engine/native/external/emscripten/bullet
    cp -r ../../bullet-release-embind/release/bullet.release.wasm.wasm ../../../../../cocos-engine/native/external/emscripten/bullet
    
    echo -e "\033[01;32m ------------ Copy Done --------------  \033[0m"
fi


now=`date +'%Y-%m-%d %H:%M:%S'`
end_time=$(date --date="$now" +%s);
echo -e "\033[01;32m Time Used: "$((end_time-start_time))"s  \033[1m"
echo -e "\033[01;32m ------------- END -----------------  \033[0m"