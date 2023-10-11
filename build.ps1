param (
    # Build mode
    [Parameter()]
    [ValidateSet('Debug', 'Release', 'Checked', 'Profile')]
    [string]
    $Mode = "Release",

    # Only
    [Parameter()]
    [ValidateSet('wasm', 'webassembly', 'asm.js', 'asm')]
    [string]
    $Only,

    # Engine path
    [Parameter()]
    [string]
    $EnginePath = "../../cocos-engine"
)

$includingWebAssembly = (-not $Only) -or ($Only.ToLower() -in "wasm", "webassembly")
$includingAsmJs = (-not $Only) -or ($Only.ToLower() -in "asm.js", "asm")

Write-Host "Start build"
Write-Host "Including WebAssembly: $includingWebAssembly"
Write-Host "Including asm.js: $includingAsmJs"

if (-not $env:EMSCRIPTEN) {
    if ($env:EMSDK) {
        $env:EMSCRIPTEN = "$env:EMSDK/upstream/emscripten"
    }
}
if (-not $env:EMSCRIPTEN) {
    Write-Error "Environment variable 'EMSCRIPTEN' is not set. " +
            "Either set it to <emsdk>/upstream/emscripten, or set environment variable 'EMSDK' to location of <emsdk>."
    exit -1
}

$buildMode = $Mode.ToLower()

if (Test-Path $EnginePath) {
    $engineDestDir = "$EnginePath/native/external/emscripten/bullet"
    New-Item -ItemType Directory -Force $engineDestDir
} else {
    $engineDestDir = $null
}

function build {
    param (
        [bool]
        $wasm
    )

    try {
        Push-Location
    
        Set-Location $PSScriptRoot/bullet
        
        Write-Host "Generating..."
        emcmake cmake `
            -S . `
            -B ./build `
            -G Ninja `
            -DBUILD_WASM="$(if ($wasm -eq $true) {'ON'} else {'OFF'})" `
            -DCMAKE_BUILD_TYPE="$buildMode" `
            -DCMAKE_TOOLCHAIN_FILE="$($env:EMSCRIPTEN)/cmake/Modules/Platform/Emscripten.cmake"

        Write-Host "Building..."
        & ninja -C ./build

        if ($engineDestDir) {
            function Output {
                param ([string]$filename)
                Copy-Item -Force "$PSScriptRoot/bullet-release-embind/$mode/$filename" "$engineDestDir/$filename" | Out-Null
            }

            if ($wasm) {
                Output -filename bullet.$buildMode.wasm.js
                Output -filename bullet.$buildMode.wasm.wasm
            } else {
                Output -filename bullet.$buildMode.asm.js
            }
        }
    } finally {
        Pop-Location
    }
}

if ($includingWebAssembly) {
    Write-Host "Start build WebAssembly"
    build -wasm $true
}

if ($includingAsmJs) {
    Write-Host "Start build asm.js"
    build -wasm $false
}