$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

Write-Host "Initializing YDLidar SDK submodule..."
git submodule update --init --recursive

$Python = (py -3 -c "import sys; print(sys.executable)").Trim()
if (-not $Python) { throw "Python 3 was not found. Install 64-bit Python 3.11 or 3.12." }

Write-Host "Installing Python build dependencies..."
& $Python -m pip install -r requirements-build.txt

if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
    throw "CMake is required. Install it and enable 'Add CMake to PATH'."
}
if (-not (Get-Command swig -ErrorAction SilentlyContinue)) {
    throw "SWIG is required. Install it and add it to PATH."
}

Write-Host "Configuring the 64-bit Windows YDLidar SDK..."
cmake -S YDLidar-SDK -B YDLidar-SDK/build-win -A x64 `
    -DBUILD_EXAMPLES=OFF `
    -DBUILD_TEST=OFF `
    -DPYTHON_EXECUTABLE="$Python"

Write-Host "Building the SDK and Python extension..."
cmake --build YDLidar-SDK/build-win --config Release

# The SDK's output layout varies between CMake/Visual Studio versions. Copy
# its Python wrapper and native output into a location recognized by run.py.
$Destination = Join-Path $PSScriptRoot "YDLidar-SDK/build/python/Release"
New-Item -ItemType Directory -Force -Path $Destination | Out-Null
$Wrapper = Get-ChildItem YDLidar-SDK -Recurse -Filter ydlidar.py |
    Where-Object { $_.FullName -notmatch "examples" } | Select-Object -First 1
$Extension = Get-ChildItem YDLidar-SDK/build-win -Recurse -Filter "_ydlidar*.pyd" |
    Select-Object -First 1
if (-not $Wrapper -or -not $Extension) {
    throw "The SDK built, but ydlidar.py or _ydlidar.pyd was not found."
}
Copy-Item $Wrapper.FullName $Destination -Force
Copy-Item $Extension.FullName $Destination -Force
Get-ChildItem YDLidar-SDK/build-win -Recurse -Filter "*.dll" |
    Copy-Item -Destination $Destination -Force

Write-Host "Building portable Windows ZIP..."
& $Python packaging/build_portable.py --clean --with-lidar
