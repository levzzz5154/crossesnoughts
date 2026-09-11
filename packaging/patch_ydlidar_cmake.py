"""Idempotently make the pinned YDLidar SDK compatible with modern CMake."""
from pathlib import Path
import re

root = Path(__file__).resolve().parent.parent / "YDLidar-SDK"
cmake = root / "CMakeLists.txt"
install = root / "cmake" / "install_package.cmake"

text = cmake.read_text(encoding="utf-8")
text = re.sub(
    r"\n\s*if\(POLICY CMP(?:0053|0037|0043)\).*?\n\s*endif\(\)",
    "", text, flags=re.DOTALL)
text = text.replace(
    "target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)",
    "target_include_directories(${PROJECT_NAME} PUBLIC\n"
    "    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>\n"
    "    $<INSTALL_INTERFACE:include>)")
cmake.write_text(text, encoding="utf-8")

text = install.read_text(encoding="utf-8")
text = re.sub(
    r"\s*if\(POLICY CMP0026\).*?\n\s*endif\(\)",
    "", text, flags=re.DOTALL)
text = re.sub(
    r"get_target_property\( _target_library \$\{PACKAGE_LIB_NAME\} LOCATION \)",
    "set( _target_library ${PACKAGE_LIB_NAME} )", text)
text = text.replace(
    "install( FILES ${_target_library} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )",
    "install( TARGETS ${PACKAGE_LIB_NAME}\n"
    "                ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib\n"
    "                LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib\n"
    "                RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin )")
install.write_text(text, encoding="utf-8")

if "CMP0026 OLD" in text or "get_target_property( _target_library" in text:
    raise SystemExit("Failed to patch the pinned YDLidar SDK CMake files")
