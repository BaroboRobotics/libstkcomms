# TODO factor this out into a FindWindowsSDK.cmake module, allow specification
# of required version.

# This CMake script is meant to be included with the include() command. It
# queries the Windows registry for the location of the latest version of
# the Windows SDK, and communicates this to the calling script via two output
# variables.
#
# Output variables:
#   WINSDK_VERSION        latest version of Windows SDK
#   WINSDK_INSTALL_DIR    absolute path to Windows SDK 

# All the versions of the Windows SDK that we're aware of. The versions
# roughly correspond to the latest version of Windows they target (where 6 is
# Vista). The [aA] suffix means it came bundled with Visual Studio. The ones
# marked tested are known to work with this script.
set(versions
  v8.1A
  v8.1    # (tested)
  v8.0A
  v8.0
  v7.1A
  v7.1    # (tested)
  v7.0a
  v7.0
  v6.1
  v6.0a
  v6.0
  # Before 6.0, there was the Platform SDK. This is too ancient to care about.
  )

if(NOT WINSDK_VERSION)
  unset(WINSDK_VERSION)
  foreach(ver ${versions})
    # Check the 64-bit location.
    get_filename_component(WINSDK_INSTALL_DIR "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Wow6432Node\\Microsoft\\Microsoft SDKs\\Windows\\${ver};InstallationFolder]" ABSOLUTE)
    if(EXISTS ${WINSDK_INSTALL_DIR} AND IS_DIRECTORY ${WINSDK_INSTALL_DIR})
      set(WINSDK_VERSION ${ver})
      break()
    endif()

    # Check the 32-bit location.
    get_filename_component(WINSDK_INSTALL_DIR "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Microsoft SDKs\\Windows\\${ver};InstallationFolder]" ABSOLUTE)
    if(EXISTS ${WINSDK_INSTALL_DIR} AND IS_DIRECTORY ${WINSDK_INSTALL_DIR})
      set(WINSDK_VERSION ${ver})
      break()
    endif()
  endforeach()

  if(WINSDK_VERSION)
    message(STATUS "Found Windows SDK ${WINSDK_VERSION} in ${WINSDK_INSTALL_DIR}")
  else()
    message(STATUS "Failed to find the Windows SDK")
  endif()
endif()
