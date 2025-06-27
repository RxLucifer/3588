# Install script for directory: /home/ztl/GeographicLib-2.4/include/GeographicLib

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/GeographicLib" TYPE FILE FILES
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/Accumulator.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/AlbersEqualArea.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/AuxAngle.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/AuxLatitude.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/AzimuthalEquidistant.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/CassiniSoldner.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/CircularEngine.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/Constants.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/DAuxLatitude.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/DMS.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/DST.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/Ellipsoid.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/EllipticFunction.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/GARS.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/GeoCoords.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/Geocentric.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/Geodesic.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/GeodesicExact.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/GeodesicLine.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/GeodesicLineExact.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/Geohash.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/Geoid.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/Georef.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/Gnomonic.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/GravityCircle.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/GravityModel.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/Intersect.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/LambertConformalConic.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/LocalCartesian.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/MGRS.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/MagneticCircle.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/MagneticModel.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/Math.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/NearestNeighbor.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/NormalGravity.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/OSGB.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/PolarStereographic.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/PolygonArea.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/Rhumb.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/SphericalEngine.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/SphericalHarmonic.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/SphericalHarmonic1.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/SphericalHarmonic2.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/TransverseMercator.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/TransverseMercatorExact.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/UTMUPS.hpp"
    "/home/ztl/GeographicLib-2.4/include/GeographicLib/Utility.hpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/GeographicLib" TYPE FILE FILES "/home/ztl/GeographicLib-2.4/BUILD/include/GeographicLib/Config.h")
endif()

