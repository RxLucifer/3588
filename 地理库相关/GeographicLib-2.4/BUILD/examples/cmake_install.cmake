# Install script for directory: /home/ztl/GeographicLib-2.4/examples

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/doc/GeographicLib-dev" TYPE FILE FILES
    "/home/ztl/GeographicLib-2.4/examples/CMakeLists.txt"
    "/home/ztl/GeographicLib-2.4/examples/example-Accumulator.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-AlbersEqualArea.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-AuxAngle.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-AuxLatitude.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-AzimuthalEquidistant.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-CassiniSoldner.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-CircularEngine.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-Constants.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-DMS.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-DST.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-Ellipsoid.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-EllipticFunction.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-GARS.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-GeoCoords.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-Geocentric.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-Geodesic.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-Geodesic-small.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-GeodesicExact.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-GeodesicLine.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-GeodesicLineExact.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-GeographicErr.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-Geohash.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-Geoid.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-Georef.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-Gnomonic.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-GravityCircle.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-GravityModel.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-Intersect.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-LambertConformalConic.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-LocalCartesian.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-MGRS.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-MagneticCircle.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-MagneticModel.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-Math.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-NearestNeighbor.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-NormalGravity.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-OSGB.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-PolarStereographic.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-PolygonArea.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-Rhumb.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-RhumbLine.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-SphericalEngine.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-SphericalHarmonic.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-SphericalHarmonic1.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-SphericalHarmonic2.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-TransverseMercator.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-TransverseMercatorExact.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-UTMUPS.cpp"
    "/home/ztl/GeographicLib-2.4/examples/example-Utility.cpp"
    "/home/ztl/GeographicLib-2.4/examples/GeoidToGTX.cpp"
    "/home/ztl/GeographicLib-2.4/examples/make-egmcof.cpp"
    )
endif()

