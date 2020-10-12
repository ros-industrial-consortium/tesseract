# Tesseract

[![codecov](https://codecov.io/gh/ros-industrial-consortium/tesseract/branch/master/graph/badge.svg)](https://codecov.io/gh/ros-industrial-consortium/tesseract)

Platform             | CI Status
---------------------|:---------
Linux (Focal)        | [![Build Status](https://github.com/ros-industrial-consortium/tesseract/workflows/Focal-Build/badge.svg)](https://github.com/ros-industrial-consortium/tesseract/actions)
Linux (Bionic)       | [![Build Status](https://github.com/ros-industrial-consortium/tesseract/workflows/Bionic-Build/badge.svg)](https://github.com/ros-industrial-consortium/tesseract/actions)
Linux (Xenial)       | [![Build Status](https://github.com/ros-industrial-consortium/tesseract/workflows/Xenial-Build/badge.svg)](https://github.com/ros-industrial-consortium/tesseract/actions)
Windows              | [![Build Status](https://github.com/ros-industrial-consortium/tesseract/workflows/Windows-Noetic-Build/badge.svg)](https://github.com/ros-industrial-consortium/tesseract/actions)
Lint  (Clang-Format) | [![Build Status](https://github.com/ros-industrial-consortium/tesseract/workflows/Clang-Format/badge.svg)](https://github.com/ros-industrial-consortium/tesseract/actions)
Lint  (CodeCov)      | [![Build Status](https://github.com/ros-industrial-consortium/tesseract/workflows/CodeCov/badge.svg)](https://github.com/ros-industrial-consortium/tesseract/actions)

[![Github Issues](https://img.shields.io/github/issues/ros-industrial-consortium/tesseract.svg)](http://github.com/ros-industrial-consortium/tesseract/issues)

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![license - bsd 2 clause](https://img.shields.io/:license-BSD%202--Clause-blue.svg)](https://opensource.org/licenses/BSD-2-Clause)

[![support level: consortium](https://img.shields.io/badge/support%20level-consortium-brightgreen.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

The planning framework (Tesseract) was designed to be light weight, limiting the number of dependencies, mainly only using standard libraries like, eigen, boost, orocos and to the packages below. The core packages are ROS agnostic and have full python support.

## Tesseract Setup Wizard and Visualization Tools

[![](https://github.com/snapcore/snap-store-badges/blob/master/EN/%5BEN%5D-snap-store-black-uneditable%401x.png)](https://snapcraft.io/tesseract-ignition)

## Tesseract Core Packages

* **tesseract** – This is the main class that manages the major component Environment, Forward Kinematics, Inverse Kinematics and loading from various data.
* **tesseract_collision** – This package contains a common interface for collision checking prividing several implementation of a Bullet collision library and FCL collision library. It includes both continuous and discrete collision checking for convex-convex, convex-concave and concave-concave shapes.
* **tesseract_common** – This package contains common functionality needed by the majority of the packages.
* **tesseract_environment** – This package contains the Tesseract Environment which provides functionality to add,remove,move and modify links and joint. It also manages adding object to the contact managers and provides the ability.
* **tesseract_geometry** – This package contains geometry types used by Tesseract including primitive shapes, mesh, convex hull mesh, octomap and signed distance field.
* **tesseract_kinematics** –  This package contains a common interface for Forward and Inverse kinematics for Chains, Trees and Graphs including implementation using KDL and OPW Kinematics.
* **tesseract_motion_planners** – This package contains a common interface for Motion Planners and includes implementation for OMPL, TrajOpt and Descartes.
* **tesseract_scene_graph** – This package contains the scene graph which is the data structure used to manage the connectivity of objects in the environment. It inherits from boost graph and provides addition functionality for adding,removing and modifying Links and Joints along with search implementation.
* **tesseract_support** – This package contains support data used for unit tests and examples throughout Tesseract.
* **tesseract_visualization** – This package contains visualization utilities and libraries.

## Documentation

* [Wiki](https://ros-industrial-tesseract.rtfd.io)
* [Doxygen](https://ros-industrial-consortium.github.io/tesseract/)
* [Benchmark](https://ros-industrial-consortium.github.io/tesseract/dev/bench)

## TODO's

Warning: These packages are under heavy development and are subject to change.

See [issue #66](https://github.com/ros-industrial-consortium/tesseract/issues/66)

## Build Instructions

1) Clone repository into your workspace
2) Clone the repositories in the dependencies.rosinstall file using wstool or some other method (e.g. manually git cloning them)
3) Build the workspace using catkin tools, colcon, or a similar tool

NOTE: To speed up clean build you may want to add tesseract_ext to an extended workspace.

NOTE: Install TaskFlow from [ROS-Industrial PPA](https://launchpad.net/~ros-industrial/+archive/ubuntu/ppa).

### Building with Clang-Tidy Enabled

Must pass the -DTESSERACT_ENABLE_CLANG_TIDY=ON to cmake when building. This is automatically enabled if cmake argument -DTESSERACT_ENABLE_TESTING_ALL=ON is passed.

### Building Tesseract Tests

Must pass the -DTESSERACT_ENABLE_TESTING=ON to cmake when wanting to build tests. This is automatically enabled if cmake argument -DTESSERACT_ENABLE_TESTING_ALL=ON is passed.

NOTE: If you are building using catkin tools, use `catkin build --force-cmake -DTESSERACT_ENABLE_TESTING=ON`.

#### Running Tesseract Tests

Tesseract packages use ctest because it is ROS agnostic, so to run the test call `catkin test --no-deps tesseract_collision tesseract_common tesseract_environment tesseract_geometry tesseract_kinematics tesseract_motion_planners tesseract_process_planners tesseract_scene_graph tesseract_urdf`

### Building Tesseract Code Coverage

Must pass the -DTESSERACT_CODE_COVERAGE=ON to cmake when wanting to build code coverage. The code coverage report is located in each individuals build directory inside a ccov/all-merged folder. Open the index.html file to see the packages code coverage report.

NOTE: Must be a clean build when generating a code coverage report. Also must build in debug.

#### Exclude Code From Code Coverage

- LCOV_EXCL_LINE
  - Lines containing this marker will be excluded.
- LCOV_EXCL_START
  - Marks the beginning of an excluded section. The current line is part of this section.
- LCOV_EXCL_STOP
  - Marks the end of an excluded section. The current line not part of this section.

.. NOTE: You can replace **LCOV** above with **GCOV** or **GCOVR**.

## Quality Tools Leverage

Tesseract currently leverages Compiler Warnigs, Clang Tidy and Code Coverage. All warnings produced by Compiler and Clang Tidy are treated as errors during CI builds.

- Compiler
  - Wall
  - Wextra
  - Wconversion
  - Wsign-conversion
  - Wno-sign-compare
- Clang Tidy
  - clang-analyzer-*
  - bugprone-*
  - cppcoreguidelines-avoid-goto
  - cppcoreguidelines-c-copy-assignment-signature
  - cppcoreguidelines-interfaces-global-init
  - cppcoreguidelines-narrowing-conversions
  - cppcoreguidelines-no-malloc
  - cppcoreguidelines-slicing
  - cppcoreguidelines-special-member-functions
  - misc-*
  - modernize-*
  - performance-*
  - readability-avoid-const-params-in-decls
  - readability-container-size-empty
  - readability-delete-null-pointer
  - readability-deleted-default
  - readability-else-after-return
  - readability-function-size
  - readability-identifier-naming
  - readability-inconsistent-declaration-parameter-name
  - readability-misleading-indentation
  - readability-misplaced-array-index
  - readability-non-const-parameter
  - readability-redundant-*
  - readability-simplify-*
  - readability-static-*
  - readability-string-compare
  - readability-uniqueptr-delete-release
  - readability-rary-objects

## Build Branch Sphinx Documentation

```
cd gh_pages
sphinx-build . output
```
Now open gh_pages/output/index.rst and remove *output* directory before commiting changes.

## Debugging Windows Build

- Search Directories CI
  - dir /s /b c:\opt\ros\noetic\*assimp*
- Location of ROS Windows Builds
  - https://ros-win.visualstudio.com/ros-win/_build
  - https://ros-win.visualstudio.com/ros-win/_build/results?buildId=8711&view=artifacts&type=publishedArtifacts
