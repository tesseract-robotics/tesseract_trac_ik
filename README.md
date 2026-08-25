# Tesseract Trac-IK plugin

Platform             | CI Status
---------------------|:---------
Linux                | [![Build Status](https://github.com/tesseract-robotics/tesseract_trac_ik/actions/workflows/ubuntu.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_trac_ik/actions/workflows/ubuntu.yml)
Lint  (Clang-Format) | [![Build Status](https://github.com/tesseract-robotics/tesseract_trac_ik/actions/workflows/clang_format.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_trac_ik/actions/workflows/clang_format.yml)
Lint  (CMake-Format) | [![Build Status](https://github.com/tesseract-robotics/tesseract_trac_ik/actions/workflows/cmake_format.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_trac_ik/actions/workflows/cmake_format.yml)
Lint  (Clang-Tidy)   | [![Build Status](https://github.com/tesseract-robotics/tesseract_trac_ik/actions/workflows/code_quality.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_trac_ik/actions/workflows/code_quality.yml)
Lint  (CodeCov)      | [![Build Status](https://github.com/tesseract-robotics/tesseract_trac_ik/actions/workflows/code_quality.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_trac_ik/actions/workflows/code_quality.yml)

This is a [Trac-IK](https://traclabs.com/projects/trac-ik/) plugin for `tesseract_kinematics`.

This plugin is not included in the main Tesseract repository, as [trac_ik_lib](https://bitbucket.org/traclabs/trac_ik/) is ROS-dependent.

## Usage

- Clone this repo to the `src` folder of your project.
- Build `trac_ik_lib` **2.2.0 or newer** from source (see `dependencies.repos`). The ROS binary releases are too old
  (Jazzy ships 2.0.2): 2.1.0 replaced the constructor's `rclcpp::Node::SharedPtr` with an `rclcpp::Logger`, and 2.2.0
  added `Manip3`. Uninstall `ros-<distro>-trac-ik-lib` or make sure the workspace overlay is sourced after
  `/opt/ros/<distro>/setup.bash`, so both the build and the loader pick up the source build.
- Add `tesseract_trac_ik_trac-ik_factory` to the `search_libraries` of your robot's `kinematic_plugins.yml`.
- Add Trac-IK to the `inv_kin_plugins` of your robot manipulator:

```
kinematic_plugins:
  search_libraries:
    - tesseract_trac_ik_trac-ik_factory
  inv_kin_plugins:
    manipulator:
      TracIKInvKinChain:
        class: TracIKInvKinChainFactory
        config:
          base_link: base
          tip_link: tool0
```

### Parameters

- _epsilon:_ Maximum deviation between target pose and IK solution.
- _max_time:_ Maximum calculation time per IK query (seconds). Speed returns as soon as a solution is found or this deadline is hit; Distance/Manip1/Manip2/Manip3 keep searching until the deadline and return the best solution found.
- _solve_type:_ (Speed/Distance/Manip1/Manip2/Manip3) Speed: return first feasible solution, Distance: return solution closest to seed, Manip1/Manip2/Manip3: use one of three manipulabilty metrics to find the best solution (Manip1 maximizes the product of the Jacobian's singular values, Manip2 its inverse condition number, Manip3 its smallest singular value).
- _bounds:_ Per-axis Cartesian tolerance applied to every IK query, as a 6-element sequence `[vx, vy, vz, rx, ry, rz]` (linear x/y/z then angular x/y/z, in meters and radians). Any solution whose pose lies within these bounds of the target is accepted. All zeros (default) disables tolerance and falls back to `epsilon`.

#### Parameter defaults
```
      TracIKInvKinChain:
        class: TracIKInvKinChainFactory
        config:
          base_link: base
          tip_link: tool0
          params:
              epsilon: 1e-5
              max_time: 0.005
              solve_type: Speed
              bounds: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

## Future work

The `trac_ik_lib` dependency should be made ROS-agnostic, so this plugin can be included in the `tesseract` repo.
