# Tesseract Trac-IK plugin

This is a [Trac-IK](https://traclabs.com/projects/trac-ik/) plugin for `tesseract_kinematics`. It is not included in the main `tesseract` repository, as it depends on a [ROS 2 fork of trac_ik_lib](https://github.com/aprotyas/trac_ik), which is ROS-dependent.

## Usage

Add `tesseract_trac_ik_trac-ik_factory` to the `search_libraries` of your robot's `kinematic_plugins.yml`.

(Note: The plugin name `tesseract_trac_ik_trac-ik_factory` will normalize to `tesseract_kinematics_trac-ik_factory` once it is integrated into `tesseract`.)

### Parameters (defaults shown below)

- _epsilon:_ Maximum deviation between target pose and IK solution.
- _max_time:_ Maximum calculation time for Distance, Manip1 and Manip2 solve types.
- _solve_type:_ (Speed/Distance/Manip1/Manip2) Speed: return first feasible solution, Distance: return solution closest to seed, Manip1/Manip2: use one of two manipulabilty metrics to find the best solution.

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
          params:
              epsilon: 1e-5
              max_time: 0.005
              solve_type: Speed
```

## Future work

The `trac_ik_lib` dependency should be made ROS-agnostic before this plugin can be included in the `tesseract` repo.
