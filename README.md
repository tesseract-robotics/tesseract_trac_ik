# Tesseract Trac-IK plugin

This is a [Trac-IK](https://traclabs.com/projects/trac-ik/) plugin for `tesseract_kinematics`.

This plugin is not included in the main Tesseract repository, as [trac_ik_lib](https://bitbucket.org/traclabs/trac_ik/) is ROS-dependent.

## Usage

- Clone this repo to the `src` folder of your project.
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
- _max_time:_ Maximum calculation time for Distance, Manip1 and Manip2 solve types.
- _solve_type:_ (Speed/Distance/Manip1/Manip2) Speed: return first feasible solution, Distance: return solution closest to seed, Manip1/Manip2: use one of two manipulabilty metrics to find the best solution.

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
```

## Future work

The `trac_ik_lib` dependency should be made ROS-agnostic, so this plugin can be included in the `tesseract` repo.
