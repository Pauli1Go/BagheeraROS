# Legacy docking controller (not built, not launched)

Hand-written AprilTag docking state machine used before the switch to Nav2's
`opennav_docking` server (2026-09-23). Kept only as reference for its
measurements and calibration history:

- `docking_controller.py`: Nav2-staged approach (`/dock/trigger`) and the
  ID 1-only `/dock/small_trigger` mode.
- `docking_controller.yaml`: its former `nav2_navigation.yaml` section,
  including the tag heading/lateral references from the manual docking
  recording.
- `dock_alignment_check.py`: stationary alignment snapshot; imports the old
  controller and does not run from here.
- `dock_vision_relay.py`: unused JPEG-to-mono relay.

The active implementation is `src/bagheera_docking` (dock plugin) plus
`bagheera_dock_tag_pose` and `bagheera_dock_trigger` in `bagheera_base`.
