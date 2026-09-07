# CAD source

`Differential Drive Robot.f3d` is the editable Fusion design archive used to
produce the robot geometry. `cad_geometry_report.json` records the geometry
checks made against that design snapshot.

The URDF is not linked live to Fusion. After a CAD change:

1. Reposition affected components and update captured positions in Fusion.
2. Re-export each affected component as an STL in metres and link-local
   coordinates.
3. Replace the corresponding file under
   `differential_drive_robot_description/meshes/`.
4. Update URDF joint origins, collision geometry, mass, and inertia values as
   required.
5. Rebuild and repeat the checks in `docs/verification.md`.

The current design uses X forward, Y left, and Z up. `base_link` is located at
the drive-axle centre and `lidar_link` at the optical scan centre.
