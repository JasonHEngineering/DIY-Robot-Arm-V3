# digital_twin – model pipeline

```
cad_source/Robot_Assembly.step      (copied from DIY-Robot-Arm-V3/CAD/Assembly)
        │  cad_tools/export_links.py   (FreeCAD 1.1, headless)
        ▼
description/meshes/<link>_<category>.stl  +  description/link_map.json
        │  cad_tools/make_urdf.py      (plain Python, uses hayley_kinematics.py)
        ▼
description/hayley_v3.urdf  ──copied to──►  ../Python program loaded on Raspberry Pi 4/twin_web/description/
```

## Adding or removing a part

**Remove a part** (no CAD edit needed): find its label in `description/link_map.json`, e.g. `"6020Fan001"`, and add it to `cad_tools/link_overrides.json`:

```json
{ "6020Fan001": "exclude" }
```

**Add a part** (gripper, sensor, bracket…):
1. Add it to the assembly *at the kinematic-zero pose*: arm upright, forearm forward (that's how `Robot_Assembly.step` is modelled). Either:
   - in Onshape: add it to the assembly and re-export the whole assembly as STEP, or
   - in FreeCAD: open `cad_source/Robot_Assembly.step`, *Import* the new part's STEP, set its Placement, then select everything and *Export* it back to `cad_source/Robot_Assembly.step`.
2. Run `rebuild_model.bat`. New parts are assigned to a link automatically by position.
3. Check `description/link_map.json`. If a part landed on the wrong link, pin it in `link_overrides.json`, e.g. `{ "Claw": "link6" }`, and rebuild.

Valid override values are `base_link`, `link1` to `link6`, and `exclude`. Moving parts that don't change the joint layout needs no kinematics change. Changing a link length or joint position also needs `hayley_kinematics.py` and the Dash UI's `local_linkage_data` updated.

**Rebuild:** double-click `rebuild_model.bat`, then refresh the twin with Ctrl+F5. It runs the two steps below and copies the result into the Pi app.

Manual steps. Run these from `digital_twin/cad_tools`:

```powershell
$env:HAYLEY_TOOLS_DIR=(Get-Location).Path; & "$env:LOCALAPPDATA\Programs\FreeCAD 1.1\bin\freecadcmd.exe" -c "exec(open(r'$PWD\export_links.py').read())"
```

```powershell
python make_urdf.py
```

- **Link assignment** uses geometric rules in `export_links.py` (`classify()`). If a part is on the wrong link, add `{"<part label>": "link4"}` to `cad_tools/link_overrides.json` and rerun. `description/link_map.json` lists every part's link, category, volume and centroid.
- **Kinematics** live in `cad_tools/hayley_kinematics.py`, a verbatim copy of `local_linkage_data` from the Dash UI. If you change the linkage in the UI, change it here too.
- The URDF uses `package://hayley_description/meshes/...`, so it drops straight into a ROS 2 `hayley_description` package for Stage 2 (RViz / MoveIt 2).
