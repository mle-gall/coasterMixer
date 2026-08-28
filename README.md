# Coaster Mixer

Coaster Mixer is a Blender add-on for authoring and driving rollercoasters from
curve objects. It provides a curve-segment editor, connected track routes,
switches, transport and brake zones, block sequencing, sensors, and
arc-length-accurate train followers in the 3D View sidebar.

> **Project status:** early prototype. The file format and Python API may change.
> Coaster Mixer currently targets Blender 5.0 and newer.

## Full coaster setup walkthrough

Watch the complete setup video to see a coaster curve configured with track
hardware, blocks, train followers, and simulation:

[![Watch the full Coaster Mixer setup walkthrough](https://img.youtube.com/vi/QcXsMtAH3MY/hqdefault.jpg)](https://www.youtube.com/watch?v=QcXsMtAH3MY)

[Watch the full coaster setup walkthrough on YouTube](https://www.youtube.com/watch?v=QcXsMtAH3MY)

## Features

- Build a track as a graph of Blender curve objects, including switches and
  reversed pieces.
- Author transport, friction-brake, and magnetic trim zones in meters.
- Sequence block behavior with a node-based control graph.
- Trigger named animation channels from sensors and block programs.
- Place train followers and cameras at accurate arc-length offsets.
- Mount ride cameras on any train car, align them to a seat with XYZ offsets,
  and add deterministic speed- and G-driven camera shake.
- Preview deterministic physics from the Blender timeline and bake the same
  trajectory to keyframes.
- Model rolling resistance and aerodynamic drag.

## Installation

Download the latest build artifact from GitHub Actions, then in Blender open
**Edit → Preferences → Extensions**, choose **Install from Disk**, and select
the `coaster_mixer-<version>.zip` file.

For development checkouts, copy or symlink the `coaster_mixer` directory into
Blender's `scripts/addons` directory, refresh the add-on list, and enable
**Coaster Mixer**.

The add-on appears under **3D View → Sidebar → Coaster**.

## Quick start

Coaster Mixer starts from a coaster-shaped Blender Curve. You can model that
curve directly in Blender or create the track with another coaster design tool,
then export and import it into Blender. Useful starting points include:

- [NoLimits 2](https://www.nolimitscoaster.com/), using the
  [Blender NoLimits CSV Importer](https://github.com/geforcefan/BlenderNoLimitsCSVImporter)
  to turn exported Professional Track Data into a Blender spline.
- [KexEdit](https://github.com/IndividualKex/KexEdit), an open-source,
  node-based Force Vector Design coaster editor.
- [open FVD++](https://github.com/altlenny/openFVD), an open-source force- and
  geometry-based coaster design tool.

Whichever workflow you use, the resulting track must be a Blender Curve object.
Coaster Mixer currently treats its first spline as the working track.

1. Select the coaster curve and open the **Coaster** tab in the 3D View sidebar.
2. Set the active curve as the coaster root.
3. Add hardware zones and connect additional curve pieces as needed.
4. Attach selected empties as train followers, or use the train setup tools.
5. Configure the simulation and play or scrub the timeline.

Authored zones and connections live on each curve piece. Train and simulation
settings live on the root piece.

## Repository layout

```text
coaster_mixer/
  runtime.py    Geometry, routes, placement, simulation, and caches
  model.py      Blender properties and control-graph node types
  operators.py  Authoring, setup, simulation, and bake operations
  ui.py         Panels, drawing helpers, and Blender handlers
  __init__.py   Add-on metadata and registration
bench_sim.py            Headless benchmark and regression harness
scripts/build_addon.py  Deterministic Blender Extension ZIP builder
```

The modules intentionally depend in one direction:
`runtime → model → operators → ui → registration`. Runtime code does not import
the user interface, keeping geometry and simulation concerns independently
testable inside Blender.

## Development

Build an installable extension archive with:

```bash
python3 scripts/build_addon.py
```

The versioned ZIP is written to `dist/`. Its manifest and add-on modules are at
the archive root, as required by Blender's Extension format. Pushes to `main`
run the same build in GitHub Actions and retain the ZIP as a workflow artifact
for 30 days.

Run the regression benchmark after changing route resolution, physics,
placement, or caching:

```bash
/path/to/blender --background --factory-startup \
  --python bench_sim.py -- local
```

The harness builds a representative multi-piece circuit, checks physics and
cache behavior, times live playback and baking, and appends results to
`bench_results.json`.

Please keep authored coaster data on curve objects unless it is genuinely
scene-level UI state. Until the data model stabilizes, changes should preserve
compatibility with Blender 5.x and the benchmark's physics assertions.

## Contributing

Issues and pull requests are welcome. When reporting a bug, include the Blender
version, steps to reproduce it, and a minimal `.blend` file when possible.
Changes to simulation or caching should include a successful benchmark run.

## License

Coaster Mixer is free software licensed under the
[GNU General Public License version 3 or later](LICENSE). You may use, study,
modify, redistribute, and sell it under the terms of that license. Distributed
modified versions must provide their corresponding source under the GPL as
well.
