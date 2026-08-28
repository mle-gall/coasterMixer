# Coaster Mixer Wiki

Coaster Mixer is a Blender 5.x add-on for turning Blender curves into a connected, animated rollercoaster. It combines track authoring, ride-control logic, deterministic physics, train rigging, and ride cameras in the **Coaster** tab of the 3D View sidebar.

## Start here

- **[Usage Guide](Usage-Guide)** — install the add-on and build a working coaster from a curve.
- **[Feature Rundown](Feature-Rundown)** — understand track pieces, switches, actuators, blocks, sensors, train followers, cameras, and simulation.
- **[Troubleshooting and Limitations](Troubleshooting-and-Limitations)** — diagnose common setup problems and review current constraints.

## The basic workflow

1. Create or import one or more Blender Curve objects.
2. Choose a curve as the coaster **Root**.
3. Add track actuators and connect additional curve pieces.
4. Create blocks and assign ride-control graphs where controlled behavior is needed.
5. Create follower empties, then parent the train car models to them.
6. Enable the simulation and play or scrub the timeline.
7. Optionally create a ride camera and bake the simulation to keyframes.

Coaster Mixer currently uses the first spline of each Curve object. Distances and speeds use meters and meters per second.

## Video walkthrough

[Watch the full coaster setup walkthrough on YouTube](https://www.youtube.com/watch?v=QcXsMtAH3MY).

## Project links

- [README](../README)
- [License](../LICENSE) — GNU GPL v3 or later
