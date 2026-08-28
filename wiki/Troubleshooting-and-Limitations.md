# Troubleshooting and Limitations

## The train does not move

- Enable the checkbox in the **Simulation** panel header.
- Read the **Startup Check** messages.
- Remember that every simulation begins at zero speed. The start location needs an active Transport, or a block graph that sets a positive transport target.
- A Friction Brake can slow or hold but cannot push. A Trim Brake can only cap speed.
- Verify that a captured block's graph reaches **Release Block**.
- Click **Recompute Simulation** after changing control or route setup.

## The route is wrong or ends early

- Inspect **Route Pieces** for the resolved order and forward/reversed labels.
- Check the connection on the end through which the train exits the current piece.
- For a switch, verify its active index and that the selected connection has a target.
- Choosing **Enter At: End** intentionally reverses the target piece.
- An unconnected end is a valid dead end; the simulation stops there.

## A zone or sensor appears in the wrong place

Positions are piece-local distances in meters, not normalized curve factors and not route-wide distances. Select the correct Curve object before editing its hardware. Use the overlays to check the authored span.

Only the first spline of a Curve object is currently evaluated.

## Followers flip on vertical track

Change **Orientation Frame** from **Z Up (Legacy)** to **Continuous Z Up** or **Minimum Twist**. If the flip occurs at the seam of a cyclic curve, also inspect **Bank Continuity** and the authored point tilts.

## A block cannot stop or dispatch correctly

- Assign at least one suitable actuator to the block.
- Use a Transport or Friction Brake for exact stops; Trim Brakes cannot stop or hold.
- Keep hold positions inside the resolved block span and inside the assigned braking hardware.
- Make sure graph nodes form one connected sequence beginning at **Block Entered**.
- Control graphs with branches or cycles are not supported as general logic programs; heed graph warnings.
- End the intended sequence with **Release Block**.

## Driver triggers do not work

Use the registered driver function exactly as a driver expression:

```python
cm_trigger('your_channel')
```

Check that the sensor or block graph reaches its Trigger action, and that its spelling matches the driver. Trigger state is deterministic and repeats with the detected simulation cycle.

## Live playback differs after editing

The trajectory is cached from timeline time zero. Most relevant edits invalidate it automatically; **Recompute Simulation** explicitly rebuilds the preview when in doubt. Baked keys override live train-front motion until **Clear Baked Keys** is used.

## Current limitations

- Blender 5.0 or newer is required.
- Only the first spline of each Curve object is used.
- The prototype currently models one root-driven train; multi-train block occupancy is not yet available.
- Switch topology is resolved from the current switch state. Animated switch keyframes are not sampled into the trajectory, so editing a switch rebuilds the preview from time zero with the newly selected route.
- The route walk is capped at 64 pieces as a safety limit.
- Trajectory discovery has a 20,000-frame safety limit before it falls back to available wrap/stop behavior.
- Data structures and Python APIs may change between releases.

## Safe reset

**Reset & Utilities → Clear / Reset Curve Setup…** removes Coaster Mixer actuators, sensors, blocks, control graphs, and outgoing connections from the target curve. This is destructive to authored setup data, so save a backup or duplicate the curve first.

When reporting a bug, include the Blender version, exact reproduction steps, and a minimal `.blend` file when possible.
