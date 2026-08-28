# Feature Rundown

## Curve-based track graph

Every Blender Curve object can be a track piece. A piece can connect from either its start or end to either end of another piece. Entering a target at its end traverses that piece in reverse, including its forward direction and bank sign.

More than one connection on an end creates a switch. The **Start Switch Position** or **End Switch Position** chooses the active exit. Coaster Mixer resolves the route from the root through the current switch choices. A route can be a closed circuit or an open, dead-end route for storage and transfer tracks.

The **Route Pieces** panel shows the resolved order, direction, and length of every piece.

## Arc-length placement and banking

Followers are positioned by distance along the resolved route, rather than by a curve's parametric factor. This keeps apparent train speed consistent through unevenly spaced curve control points.

Curve tilt supplies banking. Each piece offers three orientation modes:

- **Z Up (Legacy)** matches the original behavior but can become singular on vertical tangents.
- **Continuous Z Up** preserves a familiar world-up orientation while carrying it safely through vertical track.
- **Minimum Twist** parallel-transports the frame along the curve to avoid vertical flips.

Cyclic curves also provide automatic, exact-authored, and manual bank seam handling.

## Track actuators

Actuators are authored in piece-local meters with a start, length, speed envelope, and acceleration/braking limits.

| Type | Behavior |
| --- | --- |
| **Transport** | Drives toward a target speed. It can accelerate, brake, stop, hold, and move a train. Use it for drive tires, LSM launches, and lifts. |
| **Friction Brake** | Slows the train and can stop or hold it once it is within its controllable speed. It cannot push the train. |
| **Trim Brake** | Applies a magnetic-style speed cap. It cannot stop, hold, or propel the train. |

Transports provide drive; brakes provide caps. This means a stopped train under a station brake can still be pushed out by a transport after the brake is released by the control program.

## Blocks and ride-control graphs

A block combines three independent layers:

- an occupancy span;
- one or more assigned actuators;
- an editable control node graph.

The train is captured when its front enters the block span. The graph begins at **Block Entered** and can set transport or brake targets, brake to an exact hold point, wait for time/position/speed conditions, set a trigger, release a brake, and dispatch the train. A graph must eventually reach **Release Block** or the train remains captured.

Generated templates provide useful starting graphs:

- **Stopped Launch**
- **Rolling Launch**
- **Standard Lift**
- **Trimmed Brake Zone**
- **Load Station**
- **Unload Station**

Templates generate ordinary editable node trees. Applying a template replaces the block's current graph, so make custom edits after generating it.

## Sensors and trigger channels

Sensors are points measured from the start of a track piece. When the train crosses a sensor, it can run an ordered sequence of **Wait** and **Trigger** actions. A sensor fires once per lap and rearms after the route wraps.

A trigger assigns a numeric value to a named channel. Use that value in any Blender driver expression:

```python
cm_trigger('photo_flash')
```

This can drive lights, gates, effects, visibility, or other animatable properties. Baking adds `cm:` timeline markers at trigger frames.

## Train rigging

Coaster Mixer can create a chain of car follower empties with spacing measured along the route. Parent each car model to its corresponding empty. Existing selected empties can also be attached, and their offsets remain editable in the **Train** panel.

The root owns the train's physical settings:

- train length and weight;
- rolling resistance, including curvature-loaded wheel force;
- aerodynamic drag coefficient and frontal area.

## Ride cameras

After at least two car followers exist, Coaster Mixer can create a ride camera mounted to a car and aimed at a route-driven look-ahead target. Camera controls include:

- mounted car;
- local XYZ seat offset;
- lens and look-ahead distance;
- target height;
- deterministic speed- and G-driven shake.

## Deterministic simulation and baking

The first timeline frame is simulation time zero: the train begins at **Start Position** with zero speed. Each later frame advances by one scene-frame interval. Results are cached, so backward scrubbing and repeated playback return the same state.

When the quantized state repeats on a circuit, Coaster Mixer detects the cycle and loops it with constant-time lookups. Open routes stop at their final piece.

**Bake Simulation** writes the same train-front trajectory used by live playback to linear keyframes and adds trigger markers. **Clear Baked Keys** returns control to the live simulation.

## Viewport feedback

Optional overlays display actuator spans, braking influence, block occupancy and hold points, sensors, the simulation start cursor, and selected control-node influence. Overlays can be hidden during playback to improve viewport performance.
