# Usage Guide

## 1. Install and open Coaster Mixer

Coaster Mixer requires Blender 5.0 or newer.

1. Download the versioned extension ZIP from the project's GitHub Actions build artifacts.
2. In Blender, open **Edit → Preferences → Extensions**.
3. Choose **Install from Disk** and select `coaster_mixer-<version>.zip`.
4. Open a 3D View, press **N**, and choose the **Coaster** tab.

For a development checkout, copy or symlink the `coaster_mixer` directory into Blender's `scripts/addons` directory and enable the add-on.

## 2. Prepare the track curve

Create a coaster-shaped Curve in Blender or import one from another design tool. Coaster Mixer currently reads only its first spline; Bezier curves receive the add-on's full interpolated sampling path.

Apply or deliberately account for object transforms before detailed setup. Coaster Mixer samples the curve in world space, and all authored positions are measured in meters along the piece.

For a simple circuit, use one cyclic curve. For switches or transfer tracks, use separate Curve objects and connect them as pieces.

## 3. Set the root

1. Select the curve that should begin the route.
2. In **Coaster**, click the eyedropper beside **Root** / **Use Active Curve**.
3. Confirm that the summary reports the expected piece count, route length, and open/closed state.

The root stores train, simulation, and block data. Each selected curve piece stores its own actuators, sensors, and outgoing connections.

## 4. Configure banking

Select a route piece and open **Track Hardware → Curve Banking**.

- Start with **Continuous Z Up** for a vertical-safe version of familiar Blender orientation.
- Use **Minimum Twist** when smooth transported orientation matters more than world-up alignment.
- Keep **Automatic Seam** for most cyclic tracks.
- Use **Exact Authored Values** or **Manual Seam Winding** only when you intentionally authored roll across the closing seam.

Banking comes from each curve point's **Tilt** value in Blender Edit Mode.

## 5. Add actuators

In **Track Hardware → Actuators**, click **+**, then choose a type and configure its piece-local span.

- Add a **Transport** where the train must launch, climb, creep, or leave a stop.
- Add a **Friction Brake** for a controllable brake run or stopping area.
- Add a **Trim Brake** only to limit a moving train's speed.

Set **Start** and **Length** in meters. The maximum speed and acceleration/braking values are hardware limits; block-control nodes request behavior within those limits.

Use the hardware viewport overlay to confirm that each span lands on the intended section of curve.

## 6. Connect pieces and create switches

Select the piece you want to edit and open **Connections & Switches**.

1. Under **Piece Start** or **Piece End**, click **Add Connection**.
2. Choose the target Curve object.
3. Choose whether the route enters the target at **Start** or **End**.
4. Inspect **Route Pieces** to verify the resulting order and traversal direction.

Add multiple connections to the same end to create a switch, then change its active index to select the exit. A missing target or an unconnected end creates a dead end.

Connections are directional declarations. If the coaster must travel back through an edge in another route configuration, author the corresponding connection where needed.

## 7. Create a controlled block

Blocks are necessary when a train must follow a timed or conditional sequence rather than only react to passive hardware.

The quickest setup is:

1. Select a stop-capable Transport or Friction Brake actuator.
2. Open **Blocks** and click **Create from Selected Actuator**.
3. Choose a suitable template, then click **Apply…**.
4. Set the hold position, dwell, speed, acceleration, and braking parameters.
5. Click **Edit Control Graph** to inspect or customize the generated nodes.

For a block spanning multiple actuators, use **Assign Selected Actuator** on each relevant zone. The resolved span runs from the first assigned zone's start to the last assigned zone's end in route space.

Always ensure the connected graph reaches **Release Block**. The panel reports unresolved spans, actuator mismatches, missing dispatches, and other program warnings.

### Common block recipes

- **Station:** friction brake plus transport, using **Load Station** or **Unload Station**.
- **Launch:** transport, using **Stopped Launch** or **Rolling Launch**.
- **Lift:** transport, using **Standard Lift**.
- **Trim:** trim or friction brake, using **Trimmed Brake Zone**.

## 8. Add sensors and animation triggers

In **Track Hardware → Sensors**:

1. Add a sensor and set its position in meters from the start of the selected piece.
2. Add **Wait** and/or **Trigger** actions in the desired order.
3. Give each Trigger a channel name and numeric value.
4. On an animatable Blender property, add a driver that reads the channel, for example `cm_trigger('onride_photo')`.

Keep channel names consistent and avoid renaming them after setting up drivers.

## 9. Build the train rig

In **Train**, set the physical model first, then create or attach followers.

### Create followers

1. Click **Create Cars**.
2. Choose the number of cars, spacing, and first offset.
3. Parent each visual car object to the matching generated empty.

Offsets are distances behind the train front. A negative offset places a helper ahead of it.

### Attach an existing rig

Select one or more Empty objects and click **Attach Selected**. Edit each follower's meter offset in the panel. You can also assign a dedicated **Driven Empty** and click **Attach Driven Empty** for a root rig control.

## 10. Create a ride camera

Create at least two car followers, then open **Train → Ride Cameras** and click **Create Ride Camera**. Choose its height, look-ahead distance, lens, and whether it becomes the active scene camera.

After creation, choose a mounted car, refine the seat offset and target height, and optionally enable camera shake.

## 11. Preview the simulation

1. Set the scene FPS and timeline start before evaluating timing.
2. In **Simulation**, click the house button beside **Start Position** to use **Start at Station**.
3. Enable the checkbox in the **Simulation** panel header.
4. Click **Recompute Simulation** after substantial setup changes if needed.
5. Play or scrub the timeline.

The train starts with zero speed. The **Startup Check** explains common reasons it cannot depart, such as a missing powered Transport or control node.

For a cyclic coaster, placing a station block at the route seam makes route zero the station exit. Otherwise, **Start at Station** uses the last **Wait for Position** target it finds.

## 12. Bake for rendering

When the live result is correct:

1. Set the desired scene preview or frame range shown in the Bake box.
2. Click **Bake Simulation**.
3. Render or continue animating with the baked train-front keys.

Baking uses the same cached trajectory as live playback and writes trigger events as `cm:` timeline markers. Use **Clear Baked Keys** before returning to live simulation or changing the ride program.
