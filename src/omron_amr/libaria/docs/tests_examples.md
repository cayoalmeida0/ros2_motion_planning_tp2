# Tests and Examples

This document describes the curated CMake-based test and example targets for
`libaria`.

The original `tests/` and `examples/` directories contain many legacy programs.
Some are deterministic and safe to run automatically. Others connect to hardware,
expect operator input, or can move the robot. Those are intentionally kept out of
the automated runner.

## CMake Support

The top-level [CMakeLists.txt](../CMakeLists.txt) now includes curated build
sections for legacy tests and examples.

Available options:

- `LIBARIA_BUILD_LEGACY_TEST_PROGRAMS=ON`
- `LIBARIA_BUILD_LEGACY_EXAMPLE_PROGRAMS=ON`
- `LIBARIA_BUILD_SAFE_TEST_RUNNER=ON`
- `LIBARIA_BUILD_HARDWARE_TEST_RUNNER=ON`
- `LIBARIA_REGISTER_HARDWARE_CTEST=OFF`

Default behavior is to build the curated targets and the safe runner.

Build from the workspace root:

```bash
cd /home/ubuntu/colcon_ws
colcon build --packages-select libaria
```

## Safe Automated Runner

The master runner executable is:

```bash
/home/ubuntu/colcon_ws/build/libaria/libaria_safe_test_runner
```

It launches each curated non-motion executable, records whether it exited
successfully, prints a summary, and writes a text report to the current working
directory.

Default report file:

```text
libaria_safe_test_report.txt
```

Custom report path:

```bash
/home/ubuntu/colcon_ws/build/libaria/libaria_safe_test_runner \
  --report /tmp/libaria_safe_test_report.txt
```

You can also run it through CMake:

```bash
cd /home/ubuntu/colcon_ws/build/libaria
cmake --build . --target run_libaria_safe_tests
```

Or through CTest:

```bash
cd /home/ubuntu/colcon_ws/build/libaria
ctest --output-on-failure -R libaria_safe_test_runner
```

## Hardware Test Runner

The hardware runner is a separate executable for robot-required programs:

```bash
/home/ubuntu/colcon_ws/build/libaria/libaria_hardware_test_runner
```

This runner is intentionally separate from the safe suite because it is expected
to fail when no robot is connected.

Default report file:

```text
libaria_hardware_test_report.txt
```

You can pass common robot connector arguments to every program after `--`:

```bash
/home/ubuntu/colcon_ws/build/libaria/libaria_hardware_test_runner \
  --report /tmp/libaria_hardware_test_report.txt \
  -- -robotPort /dev/ttyUSB0
```

Or for a remote target:

```bash
/home/ubuntu/colcon_ws/build/libaria/libaria_hardware_test_runner \
  -- --remoteHost 192.168.0.10
```

There is also a build target:

```bash
cd /home/ubuntu/colcon_ws/build/libaria
cmake --build . --target run_libaria_hardware_tests
```

By default the hardware runner is not registered with CTest. If you want it
available through CTest on a robot-equipped machine, configure with:

```bash
-DLIBARIA_REGISTER_HARDWARE_CTEST=ON
```

Then run:

```bash
ctest --output-on-failure -R libaria_hardware_test_runner
```

## Programs Included in the Safe Runner

### Safe tests

- `actionArgumentTest`
- `actionAverageTest`
- `angleBetweenTest`
- `angleFixTest`
- `angleTest`
- `lineTest`
- `mathTests`
- `robotListTest`

### Safe examples

- `functorExample`
- `threadExample`

These were selected because they do not intentionally connect to a robot, drive
hardware, or require an operator to steer or supervise motion.

## Manually Run Programs

The following curated programs are built by CMake but are not part of the safe
runner because they involve a live robot connection, can move hardware, or both.

These same programs are the current members of the hardware runner suite.

### Hardware runner members

- `absoluteHeadingActionTest`
- `simpleConnect`
- `simpleMotionCommands`
- `teleopActionsExample`

### `simpleConnect`

Purpose:

- Verify that ARIA can connect to the robot controller
- Print a small amount of robot state

Run:

```bash
/home/ubuntu/colcon_ws/build/libaria/simpleConnect
```

Typical override arguments:

```bash
/home/ubuntu/colcon_ws/build/libaria/simpleConnect -robotPort /dev/ttyUSB0
```

Expected result:

- Connects successfully
- Prints pose and battery information
- Exits cleanly after a short delay

### `simpleMotionCommands`

Purpose:

- Verify basic direct motion commands

Run only in a clear, supervised area.

Run:

```bash
/home/ubuntu/colcon_ws/build/libaria/simpleMotionCommands
```

Expected result:

- Connects to the robot
- Drives forward and rotates using direct commands
- Stops and exits cleanly

Operator procedure:

1. Clear at least several meters around the robot.
2. Confirm you can reach the robot emergency stop.
3. Start the program.
4. Watch each commanded move.
5. Stop the robot manually if behavior is not as expected.

### `teleopActionsExample`

Purpose:

- Verify guarded teleoperation with ARIA actions

Run:

```bash
/home/ubuntu/colcon_ws/build/libaria/teleopActionsExample
```

Expected result:

- Connects to the robot
- Accepts keyboard or joystick teleoperation
- Uses obstacle-aware action behavior when sensors are configured

Operator procedure:

1. Start in an open area.
2. Confirm sonar or laser devices are configured if you expect guarded behavior.
3. Drive slowly at first.
4. Verify stop behavior before testing longer motion.

### `absoluteHeadingActionTest`

Purpose:

- Verify heading-control behavior using an action that commands absolute heading

Run:

```bash
/home/ubuntu/colcon_ws/build/libaria/absoluteHeadingActionTest
```

Expected result:

- Connects to the robot
- Rotates to a sequence of headings
- Prints the heading reached after each move

Operator procedure:

1. Place the robot in a clear area with rotational clearance.
2. Confirm motors can be safely enabled.
3. Start the program and observe each heading change.
4. Stop immediately if heading response is unstable or excessive.

## Recommended Workflow

Use this order when validating the library after changes:

1. Build `libaria` with colcon.
2. Run the safe runner.
3. Run the non-ROS CLI in [cli.md](cli.md) to verify ArNetworking connectivity.
4. Run the hardware runner or selected manual hardware programs on a robot-equipped machine.
5. Run motion-capable programs only with an operator present.

## Scope Notes

This is intentionally a curated first pass, not a complete migration of every
legacy source file under `tests/` and `examples/` into modern CMake targets.
Many legacy programs are hardware-specific, device-specific, or written for one-
off development investigations. More can be added incrementally after they are
classified as either:

- safe for automated execution
- build-only and manual-run
- unsupported legacy programs that should stay as source reference only