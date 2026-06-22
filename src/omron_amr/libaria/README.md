
# ARIA

Adept MobileRobots Advanced Robotics Interface for Applications.

This repository contains the ARIA and ArNetworking sources, examples, support
files, and a standalone non-ROS CLI for direct robot connectivity testing.

## Quick Start

Build from the workspace root:

```bash
cd /home/ubuntu/colcon_ws
colcon build --packages-select libaria
source /home/ubuntu/colcon_ws/install/setup.bash
```

Run the simple connectivity test client:

```bash
/home/ubuntu/colcon_ws/install/libaria/bin/omron_robot_cli
```

## Documentation

- CLI usage and operator instructions: [docs/cli.md](docs/cli.md)
- Curated test and example workflow: [docs/tests_examples.md](docs/tests_examples.md)
- Legacy full README and historical package documentation: [docs/README_old.md](docs/README_old.md)
- HTML API reference: [Aria-Reference.html](Aria-Reference.html)
- ArNetworking reference: [ArNetworking/ArNetworking-Reference.html](ArNetworking/ArNetworking-Reference.html)

## Notes

- Safe automated tests and opt-in hardware tests are split on purpose. The hardware runner is expected to fail when no robot is connected.
- License details are in `LICENSE.txt`.
- Package change history is in `Changelog`.


