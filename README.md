# pytest-ros

A Pytest plugin to provide fixtures and utilities for testing of ROS 2 systems. 

## Features

- *Automated Lifecycle Management*: Automatically initialize and clean up launched programs with some additional safeguards against rogue processes, though not guaranteed

- *Fixtures*: Included fixtures for launching ROS processes such as launch files and for monitoring ROS topics


## Installation

To install `pytest-ros`:

```
pip install .
```

This plugin has an external undeclared dependency on ROS 2 `rclpy`. Make sure ROS 2 is installed and sourced.

## Usage

### Running Tests

Execute tests normally with `pytest`.


