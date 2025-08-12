# BasicMotor

BasicMotor is a Java library designed to simplify motor control logic for FRC (FIRST Robotics Competition) robots. It provides a unified API for interacting with different types of motors, making robot code cleaner and easier to maintain.

## Features

- Unified motor interface for FRC robots
- Built-in support for simulation environments
- Integrated PID control for precise motor control
- Automatic logging of motor data using AdvantageKit for easy telemetry and debugging
- Multithreaded architecture for high-speed, responsive motor updates

## Installation

For the installation, refer to the [installation section](https://github.com/captainsoccer/BasicMotor/wiki#installation) in the wiki.

## Wiki

For detailed documentation, guides, and advanced usage tips, please visit our [Wiki](https://github.com/captainsoccer/MotorUtils/wiki).  
The wiki contains additional resources to help you get started, troubleshoot issues, and explore advanced features of the MotorUtils library.

For more sepecif information about API, check the [java doc](https://javadoc.jitpack.io/com/github/captainsoccer/MotorUtils/latest/javadoc/index.html)


## Usage

To ensure motors are updated correctly, add the following code to your `robotPeriodic()` method in the `Robot.java` file, after the `CommandScheduler` runs:

```java
@Override
public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    MotorManager.getInstance().periodic();
}
```

## License

This project is licensed under the MIT License.

[![](https://jitpack.io/v/captainsoccer/MotorUtils.svg)](https://jitpack.io/#captainsoccer/MotorUtils)
