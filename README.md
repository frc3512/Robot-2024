# FRC Team 3512's 2024 Robot

Source code for the 2024 comp robot: TBD

Source code also for the 2024 practice robot: TBD

## Setup

Install the relevant FRC toolchain for your platform (see
https://github.com/wpilibsuite/allwpilib/releases). Make sure the toolchain
is
placed in `~/wpilib/2024/roborio` (Linux) or
`C:\Users\Public\wpilib\2024\roborio` (Windows).

## Build options

### Build everything

* `./gradlew build`

This builds code for the roboRIO and tests (if present), but doesn't deploy it to the robot.

### Deploy

* `./gradlew deploy`

This runs a roboRIO build if needed, copies the resulting binary to a roboRIO at 10.35.12.2, and restarts it.

### Simulation GUI

* `./gradlew simulateJava`
Runs the simulation GUI for testing robot code without the real hardware.

### Format code
* `./gradlew spotlessApply`
Beautifies your code to make it easier to read. Required for builds/GitHub CLI to pass.

## Telemetry

Telemetry values are sent using NT4, with the ability for them to be disabled during competitions to ensure the maximum amount of network bandwitdth. When tuning, they can be viewed in the standard dashboard program like Shuffleboard or opened up in specific viewing programs like AdvantageScope.

## Simulation GUI

The simulation GUI is straightforward but can be read more about [here](https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-simulation/simulation-gui.html).

## Autonomous mode selection

Open shuffleboard and select the desired autonomous mode from the dropdown menu.
When the indicator next to the menu turns from a red X to a green checkmark, the robot has confirmed the selection.

See [this](https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html)
for details on how the robot side works.

## Game

* Game description here for Crescendo. Talk about the elements, how you can score points, and the different rank points you can get *

## Unique features

This years robot's unique features include:

- (add bullet points about all of the cool subsystems or features our robot has this year!)

## Goals of the year
|Status|Goal|Additional Description|
|------|----|----------------------|
|Placeholder|Placeholder|Hi! Add your robot goals for this year for easy access and status tracking!


## Roster
Mentor(s): Adan Silva
Lead: Jonathan Shuman
Students: Chris Colon, Jayden Furbur, Eli Miller
