# Celt-X-2023
Public Release of Team 5406's 2023 Season Codebase for our competiton robot Edwin.

## Abilities
Edwin consistently had a 2.5 gamepiece auto on the clean side of the field. It could pick up gamepieces from the single substation and score them anywhere on the grid, with the help of vision.

## Highlights
- This was our first year implementing AI-Object dectection - the code for it can be found [here](./src/main/ObjectDetection)
- Custom REV Wrapper for CAN error checking - this can be found in the setup portion of each subsystem
    - Ex. [Our Intake Subsystem](./src/main/java/frc/team5406/robot/subsystems/superstructure/WristSubsystem.java)
- Custom [AutoBalance](./src/main/java/frc/team5406/robot/commands/AutoBalance/java) for the Charge Station
- Implemented [YAGSL](https://github.com/BroncBotz3481/YAGSL) for our Swerve Drive
- Custom input scaling for circular/diagonal inputs on the translation joystick
    - Can be found at the bottom of our RobotContainer
