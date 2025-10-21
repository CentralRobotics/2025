
# 2026 base robot 
Bionic Broncos 2025 (offseason) - 2026 robot logic. 
Based off YAGSL-Example


## Authors/Maintainers 

- [@clovercrash](https://www.github.com/clovercrash)
- [@mango-rasp](https://github.com/mango-rasp)
## Structuring

```
robot/
├── commands/
│   ├── elevator/
│   │   └── ElevatorUp.Java
│   │   └── ElevatorDown.Java
│   ├── climber/
│   │   └── 
│   └── swervedrive/
│       ├── AbsoluteDrive.
│       ├── AbsoluteDriveAdv.java
│       └── AbsoluteDriveField.java
├── subsystems/
│   ├── elevator/
│   │   └── ElevatorSubsystem.java
│   ├── climber/
│   │   └── ClimberSubsystem.java
│   └── swervedrive/
│       ├── SwerveSubsystem.java
│       └── Vision.java
├── Constants.java
├── Robot.java
├── Main.java
└── RobotContainer.java
```
we're working on better documentation for future broncos!


## Acknowledgement

We owe a huge thank you to the [YAGSL](https://yetanothersoftwaresuite.com/) team for great providing FOSS example code and [community](https://discord.com/invite/yass) for the assistance in troubleshooting, and [Team 179 Children of The Swamp]() for allowing us to use their facilities! 
