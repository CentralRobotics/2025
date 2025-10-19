
# 2026 base robot 
Bionic Broncos 2025 (offseason) - 2026 Base robot logic. 



## Authors/Maintainers 

- [@clovercrash](https://www.github.com/clovercrash)
- [@mango-rasp](https://github.com/mango-rasp)
## Structuring

```
robot/
├── commands/
│   ├── climber/
│   │   └── ManualClimber
│   └── swervedrive/
│       ├── AbsoluteDrive.
│       ├── AbsoluteDriveAdv.java
│       └── AbsoluteDriveField.java
├── subsystems/
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
to-do: better documentation of structure 


## Acknowledgement

We owe a huge thank you to the [YAGSL](https://yetanothersoftwaresuite.com/) team for great providing FOSS example code and [community](https://discord.com/invite/yass) for the assistance in troubleshooting, and [Team 179 Children of The Swamp]() for allowing us to use their facilities! 