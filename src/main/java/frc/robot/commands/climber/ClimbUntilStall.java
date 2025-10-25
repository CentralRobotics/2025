// package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.climber.ClimberSubsystem;

// public class ClimbUntilStall extends Command {
//   private final ClimberSubsystem climber;

//   public ClimbUntilStall(ClimberSubsystem climber) {
//     this.climber = climber;
//     addRequirements(climber);
//   }

//   @Override
//   public void initialize() {
//     climber.engageRatchet();  // pull against
//     climber.climbUp();
//   }

//   @Override
//   public boolean isFinished() {
//     return climber.isStalled();  // hit current threshold briefly
//   }

//   @Override
//   public void end(boolean interrupted) {
//     climber.stop();
//     climber.engageRatchet(); // holding
//   }
// }
