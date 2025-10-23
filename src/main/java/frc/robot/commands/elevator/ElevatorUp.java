package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorUp extends Command {
    private final ElevatorSubsystem elevator;

    public ElevatorUp(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println("[Bionic|Elevator] started");
        if (!elevator.atTop()) {
            elevator.moveElevator(ElevatorConstants.MAX_SPEED);
        } else {
            elevator.stop();
            double position = elevator.getPosition(); 
            System.out.println("[Bionic|Elevator] Reached top at: " + position);
            System.out.println("[Bionic|Elevator] stopped");
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return elevator.atTop();
    }
}
