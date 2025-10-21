package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorReturnHome extends Command {
    private final ElevatorSubsystem elevator;

    public ElevatorReturnHome(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        if (!elevator.atBottom()) {
            elevator.moveElevator(-ElevatorConstants.MAX_SPEED);
        } else {
            elevator.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return elevator.atBottom();
    }
}
