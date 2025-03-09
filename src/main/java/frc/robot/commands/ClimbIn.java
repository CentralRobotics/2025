package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbIn extends Command{

    private final Climb climb;

    public ClimbIn(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize()
    {
        climb.retractClimb();
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return true;
    }

    @Override
    public void end(boolean interrupted)
    {
    }
}
