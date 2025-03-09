package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbOut extends Command{

    private final Climb climb;

    public ClimbOut(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize()
    {
        climb.extendClimb();
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
