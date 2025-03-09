package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCoral extends Command{

    private final Intake intake;

    public IntakeCoral(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize()
    {
        intake.set(1);
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return intake.getCurrentDrawAmps() > 15;
    }

    @Override
    public void end(boolean interrupted)
    {
        intake.stopIntake();
        //vibrate controller
    }
}
