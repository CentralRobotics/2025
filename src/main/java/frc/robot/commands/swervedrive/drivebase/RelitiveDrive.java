package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RelitiveDrive extends Command{

    private final SwerveSubsystem drive;

    private Translation2d init;
    
    public RelitiveDrive(SwerveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize()
    {
        init = drive.getPose().getTranslation();
        drive.drive(new ChassisSpeeds(0.3, 0, 0));
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return drive.getPose().getTranslation().getDistance(init) > 0.125;
    }

    @Override
    public void end(boolean interrupted)
    {
    }
}
