package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToPosition extends Command{

    private final SwerveSubsystem drive;

    private Translation2d init;
    private Pose2d pose;
    
    public DriveToPosition(SwerveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize()
    {
        init = drive.getPose().getTranslation();
        pose = drive.getPose().plus(new Transform2d(1, 0.5, new Rotation2d(30)));
        drive.driveToPose(pose);
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
        System.out.println("DriveToPosition finished");
    }
}
