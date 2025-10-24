package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmACW extends Command{

    private final ArmSubsystem arm;

    public ArmACW(ArmSubsystem arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        System.out.println("[Bionic|ArmACW] started");
    }

    @Override
    public void execute(){
        arm.setMotorPower(0.4);
    }

    @Override
    public void end(boolean interrupted){
        arm.stopMotor();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

