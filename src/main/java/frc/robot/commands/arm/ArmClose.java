package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmClose extends Command{

    private final ArmSubsystem arm;

    public ArmClose(ArmSubsystem arm){
        this.arm = arm;
        addRequirements(arm);
    }
    @Override
    public void initialize(){
        System.out.println("[Bionic|ArmClose] started");
    }

}

