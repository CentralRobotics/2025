package frc.robot.commands.arm;


import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmCW extends Command {

    private final ArmSubsystem arm;

    public ArmCW(ArmSubsystem arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        System.out.println("[Bionic|ArmCW] started");
    }

    @Override
    public execute(){
        arm.setMotorPower(-0.4);
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
