package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmACW extends Command{

    private final ArmSubsystem arm;
    private final double degreesToMove = 45.0;
    private double targetAngle;
    private double initialAngle;

    public ArmACW(ArmSubsystem arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        System.out.println("[Bionic|ArmACW] started");
        initialAngle = arm.getEncoderPosition();
        targetAngle = initialAngle + degreesToMove;
    }


    @Override
    public void execute(){

    }

}

