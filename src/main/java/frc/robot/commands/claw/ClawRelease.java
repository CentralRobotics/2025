package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ClawClose extends Command{
    private final ClawSubsystem claw;

    public ClawClose(ClawSubsystem claw){
        this.claw = claw;
        addRequirements(claw);
    }
    
    @Override
    public void initialize(){
        System.out.println("Claw Close");
    }
}
