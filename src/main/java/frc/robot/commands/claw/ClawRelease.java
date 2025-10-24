package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ClawRelease extends Command{
    private final ClawSubsystem claw;



    // i dont think im gonna use this command lol
    public ClawRelease(ClawSubsystem claw){
        this.claw = claw;
        addRequirements(claw);
    }
    
    @Override
    public void initialize(){
        System.out.println("Claw Close");
    }
}
