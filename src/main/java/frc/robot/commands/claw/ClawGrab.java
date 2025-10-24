package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawSubsystem;

public class ClawGrab extends Command{

    private final ClawSubsystem claw;

    public ClawGrab(ClawSubsystem claw){
        this.claw = claw;
        addRequirements(claw);
    }
    
    @Override
    public void execute(){
        System.out.println("Claw Running");
    }

}
