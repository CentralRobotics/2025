package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.Constants.ClawConstants;

public class ClawGrab extends Command {
    private final ClawSubsystem claw;
    private boolean isStalled = false;

    public ClawGrab(ClawSubsystem claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        isStalled = false;
        claw.setPower(ClawConstants.kGRAB_SPEED);
        System.out.println("[bionic|claw] The claw... ");
    }

    @Override
    public void execute() {
        // if it hasn’t stalled yet, check for stall
        if (!isStalled && claw.isStalled()) {
            isStalled = true;
            claw.stop(); // brake 
            System.out.println("[bionic|claw] Claw Stall detected!");

            // we may need to replace with a light hold torque if the brakes arent enough to keep the coral in the claw
            // it would literally just be claw.setPower(ClawConstants.kHoldSpeed); 
        }

        //  stay braked until released merp 
    }

    @Override
    public void end(boolean interrupted) {
        claw.stop(); // stop or brake when button released
    }

    @Override
    public boolean isFinished() {
        // this command runs as long as the button is held
        return false;
    }
}
