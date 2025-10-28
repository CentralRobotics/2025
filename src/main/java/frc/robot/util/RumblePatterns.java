package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumblePatterns {

    /**
     *  success events
     */
    public static Command success(CommandXboxController controller) {
        return Commands.sequence(
            Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1.0)),
            Commands.waitSeconds(0.25),
            Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.0))
        );
    }

    /**
     * Double pulse command completion confirmations
     */
    public static Command doublePulse(CommandXboxController controller) {
        return Commands.sequence(
            Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1.0)),
            Commands.waitSeconds(0.25),
            Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.0)),
            Commands.waitSeconds(0.25),
            Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1.0)),
            Commands.waitSeconds(0.25),
            Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.0))
        );
    }

    /**
     * Quick error buzz
     */
    public static Command error(CommandXboxController controller) {
        return Commands.sequence(
            Commands.runOnce(() -> controller.setRumble(RumbleType.kLeftRumble, 1.0)),
            Commands.waitSeconds(0.4),
            Commands.runOnce(() -> controller.setRumble(RumbleType.kLeftRumble, 0.0))
        );
    }
}
