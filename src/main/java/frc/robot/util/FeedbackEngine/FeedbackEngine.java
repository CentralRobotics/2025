package frc.robot.util.FeedbackEngine;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Haptic Feedback Engine - A library of reusable rumble feedback patterns.
 * Use these for haptic feedback cues during driver operation.
 * Written by Clovercrash
 */
public class FeedbackEngine {

    // ========================
    // Simple patterns for simple situations 
    // ========================

    /** Short success pulse (0.25s) */
    public static Command success(CommandXboxController controller) {
        return pulse(controller, 0.25, 1.0);
    }

    /** Error pulse (left motor, 0.4s) */
    public static Command error(CommandXboxController controller) {
        return pulseSideSpecific(controller, 0.4, 1.0, RumbleType.kLeftRumble);
    }

    /** Double pulse confirmation */
    public static Command doublePulse(CommandXboxController controller) {
        return Commands.sequence(
            pulse(controller, 0.25, 1.0),
            Commands.waitSeconds(0.25),
            pulse(controller, 0.25, 1.0)
        );
    }

    // ===========================
    // Curated by 6686
    // ===========================

    /** alert */
    public static Command alternating(CommandXboxController controller) {
        return Commands.sequence(
            pulseSideSpecific(controller, 0.15, 1.0, RumbleType.kLeftRumble),
            Commands.waitSeconds(0.1),
            pulseSideSpecific(controller, 0.15, 1.0, RumbleType.kRightRumble),
            Commands.waitSeconds(0.1),
            pulseSideSpecific(controller, 0.15, 1.0, RumbleType.kLeftRumble),
            Commands.waitSeconds(0.1),
            pulseSideSpecific(controller, 0.15, 1.0, RumbleType.kRightRumble)
        );
    }

    /** Soft notifications */
    public static Command warning(CommandXboxController controller) {
        return pulse(controller, 0.4, 0.4);
    }

    /** victory!! */
    public static Command celebration(CommandXboxController controller) {
        return Commands.sequence(
            pulse(controller, 0.3, 1.0),
            Commands.waitSeconds(0.15),
            pulse(controller, 0.3, 0.8),
            Commands.waitSeconds(0.15),
            pulse(controller, 0.3, 1.0)
        );
    }

    /** Rapid stutter 
     * 
     * We use this for collision or unexpected interruptions
     * 
 */
    public static Command stutter(CommandXboxController controller) {
        return Commands.sequence(
            pulse(controller, 0.1, 1.0),
            Commands.waitSeconds(0.05),
            pulse(controller, 0.1, 1.0),
            Commands.waitSeconds(0.05),
            pulse(controller, 0.1, 1.0)
        );
    }

    /** Smooth ramp-up and fade-down rumble */
    public static Command fade(CommandXboxController controller) {
        return Commands.sequence(
            pulse(controller, 0.2, 0.4),
            Commands.waitSeconds(0.05),
            pulse(controller, 0.2, 0.7),
            Commands.waitSeconds(0.05),
            pulse(controller, 0.3, 1.0),
            Commands.waitSeconds(0.05),
            pulse(controller, 0.3, 0.6)
        );
    }

    /** Idle heartbeat */
    public static Command heartbeat(CommandXboxController controller) {
        return Commands.sequence(
            pulse(controller, 0.15, 0.5),
            Commands.waitSeconds(0.35),
            pulse(controller, 0.15, 0.5)
        );
    }

    /** buzz for teleop enable */
    public static Command teleopEnable(CommandXboxController controller) {
        return Commands.sequence(
            pulse(controller, 0.2, 0.7),
            Commands.waitSeconds(0.1),
            pulse(controller, 0.3, 1.0)
        );
    }

    /** buzz for disable */
    public static Command disable(CommandXboxController controller) {
        return Commands.sequence(
            pulse(controller, 0.5, 0.3),
            Commands.waitSeconds(0.2),
            pulse(controller, 0.3, 0.1)
        );
    }

    // =======================
    // private internal methods
    // =======================

    private static Command pulse(CommandXboxController controller, double seconds, double intensity) {
        return pulseSideSpecific(controller, seconds, intensity, RumbleType.kBothRumble);
    }

    private static Command pulseSideSpecific(CommandXboxController controller, double seconds, double intensity, RumbleType side) {
        return Commands.sequence(
            Commands.runOnce(() -> controller.setRumble(side, intensity)),
            Commands.waitSeconds(seconds),
            Commands.runOnce(() -> controller.setRumble(side, 0.0))
        );
    }
}
