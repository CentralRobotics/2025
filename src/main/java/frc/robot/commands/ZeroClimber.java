package frc.robot.commands;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ZeroClimber extends Command {
    private final Climb climb;

    public ZeroClimber(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize()
    {
        SoftwareLimitSwitchConfigs disable = new SoftwareLimitSwitchConfigs();
        disable.ForwardSoftLimitEnable = false;
        disable.ReverseSoftLimitEnable = false;
        climb.getMotor().getConfigurator().apply(disable);
        climb.getMotor().setControl(new VoltageOut(2));
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return climb.getMotor().getStatorCurrent().getValueAsDouble() > 6.9;
    }

    @Override
    public void end(boolean interrupted)
    {
        climb.setClimb(0);
        SoftwareLimitSwitchConfigs enable = new SoftwareLimitSwitchConfigs();
        enable.ForwardSoftLimitEnable = true;
        enable.ReverseSoftLimitEnable = true;
        enable.ForwardSoftLimitThreshold = ClimbConstants.FORWARD_LIMIT;
        enable.ReverseSoftLimitThreshold = ClimbConstants.REVERSE_LIMIT;
        climb.getMotor().getConfigurator().apply(enable);
        climb.getMotor().setPosition(0);
    }
    
}
