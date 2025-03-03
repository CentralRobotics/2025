package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {

    PneumaticHub m_pH = new PneumaticHub(1);
    Solenoid m_solenoidClimb = m_pH.makeSolenoid(1);
    Solenoid m_solenoidRatchet = m_pH.makeSolenoid(2);

    public Pneumatics() {
        m_pH.enableCompressorAnalog(100,120);
        retractClimb();
        disengageRatchet();
    }

    public Command extendClimb() {
        return run(() -> m_solenoidClimb.set(true));
    }
    
    public Command retractClimb() {
        return run(() -> m_solenoidClimb.set(false));
    }
    
    public Command engageRatchet() {
        return run(() -> m_solenoidRatchet.set(true));
    }
    
    public Command disengageRatchet() {
        return run(() -> m_solenoidRatchet.set(false));
    }

    @Override
    public void periodic() {
    }
}
