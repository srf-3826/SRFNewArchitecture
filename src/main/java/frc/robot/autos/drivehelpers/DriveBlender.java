package frc.robot.autos.drivehelpers;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

// This helper class blends auto-drive assist chassis speeds with manually (joystick) generated chassis speeds
public class DriveBlender {
    private double          m_autoFactor;
    private ChassisSpeeds   m_finalSpeeds;

    public DriveBlender(double percentAutoBlend) {
       this.setAutoAssistBlend(percentAutoBlend);
    }
    
    public DriveBlender() {
        this.setAutoAssistBlend(0.65);              // Default is 65% auto, 35% manual
    }

    public ChassisSpeeds blend(ChassisSpeeds manual, ChassisSpeeds auto) {
        double manualFactor = 1.0 - m_autoFactor;
        m_finalSpeeds.vxMetersPerSecond = auto.vxMetersPerSecond * m_autoFactor + manual.vxMetersPerSecond * manualFactor;
        m_finalSpeeds.vyMetersPerSecond = auto.vyMetersPerSecond * m_autoFactor + manual.vyMetersPerSecond * manualFactor;
        m_finalSpeeds.omegaRadiansPerSecond = auto.omegaRadiansPerSecond * m_autoFactor + manual.omegaRadiansPerSecond * manualFactor;
        return m_finalSpeeds;
    }

    public void setAutoAssistBlend(double percentAutoBlend) {
        if (percentAutoBlend > 0.0 && percentAutoBlend <= 1.00) {
            m_autoFactor = percentAutoBlend;
        } else {
            m_autoFactor = 0.7;
            System.out.println("Invalid argument for setAutoAssistBlend(): "+percentAutoBlend+". Using default: "+m_autoFactor);
        }
    }
}