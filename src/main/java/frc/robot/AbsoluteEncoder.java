package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;

public class AbsoluteEncoder {
  private AnalogInput m_analogIn;
  private AnalogEncoder m_analogEncoder;

  private double m_offset;

  public AbsoluteEncoder(int channel, double offset) {
    m_analogIn = new AnalogInput(channel);
    m_analogEncoder = new AnalogEncoder(m_analogIn);

    m_offset = offset;
  }

  /**
   * Returns the angle as a {@link Rotation2d}.
   * 
   * @todo check if this is equal to the 0
   * @return The angle as a {@link Rotation2d}.
   */
  public Rotation2d getRotation2d() {
    return new Rotation2d(((m_analogEncoder.get() % 5 / 5) * 2 * Math.PI) - m_offset);
  }
}
