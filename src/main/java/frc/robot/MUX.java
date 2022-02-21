package frc.robot;

import edu.wpi.first.wpilibj.I2C;

/**
 * Wrapper for multiplexer plugged into the {@link I2C} port on the roboRIO.
 * There should only be one instance
 */
public class MUX {
	private final I2C m_I2C = new I2C(I2C.Port.kMXP, 0x70);

	/** Represents a port for the mux as a single byte array. */
	public enum Port {
		kOne(new byte[] { (byte) 1 }),
		kTwo(new byte[] { (byte) 2 }),
		kThree(new byte[] { (byte) 4 });

		public final byte[] value;

		Port(byte[] value) {
			this.value = value;
		}
	}

	/**
	 * Creates a new {@link MUX}. There should only be one instance.
	 */
	public MUX() {
	}

	public void switchToPort(MUX.Port port) {
		m_I2C.writeBulk(port.value);
	}
}
