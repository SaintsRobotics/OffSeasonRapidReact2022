package frc.robot;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.LEDCurrent;
import com.revrobotics.ColorSensorV3.LEDPulseFrequency;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Wrapper for the {@link ColorSensorV3} object from RevLib. Most of the
 * javadocs for these methods are the same.
 */
public class REVColorSensorV3 {
	private final MUX m_mux;
	private final MUX.Port m_port;
	private final ColorSensorV3 m_sensor;

	/**
	 * Creates a new {@link REVColorSensorV3}.
	 * 
	 * @param mux  The {@link MUX} that the color sensor is connected to.
	 * @param port The port on the {@link MUX} that the color sensor is connected
	 *             to.
	 */
	public REVColorSensorV3(MUX mux, MUX.Port port) {
		m_mux = mux;
		m_port = port;

		// The sensor needs to be connected to the right port so it can construct
		// properly.
		m_mux.switchToPort(m_port);
		m_sensor = new ColorSensorV3(I2C.Port.kMXP);
	}

	/**
	 * Configure the the IR LED used by the proximity sensor.
	 *
	 * <p>
	 * These settings are only needed for advanced users, the defaults will work
	 * fine for most teams. Consult the APDS-9151 for more information on these
	 * configuration settings and how they will affect proximity sensor
	 * measurements.
	 *
	 * @param freq   The pulse modulation frequency for the proximity sensor LED
	 * @param curr   The pulse current for the proximity sensor LED
	 * @param pulses The number of pulses per measurement of the proximity sensor
	 *               LED (0-255)
	 */
	public void configureProximitySensorLED(LEDPulseFrequency freq, LEDCurrent curr, int pulses) {
		m_mux.switchToPort(m_port);
		m_sensor.configureProximitySensorLED(freq, curr, pulses);
	}

	/**
	 * Configure the proximity sensor.
	 *
	 * <p>
	 * These settings are only needed for advanced users, the defaults will work
	 * fine for most teams. Consult the APDS-9151 for more information on these
	 * configuration settings and how they will affect proximity sensor
	 * measurements.
	 *
	 * @param res  Bit resolution output by the proximity sensor ADC.
	 * @param rate Measurement rate of the proximity sensor
	 */
	public void configureProximitySensor(ProximitySensorResolution res, ProximitySensorMeasurementRate rate) {
		m_mux.switchToPort(m_port);
		m_sensor.configureProximitySensor(res, rate);
	}

	/**
	 * Configure the color sensor.
	 *
	 * <p>
	 * These settings are only needed for advanced users, the defaults will work
	 * fine for most teams. Consult the APDS-9151 for more information on these
	 * configuration settings and how they will affect color sensor measurements.
	 *
	 * @param res  Bit resolution output by the respective light sensor ADCs
	 * @param rate Measurement rate of the light sensor
	 * @param gain Gain factor applied to light sensor (color) outputs
	 */
	public void configureColorSensor(ColorSensorResolution res, ColorSensorMeasurementRate rate, GainFactor gain) {
		m_mux.switchToPort(m_port);
		m_sensor.configureColorSensor(res, rate, gain);
	}

	/**
	 * Get the most likely color. Works best when within 2 inches and perpendicular
	 * to surface of interest.
	 * 
	 * @return Color enum of the most likely color, including unknown if the minimum
	 *         threshold is not met
	 */
	public Color getColor() {
		m_mux.switchToPort(m_port);
		return m_sensor.getColor();
	}

	/**
	 * 
	 * Get the raw proximity value from the sensor ADC (11 bit)
	 * 
	 * @return Proximity measurement value, ranging from 0 to 2047
	 */
	public int getProximity() {
		m_mux.switchToPort(m_port);
		return m_sensor.getProximity();
	}

	/**
	 * Get the raw color value from the red ADC
	 * 
	 * @return Red ADC value
	 */
	public int getRed() {
		m_mux.switchToPort(m_port);
		return m_sensor.getRed();
	}

	/**
	 * Get the raw color value from the green ADC
	 * 
	 * @return Green ADC value
	 */
	public int getGreen() {
		m_mux.switchToPort(m_port);
		return m_sensor.getGreen();
	}

	/**
	 * Get the raw color value from the blue ADC
	 * 
	 * @return Blue ADC value
	 */
	public int getBlue() {
		m_mux.switchToPort(m_port);
		return m_sensor.getBlue();
	}

	/**
	 * Indicates if the device reset. Based on the power on status flag in the
	 * status register. Per the datasheet:
	 *
	 * <p>
	 * Part went through a power-up event, either because the part was turned on or
	 * because there was power supply voltage disturbance (default at first register
	 * read).
	 *
	 * <p>
	 * This flag is self clearing
	 *
	 * @return true if the device was reset
	 */
	public boolean hasReset() {
		m_mux.switchToPort(m_port);
		return m_sensor.hasReset();
	}

	/**
	 * Indicates if the device can currently be communicated with.
	 * 
	 * @return True if yes, false if no
	 */
	public boolean isConnected() {
		m_mux.switchToPort(m_port);
		return m_sensor.isConnected();
	}
}
