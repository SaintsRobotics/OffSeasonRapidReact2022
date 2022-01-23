import static org.junit.Assert.assertEquals;
import frc.robot.Utils;

import org.junit.*;

/** Tests for all the utility functions. */
public class UtilityTest {
    public static final double DELTA = 1e-2; // acceptable deviation range
    @Test
    public void deadZonesTest() {
        double deadZonedResultOne = Utils.deadZone(10, 15);
        double deadZonedExpectedOne = 0;
        assertEquals(deadZonedExpectedOne, deadZonedResultOne, DELTA);
        double deadZonedResultTwo = Utils.deadZone(10, 7);
        double deadZonedExpectedTwo = 10;
        assertEquals(deadZonedExpectedTwo, deadZonedResultTwo, DELTA);
    }

  @Test
  public void deadZonesTest() {
    double deadZonedResultOne = Utils.deadZones(10, 15);
    double deadZonedExpectedOne = 0;
    assertEquals(deadZonedExpectedOne, deadZonedResultOne, DELTA);
    double deadZonedResultTwo = Utils.deadZones(10, 7);
    double deadZonedExpectedTwo = 10;
    assertEquals(deadZonedExpectedTwo, deadZonedResultTwo, DELTA);
  }

  @Test
  public void toleranceTest() {
    double toleranceResultOne = Utils.tolerance(0.5, 1, 1);
    double toleranceExpectedOne = 1;
    assertEquals(toleranceExpectedOne, toleranceResultOne, DELTA);
    double toleranceResultTwo = Utils.tolerance(15, 10, 0.5);
    double toleranceExpectedTwo = 15;
    assertEquals(toleranceExpectedTwo, toleranceResultTwo, DELTA);
  }

  @Test
  public void oddSquareTest() {
    double oddSquareResultOne = Utils.oddSquare(4);
    double oddSquareExpectedOne = 16;
    assertEquals(oddSquareExpectedOne, oddSquareResultOne, DELTA);
    double oddSquareResultTwo = Utils.oddSquare(-4);
    double oddSquareExpectedTwo = -16;
    assertEquals(oddSquareExpectedTwo, oddSquareResultTwo, DELTA);
  }

  @Test
  public void normalizeAngleTest() {
    double normalizeAngleResultOne = Utils.normalizeAngle(120, 360);
    double normalizeAngleExpectedOne = 120;
    assertEquals(normalizeAngleResultOne, normalizeAngleExpectedOne, DELTA);
    double normalizeAngleResultTwo = Utils.normalizeAngle(400, 360);
    double normalizeAngleExpectedTwo = 40;
    assertEquals(normalizeAngleResultTwo, normalizeAngleExpectedTwo, DELTA);
  }
}
