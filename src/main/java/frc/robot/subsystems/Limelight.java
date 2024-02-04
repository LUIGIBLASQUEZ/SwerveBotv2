package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    // Initialize a Network Table Instance
    private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private static NetworkTable table = tableInstance.getTable("limelight");

    // Network Table Entry Values
    private static NetworkTableEntry timingEntry = table.getEntry("TIMING_TEST_ENTRY");
    private static boolean timingEntryValue = false;
    boolean timingTestEntryValue2 = !timingEntryValue;

    // Network Tables update ever 0.1 seconds
    public static final long updateTime = 100_000;

    // Send the data to Smart Dashboard
    public static final boolean postSmartDashboard = true;

    // Limelight by default does not have a valid target
    public boolean m_LimelightHasValidTarget = false;

    // Do not change FOR NOW
    public double m_LimelightSteerCommand = 0.0;
    public double m_LimelightTargetArea = 0;

    static long currentTime = timingEntry.getLastChange();

    // Pipeline latency in ms (tl should be latency variable)
    public static final double IMAGE_CAPTURE_LATENCY = 11;
    private static NetworkTableEntry latencyEntry = table.getEntry("tl");
    
    // Static latency trackers
    static long lastUpdate = latencyEntry.getLastChange();
    static long timeDifference = currentTime - lastUpdate;
    static boolean connected = timeDifference < updateTime;

    // Latency sender method
    public static double getLatencyMs() {
        return latencyEntry.getDouble(0) + IMAGE_CAPTURE_LATENCY;
    }

    /*
     * Is Connected? Method for sending Limelight to Smart Dashboard
     * If anything should be initialized at startup you can put it here!
     * 
     */
    public static boolean isConnected() {
        if (postSmartDashboard) {
            SmartDashboard.putBoolean("Limelight Connected", connected);
                //SmartDashboard.putNumberConnection("Limelight Time Difference", timeDifference);
            SmartDashboard.putNumber("Limelight Time Difference", timeDifference);
        }
        return connected;
    }

    /*
     * hasAnyTarget method determines if there is any valid target available
     * variable "tv" is for the target's entry
     * 
     * 
     */
    private static NetworkTableEntry validTargetEntry = table.getEntry("tv");
    public static boolean hasAnyTarget() {
        boolean validTarget = validTargetEntry.getDouble(0) > 0.5;

        if (postSmartDashboard) {
            SmartDashboard.putBoolean("Valid Target", validTarget);
        }
        return validTarget;
    }

    // Boolean statement determining if the Limelight has a valid target
    // Is this boilerplate? It might be
    public static boolean hasValidTarget() {
        return hasAnyTarget();
    }

    /*
     * Offsets, range, and skew values for detected objects
     * tx, ty, ta, ts
     */

    // Horizontal offsets
    public static final double MIN_X_ANGLE = -27;
    public static final double MAX_X_ANGLE = 27;
    public static final double X_ANGLE_SHIFT = -1.5;
    private static NetworkTableEntry xAngleEntry = table.getEntry("tx");

    // Vertical offsets
    public static final double MIN_Y_ANGLE = -20.5;
    public static final double MAX_Y_ANGLE = 20.5;
    private static NetworkTableEntry yAngleEntry = table.getEntry("ty");

    // 0-100% value of the distance from the target to the limelight
    // (basically a range value)
    public static final double MIN_TARGET_AREA = 0;
    public static final double MAX_TARGET_AREA = 1;
    private static NetworkTableEntry targetAreaEntry = table.getEntry("ta");

    // 
    public static final double MIN_SKEW = -90;
    public static final double MAX_SKEW = 0;
    private static NetworkTableEntry targetSkewEntry = table.getEntry("ts");

    // Methods to get angle offsets (x,y), range value, and skew
    // Horiz.
    public static double getTargetXAngle() {
        double X_SHIFT = SmartDashboard.getNumber("X_SHIFT", 1000);
        if(X_SHIFT > 694) SmartDashboard.putNumber("X_SHIFT", X_ANGLE_SHIFT);
        return xAngleEntry.getDouble(0) + X_SHIFT;
    }
    // Vert.
    public static double getTargetYAngle() {
        return yAngleEntry.getDouble(0);
    }
    // Area
    public static double getTargetArea() {
        return Math.min(targetAreaEntry.getDouble(0) / 100.0, 1);
    }
    // Skew
    public static double getTargetSkew() {
        return targetSkewEntry.getDouble(0);
    }

    /*
     * Limelight Tracking command
     * updates network table
     * 
     */
    public void updateLimelight(double limeRot)
    {    
          //double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
          //double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
          m_LimelightTargetArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  
          if (hasValidTarget() == true)
          {
            m_LimelightHasValidTarget = true;
            m_LimelightSteerCommand = 0.0;
          }
    }
}
