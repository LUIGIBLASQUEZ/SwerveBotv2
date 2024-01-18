package frc.robot.subsystems;

// Imports
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {
    
    // Creates new NavX
    AHRS navX = new AHRS(SPI.Port.kMXP);

    public static NavX navx;

    public NavX() {
        navX.calibrate();
    }

    // This method will be called once per scheduler run
    public void periodic() {
    }

    // Grabs the yaw
    public float getYaw() {
        return -navX.getYaw();
    }

    // like yaw
    public float getFusedHeading() {
        return navX.getFusedHeading();
    }

    // The Rotation2D is the big brother of fused heading
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(navX.getYaw());
    }

    // Grabs the roll
    public float getRoll() {
        return navX.getRoll();
    }

    // Grabs the pitch
    public float getPitch() {
        return navX.getPitch();
    }

    // Gets the x displacement
    public float getDisplacementX() {
        return navX.getDisplacementX();
    }

    // Gets the y displacement
    public float getDisplacementY() {
        return navX.getDisplacementY();
    }

    // Gets the z displacement
    public float getDisplacementZ() {
        return navX.getDisplacementZ();
    }

    // Gets the current degrees
    public float getCompassHeading() {
        return navX.getCompassHeading();
    }

    // Resets the NavX's yaw axis to zero
    public void resetYaw() {
        navX.zeroYaw();
    }

    public void resetDisplacement() {
        resetDisplacement();
    }
}