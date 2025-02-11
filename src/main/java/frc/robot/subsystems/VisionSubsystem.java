package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    public VisionSubsystem() {
        // Initialize the Limelight here
    }

    public double getTargetAngle() {
        return NetworkTableInstance.getDefault().getTable("limelight-b").getEntry("tx").getDouble(0.0);
    }

    public double getTargetTY() {
        return NetworkTableInstance.getDefault().getTable("limelight-b").getEntry("ty").getDouble(0.0);
    }

    public double getTargetTX() {
        return NetworkTableInstance.getDefault().getTable("limelight-b").getEntry("tx").getDouble(0.0);
    }

    public double[] get3DPose() {
        return NetworkTableInstance.getDefault().getTable("limelight-b").getEntry("camtran").getDoubleArray(new double[6]);
    }
}


