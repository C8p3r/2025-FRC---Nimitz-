package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class GrappleLaserSubsystem extends SubsystemBase {
    private final LaserCan inLaser;
    private final LaserCan outLaser;
    private boolean coralInDetected = false;
    private boolean coralOutDetected = false;
    private final double threshold = 40.0; // Example threshold value in mm

    private final NetworkTableEntry inLaserDistanceEntry;
    private final NetworkTableEntry outLaserDistanceEntry;
    private final NetworkTableEntry coralInDetectedEntry;
    private final NetworkTableEntry coralOutDetectedEntry;

    public GrappleLaserSubsystem() {
        inLaser = new LaserCan(Constants.MechanicanismCANids.inLaserCANID);
        outLaser = new LaserCan(Constants.MechanicanismCANids.outLaserCANID);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inLaserDistanceEntry = inst.getTable("GrappleLaser").getEntry("InLaserDistance");
        outLaserDistanceEntry = inst.getTable("GrappleLaser").getEntry("OutLaserDistance");
        coralInDetectedEntry = inst.getTable("GrappleLaser").getEntry("CoralInDetected");
        coralOutDetectedEntry = inst.getTable("GrappleLaser").getEntry("CoralOutDetected");

        try {
            inLaser.setRangingMode(LaserCan.RangingMode.SHORT);
            inLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 8, 8)); // Tighter region of interest
            inLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS); // Reduced timing budget

            outLaser.setRangingMode(LaserCan.RangingMode.SHORT);
            outLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 8, 8)); // Tighter region of interest
            outLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS); // Reduced timing budget
        } catch (ConfigurationFailedException e) {
           // System.out.println("Configuration failed! " + e);
        }
    }

    @Override
    public void periodic() {
        LaserCan.Measurement inMeasurement = inLaser.getMeasurement();
        if (inMeasurement != null) {
           // System.out.println("IN Laser Distance: " + inMeasurement.distance_mm + " mm, Status: " + inMeasurement.status);
            coralInDetected = inMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && inMeasurement.distance_mm < threshold;
            inLaserDistanceEntry.setDouble(inMeasurement.distance_mm);
        } else {
            //System.out.println("IN Laser Measurement is null");
            inLaserDistanceEntry.setDouble(0.0);
            coralInDetected = false;
        }

        LaserCan.Measurement outMeasurement = outLaser.getMeasurement();
        if (outMeasurement != null) {
         //   System.out.println("OUT Laser Distance: " + outMeasurement.distance_mm + " mm, Status: " + outMeasurement.status);
            coralOutDetected = outMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && outMeasurement.distance_mm < threshold;
            outLaserDistanceEntry.setDouble(outMeasurement.distance_mm);
        } else {
         //   System.out.println("OUT Laser Measurement is null");
            outLaserDistanceEntry.setDouble(0.0);
            coralOutDetected = false;
        }

        // Update Shuffleboard entries
        coralInDetectedEntry.setBoolean(coralInDetected);
        coralOutDetectedEntry.setBoolean(coralOutDetected);

      //  System.out.println("Coral IN Detected: " + coralInDetected);
      //  System.out.println("Coral OUT Detected: " + coralOutDetected);
    }

    public boolean isCoralInDetected() {
        return coralInDetected;
    }

    public boolean isCoralOutDetected() {
        return coralOutDetected;
    }
}