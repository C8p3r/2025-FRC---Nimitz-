package frc.robot.subsystems.mechanisms;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Map;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevatorLeft;
    private final TalonFX elevatorRight;
    private final MotionMagicVoltage mmReq = new MotionMagicVoltage(0);
    private boolean manualOverride = false;
    private static final double POSITION_TOLERANCE = 0.5; // Tolerance in sensor units

// Constants

        public double getL1() { 
            double position = -17;
            return position;
        }
        
        public double getL2() { 
            double position = -21;
            return position;
        }
        
        public double getL3() { 
            double position = -37;
            return position;
        }
        
        public double getL4() { 
            double position = -60.5;
            return position;
        }
        
        public double getIntakeA() { 
            double position = -12.1;
          
            return position;
        }
    
        public double getIntakeB() {
            return getIntakeA();
        }

    public ElevatorSubsystem() {

        elevatorLeft = new TalonFX(Constants.MechanicanismCANids.elevatorLeftID);
        elevatorRight = new TalonFX(Constants.MechanicanismCANids.elevatorRightID);

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        /* Configure gear ratio */
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1; // rotor rotations per mechanism rotation

        /* Configure Motion Magic */
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(1000)) // 5 (mechanism) rotations per second cruise
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(1000)) // Take approximately 0.5 seconds to reach max vel
          .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(1000)); // Take approximately 0.1 seconds to reach max accel 

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 9.5; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = elevatorLeft.getConfigurator().apply(cfg);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
     //       System.out.println("Could not configure left motor. Error: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = elevatorRight.getConfigurator().apply(cfg);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
      //      System.out.println("Could not configure right motor. Error: " + status.toString());
        }

        elevatorRight.setControl(new Follower(Constants.MechanicanismCANids.elevatorLeftID, true));

    }


    public static class ElevatorConstants {
        private static final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
        private static final ShuffleboardLayout positionsLayout = tab
            .getLayout("Elevator Positions", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0);
    }

    public void setPosition(double position) {
        if (!manualOverride) {
            elevatorLeft.setControl(mmReq.withPosition(position).withSlot(0));
        }
    }

    public void setManualOverride(boolean override) {
        manualOverride = override;
    }

    public double getPosition() {
        return elevatorLeft.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition(double targetPosition) {
        double currentPosition = getPosition();
        return Math.abs(currentPosition - targetPosition) <= POSITION_TOLERANCE;
    }

    @Override
    public void periodic() {
           // Update displays with current positions
           getL1();
           getL2();
           getL3();
           getL4();
           getIntakeA();
    }
}