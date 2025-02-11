package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

//import com.ctre.phoenix6.controls.PercentOutput;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;

public class EndEffectorSubsystem extends SubsystemBase {

    private final TalonFX coralLeft;
    private final TalonFX coralRight;
    private final TalonFX algaePosition;
    private final TalonFX algaeRollers;

    public EndEffectorSubsystem() {
        coralLeft = new TalonFX(Constants.MechanicanismCANids.coralLeftID, "rio");
        coralRight = new TalonFX(Constants.MechanicanismCANids.coralRightID, "rio");
        algaePosition = new TalonFX(Constants.MechanicanismCANids.algaePositionID, "rio");
        algaeRollers = new TalonFX(Constants.MechanicanismCANids.algaeRotationID, "rio");

        // Configure the PID constants for velocity control
        TalonFXConfiguration coralConfig = new TalonFXConfiguration();
        Slot0Configs slot0 = coralConfig.Slot0;
        slot0.kP = 0.01; // Proportional gain
        slot0.kI = 0.0; // Integral gain
        slot0.kD = 0.0; // Derivative gain

        coralLeft.getConfigurator().apply(coralConfig);
        coralRight.getConfigurator().apply(coralConfig);

        // Configure the inversions
        coralConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Invert the left motor
        coralLeft.getConfigurator().apply(coralConfig);

        coralConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Do not invert the right motor
        coralRight.getConfigurator().apply(coralConfig);
    }

    public void setCoralVelocity(double velocity) {
       // System.out.println("Setting coral velocity to: " + velocity);
        coralLeft.setControl(new VelocityDutyCycle(velocity));
        coralRight.setControl(new VelocityDutyCycle(velocity));
    }

    public void setCoralDifferentialVelocity(double leftVelocity, double rightVelocity) {
    //    System.out.println("Setting coral left velocity to: " + leftVelocity + " and right velocity to: " + rightVelocity);
        coralLeft.setControl(new VelocityDutyCycle(leftVelocity));
        coralRight.setControl(new VelocityDutyCycle(rightVelocity));
    }

    public void stopCoral() {
      //  System.out.println("Stopping coral motors");
        coralLeft.setControl(new VelocityDutyCycle(0));
        coralRight.setControl(new VelocityDutyCycle(0));
    }

    // Command methods
    public Command fastIntakeCommand() {
        return new Command() {
            {
                addRequirements(EndEffectorSubsystem.this);
            }

            @Override
            public void initialize() {
                setCoralVelocity(Constants.CoralEffectorConstants.intakeVelocity);
            }

            @Override
            public boolean isFinished() {
                return false; // This command will be interrupted by the next command in the sequence
            }

            @Override
            public void end(boolean interrupted) {
                if (interrupted) {
                    stopCoral();
                }
            }
        };
    }

    public Command slowIntakeCommand() {
        return new Command() {
            {
                addRequirements(EndEffectorSubsystem.this);
            }

            @Override
            public void initialize() {
                setCoralVelocity(Constants.CoralEffectorConstants.slowintakeVelocity);
            }

            @Override
            public boolean isFinished() {
                return false; // This command will be interrupted by the next command in the sequence
            }

            @Override
            public void end(boolean interrupted) {
                if (interrupted) {
                    stopCoral();
                }
            }
        };
    }

    public Command stopIntakeCommand() {
        return new Command() {
            {
                addRequirements(EndEffectorSubsystem.this);
            }

            @Override
            public void initialize() {
                stopCoral();
            }

            @Override
            public boolean isFinished() {
                return true; // This command finishes immediately
            }
        };
    }
}