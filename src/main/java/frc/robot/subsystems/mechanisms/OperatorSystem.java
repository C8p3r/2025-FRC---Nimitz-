package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


public class OperatorSystem extends SubsystemBase {
    private final ElevatorSubsystem elevatorSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final GrappleLaserSubsystem grappleLaserSubsystem;
    private final DeepClimbSubsystem deepClimbSubsystem;
    private final LEDSubsystem ledSubsystem;


    public OperatorSystem(
            ElevatorSubsystem elevatorSubsystem,
            EndEffectorSubsystem endEffectorSubsystem,
            GrappleLaserSubsystem grappleLaserSubsystem,
            DeepClimbSubsystem deepClimbSubsystem,
            LEDSubsystem ledSubsystem
    ) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.grappleLaserSubsystem = grappleLaserSubsystem;
        this.deepClimbSubsystem = deepClimbSubsystem;
        this.ledSubsystem = ledSubsystem;
    }

    public void climb() {
        deepClimbSubsystem.climb();
    }

    public void stopClimb() {
        deepClimbSubsystem.stopClimb();
    }

    public void intake() {
        CommandScheduler.getInstance().schedule(
            new ParallelCommandGroup(
                // Strobe LED continuously
                new RunCommand(() -> ledSubsystem.patriot(), ledSubsystem),
    
                // Sequential commands for intake process
                new SequentialCommandGroup(
                    // Move elevator to intake position
                    new InstantCommand(() -> {
                        System.out.println("Moving elevator to intake position");
                        elevatorSubsystem.setPosition(elevatorSubsystem.getIntakeA());
                    }),
    
                    // Start coral motors
                    new InstantCommand(() -> {
                        System.out.println("Starting coral motors");
                        endEffectorSubsystem.setCoralVelocity(Constants.CoralEffectorConstants.intakeVelocity);
                    }),
    
                    // Wait until coral is detected
                    new WaitUntilCommand(() -> grappleLaserSubsystem.isCoralInDetected()),
    
                    // Slow down coral motors
                    new InstantCommand(() -> {
                        System.out.println("Slowing coral motors");
                        endEffectorSubsystem.setCoralVelocity(Constants.CoralEffectorConstants.slowintakeVelocity);
                    }),
    
                    // Wait until coral is not detected
                    new WaitUntilCommand(() -> !grappleLaserSubsystem.isCoralInDetected()),
    
                    // Reverse coral motors
                    new InstantCommand(() -> {
                        System.out.println("Reversing coral motors");
                        endEffectorSubsystem.setCoralVelocity(Constants.CoralEffectorConstants.retakeCoral);
                    }),
    
                    // Wait until coral is detected again
                    new WaitUntilCommand(() -> grappleLaserSubsystem.isCoralInDetected()),
    
                    // Delay for retake coral
                    new WaitCommand(Constants.CoralEffectorConstants.retakeCoralDelay),
    
                    // Stop coral motors
                    new InstantCommand(() -> {
                        System.out.println("Stopping coral motors");
                        endEffectorSubsystem.stopCoral();
                    }),

                    new InstantCommand(() -> {
                        System.out.println("Moving elevator to intake position");
                        elevatorSubsystem.setPosition(elevatorSubsystem.getIntakeA()-2);
                    }),

                    // Strobe LED white
                    new RunCommand(() -> ledSubsystem.strobe(Color.kLavenderBlush, 0.05))
                )
            )
        );
    }
    
    

    public void setL1() {
        CommandScheduler.getInstance().schedule(
            new ParallelCommandGroup(
                // Strobe LED continuously
                new RunCommand(() -> ledSubsystem.reefPose(Color.kYellow, Color.kYellowGreen), ledSubsystem),
    
                // Sequential commands for L1 process
                new SequentialCommandGroup(
                    // Move elevator to L1 position
                    new InstantCommand(() -> {
                        System.out.println("Moving elevator to L1 position");
                        elevatorSubsystem.setPosition(elevatorSubsystem.getL1());
                    }),
    
                    // Wait for elevator to reach L1 position
                    new WaitUntilCommand(() -> {
                        boolean atPosition = elevatorSubsystem.isAtPosition(elevatorSubsystem.getL1());
                        System.out.println("Elevator at L1 position: " + atPosition);
                        return atPosition;
                    }),
    
                    // Start coral motors
                    new InstantCommand(() -> {
                        System.out.println("Starting coral motors");
                        endEffectorSubsystem.setCoralDifferentialVelocity(Constants.CoralEffectorConstants.L1intakeVelocityLeft, Constants.CoralEffectorConstants.L1intakeVelocityRight);
                    }),
    
                    // Wait for OUT sensor to be FALSE
                    new WaitUntilCommand(() -> {
                        boolean notDetected = !grappleLaserSubsystem.isCoralOutDetected();
                        return notDetected;
                    }),
    
                    // Stop coral motors
                    new InstantCommand(() -> {
                        System.out.println("Stopping coral motors");
                        endEffectorSubsystem.stopCoral();
                    }),
    
                    // Delay
                    new WaitCommand(0.5),
    
                    // Start intake sequence
                    new InstantCommand(() -> {
                        System.out.println("Starting intake sequence");
                        intake();
                    })
                )
            )
        );
    }
    
    
    
    
    public void setL2() {
        CommandScheduler.getInstance().schedule(
            new ParallelCommandGroup(
                // Strobe LED continuously
                new RunCommand(() -> ledSubsystem.reefPose(Color.kForestGreen, Color.kFloralWhite), ledSubsystem),
    
                // Sequential commands for L2 process
                new SequentialCommandGroup(
                    // // Start coral motors
                    // new InstantCommand(() -> {
                    //     System.out.println("Starting coral motors");
                    //     endEffectorSubsystem.setCoralVelocity(Constants.CoralEffectorConstants.slowintakeVelocity);
                    // }),
    
                    // // Delay
                    // new WaitCommand(Constants.CoralEffectorConstants.retakeCoralDelay / 2),
    
                    // Stop coral motors
                    new InstantCommand(() -> {
                        System.out.println("Stopping coral motors");
                        endEffectorSubsystem.stopCoral();
                    }),
    
                    // Move elevator to L2 position
                    new InstantCommand(() -> {
                        System.out.println("Moving elevator to L2 position");
                        elevatorSubsystem.setPosition(elevatorSubsystem.getL2());
                    }),
    
                    // Wait until elevator reaches position
                    new WaitUntilCommand(() -> {
                        boolean atPosition = elevatorSubsystem.isAtPosition(elevatorSubsystem.getL2());
                        System.out.println("Elevator at L2 position: " + atPosition);
                        return atPosition;
                    }),
    
                    // Run coral motors for deposit
                    new InstantCommand(() -> {
                        System.out.println("Starting coral motors");
                        endEffectorSubsystem.setCoralVelocity(Constants.CoralEffectorConstants.intakeVelocity);
                    }),
    
                    // Wait until OUT sensor is FALSE
                    new WaitUntilCommand(() -> {
                        boolean notDetected = !grappleLaserSubsystem.isCoralOutDetected();
                        return notDetected;
                    }),
    
                    // Stop coral motors
                    new InstantCommand(() -> {
                        System.out.println("Stopping coral motors");
                        endEffectorSubsystem.stopCoral();
                    }),
    
                    // Start intake sequence
                    new InstantCommand(() -> {
                        System.out.println("Starting intake sequence");
                        intake();
                    })
                )
            )
        );
    }
    

    public void setL3() {
        CommandScheduler.getInstance().schedule(
            new ParallelCommandGroup(
                // Strobe LED continuously
                new RunCommand(() -> ledSubsystem.reefPose(Color.kIndianRed, Color.kDarkRed), ledSubsystem),
    
                // Sequential commands for L3 process
                new SequentialCommandGroup(
                    // Start coral motors
                    // new InstantCommand(() -> {
                    //     System.out.println("Starting coral motors");
                    //     endEffectorSubsystem.setCoralVelocity(Constants.CoralEffectorConstants.slowintakeVelocity);
                    // }),
    
                    // // Delay
                    // new WaitCommand(Constants.CoralEffectorConstants.retakeCoralDelay / 2),
    
                    // Stop coral motors
                    new InstantCommand(() -> {
                        System.out.println("Stopping coral motors");
                        endEffectorSubsystem.stopCoral();
                    }),
    
                    // Move elevator to L3 position
                    new InstantCommand(() -> {
                        System.out.println("Moving elevator to L3 position");
                        elevatorSubsystem.setPosition(elevatorSubsystem.getL3());
                    }),
    
                    // Wait until elevator reaches position
                    new WaitUntilCommand(() -> {
                        boolean atPosition = elevatorSubsystem.isAtPosition(elevatorSubsystem.getL3());
                        System.out.println("Elevator at L3 position: " + atPosition);
                        return atPosition;
                    }),
    
                    // Run coral motors for deposit
                    new InstantCommand(() -> {
                        System.out.println("Starting coral motors");
                        endEffectorSubsystem.setCoralVelocity(Constants.CoralEffectorConstants.intakeVelocity);
                    }),
    
                    // Wait until OUT sensor is FALSE
                    new WaitUntilCommand(() -> {
                        boolean notDetected = !grappleLaserSubsystem.isCoralOutDetected();
                        return notDetected;
                    }),
    
                    // Stop coral motors
                    new InstantCommand(() -> {
                        System.out.println("Stopping coral motors");
                        endEffectorSubsystem.stopCoral();
                    }),
    
                    // Start intake sequence
                    new InstantCommand(() -> {
                        System.out.println("Starting intake sequence");
                        intake();
                    })
                )
            )
        );
    }
    

    public void setL4() {
        CommandScheduler.getInstance().schedule(
            new ParallelCommandGroup(
                // Strobe LED continuously
                new RunCommand(() -> ledSubsystem.reefPose(Color.kBlueViolet, Color.kCyan), ledSubsystem),
    
                // Sequential commands for L4 process
                new SequentialCommandGroup(
                    // Start coral motors
                    new InstantCommand(() -> {
                        System.out.println("Starting coral motors");
                        endEffectorSubsystem.setCoralVelocity(Constants.CoralEffectorConstants.slowintakeVelocity);
                    }),
    
                    // Delay
                    new WaitCommand(Constants.CoralEffectorConstants.retakeCoralDelay / 2),
    
                    // Stop coral motors
                    new InstantCommand(() -> {
                        System.out.println("Stopping coral motors");
                        endEffectorSubsystem.stopCoral();
                    }),
    
                    // Move elevator to L4 position
                    new InstantCommand(() -> {
                        System.out.println("Moving elevator to L4 position");
                        elevatorSubsystem.setPosition(elevatorSubsystem.getL4());
                    }),
    
                    // Wait until elevator reaches position
                    new WaitUntilCommand(() -> {
                        boolean atPosition = elevatorSubsystem.isAtPosition(elevatorSubsystem.getL4());
                        System.out.println("Elevator at L4 position: " + atPosition);
                        return atPosition;
                    }),
    
                    // Run coral motors for deposit
                    new InstantCommand(() -> {
                        System.out.println("Starting coral motors");
                        endEffectorSubsystem.setCoralVelocity(Constants.CoralEffectorConstants.intakeVelocity);
                    }),
    
                    // Wait until OUT sensor is FALSE
                    new WaitUntilCommand(() -> {
                        boolean notDetected = !grappleLaserSubsystem.isCoralOutDetected();
                        return notDetected;
                    }),
    
                    // Stop coral motors
                    new InstantCommand(() -> {
                        System.out.println("Stopping coral motors");
                        endEffectorSubsystem.stopCoral();
                    }),
    
                    // Start intake sequence
                    new InstantCommand(() -> {
                        System.out.println("Starting intake sequence");
                        intake();
                    })
                )
            )
        );
    }
    

    public void stow() {
        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    System.out.println("Stowing");
                    elevatorSubsystem.setPosition(0);
                    endEffectorSubsystem.setCoralVelocity(0);
                })),
                new RunCommand(() -> {
                    ledSubsystem.scrollingRainbow();
                }, ledSubsystem)    
        );
    }

}

