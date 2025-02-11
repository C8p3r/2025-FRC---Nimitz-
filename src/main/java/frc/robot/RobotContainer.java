package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


import frc.robot.commands.AlignCommand;
import frc.robot.commands.TimedAlignCommand;
import frc.robot.commands.TestDriveCommand;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.mechanisms.GrappleLaserSubsystem;
import frc.robot.subsystems.mechanisms.OperatorSystem;
import frc.robot.subsystems.mechanisms.DeepClimbSubsystem;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;
import frc.robot.subsystems.mechanisms.EndEffectorSubsystem;
import frc.robot.subsystems.LEDSubsystem; 

public class RobotContainer {

    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();
    private final DeepClimbSubsystem m_deepClimbSubsystem = new DeepClimbSubsystem();
    private final GrappleLaserSubsystem m_grappleLaserSubsystem = new GrappleLaserSubsystem(); // THRESHOLD IS IN mystery grapple unit
    private final LEDSubsystem m_ledSubsystem = new LEDSubsystem(); 

    private final OperatorSystem m_operatorSystem = new OperatorSystem(m_elevatorSubsystem, m_endEffectorSubsystem, m_grappleLaserSubsystem, m_deepClimbSubsystem, m_ledSubsystem);

    private double gearShift = 0.5; // Driver Speed Limiter. Limits max speed, but prevents tips and hard hits

    private final SendableChooser<Command> autoChooser;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)*0.6;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

   // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(Constants.OperatorConstants.kOperatorControllerPort);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Command alignCommand = new AlignCommand(m_visionSubsystem, drivetrain, 0.35, 0);
    private final Command timedAlignCommand = new TimedAlignCommand(m_visionSubsystem, drivetrain, 0.35, 0, 1.5);
    
    private final Command adjustLeft = new TestDriveCommand(drivetrain, 0.75, 0.5);
    private final Command adjustRight = new TestDriveCommand(drivetrain, -0.75,0.5);
    
    public RobotContainer() {

        NamedCommands.registerCommand("L1",Commands.runOnce(m_operatorSystem::setL1, m_operatorSystem));
        NamedCommands.registerCommand("L2",Commands.runOnce(m_operatorSystem::setL2, m_operatorSystem));
        NamedCommands.registerCommand("L3",Commands.runOnce(m_operatorSystem::setL3, m_operatorSystem));
        NamedCommands.registerCommand("L4",Commands.runOnce(m_operatorSystem::setL4, m_operatorSystem));

        NamedCommands.registerCommand("L1left", L1left());
        NamedCommands.registerCommand("L1right", L1right());
        NamedCommands.registerCommand("L2left", L2left());
        NamedCommands.registerCommand("L2right", L2right());
        NamedCommands.registerCommand("L3left", L3left());
        NamedCommands.registerCommand("L3right", L3right());
        NamedCommands.registerCommand("L4left", L4left());
        NamedCommands.registerCommand("L4right", L4right());

        NamedCommands.registerCommand("Intake",Commands.runOnce(m_operatorSystem::intake, m_operatorSystem));
        NamedCommands.registerCommand("Stow",Commands.runOnce(m_operatorSystem::stow, m_operatorSystem));

        NamedCommands.registerCommand("Adjust Left", adjustLeft);
        NamedCommands.registerCommand("Adjust Right", adjustRight);

        NamedCommands.registerCommand("Align", alignCommand);

        NamedCommands.registerCommand("TimedAlign", timedAlignCommand);

        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
    
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                if (!alignCommand.isScheduled()) {
                    return drive
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed * gearShift) // SPEED REDUCTION HERE (div)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed * gearShift)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
                }
                return drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
            })
        );
    
        // DRIVER CONTROLS
    
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    
        joystick.leftTrigger().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
    
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> {
            double x = -joystick.getLeftY();
            double y = -joystick.getLeftX();
            if (Math.abs(x) < 1e-6 && Math.abs(y) < 1e-6) {
                return new SwerveRequest.PointWheelsAt().withModuleDirection(Rotation2d.fromDegrees(0)); // Default direction
            }
            return new SwerveRequest.PointWheelsAt().withModuleDirection(new Rotation2d(x, y));
        }));
    
        joystick.rightTrigger().whileTrue(alignCommand);

        // NOTE FOR ALLIGNMENT COMMAND - it is optomized for the reef, so it may not work for other heights
        // ADDTIONALY, it is set up to run until it is released, so use it breifly and at short range

        // OPERATOR CONTROLS
    
        operatorController.start().onTrue(Commands.runOnce(m_operatorSystem::intake, m_operatorSystem));
        operatorController.back().onTrue(Commands.runOnce(m_operatorSystem::stow, m_operatorSystem));
    
// Right Reef 
operatorController.y().onTrue(L1right());
operatorController.b().onTrue(L2right());
operatorController.a().onTrue(L3right());
operatorController.x().onTrue(L4right());

// Left Reef
operatorController.povUp().onTrue(L1left());
operatorController.povRight().onTrue(L2left());
operatorController.povDown().onTrue(L3left());
operatorController.povLeft().onTrue(L4left());
       
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command L1left() { return new ParallelCommandGroup(new TestDriveCommand(drivetrain, 0.75, 0.5), new InstantCommand(() -> m_operatorSystem.setL1(), m_operatorSystem)); }
    public Command L1right() { return new ParallelCommandGroup(new TestDriveCommand(drivetrain, -0.75, 0.5), new InstantCommand(() -> m_operatorSystem.setL1(), m_operatorSystem)); }
    public Command L2left() { return new ParallelCommandGroup(new TestDriveCommand(drivetrain, 0.75, 0.5), new InstantCommand(() -> m_operatorSystem.setL2(), m_operatorSystem)); }
    public Command L2right() { return new ParallelCommandGroup(new TestDriveCommand(drivetrain, -0.75, 0.5), new InstantCommand(() -> m_operatorSystem.setL2(), m_operatorSystem)); }
    public Command L3left() { return new ParallelCommandGroup(new TestDriveCommand(drivetrain, 0.75, 0.5), new InstantCommand(() -> m_operatorSystem.setL3(), m_operatorSystem)); }
    public Command L3right() { return new ParallelCommandGroup(new TestDriveCommand(drivetrain, -0.75, 0.5), new InstantCommand(() -> m_operatorSystem.setL3(), m_operatorSystem)); }
    public Command L4left() { return new ParallelCommandGroup(new TestDriveCommand(drivetrain, 0.75, 0.5), new InstantCommand(() -> m_operatorSystem.setL4(), m_operatorSystem)); }
    public Command L4right() { return new ParallelCommandGroup(new TestDriveCommand(drivetrain, -0.75, 0.5), new InstantCommand(() -> m_operatorSystem.setL4(), m_operatorSystem)); }

}
