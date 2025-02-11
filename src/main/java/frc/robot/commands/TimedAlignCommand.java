package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class TimedAlignCommand extends Command {
    private final AlignCommand alignCommand;
    private final Timer timer = new Timer();
    private final double duration;

    public TimedAlignCommand(VisionSubsystem vision, CommandSwerveDrivetrain swerve, double targetDistance, double targetAngle, double duration) {
        this.alignCommand = new AlignCommand(vision, swerve, targetDistance, targetAngle);
        this.duration = duration;
        addRequirements(vision, swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        alignCommand.initialize();
    }

    @Override
    public void execute() {
        alignCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        alignCommand.end(interrupted);
        timer.stop();
    }
}
