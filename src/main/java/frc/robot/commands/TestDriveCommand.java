
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class TestDriveCommand extends Command {
    private final CommandSwerveDrivetrain m_Swerve;
    private final double speed;
    private final double duration;
    private final Timer timer;

    public TestDriveCommand(CommandSwerveDrivetrain swerve, double speed, double duration) {
        m_Swerve = swerve;
        this.speed = speed;
        this.duration = duration;
        timer = new Timer();
        addRequirements(m_Swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        System.out.println("TestDriveCommand initialized with duration: " + duration);
    }

    @Override
    public void execute() {
        m_Swerve.setControl(new SwerveRequest.RobotCentric()
            .withVelocityY(speed)  // Forward movement
            .withVelocityX(0)      // No lateral movement
            .withRotationalRate(0) // No rotation
        );
        System.out.println("TestDriveCommand executing with speed: " + speed);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        m_Swerve.setControl(new SwerveRequest.RobotCentric()
            .withVelocityY(0)
            .withVelocityX(0)
            .withRotationalRate(0)
        );
        timer.stop();
        System.out.println("TestDriveCommand ended");
    }
}
