
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;

public class SetElevatorPositionCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double position;

    public SetElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, double position) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.position = position;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setPosition(position);
    }

    // @Override
    // public boolean isFinished() {
    //     // You can add logic here to determine when the command is finished
    //     // For example, you could check if the elevator has reached the desired position
    //     return Math.abs(elevatorSubsystem.getPosition() - position) < 0.1;
    // }
}