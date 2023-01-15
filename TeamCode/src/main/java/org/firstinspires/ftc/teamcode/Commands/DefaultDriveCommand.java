package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class DefaultDriveCommand extends CommandBase {

    private final ChassisSubsystem m_drive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier myStrafe;
    private final DoubleSupplier m_rotation;

    /**
     * Creates a new DefaultDriveCommand.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param forward   The control input for driving forwards/backwards
     * @param rotation  The control input for turning
     */
    public DefaultDriveCommand(ChassisSubsystem subsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        m_drive = subsystem;
        m_forward = forward;
        m_rotation = rotation;
        myStrafe = strafe;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.joystickDrive(m_forward.getAsDouble(), myStrafe.getAsDouble(), m_rotation.getAsDouble());
    }

}