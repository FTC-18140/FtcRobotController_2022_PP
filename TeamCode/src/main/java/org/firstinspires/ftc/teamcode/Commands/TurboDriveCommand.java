package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
@Config
public class TurboDriveCommand extends CommandBase {

    private final ChassisSubsystem m_drive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier myStrafe;
    private final DoubleSupplier m_rotation;
    private final boolean myTurbo;

    /**
     * Creates a new DefaultDriveCommand.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param forward   The control input for driving forwards/backwards
     * @param rotation  The control input for turning
     */
    public TurboDriveCommand(ChassisSubsystem subsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation ) {
        m_drive = subsystem;
        m_forward = forward;
        m_rotation = rotation;
        myStrafe = strafe;
        myTurbo = true;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        if (myTurbo)
        {
            m_drive.joystickDrive(m_forward.getAsDouble(), myStrafe.getAsDouble(), m_rotation.getAsDouble());
        }
        else
        {
            m_drive.joystickDrive(m_forward.getAsDouble()*DefaultDriveCommand.NON_TURBO_FACTOR,
                                    myStrafe.getAsDouble()*DefaultDriveCommand.NON_TURBO_FACTOR,
                                 m_rotation.getAsDouble()*DefaultDriveCommand.NON_TURBO_FACTOR);
        }
    }

}