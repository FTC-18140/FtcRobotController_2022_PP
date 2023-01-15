package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

public class DriveDistanceCommand extends CommandBase {

    private final ChassisSubsystem m_drive;
    private final double m_distance;
    private final double m_speed;

    /**
     * Creates a new DriveDistanceCommand.
     *
     * @param inches The number of inches the robot will drive
     * @param speed  The speed at which the robot will drive
     * @param drive  The drive subsystem on which this command will run
     */
    public DriveDistanceCommand(double inches, double speed, ChassisSubsystem drive) {
        m_distance = inches;
        m_speed = speed;
        m_drive = drive;
    }

    @Override
    public void initialize() {
        m_drive.resetEncoders();
        m_drive.joystickDrive(m_speed, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.joystickDrive(0, 0, 0);
    }


    @Override
    public boolean isFinished() {
        return Math.abs(m_drive.getAverageEncoderDistance()) >= m_distance;
    }

}
