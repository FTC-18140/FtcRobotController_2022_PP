package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class DiffOdometrySubsystem extends SubsystemBase
{

    protected DifferentialOdometry m_odometry;
    Telemetry telemetry;

    /**
     * Make sure you are using the supplier version of the constructor
     */
    public DiffOdometrySubsystem(DoubleSupplier leftEncoder, DoubleSupplier rightEncoder,
                                 double trackWidth, Telemetry telem) {
        m_odometry = new DifferentialOdometry( leftEncoder, rightEncoder, trackWidth);
        telemetry = telem;
    }

    public Pose2d getPose() {
        return m_odometry.getPose();
    }

    /**
     * Call this at the end of every loop
     */
    public void update() {
        m_odometry.updatePose();
    }

    /**
     * Updates the pose every cycle
     */
    @Override
    public void periodic() {
        m_odometry.updatePose();
    }

}
