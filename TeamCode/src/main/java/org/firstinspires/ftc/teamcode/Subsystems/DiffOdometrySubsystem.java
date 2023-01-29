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
    DoubleSupplier left, right;

    public final double TRACK_WIDTH = 28.2;
    public final double TRACK_WIDTH_METERS = TRACK_WIDTH/100.0;

    /**
     * Make sure you are using the supplier version of the constructor
     */
    public DiffOdometrySubsystem(DoubleSupplier leftEncoderDistance, DoubleSupplier rightEncoderDistance,
                                 Telemetry telem) {
        m_odometry = new DifferentialOdometry( leftEncoderDistance, rightEncoderDistance, TRACK_WIDTH);
        telemetry = telem;
        left = leftEncoderDistance;
        right = rightEncoderDistance;
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
        telemetry.addData("left", left.getAsDouble());
        telemetry.addData("right", right.getAsDouble());
        telemetry.addData("Robot Pose: ", getPose() );
    }

}
