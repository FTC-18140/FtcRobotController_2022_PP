package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

import static org.firstinspires.ftc.teamcode.CommandOpModes.AutoLegendary.logger;

public class DiffDriveOdometrySubsystem extends SubsystemBase
{

    protected DifferentialDriveOdometry m_odometry;
    Telemetry telemetry;
    public DoubleSupplier left, right, gyro;

    public final double TRACK_WIDTH = 28.2*2.0;
    public final double TRACK_WIDTH_METERS = TRACK_WIDTH/100.0;

    /**
     * Make sure you are using the supplier version of the constructor
     */
    public DiffDriveOdometrySubsystem(DoubleSupplier leftEncoderDistance, DoubleSupplier rightEncoderDistance, DoubleSupplier gyroAngle,
                                      double initX, double initY, double initHdg, Telemetry telem) {
        m_odometry = new DifferentialDriveOdometry( new Rotation2d(initHdg), new Pose2d(initX, initY, new Rotation2d(initHdg)));
        telemetry = telem;
        left = leftEncoderDistance;
        right = rightEncoderDistance;
        gyro = gyroAngle;
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Call this at the end of every loop
     */
    public void update() {
        m_odometry.update(new Rotation2d(gyro.getAsDouble()), left.getAsDouble(), right.getAsDouble());
    }

    /**
     * Updates the pose every cycle
     */
    @Override
    public void periodic() {
        update();
        telemetry.addData("left", left.getAsDouble());
        telemetry.addData("right", right.getAsDouble());
       // telemetry.addData("Robot Pose: ", getPose() );
        telemetry.addData("X, Y: ", "%.3f, %.3f", getPose().getX(), getPose().getY());
        telemetry.addData("Heading, deg: ", "%.3f", Math.toDegrees(getPose().getHeading()));
        telemetry.addData("Log File Path", logger.getLogFullPathName());

//        logger.addField(left.getAsDouble() );
//        logger.addField(right.getAsDouble());
//        logger.addField( getPose().getHeading() );
//        logger.addField( getPose().getX());
//        logger.addField( getPose().getY());
//        logger.newLine();
    }

}
