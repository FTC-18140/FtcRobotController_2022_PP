package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Thunderbot_2022;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous
public class RiverTestAsync extends OpMode{


    Thunderbot_2022 robot = new Thunderbot_2022();

    SampleMecanumDrive drive;
    enum State {
        START,
        TRAJECTORY_1,
        LIFT_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        IDLE
    }

    Trajectory trajectory_1;

    Trajectory trajectory_2;

    Trajectory trajectory_3;
    boolean liftDone = false;
    State currentState = State.START;

    @Override
    public void init(){robot.init(hardwareMap, telemetry);}

    @Override
    public void start(){
        drive = new SampleMecanumDrive(hardwareMap);

        trajectory_1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(20,0,Math.toRadians(90)))
                .build();

        trajectory_2 = drive.trajectoryBuilder(trajectory_1.end())
                .splineTo(new Vector2d(10,10), Math.toRadians(180))
                .build();
    }

    @Override
    public void loop(){
        switch (currentState) {
            case START:
                if(!drive.isBusy()) {
                    currentState = State.TRAJECTORY_1;
                    drive.followTrajectoryAsync(trajectory_1);
                }
                break;
            case TRAJECTORY_1:
                if(!drive.isBusy()) {
                    currentState = State.LIFT_1;
                }
                break;
            case LIFT_1:
                if (!liftDone) {
                    liftDone = robot.armstrong.liftUpDistance(10, 0.3);

                }else{
                    currentState = State.TRAJECTORY_2;
                    drive.followTrajectory(trajectory_2);
                }
                break;
            case TRAJECTORY_2:
                if(!drive.isBusy()) {
                    currentState = State.IDLE;
                }
                break;
            case IDLE:
                telemetry.addData("done", "true");
                break;
            default:
                break;
        }

        telemetry.addData("SlidePosition", robot.armstrong.getLiftPosition());


        drive.update();
        robot.update();


    }

}
