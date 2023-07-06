package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Thunderbot_2022;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous
public class RiverTestOpmode extends  OpMode {
    Thunderbot_2022 robot = new Thunderbot_2022();

    SampleMecanumDrive drive = null;

    Boolean done = false;

    Boolean movedSlide = false;
    Trajectory line = null;

    @Override
    public void init(){
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void start(){
        drive = new SampleMecanumDrive(hardwareMap);
        line = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(20, 0, -90))
                .addDisplacementMarker(5, () -> {

                    if(!movedSlide) {
                        movedSlide = robot.armstrong.liftUpDistance(5, 0.3);
                    }else{
                        robot.stop();
                    }

                })
                .build();

    }

    @Override
    public void loop(){
        robot.update();
        telemetry.addData("movedSlide", movedSlide);
        if (!done){
            drive.followTrajectory(line);
            done = true;
        }
    }

}
