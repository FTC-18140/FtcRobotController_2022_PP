package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode
{

    Thunderbot_2022 robot = new Thunderbot_2022();

    double wristPosition = 0;
    double clawPosition = 0;
    @Override
    public void init()
    {
        telemetry.addData("Init", "Start");

        robot.init(hardwareMap, telemetry);

        telemetry.addData("Init", "Done");

    }

    @Override
    public void start() {
        telemetry.addData("Starting", "...");
    }

    @Override
    public void loop()
    {

        if (gamepad1.left_stick_button) {
            robot.joystickDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            robot.joystickDrive(-gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.5);
        }

        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("rx", gamepad1.right_stick_x);
        telemetry.addData("ry", gamepad1.right_stick_y);

        if (gamepad1.dpad_up) {
            wristPosition += 0.0025;
        } else if (gamepad1.dpad_down) {
            wristPosition -= 0.0025;
        }
        wristPosition = Range.clip(wristPosition, 0, 0.625);
        robot.linearSlide.wristMove(wristPosition); // corrects position

        if (gamepad1.left_bumper) {
            clawPosition += 0.0025;
        } else if (gamepad1.right_bumper) {
            clawPosition -= 0.0025;
        }
        clawPosition = Range.clip(clawPosition, 0.2, 1);
        robot.linearSlide.clawMove(clawPosition);

        if (gamepad1.y) {
            robot.linearSlide.liftDown();
        } else if (gamepad1.a) {
            robot.linearSlide.liftUp();
        } else {
            robot.linearSlide.liftStop();
        }

        if (gamepad1.x) {
            robot.linearSlide.elbowRaise();
        } else if (gamepad1.b) {
            robot.linearSlide.elbowLower();
        } else {
            robot.linearSlide.elbowStop();
        }
    }
}
