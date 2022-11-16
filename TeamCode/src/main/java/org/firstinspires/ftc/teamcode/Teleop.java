// Makes all code compatible with the FTC variables
package org.firstinspires.ftc.teamcode;

// Specific Libraries of commands to import from the package
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

// Initializes the main framework for the program
@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode
{

    // Calls new Robot and gives it a name
    Thunderbot_2022 robot = new Thunderbot_2022();

    // Calls two new variables for the positions for claw/wrist
    double wristPosition = 0;
    double clawPosition = 0;
    double WRIST_INCREMENT = 0.0025;
    double CLAW_INCREMENT = 0.003;

    // All the things that happen when the init button is pressed
    @Override
    public void init()
    {
        telemetry.addData("Init", "Start");

        // Calls and initalizes all values and parts of the robot declared in ThunderBot_2022
        robot.init(hardwareMap, telemetry);

        // Displays Init: Done
        telemetry.addData("Init", "Done");

        // Makes sure the Claw and Wrist can move as free as they need to
        try {
            wristPosition = 0.625;
            clawPosition = 1;
            robot.linearSlide.wristMove(wristPosition);
            robot.linearSlide.clawMove(clawPosition);
        } catch (Exception e) {
            telemetry.addData("cant", "run");
        }

    }

    // All the things it does when you select Play button
    @Override
    public void start() {
        telemetry.addData("Starting", "...");
    }

    // All the things it does over and over during the period from when start is pressed to when stop is pressed.
    @Override
    public void loop()
    {

        /////////////////
        // CHASSIS
        /////////////////
        if (gamepad1.a) {
            // TURBO!!!
            robot.joystickDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            // Normal Drive
            robot.joystickDrive(-gamepad1.left_stick_y * 0.4, gamepad1.left_stick_x * 0.4, gamepad1.right_stick_x * 0.4);
        }

        /////////////////
        // WRIST
        /////////////////
        if (gamepad2.dpad_up) {
            wristPosition += WRIST_INCREMENT;
            wristPosition = Range.clip(wristPosition, robot.linearSlide.getWRIST_MIN(), robot.linearSlide.getWRIST_MAX());
            robot.linearSlide.wristMove(wristPosition);
        } else if (gamepad2.dpad_down) {
            wristPosition -= WRIST_INCREMENT;
            wristPosition = Range.clip(wristPosition, robot.linearSlide.getWRIST_MIN(), robot.linearSlide.getWRIST_MAX());
            robot.linearSlide.wristMove(wristPosition);
        }

        /////////////////
        // CLAW
        /////////////////
        if (gamepad2.left_bumper) {
            clawPosition += CLAW_INCREMENT;
        } else if (gamepad2.right_bumper) {
            clawPosition -= CLAW_INCREMENT;
        }
        clawPosition = Range.clip(clawPosition, robot.linearSlide.getCLAW_MIN(), robot.linearSlide.getCLAW_MAX());
        robot.linearSlide.clawMove(clawPosition);

        /////////////////
        // LINEAR SLIDE
        /////////////////
        if (gamepad2.a) {
            robot.linearSlide.liftDown(0.5);
        } else if (gamepad2.y) {
            robot.linearSlide.liftUp(0.5);
        } else {
            robot.linearSlide.liftStop();
        }

        /////////////////
        // ELBOW
        /////////////////
        if (gamepad2.x) {
            robot.linearSlide.elbowRaise(1);
        } else if (gamepad2.b) {
            robot.linearSlide.elbowLower(1);
        } else {
            robot.linearSlide.elbowStop();
        }

        /////////////////
        // TELEMETRY
        /////////////////
        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("rx", gamepad1.right_stick_x);

        telemetry.addData("elbow pos (deg): ", robot.linearSlide.getElbowPosition());
        telemetry.addData("wrist pos (0..1): ", robot.linearSlide.wrist.getPosition());
        telemetry.addData("claw pos (0..1):", robot.linearSlide.claw.getPosition());
        telemetry.addData("lift pos (cm): ", robot.linearSlide.getLiftPosition());

        telemetry.addData("gyro sensor", robot.updateHeading());

    }
}
