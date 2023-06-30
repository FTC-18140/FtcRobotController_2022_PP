import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config //Tells Java that FTC Dashboard will modify script in realtime
@TeleOp (name="PIDarmtuning", group="Iterative Opmode")
public class PIDF_Arm extends OpMode {
    private PIDController controller;
 
    public static double p=0, i=0, d=0; //declared as public static to allow FTCDashboard to access
    public static double f=0; //declared as public static to allow FTCDashboard to access

    public static int target = 0;

    private final double ticks_in_degree = 288/360; //Encoder Counter per Revolution for Core Hex Motor at the output

    private DcMotorEx arm_motor;


    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class,"arm_motor0");
    }

    @Override
    public void loop(){

       controller.setPID(p,i,d); //set pid components
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos,target); //calculate power
        //double ff = Math.cos(Math.toRadians(target/ticks_in_degree))*f; //calc feed forward

        double power = pid; //power to motor

        arm_motor.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
       telemetry.addData("pid",pid);
      //  //telemetry.addData("ff",ff);
        telemetry.addData("power",power);
        telemetry.update();
    }

}
