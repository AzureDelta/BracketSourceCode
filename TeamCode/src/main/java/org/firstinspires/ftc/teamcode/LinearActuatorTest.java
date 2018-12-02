package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;
import java.util.*;

//*In theory* this should also be compatible with tank drive.

@TeleOp(name ="Linear Actuator Test", group ="TeleOp")
public class LinearActuatorTest extends LinearOpMode {
    public static final double ARM_SPEED = 1;
    //public static final double ARM_SPEED = 0.5;
    public static final double intake_SPEED = 0.5;

    /* Declare OpMode members. */
    HardwareConfigVeryLite robot           = new HardwareConfigVeryLite();   //Configs hardware


    @Override
    public void runOpMode () throws InterruptedException
    {
        robot.init(hardwareMap);

        double actuatorPower;
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Shock drone going live!");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "ASSUMING DIRECT CONTROL");
        telemetry.update();

        while(opModeIsActive())
        {

            actuatorPower = (((gamepad1.right_trigger+gamepad2.right_trigger)+(0.1*-gamepad2.left_stick_y))-(gamepad1.left_trigger+gamepad2.left_trigger));
            actuatorPower *= ARM_SPEED;

            //sets maxes for each value
            actuatorPower = Range.clip(actuatorPower, -ARM_SPEED, ARM_SPEED);

            robot.yellowJacket.setPower(actuatorPower);
        }

    }
}