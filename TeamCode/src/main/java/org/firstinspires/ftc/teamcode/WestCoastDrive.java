package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

//*In theory* this should also be compatible with tank drive.

@TeleOp(name ="WestCoastDrive", group ="Pushbot")
public class WestCoastDrive extends LinearOpMode{

    /* Declare OpMode members. */
    HardwareConfig robot           = new HardwareConfig();   //Configs hardware


    @Override
    public void runOpMode () throws InterruptedException
    {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Shock drone going live!");    //
        telemetry.update();

        waitForStart();

        telemetry.addData("Say", "ASSUMING DIRECT CONTROL");    //
        telemetry.update();
        while(opModeIsActive())
        {
            //tank drive code
            robot.motorFR.setPower(gamepad1.right_stick_y);
            robot.motorFL.setPower(gamepad1.left_stick_y);
            robot.motorRR.setPower(gamepad1.right_stick_y);
            robot.motorRL.setPower(gamepad1.left_stick_y);

            idle();
        }
    }
}

