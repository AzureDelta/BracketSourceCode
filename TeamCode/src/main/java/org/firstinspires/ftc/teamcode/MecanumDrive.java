package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;
import java.util.*;

//*In theory* this should also be compatible with tank drive.

@TeleOp(name ="Mecanum Drivetrain", group ="TeleOp")
public class MecanumDrive extends LinearOpMode{

    public static final double ARM_SPEED = 1;
    public static final double SPEED = 0.75;
    //public static final double ARM_SPEED = 0.5;
    public static final double INTAKE_SPEED = 0.5;

    /* Declare OpMode members. */
    DrivetrainMap robot           = new DrivetrainMap();   //Configs hardware


    @Override
    public void runOpMode () throws InterruptedException
    {

        robot.init(hardwareMap);
        //loads hardwareMap

        double SPEED = 0.5;
        double R;
        double X;
        double Y;
        double powerFL;
        double powerFR;
        double powerRL;
        double powerRR;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Shock drone going live!");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "ASSUMING DIRECT CONTROL");
        telemetry.update();

        while(opModeIsActive())
        {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            R=gamepad1.right_stick_x;
            X=gamepad1.left_stick_x;
            Y=gamepad1.left_stick_y;

            // Combine drive and turn for blended motion.
            powerFL =  - Y + X + R;
            powerFR =  + Y + X + R;
            powerRL =  - Y - X + R;
            powerRR =  + Y - X + R;

            //maxes the values at 1
            powerFL = Range.clip(powerFL, -SPEED, SPEED);
            powerFR = Range.clip(powerFR, -SPEED, SPEED);
            powerRL = Range.clip(powerRL, -SPEED, SPEED);
            powerRR = Range.clip(powerRR, -SPEED, SPEED);

            robot.motorFL.setPower(powerFL);
            robot.motorFR.setPower(powerFR);
            robot.motorRL.setPower(powerRL);
            robot.motorRR.setPower(powerRR);

            telemetry.addData("Status", "null");
            telemetry.update();
        }
    }




    //idle();
}