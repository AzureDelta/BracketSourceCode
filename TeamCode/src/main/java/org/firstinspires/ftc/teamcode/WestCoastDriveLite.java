package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

//*In theory* this should also be compatible with tank drive.

@TeleOp(name ="WestCoastDriveLite", group ="TeleOp")
public class WestCoastDriveLite extends LinearOpMode{

    /* Declare OpMode members. */
    HardwareConfigLite robot           = new HardwareConfigLite();   //Configs hardware


    @Override
    public void runOpMode () throws InterruptedException
    {

        robot.init(hardwareMap);
        //loads hardwareMap

        double drive;
        double turn;
        double leftValue;
        double rightValue;

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
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            leftValue  = -(drive + turn);
            rightValue = -(drive - turn);

            //sets maxes for each value
            leftValue = Range.clip(leftValue, -0.9, 0.9);
            rightValue = Range.clip(rightValue, -0.9, 0.9);

            robot.motorFL.setPower(leftValue);
            robot.motorFR.setPower(rightValue);
            robot.motorRL.setPower(leftValue);
            robot.motorRR.setPower(rightValue);

            telemetry.addData("Status", "Left: "+ leftValue+"        Right: "+ rightValue+"\n" +
                    "Power: "+ drive +"        Turn: "+turn);
            telemetry.update();

            //idle();
        }
    }
}

