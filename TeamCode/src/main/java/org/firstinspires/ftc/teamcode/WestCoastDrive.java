package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

//*In theory* this should also be compatible with tank drive.

@TeleOp(name ="WestCoastDrive", group ="TeleOp")
public class WestCoastDrive extends LinearOpMode{

    /* Declare OpMode members. */
    HardwareConfig robot           = new HardwareConfig();   //Configs hardware


    @Override
    public void runOpMode () throws InterruptedException
    {

        robot.init(hardwareMap);
        //loads hardwareMap

        double R;
        double X;
        double Y;
        double FLvalue;
        double FRvalue;
        double RLvalue;
        double RRvalue;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Shock drone going live!");    //
        telemetry.update();

        waitForStart();

        telemetry.addData("Say", "ASSUMING DIRECT CONTROL");    //
        telemetry.update();

        while(opModeIsActive())
        {
            //tank drive code
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            R=gamepad1.right_stick_x;
            X=gamepad1.left_stick_x;
            Y=gamepad1.left_stick_y;

            // Combine drive and turn for blended motion.
            FLvalue =  - Y + X +R;
            FRvalue =  + Y + X + R;
            RLvalue =  - Y -X+ R;
            RRvalue =  + Y -X + R;

            //maxes the values at 1
            FLvalue = Range.clip(FLvalue, -1, 1 );
            FRvalue = Range.clip(FRvalue, -1, 1 );
            RLvalue = Range.clip(RLvalue, -1, 1 );
            RRvalue = Range.clip(RRvalue, -1, 1 );

            robot.motorFL.setPower(FLvalue);
            robot.motorFR.setPower(FRvalue);
            robot.motorRL.setPower(RLvalue);
            robot.motorRR.setPower(RRvalue);

            idle();
        }
    }
}

