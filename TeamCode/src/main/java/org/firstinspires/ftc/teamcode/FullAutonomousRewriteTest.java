package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

@Autonomous(name="Full Autonomous Rewrite Test", group="F.A.R.T.")

/* Declare OpMode members. */


public class FullAutonomousRewriteTest extends LinearOpMode {

    AutonMap robot = new AutonMap();

    private GoldAlignDetector detector;

    private ElapsedTime runtime = new ElapsedTime();

    public static double DRIVE_SPEED = 0.5;

    /*
    660 counts of encoder = 4 inches
    1 inch = 165 counts
    */

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);

        // Look for the audio file
        boolean soundFound;
        //int soundID = hardwareMap.appContext.getResources().getIdentifier("attackontitan", "raw", hardwareMap.appContext.getPackageName());

        // Preload the audio if the file has a valid ID
        //if (soundID != 0)
            //soundFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, soundID);


        //send telemetry
        telemetry.addData("Status", "Ready to run Test Autonomous");
        telemetry.update();

        waitForStart();



        // Play the audio
        //SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);


            robot.strafe(-DRIVE_SPEED, -1428);
            robot.OFFSET-=68;

            robot.strafe(DRIVE_SPEED, 2856);
            robot.OFFSET+=136;

        //runs loop until robot is aligned with mineral

        int offsetTotal = robot.OFFSET*4;
        robot.strafe(DRIVE_SPEED, offsetTotal);



        telemetry.addData("Status", "I've got a good lock! Firing!");
        telemetry.update();

        //ONE TILE IS 24 INCHES X 24 INCHES

        //drive to crater
        robot.drive(DRIVE_SPEED, 2856);



        intake(0.9, 2.25);

        //test intake
/*        intake(0.5, 3);
        sleep(1000);
        //test actuator
        actuate(1, 5);*/
    }
    public void alignGold(){
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (detector.getAligned() != true && runtime.seconds() < 20 && detector.isFound()) {
            telemetry.addData("OFFSET", robot.OFFSET);
            if (detector.getXPosition() < 320 && detector.isFound()) {
                robot.strafe(DRIVE_SPEED, -21);
                robot.OFFSET--;
                telemetry.addData("Status", "Target left.");
                telemetry.update();
            } else if (detector.getXPosition() > 320 && detector.isFound()) {
                robot.strafe(DRIVE_SPEED, 21);
                robot.OFFSET++;
                telemetry.addData("Status", "Target Right");
                telemetry.update();
            }
        }
        robot.motorFL.setPower(0);
        robot.motorFR.setPower(0);
        robot.motorRL.setPower(0);
        robot.motorRR.setPower(0);
    }

    public void intake(double speed, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Status:", "Actuating", runtime.seconds());
            telemetry.update();
            robot.intake.setPower(speed);
        }
    }

/*    public void actuate(double speed, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Status:", "Actuating", runtime.seconds());
            telemetry.update();
            robot.actuator.setPower(speed);
        }
    }*/

    /*public void intake(double speed, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Status:", "Actuating", runtime.seconds());
            telemetry.update();
            robot.intake.setPower(speed);
        }
    }*/
}