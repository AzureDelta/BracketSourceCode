package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="Full Autonomous Rewrite Test", group="F.A.R.T.")

/* Declare OpMode members. */


public class FullAutonomousRewriteTest extends LinearOpMode {

    VuforiaLocalizer vuforia;

    AutonMap robot = new AutonMap();

    private GoldAlignDetector detector;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder (40:1)
    static final double DRIVE_GEAR_REDUCTION = 80/120;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_ROTATION = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;     //used to compute degrees
    static final double INCHES = (COUNTS_PER_ROTATION * 0.5) / (WHEEL_DIAMETER_INCHES * Math.PI); //calculates counts per inch
    public static final double M = (2 / Math.sqrt(2));
    /*
    660 counts of encoder = 4 inches
    1 inch = 165 counts
    */

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);

        // Look for the audio file
        boolean soundFound;

        int soundID = hardwareMap.appContext.getResources().getIdentifier("spritecranberry", "raw", hardwareMap.appContext.getPackageName());

        // Preload the audio if the file has a valid ID
        if (soundID != 0)
            soundFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, soundID);

        //Vuforia documentation - Brandon look at this dude
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "ARvoW7v/////AAABmYoAtzjbfUl5gYNuLdrfUl8xlfcBiKF/LznPg4EMgSTGYH6BSBuFXw6l0WYTIwevC/nUQjfQ2KFn2j9YE1doWfQ/Tip4ONRj1SiKI8Yd1bTcgVrdPJYTynrkFNlUWg13P8wxc1KxgOd1KFyGpyQwyKAgUz454AhxkYxeAY8FxynFozAMvVojpLrUNxkAi6Ph16wu/1ykQScD14i87X3nVZyd0NfSGCimTKUryARPQf+WCZuSCIid4nPX1WTVIyEa5DXoTXnWZhvsb6/c8tN0GaVC+s6MoKVWSC1Lu4syK6tRbWoX5OirzW20nb8F8ZOUe2gM8KsON7UEMWTet2EyNZiLNIluxBIDG9wtJbgl3rf7\n";

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        CameraDevice.getInstance().setFlashTorchMode(true);

        //send telemetry
        telemetry.addData("Status", "Ready to run Test Autonomous");
        telemetry.update();

        waitForStart();



        // Play the audio
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);

        //drive, strafe
        drive(0.5, 3 * INCHES * M);
        sleep(500);
        drive(0.5, -3 * INCHES * M);
        sleep(500);
        //positive dist. is right
        //negative dist. is left
        //first right 3 inches
        strafe(0.5, 3 * INCHES * M);
        sleep(500);
        //then left 3 inches
        strafe(0.5, -3 * INCHES * M);
        sleep(500);

        //test intake
/*        intake(0.5, 3);
        sleep(1000);
        //test actuator
        actuate(1, 5);*/

        CameraDevice.getInstance().setFlashTorchMode(false);
    }

    public void drive(double speed, double distance) {
        if (opModeIsActive()) {
            // Resets encoder values so that it doesn't attempt to run to outdated values
            robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Sets motors to run to a given encoder value
            robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Declares target point storage variables
            int targetFL;
            int targetFR;
            int targetRL;
            int targetRR;

            // Determines new target position, and pass to motor controller
            targetFL = robot.motorFL.getCurrentPosition() + (int) (distance);
            targetFR = robot.motorFR.getCurrentPosition() + (int) (distance);
            targetRL = robot.motorRL.getCurrentPosition() + (int) (distance);
            targetRR = robot.motorRR.getCurrentPosition() + (int) (distance);
            robot.motorFL.setTargetPosition(targetFL);
            robot.motorFR.setTargetPosition(targetFR);
            robot.motorRL.setTargetPosition(targetRL);
            robot.motorRR.setTargetPosition(targetRR);

            // Motors are set to run at a certain speed until one reaches its target position
            while (robot.motorFL.isBusy() && robot.motorFR.isBusy() && robot.motorRL.isBusy() && robot.motorRR.isBusy()) {
                robot.motorFL.setPower(Math.abs(speed));
                robot.motorFR.setPower(Math.abs(speed));
                robot.motorRL.setPower(Math.abs(speed));
                robot.motorRR.setPower(Math.abs(speed));
                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
            }
        }
        // The motors are shutdown when a motor gets to its target position
        robot.motorFL.setPower(0);
        robot.motorFR.setPower(0);
        robot.motorRL.setPower(0);
        robot.motorRR.setPower(0);
    }

    public void strafe(double speed, double distance) {
        if (opModeIsActive()) {
            // Resets encoder values so that it doesn't attempt to run to outdated values
            robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Sets motors to run to a given encoder value
            robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Declares target point storage variables
            int targetFL;
            int targetFR;
            int targetRL;
            int targetRR;

            // Determines new target position, and pass to motor controller
            targetFL = robot.motorFL.getCurrentPosition() + (int) (-distance);
            targetFR = robot.motorFR.getCurrentPosition() + (int) (distance);
            targetRL = robot.motorRL.getCurrentPosition() + (int) (distance);
            targetRR = robot.motorRR.getCurrentPosition() + (int) (-distance);
            robot.motorFL.setTargetPosition(targetFL);
            robot.motorFR.setTargetPosition(targetFR);
            robot.motorRL.setTargetPosition(targetRL);
            robot.motorRR.setTargetPosition(targetRR);

            // Motors are set to run at a certain speed until one reaches its target position
            while (robot.motorFL.isBusy() && robot.motorFR.isBusy() && robot.motorRL.isBusy() && robot.motorRR.isBusy()) {
                robot.motorFL.setPower(Math.abs(speed));
                robot.motorFR.setPower(Math.abs(speed));
                robot.motorRL.setPower(Math.abs(speed));
                robot.motorRR.setPower(Math.abs(speed));
                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
            }
        }
        // The motors are shutdown when a motor gets to its target position
        robot.motorFL.setPower(0);
        robot.motorFR.setPower(0);
        robot.motorRL.setPower(0);
        robot.motorRR.setPower(0);
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