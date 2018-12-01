package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Dropping Dusty Divot (CRATER FULL)", group = "Auto")

/* Declare OpMode members. */


public class Depot extends LinearOpMode {

    HardwareConfig robot = new HardwareConfig();

    private GoldAlignDetector detector;

    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder (40:1)
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_ROTATION = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;     //used to compute degrees
    static final double INCHES = (COUNTS_PER_MOTOR_REV * 0.5) / (WHEEL_DIAMETER_INCHES * Math.PI); //calculates counts per inch
    static final double FEET = 12 * INCHES; //calculates counts per foot
    static final double DEGREES = (1120) / 360; //calculates counts per degree
    public static final double M = (2 / Math.sqrt(2));
    public static final double ARM_SPEED = 0.1;
    public static final double DRIVE_SPEED = 0.5;
    /*
    660 counts of encoder = 4 inches
    1 inch = 165 counts
    */

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);
        //this section of the code runs what would normally be run in the initialization method
        //consider abstracting later

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        telemetry.addData("Status", "Insertion checklist complete. All systems GO.");    //
        telemetry.update();

        waitForStart();
        ////////////////////////////////////////////////////////////////////////////////////////////

        //write main portion of the opMode here

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Dropping Dusty!");
        telemetry.update();

        //lower the robot
        actuate(0.5, 1.9);
        //detach arm
        strafe(0.5, -3 * INCHES * M);
        //store arm
        actuate(-0.5, 1.9);
        //reset position
        drive(0.5, 3 * INCHES * M);
        //detach arm
        strafe(0.5, 3 * INCHES * M);

        //declare counter variable
        int OFFSET = 0;

        //declare sentinel variable
        boolean runLoop = true;

        telemetry.addData("Status", "I'm going for missile lock!");
        telemetry.update();

        //runs loop until robot is aligned with mineral
        while (detector.getAligned() != true && runLoop == true && runtime.seconds() < 20) {
            if (detector.getXPosition() < 320) {
                strafe(DRIVE_SPEED, -0.1 * INCHES * M);
                OFFSET--;
                telemetry.addData("Status", "Target left.");
                telemetry.update();

            } else if (detector.getXPosition() > 320) {
                strafe(DRIVE_SPEED, 0.1 * INCHES * M);
                OFFSET++;
                telemetry.addData("Status", "Target Right");
                telemetry.update();
            } else if (!detector.isFound()) {
                //performs 4B0R7N173
                runLoop = false;
                telemetry.addData("Status", "I lost him Goose!");
                telemetry.update();
            }


        }

        if (runLoop == true) {

            telemetry.addData("Status", "I've got a good lock! Firing!");
            telemetry.update();

            //ONE TILE IS 23.5 INCHES X 23.5 INCHES
            //TWO TITLES ARE 47 INCHES X 47 INCHES

            //drive to crater
            //current implementation of rotation count is a placeholder

            //CENTER OF THE LANDER TO A THE CRATER IS ROUGHLY 1.8 TILES IF LOOKING AT IT STRAIGHT ON
            //1.8 tiles is equal to approximately 42.3 inches (THIS IS A HIGH ESTIMATE)
            //According to the field setup guide, it is more around 34.
            //Brandon, I don't know the values to change, but I did some calculations

            drive(0.5, 19 * INCHES);
            telemetry.addData("Status", "Performing correction burn.");
            telemetry.update();

        }

        ////////////////////////////////////////////////////////////////////////////////////////////

        //this section of the code runs what normally would be written in the stop method

        detector.disable();

    }

    public void rotateIntake(double speed, double distance, double timeout) {
        int targetL;
        int targetR;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        targetL = robot.slide.getCurrentPosition() + (int) (distance);
        targetR = robot.actuator.getCurrentPosition() + (int) (distance);
        robot.slide.setTargetPosition(targetL);
        robot.actuator.setTargetPosition(targetR);

        // Turn On RUN_TO_POSITION
        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        robot.slide.setPower(Math.abs(speed));
        robot.actuator.setPower(Math.abs(speed));
        ;

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

        // Stop all motion;
        robot.slide.setPower(0);
        robot.actuator.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotateArm(double speed, double distance) {
        int targetL;
        int targetR;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        targetL = robot.slide.getCurrentPosition() + (int) (distance);
        targetR = robot.actuator.getCurrentPosition() + (int) (distance);
        robot.slide.setTargetPosition(targetL);
        robot.actuator.setTargetPosition(targetR);

        // Turn On RUN_TO_POSITION
        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (opModeIsActive()) {

            // reset the timeout time and start motion.
            robot.slide.setPower(Math.abs(speed));
            robot.actuator.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            // Stop all motion;
            robot.slide.setPower(0);
            robot.actuator.setPower(0);
        }

        // Turn off RUN_TO_POSITION
        robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void drive(double speed, double distance) {
        //declares target point storage variables
        int targetFL;
        int targetFR;
        int targetRL;
        int targetRR;

        // Determine new target position, and pass to motor controller
        targetFL = robot.motorFL.getCurrentPosition() + (int) (distance);
        targetFR = robot.motorFR.getCurrentPosition() + (int) (distance);
        targetRL = robot.motorRL.getCurrentPosition() + (int) (distance);
        targetRR = robot.motorRR.getCurrentPosition() + (int) (distance);
        robot.motorFL.setTargetPosition(targetFL);
        robot.motorFR.setTargetPosition(targetFR);
        robot.motorRL.setTargetPosition(targetRL);
        robot.motorRR.setTargetPosition(targetRR);

        // Turn On RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // reset the timeout time and start motion.

        if (opModeIsActive()) {
            robot.motorFL.setPower(Math.abs(speed));
            robot.motorFR.setPower(Math.abs(speed));
            robot.motorRL.setPower(Math.abs(speed));
            robot.motorRR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            // Stop all motion;
            robot.motorFL.setPower(0);
            robot.motorFR.setPower(0);
            robot.motorRL.setPower(0);
            robot.motorRR.setPower(0);
        }

        // Turn off RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void turn(double speed, double angle) {
        //declares target point storage variables
        int targetFL;
        int targetFR;
        int targetRL;
        int targetRR;

        //rotates counter clockwise based on angle
        targetFL = robot.motorFL.getCurrentPosition() + (int) (COUNTS_PER_ROTATION - angle);
        targetFR = robot.motorFR.getCurrentPosition() + (int) (COUNTS_PER_ROTATION + angle);
        targetRL = robot.motorRL.getCurrentPosition() + (int) (COUNTS_PER_ROTATION - angle);
        targetRR = robot.motorRR.getCurrentPosition() + (int) (COUNTS_PER_ROTATION + angle);
        robot.motorFL.setTargetPosition(targetFL);
        robot.motorFR.setTargetPosition(targetFR);
        robot.motorRL.setTargetPosition(targetRL);
        robot.motorRR.setTargetPosition(targetRR);

        // Turn On RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (opModeIsActive()) {

            // reset the timeout time and start motion.
            robot.motorFL.setPower(Math.abs(speed));
            robot.motorFR.setPower(Math.abs(speed));
            robot.motorRL.setPower(Math.abs(speed));
            robot.motorRR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            // Stop all motion;
            robot.motorFL.setPower(0);
            robot.motorFR.setPower(0);
            robot.motorRL.setPower(0);
            robot.motorRR.setPower(0);
        }

        // Turn off RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafe(double speed, double distance) {
        //declares target point storage variables
        int targetFL;
        int targetFR;
        int targetRL;
        int targetRR;

        // Determine new target position, and pass to motor controller
        targetFL = robot.motorFL.getCurrentPosition() + (int) (distance);
        targetFR = robot.motorFR.getCurrentPosition() + (int) (-distance);
        targetRL = robot.motorRL.getCurrentPosition() + (int) (-distance);
        targetRR = robot.motorRR.getCurrentPosition() + (int) (distance);
        robot.motorFL.setTargetPosition(targetFL);
        robot.motorFR.setTargetPosition(targetFR);
        robot.motorRL.setTargetPosition(targetRL);
        robot.motorRR.setTargetPosition(targetRR);

        // Turn On RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // reset the timeout time and start motion.

        if (opModeIsActive()) {
            robot.motorFL.setPower(Math.abs(speed));
            robot.motorFR.setPower(Math.abs(speed));
            robot.motorRL.setPower(Math.abs(speed));
            robot.motorRR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            // Stop all motion;
            robot.motorFL.setPower(0);
            robot.motorFR.setPower(0);
            robot.motorRL.setPower(0);
            robot.motorRR.setPower(0);
        }

        // Turn off RUN_TO_POSITION
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void actuate(double speed, double time) {
        // Step 1:  Drive forward for 3 seconds
        robot.actuator.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Status:", "Actuating", runtime.seconds());
            telemetry.update();
        }
    }

    public void intake(double speed, double time) {
        // Step 1:  Drive forward for 3 seconds
        robot.intakeL.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Status:", "Actuating", runtime.seconds());
            telemetry.update();
        }
    }
}
