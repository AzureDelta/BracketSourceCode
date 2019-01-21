package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

@Autonomous(name = "Full Depot", group = "Depot")

/* Declare OpMode members. */

public class FacingDepotAuton extends LinearOpMode {

    AutonMap robot = new AutonMap();

    private GoldAlignDetector detector;
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder (40:1)
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_ROTATION = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;     //used to compute degrees
    static final double INCHES = (COUNTS_PER_MOTOR_REV * (80/120) / (WHEEL_DIAMETER_INCHES * Math.PI)); //calculates counts per inch
    static final double FEET = 12 * INCHES;
    double OFFSET = 0;
    public static final double M = (2 / Math.sqrt(2));
    public static final double DRIVE_SPEED = 0.5;

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

        robot.open(1, 8);
        //lower the robot
        //actuate(1.0, 12.5);
        //detach arm
        robot.strafe(DRIVE_SPEED,168);
        //move forward
        robot.drive(DRIVE_SPEED, 168);
        //declare sentinel variable
        boolean runLoop = true;

        telemetry.addData("Status", "I'm going for missile lock!");
        telemetry.update();

        //length diagonally across a tile is 33.9411255
        //basically 34

        alignGold();
        if(!detector.isFound()){
            robot.strafe(-DRIVE_SPEED, -1428);
            robot.OFFSET-=68;
            alignGold();
        }
        if(!detector.isFound()){
            robot.strafe(DRIVE_SPEED, 2856);
            robot.OFFSET+=136;
            alignGold();
        }

        //runs loop until robot is aligned with mineral

        //drive to crater
        robot.drive(DRIVE_SPEED, 2856);

        int offsetTotal = robot.OFFSET*4;
        robot.strafe(DRIVE_SPEED, offsetTotal);



            telemetry.addData("Status", "I've got a good lock! Firing!");
            telemetry.update();

            //ONE TILE IS 24 INCHES X 24 INCHES





        intake(0.9, 2.25);

        ////////////////////////////////////////////////////////////////////////////////////////////

        //this section of the code runs what normally would be written in the stop method

        detector.disable();

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
}