package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

@Autonomous(name="Crater", group="Auto")

/* Declare OpMode members. */


public class Crater extends LinearOpMode {

    AutonomousControls auto = new AutonomousControls(); //AutonomousControls

    private GoldAlignDetector detector;

    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder (40:1)
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_ROTATION = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;     //used to compute degrees
    static final double INCHES = (COUNTS_PER_MOTOR_REV * 0.5) / (WHEEL_DIAMETER_INCHES * Math.PI); //calculates counts per inch
    static final double FEET = 12 * INCHES; //calculates counts per foot
    static final double DEGREES = (1120) / 360; //calculates counts per degree

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override

    public void runOpMode() {

        //this section of the code runs what would normally be run in the initialization method
        //consider abstracting later

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        AutonomousControls auto = new AutonomousControls(); //AutonomousControls

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

        telemetry.addData("Say", "Insertion checklist complete. All systems GO.");    //
        telemetry.update();

        waitForStart();
        ////////////////////////////////////////////////////////////////////////////////////////////

        //write main portion of the opMode here

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Dropping Dusty!");    //
        telemetry.update();

        //lower the robot
        auto.rotateArm(0.5, 1120);

        //declare counter variable
        int rotationCount = 0;

        //declare sentinel variable
        boolean runLoop = true;

        telemetry.addData("Say", "I'm going for missile lock!");    //
        telemetry.update();

        //runs loop until robot is aligned with mineral
        while (detector.getAligned() != true&&runLoop==true) {
            if (detector.getXPosition() < 170) {
                auto.turn(0.25, -3 * DEGREES);
                rotationCount--;

            } else if (detector.getXPosition() > 170) {
                auto.turn(0.25, 3 * DEGREES);
                rotationCount++;

            } else {
                //performs 4B0R7N173
                runLoop = false;
                telemetry.addData("Say", "I lost him Goose!");    //
                telemetry.update();
            }

            if(runLoop==true) {

                telemetry.addData("Say", "I've got a good lock! Firing!");    //
                telemetry.update();

                //drive to crater
                //current implementation of rotation count is a placeholder
                auto.drive(0.5, 19 * INCHES);
                auto.turn(0.25, 2 * 3 * rotationCount * DEGREES);
                telemetry.addData("Say", "This is Voodoo Three, remaining MiGs are bugging out. \n" +
                        "This is Maverick, requesting fly-by.");
                telemetry.update();
            }
        }

        ////////////////////////////////////////////////////////////////////////////////////////////

        //this section of the code runs what normally would be written in the stop method

        detector.disable();

    }
}
