package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "gOLD aLIGN tEST", group = "Auto")

/* Declare OpMode members. */

public class GoldAlignTest extends LinearOpMode {

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
        telemetry.addData("Status", "Testing!");
        telemetry.update();

        while(opModeIsActive()) {
            alignGold();
            telemetry.addData("OFFSET", OFFSET);
            telemetry.update();
        }

        //runs loop until robot is aligned with mineral

        ////////////////////////////////////////////////////////////////////////////////////////////

        //this section of the code runs what normally would be written in the stop method

        detector.disable();

    }

    public void alignGold(){
        robot.motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (detector.getAligned() != true && runtime.seconds() < 20 && detector.isFound()) {
            if (detector.getXPosition() < 320 && detector.isFound()) {
                robot.motorFL.setPower(-DRIVE_SPEED);
                robot.motorFR.setPower(-DRIVE_SPEED);
                robot.motorRL.setPower(DRIVE_SPEED);
                robot.motorRR.setPower(DRIVE_SPEED);
                OFFSET--;
                telemetry.addData("Status", "Target left.");
                telemetry.update();
            } else if (detector.getXPosition() > 320 && detector.isFound()) {
                robot.motorFL.setPower(DRIVE_SPEED);
                robot.motorFR.setPower(DRIVE_SPEED);
                robot.motorRL.setPower(-DRIVE_SPEED);
                robot.motorRR.setPower(-DRIVE_SPEED);
                OFFSET++;
                telemetry.addData("Status", "Target Right");
                telemetry.update();
            }
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