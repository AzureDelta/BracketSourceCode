package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="CV Test", group="Auto")

/* Declare OpMode members. */


public class ComputerVisionTest extends LinearOpMode {

    HardwareConfigLite robot = new HardwareConfigLite();

    private GoldAlignDetector detector;

    private ElapsedTime runtime = new ElapsedTime();

    @Override

    public void runOpMode() {

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

        telemetry.addData("Say", "Insertion checklist complete. All systems GO.");    //
        telemetry.update();

        waitForStart();
        ////////////////////////////////////////////////////////////////////////////////////////////

        //write main portion of the opMode here

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Dropping Dusty!");
        telemetry.update();

        boolean runLoop = true;

        telemetry.addData("Say", "I'm going for missile lock!");
        telemetry.update();

        //runs loop until robot is aligned with mineral
        while (detector.getAligned() != true&&runLoop==true) {
            if (detector.getXPosition() < 170) {
                telemetry.addData("Say", "Target left.");
                telemetry.update();

            } else if (detector.getXPosition() > 170) {
                telemetry.addData("Say", "Target Right");
                telemetry.update();

            } else {
                //performs 4B0R7N173
                runLoop = false;
                telemetry.addData("Say", "I lost him Goose!");
                telemetry.update();
            }

            if(runLoop==true) {

                telemetry.addData("Say", "I've got a good lock! Firing!");
                telemetry.update();
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
