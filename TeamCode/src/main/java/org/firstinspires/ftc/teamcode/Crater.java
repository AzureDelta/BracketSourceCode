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

@Autonomous(name="Crater", group="Auto")

/* Declare OpMode members. */


public class Crater extends LinearOpMode {

    AutonomousControls auto = new AutonomousControls(); //AutonomousControls
    LandAndSort landAndSort = new LandAndSort();
    CraterPartTwo crater = new CraterPartTwo();

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
        ////////////////////////////////////////////////////////////////////////////////////////////

        landAndSort.partOne();
        crater.partTwo();

        ////////////////////////////////////////////////////////////////////////////////////////////

        //this section of the code runs what normally would be written in the stop method

        detector.disable();

    }
}
