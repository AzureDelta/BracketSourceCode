package BracketSourceCode.TeamCode.src.main.java.org.vision

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp
@Disabled
public class VisionJavaExample extends LinearOpMode{
    MasterVision vision;
    SampleRandomizedPositions goldPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "IAbGzTCD/////AAABmTyRWQKgu0TatF+00V7AzV4MLLLhKSgLtnneljO0P6prb7zQZd3Uy2ABiYtDtW9W2zgAfW+vqfVy2Yti8XJYEtPmT0SA2u2LYQzzWMajVW5p3uQZBnVaSrs4QKhzL0GHTLYtxNZnXfgq8UUd2OAKXZaJ0Wqr7aIagekS5m1q0Yhcs9zCjkdW5RI9oLHh1MQkhf2QzsBv9kYd75URli6M47ANABYoyFYM5F3fcxJOhkeCzwDFxSuO52cvVX8NWfNEmJUP2iVYc/bhJVB8yLn6GxFkfDDcGidkx4gwrzVvdNl6FxgnRdsnWD+VIFlMgCQ4ZIOTMCseQKxJOj2VgCy/1bDz4dHB+7UF7Em89VG0P0Uj";

        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_NONE);
        vision.init();// enables the camera overlay. this will take a couple of seconds
        vision.enable();// enables the tracking algorithms. this might also take a little time

        waitForStart();

        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        while(opModeIsActive()){
            telemetry.addData("goldPosition was", goldPosition);// giving feedback

            switch (goldPosition){ // using for things in the autonomous program
                case LEFT:
                    telemetry.addLine("going to the left");
                    break;
                case CENTER:
                    telemetry.addLine("going straight");
                    break;
                case RIGHT:
                    telemetry.addLine("going to the right");
                    break;
                case UNKNOWN:
                    telemetry.addLine("staying put");
                    break;
            }

            telemetry.update();
        }

        vision.shutdown();
    }
}
