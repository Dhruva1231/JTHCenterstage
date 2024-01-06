package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Auto.Random.left;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Auto.Random.middle;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Auto.Random.right;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Auto.State.CYCLE1_INTAKE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Auto.State.CYCLE1_PRE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Auto.State.IDLE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Auto.State.PRELOAD_PURPLE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Auto.State.PRELOAD_PURPLE_DROP;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Auto.State.PRELOAD_PURPLE_INTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Auto.State.PRELOAD_YELLOW;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Auto.State.PRELOAD_YELLOW_DROP;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Auto.State.PRELOAD_YELLOW_INTER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.Barcode;
import org.firstinspires.ftc.teamcode.vision.BlueScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Autonomous Program")
public class Auto extends LinearOpMode {
    OpenCvWebcam webcam;

    enum Random {
        left,
        middle,
        right
    }
    Random Random = left;
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    enum State {
        PRELOAD_YELLOW,            // Drop yellow preload on backdrop
        PRELOAD_YELLOW_INTER,
        PRELOAD_YELLOW_DROP,           // Drop yellow preload on backdrop
        PRELOAD_PURPLE,    // Drop purple preload on ground
        PRELOAD_PURPLE_INTER,    // Drop purple preload on ground
        PRELOAD_PURPLE_DROP,   // Drop purple preload on ground

        CYCLE1_PRE,        // Move servos into proper position and drive center
        CYCLE1_INTAKE,     // Extend intake and intake pixels
        CYCLE1_INTER,      // Start path to backdrop
        CYCLE1_TRANSFER,   // Retract intake and transfer pixels
        CYCLE1_DEPOSIT,    // Drop pixels on backdrop

        IDLE,              // Our bot will enter the IDLE state when done
    }

    State state = PRELOAD_YELLOW;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence preload1 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-5, 0), Math.toRadians(180))
                .splineTo(new Vector2d(-27, -36), Math.toRadians(-90))
                .build();

        TrajectorySequence preload2_v1 = drive.trajectorySequenceBuilder(preload1.end())
                .lineTo(new Vector2d(-33, -36))
                .build();

        TrajectorySequence preload2_v2 = drive.trajectorySequenceBuilder(preload1.end())
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-50.5, -10, Math.toRadians(45)), Math.toRadians(90))
                .build();

        TrajectorySequence cycle1_v1 = drive.trajectorySequenceBuilder(preload2_v1.end())
                .setReversed(false)
                .splineTo(new Vector2d(-50.5, -10), Math.toRadians(90))
                .splineTo(new Vector2d(-50.5, 30), Math.toRadians(90))
                .build();

        TrajectorySequence cycle1_v2 = drive.trajectorySequenceBuilder(preload2_v2.end())
                .setReversed(false)
                .lineToSplineHeading(new Pose2d(-51.5, -8, Math.toRadians(90)))
                .splineTo(new Vector2d(-51.5, 30), Math.toRadians(90))
                .build();

        TrajectorySequence deposit_v1 = drive.trajectorySequenceBuilder(cycle1_v1.end())
                .setReversed(true)
                .waitSeconds(0.25)
                .splineTo(new Vector2d(-50.5, 10), Math.toRadians(-90))
                .splineTo(new Vector2d(-27, -36), Math.toRadians(-90))
                .build();

        TrajectorySequence deposit_v2 = drive.trajectorySequenceBuilder(cycle1_v2.end())
                .setReversed(true)
                .waitSeconds(0.25)
                .splineTo(new Vector2d(-50.5, 10), Math.toRadians(-90))
                .splineTo(new Vector2d(-27, -36), Math.toRadians(-90))
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        BlueScanner scanner = new BlueScanner(telemetry);
        webcam.setPipeline(scanner);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode)
            {
            }
        });

        telemetry.setMsTransmissionInterval(50);

        waitForStart();
        Barcode result = scanner.getResult();
        switch (result) {
            case LEFT:
                telemetry.addData("Detected", "Left");
                break;
            case MIDDLE:
                telemetry.addData("Detected", "Middle");
                break;
            case RIGHT:
                telemetry.addData("Detected", "Right");
                break;
        }

        runTime.reset();

        if (isStopRequested()) return;

        state = PRELOAD_YELLOW;
        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            switch(state){
                case PRELOAD_YELLOW:
                    drive.followTrajectorySequenceAsync(preload1);
                    timer.reset();
                    state = PRELOAD_YELLOW_INTER;
                    break;

                case PRELOAD_YELLOW_INTER:
                    //Move intake servos perpendicular to the ground
                    //Move deposit out and deposit slides up
                    if(!drive.isBusy()){
                        timer.reset();
                        state = PRELOAD_YELLOW_DROP;
                    }
                    break;

                case PRELOAD_YELLOW_DROP:
                    //Drop pixel
                    state = PRELOAD_PURPLE;
                    break;

                case PRELOAD_PURPLE:
                    if(result == Barcode.LEFT || result == Barcode.RIGHT){
                        drive.followTrajectorySequenceAsync(preload2_v1);
                        state = PRELOAD_PURPLE_INTER;
                    }
                    if(result == Barcode.MIDDLE){
                        drive.followTrajectorySequenceAsync(preload2_v2);
                        state = PRELOAD_PURPLE_INTER;
                    }
                    break;

                case PRELOAD_PURPLE_INTER:
                    if(!drive.isBusy()){
                        state = PRELOAD_PURPLE_DROP;
                    }
                    break;

                case PRELOAD_PURPLE_DROP:
                    if(result == Barcode.LEFT || result == Barcode.RIGHT){
                        //extend intake
                        //move intake servos
                        //"spit" out purple pixel
                        state = CYCLE1_PRE;
                    }

                    if(result == Barcode.MIDDLE){
                        //move intake servos
                        //"spit" out purple pixel
                        state = CYCLE1_PRE;
                    }
                    break;

                case CYCLE1_PRE:
                    if(result == Barcode.LEFT || result == Barcode.RIGHT){
                        drive.followTrajectorySequenceAsync(cycle1_v1);
                        state = CYCLE1_INTAKE;
                    }
                    if(result == Barcode.MIDDLE){
                        drive.followTrajectorySequenceAsync(cycle1_v2);
                        state = CYCLE1_INTAKE;
                    }
                    break;

                case CYCLE1_INTAKE:
                    //
                    break;


                }

            drive.update();

            }
        }
}