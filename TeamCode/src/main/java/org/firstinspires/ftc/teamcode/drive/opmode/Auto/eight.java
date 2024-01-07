package org.firstinspires.ftc.teamcode.drive.opmode.Auto;


import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.Random.left;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE1_DEPOSIT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE1_INTAKE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE1_INTER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE1_INTER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE1_PRE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE1_TRANSFER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE1_TRANSFER1_5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE1_TRANSFER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE2_DEPOSIT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE2_INTAKE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE2_INTER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE2_INTER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE2_INTER_5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE2_TRANSFER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE2_TRANSFER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.CYCLE2_TRANSFER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.PRELOAD_PURPLE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.PRELOAD_PURPLE_DROP;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.PRELOAD_PURPLE_INTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.PRELOAD_YELLOW;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.PRELOAD_YELLOW_DROP;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.PRELOAD_YELLOW_INTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.SPIT1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.eight.State.SPIT2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

@Autonomous(name="Two + Six")
public class eight extends LinearOpMode {
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
        CYCLE1_INTER1,      // Start path to backdrop
        SPIT1,
        SPIT2,
        CYCLE1_INTER2,
        CYCLE1_TRANSFER1,   // Retract intake and transfer pixels
        CYCLE1_TRANSFER1_5,
        CYCLE1_TRANSFER2,   // Retract intake and transfer pixels
        CYCLE1_DEPOSIT,    // Drop pixels on backdrop
        CYCLE2_INTER_5,
        CYCLE2_INTER1,
        CYCLE2_INTAKE,
        CYCLE2_INTER2,
        CYCLE2_TRANSFER1,
        CYCLE2_TRANSFER2,
        CYCLE2_DEPOSIT,

        IDLE,              // Our bot will enter the IDLE state when done
    }

    private PIDController Lcontroller;
    public static double Lp = 0.006, Li = 0, Ld = 0.0001;

    //Ltarget Max 750, Min -75
    public static int Ltarget;

    private PIDController Ocontroller;
    public static int Otarget;
    public static double Op = 0.012, Oi = 0, Od = 0.0002;
    public static double Of = -0.08;
    public static double intakePower = 0;

    public Servo pivotleft;
    public Servo pivotright;
    public Servo elbowleft;
    public Servo elbowright;
    public Servo pivotOut;
    public Servo fourbar;
    public Servo wrist;
    public Servo outLeft;
    public Servo outRight;
    public Servo frontgrab;


    public DcMotorEx outtakeMotor;
    public DcMotorEx intakeMotor;
    public DcMotorEx intakeLeftExt;
    public DcMotorEx intakeRightExt;


    public static double p = 0.7;
    public static double e = 0.52;
    public static double r = 0.92;
    public static double v = 0.86;
    public static double x1 = 0.4;
    public static double y1 = 0.6;
    public static double z1 = 0.51;
    State state = PRELOAD_YELLOW;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    public TouchSensor intSensor1;
    public TouchSensor intSensor2;

    public boolean intake1 = false;
    public boolean intake2 = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Ocontroller = new PIDController(Op,Oi,Od);
        Lcontroller = new PIDController(Lp,Li,Ld);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);


        intSensor1 = hardwareMap.get(TouchSensor.class, "2");
        intSensor2 = hardwareMap.get(TouchSensor.class, "4");

        outtakeMotor = hardwareMap.get(DcMotorEx.class, "out");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        frontgrab = hardwareMap.get(Servo.class, "front");

        elbowleft = hardwareMap.get(Servo.class, "el");
        elbowright = hardwareMap.get(Servo.class, "er");
        pivotleft = hardwareMap.get(Servo.class, "pl");
        pivotright = hardwareMap.get(Servo.class, "pr");
        pivotOut = hardwareMap.get(Servo.class, "arm1");
        fourbar = hardwareMap.get(Servo.class, "arm2");
        wrist = hardwareMap.get(Servo.class, "wrist");

        outRight = hardwareMap.get(Servo.class, "outRight");
        outLeft = hardwareMap.get(Servo.class, "outLeft");

        intakeLeftExt = hardwareMap.get(DcMotorEx.class, "lint");
        intakeRightExt = hardwareMap.get(DcMotorEx.class, "rint");
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeftExt.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRightExt.setDirection(DcMotorSimple.Direction.REVERSE);

        //Preload 1
//                .splineTo(new Vector2d(10, 0), Math.toRadians(0))
//                .splineTo(new Vector2d(29, -7), Math.toRadians(-40))
//                .addDisplacementMarker(() -> {
//            frontgrab.setPosition(0.4);
//        })
//                .setReversed(true)
//                .back(2)
//                .splineTo(new Vector2d(20, 10), Math.toRadians(90))
//                .splineTo(new Vector2d(20, 36), Math.toRadians(90))

        //Preload 2
//                .lineToLinearHeading(new Pose2d(33, 9, Math.toRadians(-55)))
//                        .addDisplacementMarker(() -> {
//            frontgrab.setPosition(0.4);
//        })
//                .setReversed(true)
//                .back(4)
//                .splineTo(new Vector2d(25, 36), Math.toRadians(90))


        //Preload 3
//                .lineToLinearHeading(new Pose2d(37, 18, Math.toRadians(-90)))
//                        .addDisplacementMarker(() -> {
//            frontgrab.setPosition(0.4);
//        })
//                .setReversed(true)
//                .back(2)
//                .splineTo(new Vector2d(32, 36), Math.toRadians(90))

        TrajectorySequence preload1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(10, 0), Math.toRadians(0))
                .splineTo(new Vector2d(29, -7), Math.toRadians(-40))
                .addDisplacementMarker(() -> {
                    frontgrab.setPosition(0.4);
                })
                .setReversed(true)
                .back(2)
                .splineTo(new Vector2d(20, 10), Math.toRadians(90))
                .splineTo(new Vector2d(20, 36), Math.toRadians(90))
                .build();

        TrajectorySequence preload2 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(33, 9, Math.toRadians(-55)))
                .addDisplacementMarker(() -> {
                    frontgrab.setPosition(0.4);
                })
                .setReversed(true)
                .back(4)
                .splineTo(new Vector2d(25, 36), Math.toRadians(90))
                .build();

        TrajectorySequence preload3 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(37, 18, Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    frontgrab.setPosition(0.4);
                })
                .setReversed(true)
                .back(2)
                .splineTo(new Vector2d(32, 36), Math.toRadians(90))
                .build();

        TrajectorySequence preload2_v1_5 = drive.trajectorySequenceBuilder(preload1.end())
                .setReversed(false)
                .splineTo(new Vector2d(48, 10), Math.toRadians(-92.5))
                .splineTo(new Vector2d(48, -30.5), Math.toRadians(-92.5))
                .build();

        TrajectorySequence preload2_v2_5 = drive.trajectorySequenceBuilder(preload2.end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(48, 10, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(48, -18, Math.toRadians(-90)))
                .build();

        TrajectorySequence preload2_v3_5 = drive.trajectorySequenceBuilder(preload3.end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(48, 10, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(48, -18, Math.toRadians(-90)))
                .build();

        TrajectorySequence deposit_v1 = drive.trajectorySequenceBuilder(preload2_v1_5.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(48, 10))
                .lineToConstantHeading(new Vector2d(26, 35.5))
                .build();

        TrajectorySequence deposit_v2 = drive.trajectorySequenceBuilder(preload2_v2_5.end())
                .setReversed(true)
                .splineTo(new Vector2d(-50.5, -10), Math.toRadians(-90))
                .splineTo(new Vector2d(-27, -36.75), Math.toRadians(-90))
                .build();

        TrajectorySequence deposit_v3 = drive.trajectorySequenceBuilder(preload2_v3_5.end())
                .setReversed(true)
                .splineTo(new Vector2d(-50.5, -10), Math.toRadians(-90))
                .splineTo(new Vector2d(-27, -36.75), Math.toRadians(-90))
                .build();

        TrajectorySequence cycle2_v1 = drive.trajectorySequenceBuilder(deposit_v1.end())
                .setReversed(false)
                .splineTo(new Vector2d(48, 10), Math.toRadians(-92.5))
                .splineTo(new Vector2d(48, -32), Math.toRadians(-92.5))
                .build();

        TrajectorySequence cycle2_v1_5 = drive.trajectorySequenceBuilder(cycle2_v1.end())
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(-51, 31))
                .build();

        TrajectorySequence cycle2_v2 = drive.trajectorySequenceBuilder(deposit_v2.end())
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(-51, -15))
                .lineToConstantHeading(new Vector2d(-51, 31))
                .build();

        TrajectorySequence cycle2_v3 = drive.trajectorySequenceBuilder(deposit_v3.end())
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(-51, -15))
                .lineToConstantHeading(new Vector2d(-51, 31))
                .build();

        TrajectorySequence deposit_v1_2 = drive.trajectorySequenceBuilder(cycle2_v1.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(49, 10))
                .lineToConstantHeading(new Vector2d(26, 35.5))
                .build();

        TrajectorySequence deposit_v2_2 = drive.trajectorySequenceBuilder(cycle2_v2.end())
                .setReversed(true)
                .splineTo(new Vector2d(-50.5, -10), Math.toRadians(-90))
                .splineTo(new Vector2d(-27, -36), Math.toRadians(-90))
                .build();

        TrajectorySequence deposit_v3_2 = drive.trajectorySequenceBuilder(cycle2_v3.end())
                .setReversed(true)
                .splineTo(new Vector2d(-50.5, -10), Math.toRadians(-90))
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

        elbowleft.setPosition(0.4);
        elbowright.setPosition(1-0.4);
        pivotleft.setPosition(1-0.7);
        pivotright.setPosition(0.7);
        pivotOut.setPosition(0.92);
        fourbar.setPosition(0.86);
        outRight.setPosition(0.4);
        outLeft.setPosition(0.6);
        wrist.setPosition(0.51);
        frontgrab.setPosition(0);

        waitForStart();

        e = 0.4;
        p = 0.7;
        r = 0.92;
        v = 0.86;
        x1 = 0.4;
        y1 = 0.6;

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
                    Ltarget = 0;
                    Otarget = -5;
                    if(result == Barcode.LEFT){
                        drive.followTrajectorySequenceAsync(preload1);
                    }else if(result == Barcode.MIDDLE){
                        drive.followTrajectorySequenceAsync(preload2);
                    }else if(result == Barcode.RIGHT){
                        drive.followTrajectorySequenceAsync(preload3);
                    }
                    timer.reset();
                    state = PRELOAD_YELLOW_INTER;
                    break;

                case PRELOAD_YELLOW_INTER:
                    if(timer.seconds() > 0.5){
                        Otarget = 75;
                        e = 0.8;
                        p = 0.5;
                    }
                    if(timer.seconds() > 1.5){
                        r = 0.1;
                        v = 0.40;
                    }

                    if(!drive.isBusy() && timer.seconds() > 1.5){
                        timer.reset();
                        state = PRELOAD_YELLOW_DROP;
                    }
                    break;

                case PRELOAD_YELLOW_DROP:
                    if(timer.seconds() > 0){
                        x1 = 0;
                    }
                    //Drop pixel
                    if(timer.seconds() > 0.05){
                        e = 1;
                        p = 0.1;
                        timer.reset();
                        state = CYCLE1_PRE;
                    }
                    break;

                case CYCLE1_PRE:
                    if(timer.seconds() > 0){
                        e = 0.75;
                        Ltarget = 0;
                    }
                    if(timer.seconds() > 0.05){
                        if(result == Barcode.LEFT){
                            drive.followTrajectorySequenceAsync(preload2_v1_5);
                            timer.reset();
                            state = CYCLE1_INTAKE;
                        }
                        if(result == Barcode.MIDDLE){
                            drive.followTrajectorySequenceAsync(preload2_v2_5);
                            timer.reset();
                            state = CYCLE1_INTAKE;
                        }
                        if(result == Barcode.RIGHT){
                            drive.followTrajectorySequenceAsync(preload2_v3_5);
                            timer.reset();
                            state = CYCLE1_INTAKE;
                        }
                    }
                    break;

                case CYCLE1_INTAKE:
                    if(timer.seconds() > 0.25){
                        e = 0.55;
                        p = 0.23;
                        r = 0.92;
                        v = 0.86;
                    }
                    if(timer.seconds() > 0.75){
                        Otarget = 0;
                    }
                    if(!drive.isBusy()){
                        timer.reset();
                        state = CYCLE1_INTER1;
                    }
                    break;

                case CYCLE1_INTER1:
                    if(timer.seconds() > 0){
                        Ltarget = 870;
                        intakePower = -1;
                    }
                    if((timer.seconds() > 0.2 && Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 50) || timer.seconds() > 1){
                        timer.reset();
                        state = SPIT1;
                    }
                    break;

                case SPIT1:
                    if(timer.seconds() > 0.5){
                        timer.reset();
                        p = 0.21;
                        state = SPIT2;
                    }
                    break;

                case SPIT2:
                    if(intSensor1.isPressed()){
                        intake1 = true;
                    }
                    if(intSensor2.isPressed()){
                        intake2 = true;
                    }
                    if((intake1 && intake2)||timer.seconds() > 2){
                        timer.reset();
                        state = CYCLE1_INTER2;
                    }else if(timer.seconds() > 1 && timer.seconds() < 2){
                        p = 0.2;
                    }
                    break;

                case CYCLE1_INTER2:
                    if(timer.seconds() > 0 && timer.seconds() < 0.15){
                        intakePower = 1;
                    }else if(timer.seconds() > 0.15){
                        Ltarget = 0;
                        intakePower = -0.3;
                    }

                    if(timer.seconds() > 0.25 && Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 400){
                        if(result == Barcode.LEFT){
                            drive.followTrajectorySequenceAsync(deposit_v1);
                            timer.reset();
                            state = CYCLE1_TRANSFER1;
                        }
                        if(result == Barcode.MIDDLE){
                            drive.followTrajectorySequenceAsync(deposit_v2);
                            timer.reset();
                            state = CYCLE1_TRANSFER1;
                        }
                        if(result == Barcode.RIGHT){
                            drive.followTrajectorySequenceAsync(deposit_v3);
                            timer.reset();
                            state = CYCLE1_TRANSFER1;
                        }
                    }
                    break;

                case CYCLE1_TRANSFER1:
                    if(timer.seconds() > 0.5){
                        x1 = 0;
                        y1 = 1;
                        Otarget = 60;
                    }
                    if((Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 50)&& timer.seconds() < 0.51){
                        timer.reset();
                        state = CYCLE1_TRANSFER1_5;
                    }
                    break;

                case CYCLE1_TRANSFER1_5:
                    if(timer.seconds() > 0){
                        e = 0.785;
                        p = 0.8;
                    }
                    if(timer.seconds() > 0.75){
                        intakePower = 0.7;
                    }else if(timer.seconds() > 1.5){
                        intakePower = 0;
                    }

                    if(timer.seconds() > 1.51){
                        Otarget = -15;
                        timer.reset();
                        state = CYCLE1_TRANSFER2;
                    }
                break;

                case CYCLE1_TRANSFER2:
                    if(timer.seconds() > 0.25){
                        x1 = 0.4;
                        y1 = 0.6;
                    }

                    if(timer.seconds() > 0.5){
                        Otarget = 225;
                    }
                    if(timer.seconds() > 0.75){
                        r = 0.1;
                        v = 0.40;
                        e = 0.55;
                        p = 0.21;
                    }

                    if(!drive.isBusy() && timer.seconds() > 0.76){
                        p = 0.21;
                        timer.reset();
                        state = CYCLE1_DEPOSIT;
                    }
                    break;

                case CYCLE1_DEPOSIT:
                    if(timer.seconds() > 0){
                        x1 = 0;
                        y1 = 1;
                    }
                    if(timer.seconds() > 0.25){
                        r = 0.92; //deposit in correct position
                        v = 0.86; //deposit in correct position
                        e = 0.55;
                        p = 0.21;
                    }

                    if(timer.seconds() > 0.4){
                        if(result == Barcode.LEFT){
                            drive.followTrajectorySequenceAsync(cycle2_v1);
                            timer.reset();
                            state = CYCLE2_INTER_5;
                        }
                        if(result == Barcode.MIDDLE){
                            drive.followTrajectorySequenceAsync(cycle2_v2);
                            timer.reset();
                            state = CYCLE2_INTER1;
                        }
                        if(result == Barcode.RIGHT){
                            drive.followTrajectorySequenceAsync(cycle2_v3);
                            timer.reset();
                            state = CYCLE2_INTER1;
                        }
                    }
                    break;

                case CYCLE2_INTER_5:
                    if(timer.seconds() > 0.5){
                        Otarget = 0;
                        e = 0.55;
                        p = 0.21;
                    }
                    if(!drive.isBusy()){
                        timer.reset();
                        state = CYCLE2_INTAKE;
                    }
                    break;

                case CYCLE2_INTAKE:
                    if(timer.seconds() > 0){
                        Ltarget = 876;
                        intakePower = -1;
                    }
                    if((timer.seconds() > 0.2 && Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 50) || timer.seconds() > 1){
                        timer.reset();
                        state = CYCLE2_INTER2;
                    }
                    break;

                case CYCLE2_INTER2:
                    if(timer.seconds() > 0.5){
                        p = 0.17;
                    }

                    if(timer.seconds() > 1.3 && timer.seconds() < 1.4){
                        intakePower = 1;
                    }else if(timer.seconds() > 1.4){
                        intakePower = -0.3;
                    }

                    if(timer.seconds() > 1.5){
                        e = 0.785;
                        p = 0.8;
                        intakePower = 0;
                    }

                    if(timer.seconds() > 1.52){
                        Ltarget = 0;
                    }

                    if(timer.seconds() > 1.75 && Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 50){
                        if(result == Barcode.LEFT){
                            drive.followTrajectorySequenceAsync(deposit_v1_2);
                            timer.reset();
                            state = CYCLE2_TRANSFER1;
                        }
                        if(result == Barcode.MIDDLE){
                            drive.followTrajectorySequenceAsync(deposit_v2_2);
                            timer.reset();
                            state = CYCLE2_TRANSFER1;
                        }
                        if(result == Barcode.RIGHT){
                            drive.followTrajectorySequenceAsync(deposit_v3_2);
                            timer.reset();
                            state = CYCLE2_TRANSFER1;
                        }
                    }
                    break;

                case CYCLE2_TRANSFER1:
                    if(timer.seconds() > 0.5){
                        x1 = 0;
                        y1 = 1;
                        Otarget = 60;
                    }
                    if(timer.seconds() > 0.75){
                        intakePower = 0.7;
                    }

                    if(timer.seconds() > 1.5){
                        intakePower = 0;
                    }

                    if(timer.seconds() > 1.51){
                        Otarget = -15;
                        timer.reset();
                        state = CYCLE2_TRANSFER2;
                    }
                    break;

                case CYCLE2_TRANSFER2:
                    if(timer.seconds() > 0.25){
                        x1 = 0.4;
                        y1 = 0.6;
                    }

                    if(timer.seconds() > 0.5){
                        Otarget = 225;
                    }
                    if(timer.seconds() > 0.75){
                        r = 0.1;
                        v = 0.40;
                    }

                    if(!drive.isBusy() && timer.seconds() > 0.76){
                        timer.reset();
                        state = CYCLE2_DEPOSIT;
                    }
                    break;

                case CYCLE2_DEPOSIT:
                    if(timer.seconds() > 0){
                        x1 = 0;
                        y1 = 1;
                    }
                    if(timer.seconds() > 0.25){
                        r = 0.92; //deposit in correct position
                        v = 0.86; //deposit in correct position
                    }
                    break;


            }

            elbowleft.setPosition(e);
            elbowright.setPosition(1-e);
            pivotleft.setPosition(1-p);
            pivotright.setPosition(p);
            pivotOut.setPosition(r);
            fourbar.setPosition(v);
            outRight.setPosition(x1);
            outLeft.setPosition(y1);
            wrist.setPosition(z1);
            intakeMotor.setPower(intakePower);

            Lcontroller.setPID(Lp, Li, Ld);

            int armPos = intakeLeftExt.getCurrentPosition();

            double Lpid = Lcontroller.calculate(armPos, Ltarget);

            double Lpower = Lpid;

            intakeLeftExt.setPower(Lpower);
            intakeRightExt.setPower(Lpower);


            Ocontroller.setPID(Op, Oi, Od);

            int outPos = outtakeMotor.getCurrentPosition();

            double Opid = Ocontroller.calculate(outPos, -Otarget);
            double Off = Of;


            double Opower = Opid + Off;

            outtakeMotor.setPower(Opower);

            drive.update();

        }
    }
}