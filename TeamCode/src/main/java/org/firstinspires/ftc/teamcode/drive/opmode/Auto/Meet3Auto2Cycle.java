package org.firstinspires.ftc.teamcode.drive.opmode.Auto;


import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.Random.left;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE1_DEPOSIT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE1_INTAKE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE1_INTER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE1_INTER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE1_PRE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE1_TRANSFER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE1_TRANSFER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE2_DEPOSIT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE2_INTAKE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE2_INTER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE2_INTER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE2_INTER_5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE2_TRANSFER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE2_TRANSFER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE3_DEPOSIT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE3_INTAKE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE3_INTER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE3_INTER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE3_INTER_5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE3_TRANSFER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.CYCLE3_TRANSFER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.PARK;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.PRELOAD_YELLOW;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.PRELOAD_YELLOW_DROP;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.PRELOAD_YELLOW_INTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.SPIT1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Meet3Auto2Cycle.State.SPIT2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.Barcode;
import org.firstinspires.ftc.teamcode.vision.BlueScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="meet 3 auto 2 cycle")
public class Meet3Auto2Cycle extends LinearOpMode {
    OpenCvWebcam webcam;

    public enum Random {
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
        CYCLE1_TRANSFER2,   // Retract intake and transfer pixels
        CYCLE1_DEPOSIT,    // Drop pixels on backdrop
        CYCLE2_INTER_5,
        CYCLE2_INTER1,
        CYCLE2_INTAKE,
        CYCLE2_INTER2,
        CYCLE2_TRANSFER1,
        CYCLE2_TRANSFER2,
        CYCLE2_DEPOSIT,
        CYCLE3_INTER_5,
        CYCLE3_INTER1,
        CYCLE3_INTAKE,
        CYCLE3_INTER2,
        CYCLE3_TRANSFER1,
        CYCLE3_TRANSFER2,
        CYCLE3_DEPOSIT,
        PARK,

        IDLE,              // Our bot will enter the IDLE state when done
    }
    private PIDController Lcontroller;
    public static double Lp = 0.006, Li = 0, Ld = 0.0001;

    //Ltarget Max 750, Min -75
    public static int Ltarget;

    private PIDController Ocontroller;
    public static int Otarget;

    public static double Op = 0.009, Oi = 0, Od = 0.0004;
    public static double Of = -0.17;
    public static double intakePower = 0;
    public Servo pivotleft;
    public Servo pivotright;
    public Servo elbowleft;
    public Servo elbowright;
    public Servo pan;
    public Servo tilt;
    public Servo wrist;
    public Servo claw1;
    public Servo claw2;
    public Servo frontgrab;

    private IMU imu;

    public DcMotorEx outtakeMotor;
    public DcMotorEx intakeMotor;
    public DcMotorEx intakeLeftExt;
    public DcMotorEx intakeRightExt;

    public static double p = 0.7;
    public static double e = 0.52;

    public static double wpos = 0.48;
    public static double ppos = 0.48;
    public static double tpos = 0.31;
    public static double c1pos = 0.5;
    public static double c2pos = 0.42;
    public static double apos = 0.51;
    public static double pposadd = 0.468;
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
        intakeLeftExt = hardwareMap.get(DcMotorEx.class, "lint");
        intakeRightExt = hardwareMap.get(DcMotorEx.class, "rint");

        frontgrab = hardwareMap.get(Servo.class, "front");

        elbowleft = hardwareMap.get(Servo.class, "el");
        elbowright = hardwareMap.get(Servo.class, "er");
        pivotleft = hardwareMap.get(Servo.class, "pl");
        pivotright = hardwareMap.get(Servo.class, "pr");

        pan = hardwareMap.get(Servo.class, "0.");
        wrist = hardwareMap.get(Servo.class, "1.");
        tilt = hardwareMap.get(Servo.class, "5.");
        claw1 = hardwareMap.get(Servo.class, "2.");
        claw2 = hardwareMap.get(Servo.class, "4.");

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeftExt.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRightExt.setDirection(DcMotorSimple.Direction.REVERSE);

        TrajectorySequence preload1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(21, 17, Math.toRadians(-40)))
                .addDisplacementMarker(() -> {
                    frontgrab.setPosition(0.4);
                })
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(22, 37, Math.toRadians(-60)))
                .build();

        TrajectorySequence preload2 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(33, 9, Math.toRadians(-55)))
                .setReversed(true)
                .back(4)
                .splineTo(new Vector2d(24, 37), Math.toRadians(90))
                .build();

        TrajectorySequence preload3 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(10, 0), Math.toRadians(0))
                .splineTo(new Vector2d(29, -7), Math.toRadians(-40))
                .setReversed(true)
                .back(2)
                .splineTo(new Vector2d(25, 10), Math.toRadians(90))
                .splineTo(new Vector2d(31, 37), Math.toRadians(90))
                .build();

        TrajectorySequence preload2_v1_5 = drive.trajectorySequenceBuilder(preload1.end())
                .setReversed(false)
                .splineTo(new Vector2d(50, 18), Math.toRadians(-91))
                .splineToLinearHeading(new Pose2d(50, -30.5, Math.toRadians(-91)), Math.toRadians(270))
                .build();

        TrajectorySequence preload2_v2_5 = drive.trajectorySequenceBuilder(preload2.end())
                .setReversed(false)
                .splineTo(new Vector2d(50, 18), Math.toRadians(-91))
                .splineToLinearHeading(new Pose2d(50, -30.5, Math.toRadians(-91)), Math.toRadians(270))
                .build();

        TrajectorySequence preload2_v3_5 = drive.trajectorySequenceBuilder(preload3.end())
                .setReversed(false)
                .splineTo(new Vector2d(50, 18), Math.toRadians(-91))
                .splineToLinearHeading(new Pose2d(50, -30.5, Math.toRadians(-91)), Math.toRadians(270))
                .build();

        TrajectorySequence deposit_v1 = drive.trajectorySequenceBuilder(preload2_v1_5.end())
                .setReversed(true)
                .splineTo(new Vector2d(50, 10), Math.toRadians(90))
                .splineTo(new Vector2d(34, 37.75), Math.toRadians(-250))
                .build();

        TrajectorySequence deposit_v2 = drive.trajectorySequenceBuilder(preload2_v2_5.end())
                .setReversed(true)
                .splineTo(new Vector2d(50, 10), Math.toRadians(90))
                .splineTo(new Vector2d(34, 37.75), Math.toRadians(-250))
                .build();

        TrajectorySequence deposit_v3 = drive.trajectorySequenceBuilder(preload2_v3_5.end())
                .setReversed(true)
                .splineTo(new Vector2d(50, 10), Math.toRadians(90))
                .splineTo(new Vector2d(34, 37.75), Math.toRadians(-250))
                .build();

        TrajectorySequence cycle2_v1 = drive.trajectorySequenceBuilder(deposit_v1.end())
                .setReversed(false)
                .splineTo(new Vector2d(50, 18), Math.toRadians(-90.75))
                .splineTo(new Vector2d(50, -30.5), Math.toRadians(-90.75))
                .build();

        TrajectorySequence cycle2_v2 = drive.trajectorySequenceBuilder(deposit_v2.end())
                .setReversed(false)
                .splineTo(new Vector2d(50, 18), Math.toRadians(-90.75))
                .splineTo(new Vector2d(50, -30.5), Math.toRadians(-90.75))
                .build();

        TrajectorySequence cycle2_v3 = drive.trajectorySequenceBuilder(deposit_v3.end())
                .setReversed(false)
                .splineTo(new Vector2d(50, 18), Math.toRadians(-90.75))
                .splineTo(new Vector2d(50, -30.5), Math.toRadians(-90.75))
                .build();

        TrajectorySequence deposit_v1_2 = drive.trajectorySequenceBuilder(cycle2_v1.end())
                .setReversed(true)
                .splineTo(new Vector2d(49, 10), Math.toRadians(90))
                .splineTo(new Vector2d(34, 37.75), Math.toRadians(-250))
                .build();

        TrajectorySequence deposit_v2_2 = drive.trajectorySequenceBuilder(cycle2_v2.end())
                .setReversed(true)
                .splineTo(new Vector2d(49, 10), Math.toRadians(90))
                .splineTo(new Vector2d(34, 37.75), Math.toRadians(-250))
                .build();

        TrajectorySequence deposit_v3_2 = drive.trajectorySequenceBuilder(cycle2_v3.end())
                .setReversed(true)
                .splineTo(new Vector2d(49, 10), Math.toRadians(90))
                .splineTo(new Vector2d(34, 37.75), Math.toRadians(-250))
                .build();

        TrajectorySequence cycle3_v1 = drive.trajectorySequenceBuilder(deposit_v1_2.end())
                .setReversed(false)
                .splineTo(new Vector2d(52, 18), Math.toRadians(-90.75))
                .splineTo(new Vector2d(52, -30.5), Math.toRadians(-90.75))
                .turn(Math.toRadians(-15))
                .build();

        TrajectorySequence deposit_v1_3 = drive.trajectorySequenceBuilder(cycle3_v1.end())
                .setReversed(true)
                .turn(Math.toRadians(15))
                .splineTo(new Vector2d(49, 10), Math.toRadians(90))
                .splineTo(new Vector2d(34, 36.25), Math.toRadians(-250))
                .build();

        TrajectorySequence park_1 = drive.trajectorySequenceBuilder(deposit_v1_2.end())
                .forward(1)
                .build();

        TrajectorySequence park_2 = drive.trajectorySequenceBuilder(deposit_v2_2.end())
                .forward(1)
                .build();

        TrajectorySequence park_3 = drive.trajectorySequenceBuilder(deposit_v3_2.end())
                .forward(1)
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

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        telemetry.setMsTransmissionInterval(50);

        elbowleft.setPosition(0.4);
        elbowright.setPosition(1-0.4);
        pivotleft.setPosition(1-0.7);
        pivotright.setPosition(0.7);
        wrist.setPosition(0.48);
        pan.setPosition(0.47);
        claw1.setPosition(0.5);
        claw2.setPosition(0.44);

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
                    if(timer.seconds() > 1 && result != Barcode.LEFT){
                        frontgrab.setPosition(0.4);
                    }else if(timer.seconds() > 1.4){
                        frontgrab.setPosition(0.4);
                    }
                    if(timer.seconds() > 0.25){
                        Otarget = 250;
                        e = 0.8;
                        p = 0.5;
                    }
                    if(timer.seconds() > 0.75){
                        tpos = 0.68;
                    }

                    double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    double angle = Math.toDegrees(currentHeading) + 90;

                    if(timer.seconds() > 0.95){
                        ppos = Range.clip(-0.00603*(angle)+pposadd, 0.22, 0.72);
                    }

                    if(!drive.isBusy() && timer.seconds() > 1.5){
                        timer.reset();
                        state = PRELOAD_YELLOW_DROP;
                    }
                    break;

                case PRELOAD_YELLOW_DROP:
                    if(timer.seconds() > 0){
                        c1pos = 0;
                        c2pos = 0.98;
                        e = 0.38;
                        p = 0.26;
                        timer.reset();
                        state = CYCLE1_PRE;
                    }
                    break;

                case CYCLE1_PRE:
                    if(timer.seconds() > 0){
                        Ltarget = 0;
                    }
                    if(timer.seconds() > 0 && Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 50){
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
                    if(timer.seconds() > 0.2){
                        ppos = 0.48;
                        e = 0.38;
                        p = 0.26;
                    }
                    if(timer.seconds() > 0.35){
                        Otarget = 0;
                    }
                    if(timer.seconds() > 0.25){
                        tpos = 0.31;
                    }
                    if(timer.seconds() > 2.3){
                        intakePower = -1;
                        Ltarget = 825;
                    }
                    if(!drive.isBusy()){
                        timer.reset();
                        state = CYCLE1_INTER1;
                    }
                    break;

                case CYCLE1_INTER1:
                    if((Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 50) || timer.seconds() > 0.5){
                        timer.reset();
                        state = SPIT1;
                    }
                    break;

                case SPIT1:
                    if(timer.seconds() > 0){
                        timer.reset();
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
                    if((intake1 && intake2)||timer.seconds() > 1){
                        intake1 = false;
                        intake2 = false;
                        timer.reset();
                        state = CYCLE1_INTER2;
                    }
                    break;

                case CYCLE1_INTER2:
                    if(timer.seconds() > 0){
                        Ltarget = 0;
                        e = 0.72;
                        p = 0.83;
                        intakePower = 0;
                    }

                    if(timer.seconds() > 0.1 && Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 650){
                        if(result == Barcode.LEFT){
                            p = 0.83;
                            drive.followTrajectorySequenceAsync(deposit_v1);
                            timer.reset();
                            state = CYCLE1_TRANSFER1;
                        }
                        if(result == Barcode.MIDDLE){
                            p = 0.83;
                            drive.followTrajectorySequenceAsync(deposit_v2);
                            timer.reset();
                            state = CYCLE1_TRANSFER1;
                        }
                        if(result == Barcode.RIGHT){
                            p = 0.83;
                            drive.followTrajectorySequenceAsync(deposit_v3);
                            timer.reset();
                            state = CYCLE1_TRANSFER1;
                        }
                    }
                    break;

                case CYCLE1_TRANSFER1:
                    if(timer.seconds() > 0){
                        c1pos = 0;
                        c2pos = 0.98;
                        Otarget = 50;
                    }
                    if(timer.seconds() > 0.65){
                        intakePower = 1;
                    }

                    if(timer.seconds() > 2){
                        Otarget = -150;
                    }

                    if(timer.seconds() > 2.5){
                        intakePower = 0;
                        Otarget = -300;
                        c1pos = 0.5;
                        c2pos = 0.42;
                        timer.reset();
                        state = CYCLE1_TRANSFER2;
                    }
                    break;

                case CYCLE1_TRANSFER2:
                    if(timer.seconds() > 1.25){
                        Otarget = 225;
                    }
                    if(timer.seconds() > 1.75){
                        tpos = 0.69;
                    }

                    currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    angle = Math.toDegrees(currentHeading) + 90;

                    if(timer.seconds() > 1.9){
                        ppos = Range.clip(-0.00603*(angle)+pposadd, 0.22, 0.72);
                    }
                    if(!drive.isBusy() && timer.seconds() > 2.5){
                        p = 0.21;
                        c1pos = 0;
                        c2pos = 0.98;
                        timer.reset();
                        state = CYCLE1_DEPOSIT;
                    }
                    break;

                case CYCLE1_DEPOSIT:
                    if(timer.seconds() > 0.2){
                        if(result == Barcode.LEFT){
                            drive.followTrajectorySequenceAsync(cycle2_v1);
                            timer.reset();
                            state = CYCLE2_INTER_5;
                        }
                        if(result == Barcode.MIDDLE){
                            drive.followTrajectorySequenceAsync(cycle2_v2);
                            timer.reset();
                            state = CYCLE2_INTER_5;
                        }
                        if(result == Barcode.RIGHT){
                            drive.followTrajectorySequenceAsync(cycle2_v3);
                            timer.reset();
                            state = CYCLE2_INTER_5;
                        }
                    }
                    break;

                case CYCLE2_INTER_5:
                    if(timer.seconds() > 0.25){
                        ppos = 0.48;
                    }

                    if(timer.seconds() > 0.4){
                        tpos = 0.31;
                    }

                    if(timer.seconds() > 1){
                        Otarget = 0;
                        e = 0.55;
                        p = 0.18;
                    }

                    if(timer.seconds() > 2){
                        intakePower = -1;
                        Ltarget = 880;
                    }

                    if(!drive.isBusy()){
                        timer.reset();
                        state = CYCLE2_INTER1;
                    }
                    break;

                case CYCLE2_INTER1:
                    if((Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 50) || timer.seconds() > 0.5){
                        timer.reset();
                        state = CYCLE2_INTAKE;
                    }
                    break;

                case CYCLE2_INTAKE:
                    if(intSensor1.isPressed()){
                        intake1 = true;
                    }
                    if(intSensor2.isPressed()){
                        intake2 = true;
                    }
                    if((intake1 && intake2)||timer.seconds() > 1){
                        intake1 = false;
                        intake2 = false;
                        timer.reset();
                        state = CYCLE2_INTER2;
                    }
                    break;

                case CYCLE2_INTER2:
                    if(timer.seconds() > 0){
                        Ltarget = 0;
                        e = 0.72;
                        p = 0.83;
                        intakePower = 0;
                    }

                    if(timer.seconds() > 0.1 && Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 650){
                        if(result == Barcode.LEFT){
                            p = 0.83;
                            drive.followTrajectorySequenceAsync(deposit_v1_2);
                            timer.reset();
                            state = CYCLE2_TRANSFER1;
                        }
                        if(result == Barcode.MIDDLE){
                            p = 0.83;
                            drive.followTrajectorySequenceAsync(deposit_v2_2);
                            timer.reset();
                            state = CYCLE2_TRANSFER1;
                        }
                        if(result == Barcode.RIGHT){
                            p = 0.83;
                            drive.followTrajectorySequenceAsync(deposit_v3_2);
                            timer.reset();
                            state = CYCLE2_TRANSFER1;
                        }
                    }
                    break;

                case CYCLE2_TRANSFER1:
                    if(timer.seconds() > 0){
                        c1pos = 0;
                        c2pos = 0.98;
                        Otarget = 50;
                    }
                    if(timer.seconds() > 0.65){
                        intakePower = 1;
                    }

                    if(timer.seconds() > 2){
                        Otarget = -150;
                    }

                    if(timer.seconds() > 2.5){
                        intakePower = 0;
                        Otarget = -300;
                        c1pos = 0.5;
                        c2pos = 0.42;
                        timer.reset();
                        state = CYCLE2_TRANSFER2;
                    }
                    break;

                case CYCLE2_TRANSFER2:
                    if(timer.seconds() > 1){
                        Otarget = 350;
                    }

                    if(timer.seconds() > 1.65){
                        tpos = 0.69;
                        //HERE
                    }
                    currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    angle = Math.toDegrees(currentHeading) + 90;

                    if(timer.seconds() > 1.8){
                        ppos = Range.clip(-0.00603*(angle)+pposadd, 0.22, 0.72);
                    }
                    if(!drive.isBusy() && timer.seconds() > 2.5){
                        p = 0.21;
                        c1pos = 0;
                        c2pos = 0.98;
                        timer.reset();
                        state = CYCLE2_DEPOSIT;
                    }
                    break;

                case CYCLE2_DEPOSIT:
                    if(timer.seconds() > 0.15){
                        if(result == Barcode.LEFT){
//                            drive.followTrajectorySequenceAsync(cycle3_v1);
//                            timer.reset();
//                            state = CYCLE3_INTER_5;
                        }
                        if(result == Barcode.MIDDLE){
//                            drive.followTrajectorySequenceAsync(cycle_v2);
//                            timer.reset();
//                            state = CYCLE2_INTER_5;
                        }
                        if(result == Barcode.RIGHT){
//                            drive.followTrajectorySequenceAsync(cycle2_v3);
//                            timer.reset()
//                            state = CYCLE2_INTER_5;
                        }
                    }
                    break;

                case CYCLE3_INTER_5:
                    if(timer.seconds() > 0.25){
                        ppos = 0.48;
                    }

                    if(timer.seconds() > 0.4){
                        tpos = 0.31;
                    }

                    if(timer.seconds() > 1){
                        Otarget = 0;
                        e = 0.55;
                        p = 0.18;
                    }

                    if(!drive.isBusy()){
                        intakePower = -1;
                        Ltarget = 950;
                        timer.reset();
                        state = CYCLE3_INTER1;
                    }
                    break;

                case CYCLE3_INTER1:
                    if((Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 50)){
                        timer.reset();
                        state = CYCLE3_INTAKE;
                    }
                    break;

                case CYCLE3_INTAKE:
                    if(intSensor1.isPressed()){
                        intake1 = true;
                    }
                    if(intSensor2.isPressed()){
                        intake2 = true;
                    }
                    if((intake1 && intake2)||timer.seconds() > 2){
                        intake1 = false;
                        intake2 = false;
                        timer.reset();
                        state = CYCLE3_INTER2;
                    }
                    break;

                case CYCLE3_INTER2:
                    if(timer.seconds() > 0){
                        Ltarget = 0;
                        e = 0.72;
                        p = 0.83;
                        intakePower = 0;
                    }


                    if(timer.seconds() > 0.1 && Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 650){
                        if(result == Barcode.LEFT){
                            p = 0.83;
                            drive.followTrajectorySequenceAsync(deposit_v1_3);
                            timer.reset();
                            state = CYCLE3_TRANSFER1;
                        }
                        if(result == Barcode.MIDDLE){
                            p = 0.83;
////                            drive.followTrajectorySequenceAsync(deposit_v2_3);
//                            timer.reset();
//                            state = CYCLE3_TRANSFER1;
                        }
                        if(result == Barcode.RIGHT){
                            p = 0.83;
//                            drive.followTrajectorySequenceAsync(deposit_v3_3);
//                            timer.reset();
//                            state = CYCLE3_TRANSFER1;
                        }
                    }
                    break;

                case CYCLE3_TRANSFER1:
                    if(timer.seconds() > 0){
                        c1pos = 0;
                        c2pos = 0.98;
                        Otarget = 100;
                    }
                    if(timer.seconds() > 0.65){
                        intakePower = 1;
                    }

                    if(timer.seconds() > 1.5){
                        Otarget = -150;
                    }

                    if(timer.seconds() > 1.8){
                        intakePower = 0;
                        Otarget = -300;
                        c1pos = 0.5;
                        c2pos = 0.42;
                        timer.reset();
                        state = CYCLE3_TRANSFER2;
                    }
                    break;

                case CYCLE3_TRANSFER2:
                    if(timer.seconds() > 0.55){
                        Otarget = 400;
                    }

                    if(timer.seconds() > 0.7){
                        tpos = 0.69;
                    }

                    currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    angle = Math.toDegrees(currentHeading) + 90;

                    if(timer.seconds() > 0.75){
                        ppos = Range.clip(-0.00603*(angle)+pposadd, 0.22, 0.72);
                    }
                    if(!drive.isBusy() && timer.seconds() > 0.81){
                        p = 0.21;
                        c1pos = 0;
                        c2pos = 0.98;
                        timer.reset();
                        state = CYCLE3_DEPOSIT;
                    }
                    break;

                case CYCLE3_DEPOSIT:
                    if(timer.seconds() > 0.25){
                        ppos = 0.48;
                    }

                    if(timer.seconds() > 0.41){
                        tpos = 0.31;
                    }
                    break;

                case PARK:
                    if(!drive.isBusy()){
//                        r = 0.92; //deposit in correct position
//                        v = 0.86; //deposit in correct position
                    }
                    if(timer.seconds() > 0.5){
//                        r = 0.92; //deposit in correct position
//                        v = 0.86; //deposit in correct position
                    }
                    break;

            }

            elbowleft.setPosition(e);
            elbowright.setPosition(1-e);
            pivotleft.setPosition(1-p);
            pivotright.setPosition(p);

            pan.setPosition(ppos);
            tilt.setPosition(tpos);
            wrist.setPosition(wpos);
            claw1.setPosition(c1pos);
            claw2.setPosition(c2pos);
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