package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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

@Autonomous(group = "Auto")
public class Auto extends LinearOpMode {
    public DcMotorEx intakeLeftExt;
    public DcMotorEx outtakeMotor;
    public DcMotorEx intakeRightExt;
    private PIDController Lcontroller;
    private PIDController Ocontroller;

    public static double r = 0.9;
    public static double v = 0.9;

    public static double p = 0.7;
    public static double e = 0.52;

    public static double x1 = 0.4;
    public static double y1 = 0.6;
    public static double z1 = 0.51;
    public static double Lp = 0.006, Li = 0, Ld = 0.0001;

    //Ltarget Max 750, Min -75
    public static int Ltarget;

    //Otarget Max 800, Min 25
    public static int Otarget;

    public static double Op = 0.012, Oi = 0, Od = 0.0002;
    public static double Of = -0.08;
    public Servo outLeft;
    public Servo outRight;
    public Servo pivotleft;

    public Servo pivotright;

    public Servo elbowleft;

    public Servo elbowright;
    public Servo pivotOut;
    public Servo fourbar;
    public Servo wrist;
    public CRServo lift;

    private boolean test = false;
    private ElapsedTime runTime = new ElapsedTime();


    OpenCvWebcam webcam;


    private enum Random {
        left,
        middle,
        right
    }

    private DcMotorEx intakeMotor;
    Random randomization = Random.left;

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        TURN_1,         // Then we want to do a point turn
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        WAIT_1,         // Then we're gonna wait a second
        TURN_2,         // Finally, we're gonna turn again
        IDLE,            // Our bot will enter the IDLE state when done
        AAA,
        BBB
    }

    State currentState = State.IDLE;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d cyclePose = new Pose2d(0, 0, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        Lcontroller = new PIDController(Lp,Li,Ld);
        Ocontroller = new PIDController(Op,Oi,Od);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");


        intakeLeftExt = hardwareMap.get(DcMotorEx.class, "lint");
        intakeRightExt = hardwareMap.get(DcMotorEx.class, "rint");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "out");

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeftExt.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRightExt.setDirection(DcMotorSimple.Direction.REVERSE);

        elbowleft = hardwareMap.get(Servo.class, "el");
        elbowright = hardwareMap.get(Servo.class, "er");
        pivotleft = hardwareMap.get(Servo.class, "pl");
        pivotright = hardwareMap.get(Servo.class, "pr");
        pivotOut = hardwareMap.get(Servo.class, "arm1");
        fourbar = hardwareMap.get(Servo.class, "arm2");
        wrist = hardwareMap.get(Servo.class, "wrist");

        outRight = hardwareMap.get(Servo.class, "outRight");
        outLeft = hardwareMap.get(Servo.class, "outLeft");

        ElapsedTime timer  = new ElapsedTime();

        elbowleft.setPosition(0.4);
        elbowright.setPosition(1-0.4);
        pivotleft.setPosition(1-0.7);
        pivotright.setPosition(0.7);
        pivotOut.setPosition(0.9);
        fourbar.setPosition(0.9);
        outRight.setPosition(0.4);
        outLeft.setPosition(0.6);
        wrist.setPosition(0.51);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(15, 0, Math.toRadians(0)))
                .build();

        TrajectorySequence trajectoryright = drive.trajectorySequenceBuilder(trajectory1.end())
                .turn(Math.toRadians(-48))
                .waitSeconds(7)
                .turn(Math.toRadians(144))
                .build();

        TrajectorySequence trajectoryleft = drive.trajectorySequenceBuilder(trajectory1.end())
                .turn(Math.toRadians(38))
                .turn(Math.toRadians(58))
                .build();

        TrajectorySequence trajectorymid = drive.trajectorySequenceBuilder(trajectory1.end())
                .forward(7)
                .waitSeconds(1)
                .back(7)
                .waitSeconds(7)
                .turn(Math.toRadians(58))
                .turn(Math.toRadians(38))
                .build();

        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(cyclePose)
                .back(38)
                .waitSeconds(0.5)
                .back(2)
                .build();

        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(cyclePose)
                .lineToConstantHeading(new Vector2d(0, -35))
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(0, 20))
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(0, -35))
                .build();

        // Define the angle to turn at
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

        waitForStart();

        runTime.reset();

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
        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.TRAJECTORY_1;

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    intakeMotor.setPower(-0.2);
                    p = 0.1;
                    Ltarget = 600;
                    timer.reset();
                    currentState = State.TRAJECTORY_2;
                    break;
                case TRAJECTORY_2:
                    if(timer.seconds() > 0.25){
                        e = 0.75;
                    }
                    if (!drive.isBusy()) {
                        e = 0.75;
                        timer.reset();
                        currentState = State.AAA;
                        timer.reset();
                    }
                    break;

                case TURN_1:
                    if(result == Barcode.LEFT){
                        drive.followTrajectorySequenceAsync(trajectoryleft);
                    }
                    else if(result == Barcode.RIGHT){
                        drive.followTrajectorySequenceAsync(trajectoryright);
                    }
                    else if(result == Barcode.MIDDLE){
                        drive.followTrajectorySequenceAsync(trajectorymid);
                    }
                    timer.reset();
                    currentState = State.TRAJECTORY_3;
                    break;
                case TRAJECTORY_3:
                    if(timer.seconds() > 4){
                        x1 = 0;
                    }
                    if(timer.seconds() > 5){
                    if (!drive.isBusy()) {
                        r = 0.08;
                        drive.setPoseEstimate(cyclePose);
                        currentState = State.AAA;
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                    }
                    }
                    break;
                case WAIT_1:
                    drive.followTrajectorySequenceAsync(trajectory3);
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    timer.reset();
                    currentState = State.TURN_2;
                    break;
                case TURN_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (timer.seconds() > 4) {
                        timer.reset();
                        y1 = 1;
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    if(timer.seconds() > 1){
                    r = 0.9;
                    v = 0.9;
                    currentState = State.AAA;
                    }
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;

                case AAA:
                    if(timer.seconds() > 5){
                        intakeMotor.setPower(1);
                    }
                    else if(timer.seconds() > 8) {
                        intakeMotor.setPower(0);
                    }
                    break;
            }

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

            elbowleft.setPosition(e);
            elbowright.setPosition(1-e);
            pivotleft.setPosition(1-p);
            pivotright.setPosition(p);
            pivotOut.setPosition(r);
            fourbar.setPosition(v);
            outRight.setPosition(x1);
            outLeft.setPosition(y1);
            wrist.setPosition(z1);

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }


    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware
        }

        public void update() {
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift
        }
    }
}