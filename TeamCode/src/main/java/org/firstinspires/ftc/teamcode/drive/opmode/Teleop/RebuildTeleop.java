package org.firstinspires.ftc.teamcode.drive.opmode.Teleop;


import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.barrier;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.barrierinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.base;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.cancel;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.cancelinter1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.cancelinter2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.deposit;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.initialize;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.intake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.intakeinter1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.outtake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.outtakeinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.outtakepre;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.pre;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.transfer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name="Teleops ")
public class RebuildTeleop extends OpMode {

    private IMU imu;
    private double slow = 1.0;
    private double turnslow = 1.0;
    private double progSlowmode = 1.0;

    private boolean left = false;
    private boolean right = false;
    boolean onOff = false;
    public DcMotorEx intakeLeftExt;
    public DcMotorEx outtakeMotor;
    public DcMotorEx intakeRightExt;
    private PIDController Lcontroller;
    private PIDController Ocontroller;


    public static double Lp = 0.006, Li = 0, Ld = 0.0001;

    //Ltarget Max 750, Min -75
    public static int Ltarget;

    //Otarget Max 800, Min 25
    public static int Otarget;

    public static double Op = 0.012, Oi = 0, Od = 0.0002;
    public static double Of = -0.08;



    public DcMotorEx intakeMotor;

    public static double p = 0.7;
    public static double e = 0.52;

    public static double r = 0.9;
    public static double v = 0.9;
    public static double x1 = 0;
    public static double y1 = 1;
    public static double z1 = 0.95;

    public Servo pivotleft;

    public Servo pivotright;

    public Servo elbowleft;

    public Servo elbowright;
    public Servo pivotOut;
    public Servo fourbar;
    public Servo wrist;
    public CRServo lift;

    private int counter = 0;

    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private boolean posLockOut = false;
    private boolean posLockInt = false;
    public enum state {
        cancel,
        cancelinter1,
        cancelinter2,
        pre,
        initialize,
        intslides,
        base,
        intake,
        intakeinter1,
        transfer,
        outtake,
        outtakepre,
        outtakeinter,
        barrier,
        barrierinter,
        deposit
    }

    public double targetAngleOut = Math.toRadians(90);
    public double targetAngleInt = Math.toRadians(90);
    public double currentAngle;

    private boolean fortnite = false;

    state New = pre;
    ElapsedTime timer  = new ElapsedTime();
    ElapsedTime holdtimer  = new ElapsedTime();

    public Servo outLeft;
    public Servo outRight;

    public static double Hp = 0.03;




    @Override
    public void init(){


        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        Lcontroller = new PIDController(Lp,Li,Ld);
        Ocontroller = new PIDController(Op,Oi,Od);

        intakeLeftExt = hardwareMap.get(DcMotorEx.class, "lint");
        intakeRightExt = hardwareMap.get(DcMotorEx.class, "rint");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "out");

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeftExt.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRightExt.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "lf");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rb");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rf");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = hardwareMap.get(CRServo.class, "lift");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
//
        elbowleft = hardwareMap.get(Servo.class, "el");
        elbowright = hardwareMap.get(Servo.class, "er");
        pivotleft = hardwareMap.get(Servo.class, "pl");
        pivotright = hardwareMap.get(Servo.class, "pr");
        pivotOut = hardwareMap.get(Servo.class, "arm1");
        fourbar = hardwareMap.get(Servo.class, "arm2");
        wrist = hardwareMap.get(Servo.class, "wrist");

        outRight = hardwareMap.get(Servo.class, "outRight");
        outLeft = hardwareMap.get(Servo.class, "outLeft");

        elbowleft.setPosition(0.4);
        elbowright.setPosition(1-0.4);
        pivotleft.setPosition(1-0.7);
        pivotright.setPosition(0.7);
        pivotOut.setPosition(0.9);
        fourbar.setPosition(0.9);
        outRight.setPosition(0);
        outLeft.setPosition(1);
        wrist.setPosition(0.95);
    }

    @Override
    public void start() {
        timer.reset();
        // Get the current time
    }

    @Override
    public void loop(){


        double max;
        double axial   = -gamepad1.left_stick_y * slow * progSlowmode;
        double lateral =  gamepad1.left_stick_x * slow * progSlowmode * 1.4;
        double yaw     =  gamepad1.right_stick_x * turnslow * progSlowmode;

        progSlowmode = Range.clip(-0.83*(gamepad1.left_trigger) + 1, 0, 1);

        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double modifiedHeading = modify(Math.toDegrees(currentHeading));

        double errorOut =  modifiedHeading - Math.toDegrees(targetAngleOut);
        double errorInt = Math.toDegrees(currentHeading) - Math.toDegrees(targetAngleInt);
        double admissibleError = Math.toRadians(1);

        if(gamepad1.options){
            targetAngleOut = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
        if(gamepad1.share){
            targetAngleInt = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        if((Math.abs(errorInt) > admissibleError) && posLockInt){
            leftFrontPower += errorInt*Hp;
            leftBackPower += errorInt*Hp;
            rightBackPower += -errorInt*Hp;
            rightFrontPower += -errorInt*Hp;
        }

        if((Math.abs(errorOut) > admissibleError) && posLockOut){
            leftFrontPower += errorOut*Hp;
            leftBackPower += errorOut*Hp;
            rightBackPower += -errorOut*Hp;
            rightFrontPower += -errorOut*Hp;
        }

        telemetry.addData("currentheading", Math.toDegrees(currentHeading));
        telemetry.addData("error", errorOut);


        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if(gamepad2.dpad_down){
            posLockInt = true;
            posLockOut = false;
        }else if(gamepad1.dpad_right){
            posLockInt = false;
            posLockOut = true;
        }else if(gamepad1.dpad_left || gamepad2.dpad_left){
            posLockInt = false;
            posLockOut = false;
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

        //1 is zero pos
        //0.75 is outtake

        //y is 1 close
        //y is 0.3 open

        //x is 0 close
        //x is 0.7 open




//        From
//        0.65
//        0.93
//        Transfer
//
//        0.75 pivot
//
//                Then
//        0.83 elbow
//
//        Then 0.12 pivot
//
//        Then 0.3 pivot
//
//        Then 0.65 elbow
//
//        Then 0.93 pivot
//
//                Repeat

        switch (New) {
            case cancel:
                if(Ltarget > 100){
                    p = 0.8;
                    intakeMotor.setPower(0.2);
                    timer.reset();
                    New = cancelinter1;
                }else{
                    Otarget = -5;
                    p = 0.8;
                    e = 0.765;
                    timer.reset();
                    New = base;
                }

                break;

            case cancelinter1:
                if(timer.seconds() > 0.25){
                    Ltarget = 50;
                    timer.reset();
                    New = cancelinter2;
                }
                break;

            case cancelinter2:
                if(timer.seconds() > 0.5){
                    Ltarget = -50;
                }
                if(timer.seconds() > 0.75){
                    Otarget = -5;
                    p = 0.8;
                    e = 0.78;
                    timer.reset();
                    New = base;
                }
                break;

            case pre:
                //move to intake
                Otarget = 38;
                if(counter == 0){
                    p = 0.3;
                }else{
                    p = 0.1;
                }
                timer.reset();
                New = initialize;
                break;

            case initialize:
                if(timer.seconds() > 0.25){
                    e = 0.75;
                    timer.reset();
                    New = base;
                }
                break;

            case base:
                //move pivot to intake
                if(timer.seconds() > 0.15){
                    p = 0.1;
                    timer.reset();
                    New = intake;
                }
                break;

            case intake:
                if(gamepad2.right_bumper){
                    slow = 0.5;
                    turnslow = 0.3;
                    e = 0.75;
                    Ltarget = 975;
                }
                if(gamepad2.left_bumper){
                    slow = 0.5;
                    turnslow = 0.3;
                    e = 0.75;
                    Ltarget = 700;
                }
                if(gamepad2.right_trigger > 0.2){
                    slow = 0.75;
                    turnslow = 0.5;
                    e = 0.75;
                    Ltarget = 400;
                }
                if(gamepad2.left_trigger > 0.2){
                    slow = 1;
                    turnslow = 1;
                    e = 0.75;
                    Ltarget = -100;
                }

                int lstickpos1 = (int) (20 * gamepad2.right_stick_y);

                if(gamepad2.right_stick_y != 0){
                    Ltarget = Ltarget - lstickpos1;
                }

                if(gamepad1.right_bumper && !onOff && holdtimer.seconds() > 0.25 && !(intakeLeftExt.getVelocity() > 25)){
                    holdtimer.reset();
                    onOff = true;
                    e = 0.7;
                    intakeMotor.setPower(-1);
                }else if(gamepad1.right_bumper && onOff && holdtimer.seconds() > 0.25 && !(intakeLeftExt.getVelocity() > 25)){
                    holdtimer.reset();
                    onOff = false;
                    intakeMotor.setPower(0);
                    e = 0.75;
                }else if(gamepad1.left_bumper){
                    holdtimer.reset();
                    onOff = false;
                    intakeMotor.setPower(1);
                }

                if(gamepad2.dpad_up || gamepad1.dpad_up){
                    slow = 1;
                    turnslow = 1;
                    e = 0.765;
                    p = 0.8;
                    intakeMotor.setPower(-0.5);
                    timer.reset();
                    New = intakeinter1;
                }
                break;

            case intakeinter1:
                if(timer.seconds() > 0.25){
                    Ltarget = -15;
                    timer.reset();
                    New = transfer;
                }
                break;

            case transfer:
                int lstickpos3 = (int) (20 * gamepad2.right_stick_y);

                if(gamepad2.right_stick_y != 0){
                    Ltarget = Ltarget - lstickpos3;
                }

                if(timer.seconds() > 0.35 && intakeLeftExt.getCurrentPosition() < 25){
                    intakeMotor.setPower(0);
                    p = 0.8;
                    timer.reset();
                    New = outtake;
                }
                break;

            case outtake:
                if(timer.seconds() > 0.35){
                    timer.reset();
                    New = outtakepre;
                }
                break;

            case outtakepre:
                intakeMotor.setPower(1);
//                intakeMotor.setPower((Math.pow((timer.seconds()), 2) * 1.25));
                if(timer.seconds() > 1.25){
                    timer.reset();
                    New = outtakeinter;
                }
                break;

            case outtakeinter:
                if(timer.seconds() > 0.25){
                    Otarget = -50;
                    intakeMotor.setPower(0);
                    timer.reset();
                    New = barrier;
                }
                break;

            case barrier:

                x1 = 0.4;
                y1 = 0.6;
                if(timer.seconds() > 0.75 && gamepad2.right_bumper){
                    Otarget = 200;
                    timer.reset();
                    New = barrierinter;
                }
                break;

            case barrierinter:
                int lstickpos2 = (int) (15 * gamepad2.right_stick_y);
                if(gamepad2.right_stick_y != 0){
                    Otarget = Otarget - lstickpos2;
                }
                if(timer.seconds() > 0){
                    r = 0.05;
                }
                if(timer.seconds() > 0.2){
                    v = 0.7;
                }
                if(gamepad2.right_bumper && timer.seconds() > 1.25){
                    Otarget = 800;
                }
                if(gamepad2.left_bumper && timer.seconds() > 0.5){
                    Otarget = 700;
                }

                if(gamepad2.right_trigger > 0.1 && timer.seconds() > 0.5){
                    Otarget = 500;
                }

                if(gamepad2.left_trigger > 0.1 && timer.seconds() > 0.5){
                    Otarget = 350;
                }

                if(gamepad1.left_bumper && timer.seconds() > 0.5){
                    p = 0.3;
                    left = true;
                    x1 = 0;
                }
                if(gamepad1.right_bumper && timer.seconds() > 0.5){
                    p = 0.3;
                    right = true;
                    y1 = 1;
                }
                if(left && right){
                    timer.reset();
                    New = deposit;
                }
                break;

            case deposit:
                if(timer.seconds() > 0.5){
                    leftBackPower += -0.3;
                    leftFrontPower += -0.3;
                    rightBackPower += -0.3;
                    rightFrontPower += -0.3;
                }

                if(timer.seconds() > 1.25){
                leftBackPower -= -0.3;
                leftFrontPower -= -0.3;
                rightBackPower -= -0.3;
                rightFrontPower -= -0.3;
                e = 0.75;
                left = false;
                right = false;
                y1 = 1;
                x1 = 0;
                r = 0.9;
                v = 0.9;
                timer.reset();
                New = pre;
                break;
                }
        }

        if(gamepad1.triangle){
            New = cancel;
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


        if(gamepad2.cross){
            lift.setPower(1);
        }else if(gamepad2.square){
            lift.setPower(-1);
        }else{
            lift.setPower(0);
        }
        double angleError = 0;

        double errorOuttest = Math.toDegrees(currentHeading) - Math.toDegrees(targetAngleOut);

        errorOuttest -= (360*Math.floor(0.5+((errorOuttest)/360.0)));

        telemetry.addData("testinginging", intakeLeftExt.getCurrentPosition());
        telemetry.addData("egvhgvhgrror", errorOuttest);
        telemetry.addData("axon pos", intakeMotor.getCurrentPosition());

        telemetry.addData("modifiedHeading", modifiedHeading);

        telemetry.addData("left", intakeLeftExt.getCurrentPosition());
        telemetry.addData("right", intakeRightExt.getCurrentPosition());
        telemetry.addData("out", outtakeMotor.getCurrentPosition());

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        telemetry.update();
    }

    public double modify(double heading){
        //if it is negative
        if(heading < 0){
            heading += 360;
        }
        return heading % 360;

    }

}


