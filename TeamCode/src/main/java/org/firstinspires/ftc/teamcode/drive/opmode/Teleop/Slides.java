package org.firstinspires.ftc.teamcode.drive.opmode.Teleop;


import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.barrier;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.barrierinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.base;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.cancel;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.cancelinter1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.cancelinter2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.deposit;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.idle;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.initialize;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.intake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.intakeinter1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.outtake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.outtakeinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.outtakepre;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.pre;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.shoot;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.RebuildTeleop.state.transfer;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Slides.slides.extend;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Slides.slides.inter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Slides.slides.inter2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Slides.slides.inter3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Slides.slides.retract;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name="sliudes")
public class Slides extends OpMode {

    public DcMotorEx intakeLeftExt;
    public DcMotorEx outtakeMotor;
    public DcMotorEx intakeRightExt;
    private PIDController Lcontroller;
    private PIDController Ocontroller;

    public static double amp1;
    public static double amp2;

    //right claw grab = 0.5, retract = 0
    //left claw grab = 0.44, retract = 0.98

    public static double Lp = 0.006, Li = 0, Ld = 0.0001;

    public static double Lf;

    //Ltarget Max 750, Min -75
    public static int Ltarget;

    //Otarget Max 800, Min 25
    public static int Otarget;

    public static double Op = 0.012, Oi = 0, Od = 0.0002;
    public static double Of = -0.08;

    public DcMotorEx intakeMotor;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double finalconvert;

    public double targetAngleOut = Math.toRadians(90);
    public double targetAngleInt = Math.toRadians(90);
    public double currentAngle;

    private boolean fortnite = false;
    //pan center = 0.47
    public static double p = 0.7;
    public static double e = 0.2;

    public boolean test = false;

    public double time;

    public Servo wrist;
    public Servo airplane;
    public Servo pan;
    public Servo tilt;
    public Servo claw1;
    public Servo claw2;
    public DcMotorEx leftFrontDrive;
    public DcMotorEx leftBackDrive;
    public DcMotorEx rightFrontDrive;
    public DcMotorEx rightBackDrive;
    ElapsedTime timer  = new ElapsedTime();
    ElapsedTime speed  = new ElapsedTime();


    public enum slides {
        retract,
        inter,
        extend,
        inter2,
        inter3
    }

    slides slides = retract;
        @Override
    public void init(){
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


        pan = hardwareMap.get(Servo.class, "0.");
        wrist = hardwareMap.get(Servo.class, "1.");
        tilt = hardwareMap.get(Servo.class, "5.");
        claw1 = hardwareMap.get(Servo.class, "2.");
        claw2 = hardwareMap.get(Servo.class, "4.");
        airplane = hardwareMap.get(Servo.class, "3.");



        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void start() {
            timer.reset();
        // Get the current time
    }

    @Override
    public void loop(){



        double max;


        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;


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

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        Lcontroller.setPID(Lp, Li, Ld);

        int armPos = intakeLeftExt.getCurrentPosition();

        double Lpid = Lcontroller.calculate(armPos, Ltarget);

        double Lpower = Lpid;

//        intakeLeftExt.setPower(Lpower);
//        intakeRightExt.setPower(Lpower);

        Ocontroller.setPID(Op, Oi, Od);

        int outPos = outtakeMotor.getCurrentPosition();

        double Opid = Ocontroller.calculate(outPos, -Otarget);
        double Off = Of;


        double Opower = Opid + Off;

        outtakeMotor.setPower(Opower);


        pan.setPosition(0.47);
        wrist.setPosition(0.48);
        claw1.setPosition(0);
        claw2.setPosition(0.98);
        tilt.setPosition(0.32);

        switch (slides) {
            case retract:
                if(intakeLeftExt.getCurrentPosition() > 50){
                    intakeLeftExt.setPower(-0.5);
                    intakeRightExt.setPower(-0.5);
                }
                timer.reset();
                slides = inter;
                break;

            case inter:
                if(timer.seconds() > 1 && intakeLeftExt.getCurrentPosition() < 50){
                    intakeLeftExt.setPower(0);
                    intakeRightExt.setPower(0);
                    timer.reset();
                    slides = extend;
                }
                break;

            case extend:
                if(timer.seconds() > 1){
                    intakeLeftExt.setPower(1);
                    intakeRightExt.setPower(1);
                    speed.reset();
                    timer.reset();
                    slides = inter2;
                }
                break;

            case inter2:
                if(intakeLeftExt.getCurrentPosition() > 975){
                    intakeLeftExt.setPower(0);
                    intakeRightExt.setPower(0);
                    timer.reset();
                    slides = inter3;
                }
                break;

            case inter3:
                if(!test){
                    time = speed.seconds();
                }
                test = true;
                if(timer.seconds() > 2){
                    slides = retract;
                }
                break;


        }

            if(gamepad1.cross){

        }



        telemetry.addData("left", intakeLeftExt.getCurrentPosition());
        telemetry.addData("right", intakeRightExt.getCurrentPosition());
        telemetry.addData("out", outtakeMotor.getCurrentPosition());
        telemetry.addData("amp1", intakeLeftExt.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("amp2", intakeRightExt.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("power", Lpower);
        telemetry.addData("time", time);

        amp1 = intakeLeftExt.getCurrent(CurrentUnit.MILLIAMPS);
        amp2 = intakeRightExt.getCurrent(CurrentUnit.MILLIAMPS);
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


