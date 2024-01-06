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
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.ServoTest.state.move;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.ServoTest.state.sleep;

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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name="ServoTest")
public class ServoTest extends OpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();


    public static double finalconvert;
    public Servo frontgrab;
    public AnalogInput distance1;
    public TouchSensor one;
    public TouchSensor two;
    public TouchSensor three;
    public TouchSensor four;
    public TouchSensor five;
    public TouchSensor six;
    public TouchSensor seven;
    public TouchSensor eight;

    public static double r = 0.15;
    public static double v = 0.86;
    public Servo wrist;

    public static double newposition;
    public static double position;
    public AnalogInput distance2;
    public Servo pivotOut;
    public Servo fourbar;


    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;

    public enum state {
        move,
        sleep
    }

    state State = move;

    ElapsedTime timer  = new ElapsedTime();


    @Override
    public void init(){

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

//        distance1 = hardwareMap.get(AnalogInput.class, "one");
//        three = hardwareMap.get(TouchSensor.class, "3");
//        four = hardwareMap.get(TouchSensor.class, "4");
//        five = hardwareMap.get(TouchSensor.class, "5");
//        six = hardwareMap.get(TouchSensor.class, "6");
//        seven = hardwareMap.get(TouchSensor.class, "7");
//        eight = hardwareMap.get(TouchSensor.class, "8");
////
////        wrist = hardwareMap.get(Servo.class, "wrist");
////
////        pivotOut = hardwareMap.get(Servo.class, "arm1");
////        fourbar = hardwareMap.get(Servo.class, "arm2");
//////        pivotOut.setPosition(0.15);
//////        fourbar.setPosition(0.86);
//////        wrist.setPosition(0.51);

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
        double lateral =  gamepad1.left_stick_x ;
        double yaw     =  gamepad1.right_stick_x ;

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
//
//        v = finalconvert;
//
//        switch (State) {
//            case move:
//                timer.reset();
//                State = sleep;
//                break;
//
//            case sleep:
//                if(timer.seconds() > 0.008){
//                    State = move;
//                }
//                break;
//
//        }
//
//
//        pivotOut.setPosition(r);
//        fourbar.setPosition(v);
//
//        telemetry.addData("1", distance1.getVoltage());
////        telemetry.addData("2", distance2.getVoltage());
//
////        double volttocm = ((611.47996888)*Math.pow((1/distance1.getVoltage()), 0.719424460432))/100;
////        telemetry.addData("convertnigganigger", volttocm);
//
////        position = Range.clip(((volttocm-2.8)+0.5), 0.5, 0.86);
////        newposition = Range.clip(Math.pow(position, 1.9), 0.5, 0.86);
//
//        finalconvert = Range.clip((0.8702699 + (-506713.97027)/(1+(Math.pow(distance1.getVoltage(), 2.126704))/0.00000126998012639)), 0.5, 0.85);

        telemetry.addData("convertnigganiggers", finalconvert);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        telemetry.update();
    }

}


