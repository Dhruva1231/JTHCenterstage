package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;


@Autonomous(name="CSZero")
public class CSZero extends LinearOpMode{
    public DcMotorEx intakeLeftExt;
    public DcMotorEx outtakeMotor;
    public DcMotorEx intakeRightExt;
    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;
    public void runOpMode() throws InterruptedException {



        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "lf");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rb");


        intakeLeftExt = hardwareMap.get(DcMotorEx.class, "lint");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "out");
        intakeRightExt = hardwareMap.get(DcMotorEx.class, "rint");

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeftExt.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRightExt.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeRightExt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLeftExt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        leftBackDrive.setPower(0.5);
        sleep(1000);
        rightBackDrive.setPower(0.5);
        sleep(1000);

        rightFrontDrive.setPower(0.5);
        sleep(1000);

        leftFrontDrive.setPower(0.5);

    }


}