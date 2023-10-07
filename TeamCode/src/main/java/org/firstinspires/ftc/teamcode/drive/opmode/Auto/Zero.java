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


@Autonomous(name="Zero")
public class Zero extends LinearOpMode{

    private DcMotorEx leftArm;
    private DcMotorEx rightArm;
    private DcMotorEx leftLift;
    private DcMotorEx rightLift;
    private DcMotorEx test;


    public void runOpMode() throws InterruptedException {


        test = hardwareMap.get(DcMotorEx.class,"leftFront");


        leftArm = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightArm = hardwareMap.get(DcMotorEx.class,"rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        rightLift = hardwareMap.get(DcMotorEx.class,"intakeRight");

        leftLift.setDirection(DcMotorEx.Direction.REVERSE);
        leftArm.setDirection(DcMotorEx.Direction.REVERSE);

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

    }


}