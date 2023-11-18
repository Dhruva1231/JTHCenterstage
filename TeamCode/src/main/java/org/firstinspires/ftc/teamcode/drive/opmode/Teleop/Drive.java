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
@TeleOp(name="Drivebase")
public class Drive extends OpMode{

    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();


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

    }

    @Override
    public void start() {
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


