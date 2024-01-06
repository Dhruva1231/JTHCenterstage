package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.Zero.State.IDLE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Encoder;


@Autonomous(name="Zero")
public class Zero extends LinearOpMode{
    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        TURN_1,         // Then we want to do a point turn
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        IDLE,            // Our bot will enter the IDLE state when done
        AAA,
    }
    State currentState = IDLE;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));


    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);


        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-5, 0), Math.toRadians(180))
                .splineTo(new Vector2d(-27, -36), Math.toRadians(-90))
                .waitSeconds(0.5)
//                .setReversed(false)
//                .splineTo(new Vector2d(-50.5, -10), Math.toRadians(90))
//                .splineTo(new Vector2d(-50.5, 30), Math.toRadians(90))
//                .setReversed(true)
//                .waitSeconds(0.25)
//                .splineTo(new Vector2d(-50.5, 10), Math.toRadians(-90))
//                .splineTo(new Vector2d(-27, -36), Math.toRadians(-90))
//                .waitSeconds(0.5)
//                .setReversed(false)
//                .splineTo(new Vector2d(-50.5, -10), Math.toRadians(90))
//                .splineTo(new Vector2d(-50.5, 30), Math.toRadians(90))
//                .setReversed(true)
//                .waitSeconds(0.5)
//                .splineTo(new Vector2d(-50.5, 10), Math.toRadians(-90))
//                .splineTo(new Vector2d(-27, -36), Math.toRadians(-90))
                .build();

        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .setReversed(false)
                .splineTo(new Vector2d(-50.5, -10), Math.toRadians(90))
                .splineTo(new Vector2d(-50.5, 30), Math.toRadians(90))
                .build();

        waitForStart();
        if (isStopRequested()) return;
        currentState = State.TRAJECTORY_1;

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case TRAJECTORY_1:
                    drive.followTrajectorySequence(trajectory1);
                    currentState = State.TRAJECTORY_2;
                    break;

                case TRAJECTORY_2:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_3;
                    }
                    break;
                case TRAJECTORY_3:
                    drive.followTrajectorySequence(trajectory2);
                    currentState = State.IDLE;
                    break;

                case IDLE:
                    if(!drive.isBusy()){
                        currentState = State.AAA;
                    }
                    break;
                case AAA:



                    break;
            }
        }

    }


}