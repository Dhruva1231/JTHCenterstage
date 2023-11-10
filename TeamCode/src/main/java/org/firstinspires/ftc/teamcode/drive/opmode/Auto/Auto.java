package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "Auto")
public class Auto extends LinearOpMode {
    private boolean test = false;

    private enum Random {
        left,
        middle,
        right
    }

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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-27.25, 0, Math.toRadians(0)))
                .build();

        TrajectorySequence trajectoryright = drive.trajectorySequenceBuilder(trajectory1.end())
                .turn(Math.toRadians(-58))
                .waitSeconds(7)
                .turn(Math.toRadians(154))
                .build();

        TrajectorySequence trajectoryleft = drive.trajectorySequenceBuilder(trajectory1.end())
                .turn(Math.toRadians(58))
                .waitSeconds(7)
                .turn(Math.toRadians(38))
                .build();

        TrajectorySequence trajectorymid = drive.trajectorySequenceBuilder(trajectory1.end())
                .waitSeconds(7)
                .turn(Math.toRadians(58))
                .turn(Math.toRadians(38))
                .build();

        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(cyclePose)
                .back(38)
                .build();

        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(cyclePose)
                .lineToConstantHeading(new Vector2d(0, -35))
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(0, 20))
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(0, -35))
                .build();

        // Define the angle to turn at

        waitForStart();

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
                    drive.followTrajectorySequenceAsync(trajectory1);
                    currentState = State.TRAJECTORY_2;
                    break;
                case TRAJECTORY_2:
                    if (!drive.isBusy()) {
                        currentState = State.TURN_1;
                    }
                    break;
                case TURN_1:
                    if(randomization == Random.left){
                        drive.followTrajectorySequenceAsync(trajectoryleft);
                    }
                    else if(randomization == Random.right){
                        drive.followTrajectorySequenceAsync(trajectoryright);
                    }
                    else if(randomization == Random.middle){
                        drive.followTrajectorySequenceAsync(trajectorymid);
                    }
                    currentState = State.TRAJECTORY_3;
                    break;
                case TRAJECTORY_3:
                    if (!drive.isBusy()) {
                        drive.setPoseEstimate(cyclePose);
                        currentState = State.WAIT_1;
                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                    }
                    break;
                case WAIT_1:
                    drive.followTrajectorySequenceAsync(trajectory3);
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    currentState = State.TURN_2;
                    break;
                case TURN_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    currentState = State.AAA;
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;

                case AAA:
                    if(!drive.isBusy()){
                        currentState = State.BBB;
                    }

                    break;
            }

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