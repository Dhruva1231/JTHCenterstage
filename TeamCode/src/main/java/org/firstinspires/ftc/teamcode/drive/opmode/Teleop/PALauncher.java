package org.firstinspires.ftc.teamcode.drive.opmode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="PALauncher", group="Linear Opmode")

public class PALauncher extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private ElapsedTime runtime = new ElapsedTime();
    private Servo launcher = null;
    public static double servoPos = 0.5;

    @Override
    public void runOpMode() {

        launcher = hardwareMap.get(Servo.class, "allah 'akbar");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                launcher.setPosition(0.2);
            }else{
                launcher.setPosition(0.5);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Servo Pos", "%4.2f", servoPos);
            Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
            telemetry.update();
        }
    }}

