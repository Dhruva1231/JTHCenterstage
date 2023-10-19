package org.firstinspires.ftc.teamcode.drive.opmode.Teleop;



import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.ScrimmageTeleop.state.base;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.ScrimmageTeleop.state.initialize;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.ScrimmageTeleop.state.intake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.ScrimmageTeleop.state.outtake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.ScrimmageTeleop.state.pre;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.ScrimmageTeleop.state.transfer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="Scrimmage Teleop")
public class ScrimmageTeleop extends OpMode {

    public DcMotorEx intakeLeftExt;
    public DcMotorEx intakeRightExt;
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    public DcMotorEx intakeMotor;

    public static double p;
    public static double e;

    public Servo pivotleft;

    public Servo pivotright;

    public Servo elbowleft;

    public Servo elbowright;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public enum state {
        pre,
        initialize,
        base,
        intake,
        transfer,
        outtake,
        barrier
    }

    private boolean fortnite = false;

    state state = pre;
    ElapsedTime timer  = new ElapsedTime();

    @Override
    public void init(){
        intakeLeftExt = hardwareMap.get(DcMotorEx.class, "lint");
        intakeRightExt = hardwareMap.get(DcMotorEx.class, "rint");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        elbowleft = hardwareMap.get(Servo.class, "el");
        elbowright = hardwareMap.get(Servo.class, "er");
        pivotleft = hardwareMap.get(Servo.class, "pl");
        pivotright = hardwareMap.get(Servo.class, "pr");

        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_rear");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


    }

    @Override
    public void start() {
        timer.reset();
        // Get the current time
    }

    @Override
    public void loop(){


        elbowleft.setPosition(e);
        elbowright.setPosition(1-e);
        pivotleft.setPosition(1-p);
        pivotright.setPosition(p);
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

        switch (state) {
            case pre:
                //move to intake
                if(timer.seconds() > 0.5){
                    p = 0.83;
                    timer.reset();
                    state = initialize;
                }
                break;

            case initialize:

                

                break;

            case base:
                //move elbow to intake
                if(timer.seconds() > 1){
                    p = 0.7;
                    timer.reset();
                    state = intake;
                }
                break;

            case intake:
                if(gamepad1.x){
                    intakeMotor.setPower(-1);
                }else{
                    intakeMotor.setPower(0);
                }

                if(gamepad1.dpad_down){
                    intakeMotor.setPower(0);
                    timer.reset();
                    state = transfer;
                }
                break;

            case transfer:
                if(timer.seconds() > 0.5){
                    e = 0.55;
                }
                if(timer.seconds() > 1){
                    p = 0.92;
                    timer.reset();
                    state = outtake;
                }

                break;

            case outtake:
                if(gamepad1.dpad_down){
                    intakeMotor.setPower(1);
                }
                break;
        }
    telemetry.addData("Status", "Run Time: " + timer.toString());
    telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
    telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
    telemetry.update();
    telemetry.addData("turret-rot-position", intakeLeftExt.getCurrentPosition());
    telemetry.addData("arm-pivot-position", intakeRightExt.getCurrentPosition());
    Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
    telemetry.update();
    }

}


