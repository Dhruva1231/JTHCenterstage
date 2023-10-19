package org.firstinspires.ftc.teamcode.drive.opmode.Teleop;



import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.ScrimmageTeleop.state.barrier;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.ScrimmageTeleop.state.base;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.ScrimmageTeleop.state.deposit;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="Scrimmage Teleop")
public class ScrimmageTeleop extends OpMode {

    public DcMotorEx intakeLeftExt;
    public DcMotorEx intakeRightExt;

    public DcMotorEx intakeMotor;

    public static double p = 0.93;
    public static double e = 0.65;

    public static double r = 0.95;
    public static double x = 0;
    public static double y = 1;

    public Servo pivotleft;

    public Servo pivotright;

    public Servo elbowleft;

    public Servo elbowright;
    public Servo pivotOut;

    public DcMotorEx outtakeMotor;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public enum state {
        pre,
        initialize,
        base,
        intake,
        transfer,
        outtake,
        barrier,
        deposit
    }

    private boolean fortnite = false;

    state state = pre;
    ElapsedTime timer  = new ElapsedTime();

    public Servo outLeft;
    public Servo outRight;

    @Override
    public void init(){
        intakeLeftExt = hardwareMap.get(DcMotorEx.class, "lint");
        intakeRightExt = hardwareMap.get(DcMotorEx.class, "rint");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        elbowleft = hardwareMap.get(Servo.class, "el");
        elbowright = hardwareMap.get(Servo.class, "er");
        pivotleft = hardwareMap.get(Servo.class, "pl");
        pivotright = hardwareMap.get(Servo.class, "pr");
        pivotOut = hardwareMap.get(Servo.class, "outElbow");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");

        outRight = hardwareMap.get(Servo.class, "outRight");
        outLeft = hardwareMap.get(Servo.class, "outLeft");

        elbowleft.setPosition(0.65);
        elbowright.setPosition(1-0.65);
        pivotleft.setPosition(1-0.93);
        pivotright.setPosition(0.93);

        pivotOut.setPosition(0.95);
        outRight.setPosition(0);
        outLeft.setPosition(1);
    }

    @Override
    public void start() {
        timer.reset();
        // Get the current time
    }

    @Override
    public void loop(){

        pivotOut.setPosition(r);
        outRight.setPosition(x);
        outLeft.setPosition(y);


        elbowleft.setPosition(e);
        elbowright.setPosition(1-e);
        pivotleft.setPosition(1-p);
        pivotright.setPosition(p);

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

        switch (state) {
            case pre:
                //move to intake
                if(gamepad1.cross){
                    p = 0.75;
                    timer.reset();
                    state = initialize;
                }
                break;

            case initialize:
                if(timer.seconds() > 0.5){
                    e = 0.83;
                    timer.reset();
                    state = base;
                }
                break;

            case base:
                //move elbow to intake
                if(timer.seconds() > 0.25){
                    p = 0.1;
                    timer.reset();
                    state = intake;
                }
                break;

            case intake:
                if(gamepad1.cross){
                    intakeMotor.setPower(-1);
                }else{
                    intakeMotor.setPower(0);
                }

                if(gamepad1.dpad_down){
                    p = 0.3;
                    intakeMotor.setPower(-0.5);
                    timer.reset();
                    state = transfer;
                }
                break;

            case transfer:
                if(timer.seconds() > 0.25){
                    e = 0.52;
                }
                if(timer.seconds() > 0.65){
                    p = 0.8;
                }
                if(timer.seconds() > 0.95){
                    p = 0.91;
                    timer.reset();
                    state = outtake;
                }

                break;

            case outtake:
                if(timer.seconds() > 0.5){
                    intakeMotor.setPower(1);
                }
                if(timer.seconds() > 1.5 && gamepad1.x){
                    intakeMotor.setPower(0);
                    state = barrier;
                }
                break;

            case barrier:
                if(gamepad1.cross){
                    r = 0.75;
                }
                if(gamepad1.right_bumper){
                    x = 0.7;
                }
                if(gamepad1.left_bumper){
                    y = 0.3;
                }
                if(gamepad1.square){
                    state = deposit;
                }
                break;

            case deposit:
                if(gamepad1.cross){
                    y = 1;
                    x = 0;
                    r = 0.95;
                    timer.reset();
                    state = pre;
                }

                break;
        }


        telemetry.addData("turret-rot-position", intakeLeftExt.getCurrentPosition());
        telemetry.addData("arm-pivot-position", intakeRightExt.getCurrentPosition());
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        telemetry.update();
    }

}


