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
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="Intake Slides")
public class IntakeSlidesTest extends OpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public DcMotorEx intakeLeftExt;
    public DcMotorEx outtakeMotor;
    public DcMotorEx intakeRightExt;
    private PIDController Lcontroller;
    private PIDController Ocontroller;


    public static double a;
    public static double b;
    public static double c;
    public static double Lp = 0.006, Li = 0, Ld = 0.0001;

    //Ltarget Max 600, Min -75
    public static int Ltarget;

    //Otarget Max 800, Min 25
    public static int Otarget;

    public static double Op = 0.012, Oi = 0, Od = 0.0002;
    public static double Of = -0.08;


    @Override
    public void init(){
        Lcontroller = new PIDController(Lp,Li,Ld);
        Ocontroller = new PIDController(Op,Oi,Od);

        intakeLeftExt = hardwareMap.get(DcMotorEx.class, "lint");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        intakeRightExt = hardwareMap.get(DcMotorEx.class, "rint");

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeftExt.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRightExt.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void start() {
        // Get the current time
    }

    @Override
    public void loop(){

        Lcontroller.setPID(Lp, Li, Ld);

        int armPos = intakeLeftExt.getCurrentPosition();

        double Lpid = Lcontroller.calculate(armPos, Ltarget);

        double Lpower = Lpid;

        outtakeMotor.setPower(Lpower);
        intakeRightExt.setPower(Lpower);


        Ocontroller.setPID(Op, Oi, Od);

        int outPos = outtakeMotor.getCurrentPosition();

        double Opid = Ocontroller.calculate(outPos, -Otarget);
        double Off = Of;


        double Opower = Opid + Off;

        intakeLeftExt.setPower(Opower);


        telemetry.addData("left", intakeLeftExt.getCurrentPosition());
        telemetry.addData("right", intakeRightExt.getCurrentPosition());
        telemetry.addData("out", outtakeMotor.getCurrentPosition());
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        telemetry.update();
    }

}


