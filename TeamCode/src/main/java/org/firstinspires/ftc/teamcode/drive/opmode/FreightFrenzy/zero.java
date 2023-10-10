package org.firstinspires.ftc.teamcode.drive.opmode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="zero", group="Iterative Opmode")
public class zero extends OpMode {

    private DcMotorEx turretDrive = null;
    private DcMotorEx armVert = null;

    private DcMotorEx armPivot = null;

    private int turretTarget = -630;
    private int armVertTarget = -2000;
    private int armPivotTarget = 1900;
    private int carouselPower;
    private double intakePosition;
    private double intakeAdjuster = 0;
    private double carousel;
    private double intakePower;
    public boolean mode1 = true;
    public boolean mode2 = false;
    public double spoolSpeed = 1;
    private double test;

    ElapsedTime timer  = new ElapsedTime();
    ElapsedTime timer1  = new ElapsedTime();
    ElapsedTime carouselTimer  = new ElapsedTime();


    private ElapsedTime holdTimerA = new ElapsedTime();

    @Override
    public void init() {


        turretDrive = hardwareMap.get(DcMotorEx.class, "Level");

        armVert = hardwareMap.get(DcMotorEx.class, "armTurn");
        armPivot = hardwareMap.get(DcMotorEx.class, "armPivot");


        turretDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {


    }

    void TargetPosition(int ticks, DcMotorEx attachment) {
        {
            attachment.setTargetPosition(ticks);
        }
    }
    void setMax(int min, int max, DcMotorEx attachment, double speed){
        if(speed > 0.0 && attachment.getCurrentPosition() <= max){
            attachment.setPower(speed);
        }
        else if(speed < 0.0 && attachment.getCurrentPosition() >= min){
            attachment.setPower(speed);
        }
        else{
            attachment.setPower(0);
        }
    }
}