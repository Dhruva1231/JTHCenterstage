package org.firstinspires.ftc.teamcode.drive.opmode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="FF Bot", group="Iterative Opmode")
public class FFWorldsBackup extends OpMode {

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotorEx turretDrive = null;
    private DcMotorEx armVert = null;
    private Servo leftServo = null;
    private Servo rightServo = null;
    private Servo intakeServo = null;
    private DcMotorEx armPivot = null;
    private DcMotorEx intakeMotor = null;
    private CRServo carousel1 = null;
    private CRServo carousel2 = null;
    private CRServo spoolSpin = null;
    private Servo intakeMaster = null;
    private Servo intakeTilt = null;
    private Servo capPivot = null;
    private Servo capTurn = null;
    private double multiplier = 1;
    private double capMultiplier = 1;
    private double capMult = 1;
    private double Turnmultiplier = 1;
    private double Strafemultiplier = 1;
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
    private double position = 0.5;
    private double newPosition = 0.7;
    private double timeVal1 = 0.03;
    private double timeVal2 = 0.03;
    ElapsedTime timer  = new ElapsedTime();
    ElapsedTime timer1  = new ElapsedTime();
    ElapsedTime carouselTimer  = new ElapsedTime();


    private ElapsedTime holdTimerA = new ElapsedTime();

    @Override
    public void init() {

        front_left = hardwareMap.get(DcMotor.class, "leftFront");
        front_right = hardwareMap.get(DcMotor.class, "rightFront");
        back_left = hardwareMap.get(DcMotor.class, "leftRear");
        back_right = hardwareMap.get(DcMotor.class, "rightRear");

        turretDrive = hardwareMap.get(DcMotorEx.class, "Level");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        armVert = hardwareMap.get(DcMotorEx.class, "armTurn");
        armPivot = hardwareMap.get(DcMotorEx.class, "armPivot");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "carousel");
        intakeTilt = hardwareMap.get(Servo.class, "intakeSlave");
        intakeMaster = hardwareMap.get(Servo.class, "intakeMaster");
        carousel1 = hardwareMap.get(CRServo.class, "carousel1");
        carousel2 = hardwareMap.get(CRServo.class, "carousel2");
        spoolSpin = hardwareMap.get(CRServo.class, "spool");
        capPivot = hardwareMap.get(Servo.class, "pivot");
        capTurn = hardwareMap.get(Servo.class, "turn");
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretDrive.setTargetPosition(-375);
        turretDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretDrive.setPower(1);

        armVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armVert.setTargetPosition(-50);
        armVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armVert.setPower(1);


        armPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPivot.setTargetPosition(1000);
        armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armPivot.setPower(1);

        intakeMaster.setPosition(0.2);
        intakeTilt.setPosition(0.1);
    }

    @Override
    public void loop() {

        double drive = gamepad1.right_stick_y * multiplier;
        double strafe = gamepad1.right_stick_x * Strafemultiplier;
        double twist = gamepad1.left_stick_x * Turnmultiplier;

        double[] speeds = {
                (drive - strafe - twist),
                (-drive - strafe - twist),
                (drive + strafe - twist),
                (-drive + strafe - twist),
        };

        double max = Math.abs(speeds[0]);
        for (int i = 0; i > speeds.length; i++) {
            if (max > Math.abs(speeds[-i])) max = Math.abs(speeds[-i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);

        if(gamepad1.a){
            carousel1.setPower(-0.5 * carouselTimer.seconds()/0.8);
            carousel2.setPower(-0.5 * carouselTimer.seconds()/0.8);
        }
        else{
            carouselTimer.reset();
            carousel1.setPower(0);
            carousel2.setPower(0);
        }

        if(mode1){
            turretTarget += gamepad2.right_trigger * -30;
            turretTarget -= gamepad2.left_trigger * -30;
//            armPivotTarget += gamepad2.right_stick_y * -20;
        }
        spoolSpin.setPower(gamepad2.left_stick_y * spoolSpeed);




        if(gamepad2.right_stick_y>0.1 && timer.seconds()>timeVal1 && mode2){
            newPosition += gamepad2.right_stick_y * 0.01 * capMultiplier;
            newPosition = Range.clip(newPosition,0.45,0.8);
            timer.reset();
        }
        else if(gamepad2.right_stick_y<-0.1 && timer.seconds()>timeVal1 && mode2){
            newPosition += gamepad2.right_stick_y * 0.01 * capMultiplier;
            newPosition = Range.clip(newPosition,0.45,0.8);
            timer.reset();
        }


        if(gamepad2.right_stick_x>0.1 && timer1.seconds()>timeVal2 && mode2){
            position += gamepad2.right_stick_x * 0.01 * capMultiplier;
            position = Range.clip(position,0,1);
            timer1.reset();
        }
        else if(gamepad2.right_stick_x<-0.1 && timer1.seconds()>timeVal2 && mode2){
            position += gamepad2.right_stick_x * 0.01 * capMultiplier;
            position = Range.clip(position,0,1);
            timer1.reset();
        }




        if(gamepad2.right_bumper && mode2){
            spoolSpeed = 0.1;
            capMultiplier = 0.1;
            timeVal1 = 0.01;
            timeVal2 = 0.01;
        }
        if(gamepad2.left_bumper  && mode2){
            capMultiplier = 1;
            timeVal1 = 0.03;
            timeVal2 = 0.03;
            spoolSpeed = 1;
        }


        if(gamepad2.a && mode2){
            newPosition = 0.7;
            position = 0.72;
            turretTarget = -1780;
        }

        //Slowmode
        if(gamepad1.dpad_up){
            Turnmultiplier = 1;
            multiplier = 1;
            Strafemultiplier = 1;
        }
        if(gamepad1.dpad_right){
            Turnmultiplier = 0.70;
            Strafemultiplier = 0.5;
            multiplier = 1;
        }

        //Super slow
        if(gamepad1.dpad_down){
            Turnmultiplier = 0.4;
            multiplier = 0.5;
            Strafemultiplier = 0.5;
        }

        if(gamepad1.dpad_left){
            Turnmultiplier = 0.5;
            multiplier = 0.6;
            Strafemultiplier = 0.5;
        }


        //Outtake Pos
        if(gamepad1.right_bumper){
            intakeMaster.setPosition(0.39);
            intakePower = 0.35;
            intakeTilt.setPosition(0.2);
        }
        if(gamepad1.left_bumper){
            intakeMaster.setPosition(0.39);
            intakeTilt.setPosition(0.2);
            intakePower = 0.1;
        }
        if(gamepad1.y){
            intakeTilt.setPosition(0.1);
            intakeMaster.setPosition(0.34);
            intakePower = 0.0;
        }

        //Intake
        if(gamepad2.b && mode1) {
            intakeMaster.setPosition(0.34);
            turretTarget = -2100;
            armVertTarget = -50;
            armPivotTarget = 1350;

        }
        //Lowering suspension

        //Carousel
        if(gamepad1.x){
            intakePower = 0.8;
            intakeTilt.setPosition(0.2);
        }

        /*CAPPING*/
        if(gamepad2.a && mode1){
            armVertTarget = -50;
            armPivotTarget = 1140;
            intakeMaster.setPosition(0.36);
        }


        /*CAPPING DROP*/

        /*SHARED*/
        if(gamepad2.dpad_right && mode1){
            intakeMaster.setPosition(0.34);
            turretTarget = 890; //-1650
            armVertTarget = -50;
            armPivotTarget = 1350;

        }

        /*MIDDLE POSITION*/
        if(gamepad2.dpad_left && mode1){
            intakeMaster.setPosition(0.34);
            turretTarget = -630; //-350
            armPivotTarget = 1900;
            armVertTarget = -2000;
            intakeTilt.setPosition(0.1);

        }

        /*OUTTAKE POSITION*/
        if(gamepad2.dpad_up && mode1){
            intakeMaster.setPosition(0.37);
            armPivotTarget = 2550;//2575
            armVertTarget = -90;
            turretTarget = -2000;//-1100
        }



        /*INTAKE POSITION*/
        if(gamepad2.dpad_down && mode1){
            intakeMaster.setPosition(0.37);
            turretTarget = -630;//-375
            armPivotTarget = 1075;
            armVertTarget = -2150; //-2650
        }
        /*special INTAKE POSITION*/
        if(gamepad2.right_bumper && mode1){
            intakeMaster.setPosition(0.35);
            turretTarget = -630;
            armPivotTarget = 390;
            armVertTarget = -750;
        }
        //Intake Motor
        if(gamepad2.x && mode1){
            intakePower = -1;
        }

        if(gamepad2.y && mode1){
            intakePower = 0;
        }

        if(gamepad2.left_bumper && mode1){
            intakePower = 1;
        }
        if(gamepad1.b){
            intakeTilt.setPosition(0.2);
        }

        //Switching Modes
        if (gamepad2.back) {
            if ((mode1) && (holdTimerA.time() > 0.5)) {

                telemetry.addLine("among");
                mode2 = true;
                mode1 = false;
                holdTimerA.reset();
            } else if ((mode2) && (holdTimerA.time() > 0.5)) {
                telemetry.addLine("us");
                mode1 = true;
                mode2 = false;
                holdTimerA.reset();
            }
        }

        intakeMotor.setPower(intakePower);
        turretDrive.setTargetPosition(turretTarget);
        armVert.setTargetPosition(armVertTarget);
        armPivot.setTargetPosition(armPivotTarget);
//        if(gamepad2.left_stick_y != 0){
//        }
        capPivot.setPosition(newPosition);
        capTurn.setPosition(position);
        telemetry.addData("turret-rot-position", turretDrive.getCurrentPosition());
        telemetry.addData("arm-pivot-position", armPivot.getCurrentPosition());
        telemetry.addData("arm-vertical-position", armVert.getCurrentPosition());
        telemetry.addData("test", intakeMotor.getCurrentPosition());
        telemetry.addData("pivot", capPivot.getPosition());
        telemetry.addData("turn", capTurn.getPosition());

        telemetry.addData("mode1 = ", mode1 + "     mode2 = " + mode2);
        telemetry.update();
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


