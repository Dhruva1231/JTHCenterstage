package org.firstinspires.ftc.teamcode.drive.opmode.Teleop;

import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.backstopconeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.elevatortransfer;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.PoleHeight.h;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.PoleHeight.l;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.armlow;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.baseinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.beacondeposit;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.beacongrab2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.beaconinter2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.beaconinter3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.beaconscore;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.beaconscoreinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.beacontransfer;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.deposit;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.drop;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.elevatorgrab2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.elevatorgrab3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.elevatorgrab4;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.grab2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.inter2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.mid;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.outtake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.score;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.specialouttake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.specialtrans;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.terminalDeposit;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.terminalOuttake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.terminalinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.vomitbase;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.vomitbaseinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.vomitdeposit;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.vomitdrop;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.vomitgrab2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.vomithigh;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.vomitinter2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.vomitouttake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.vomitouttakeinter;

import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.backstopconeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.backstopintaketeleop;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.backstopretract;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.clawdepositpos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.clawelevatorpos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.clawinitializepos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.clawouttakepos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.clawparkpos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.clawtransferpos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.elevatorbackwards;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.elevatorbackwardsdown;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.elevatorhigh;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.elevatorlow;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.elevatorlowinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.elevatormid;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.elevatortransfer;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.elevatorvomittransfer;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.PoleHeight.m;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.base;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.baseclaw;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.clawflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.elevatorgrab;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.grab;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.intake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.inter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.pre;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.right;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.rightinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.terminal;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.transfer;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.vomitinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.RobotState.vomittransfer;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.flip.betweenflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.flip.depositflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.flip.elevatorflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.flip.initializeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.flip.outtakeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop.flip.transferflip;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="MTI Teleop")
public class MTITeleop extends OpMode {

    public double timeTest1;
    public double timeTest2;
    ElapsedTime grabTest = new ElapsedTime();

    private PIDController Lcontroller;
    private PIDController Acontroller;

    public static double Lp = 0.014, Li = 0, Ld = 0.0004;
    public static double Lf = 0.01;//0-770
    public static int Ltarget = 200;//350 - grab

    public enum PoleHeight{
        l,
        m,
        h
    }

    public static double Ap = 0.0064, Ai = 0, Ad = 0.0005;
    public static double Af;
    public static int Atarget; //0-530

    private boolean vomit = false;
    private boolean beacon = false;

    private boolean frontscore = false;
    private boolean stack = false;

    private DcMotorEx leftLift;
    private DcMotorEx rightLift;

    private DcMotorEx leftArm;
    private DcMotorEx rightArm;

    private boolean liftVar = true;

    private Servo lturn;
    private Servo rturn;
    private Servo claw;
    private Servo wrist;
    private Servo guide;
    private Servo backstop;

    private int loffset = 10;

    private Servo leftIntServo;
    private Servo rightIntServo;

    private double lpos = 0;
    private double rpos = 1;
    public static double x = 0.475;

    private boolean endgame = true;

    private RevTouchSensor clawsensor;
    private RevTouchSensor pgsensor;
    private RevTouchSensor limit;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public enum RobotState {
        pre,
        base,
        baseclaw,
        intake,
        grab,
        transfer,
        transfer2,
        inter,
        drop,
        grab2,
        inter2,
        armlow,
        low,
        mid,
        high,
        outtake,
        deposit,
        score,
        specialouttake,
        parkouttake,
        specialtrans,
        vomittransfer,
        vomittransfer1,
        vomitinter,
        vomitdrop,
        vomitgrab2,
        vomitinter2,
        vomithigh,
        vomitouttake,
        vomitouttakeinter,
        vomitdeposit,
        vomitscore,
        vomitbase,
        vomitbaseinter,
        vomitintake,
        terminal,
        terminalOuttake,
        terminalDeposit,
        terminalinter,
        elevatorgrab,
        elevatorgrab2,
        elevatorgrab3,
        elevatorgrab4,
        beacongrab2,
        beaconinter2,
        beaconinter3,
        beacondeposit,
        beacontransfer,
        beaconscore,
        beaconscoreinter,
        knocked,
        right,
        rightinter,
        baseinter,
        clawflip

    }


    enum flip{
        initializeflip,
        transferflip,
        outtakeflip,
        depositflip,
        parkflip,
        elevatorflip,
        betweenflip
    }

    flip flip = initializeflip;
    private boolean lgrab = false;

    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorFrontRight;

    RobotState botState = pre;
    PoleHeight poleHeight = m;

    private double multiplier = 1;
    private double Turnmultiplier = 1;
    private double progSlowmode = 1;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime gametimer = new ElapsedTime();
    ElapsedTime droptimer = new ElapsedTime();
    ElapsedTime testimer = new ElapsedTime();
    ElapsedTime starttimer = new ElapsedTime();

    private Servo servo;
    private Servo servotwo;
    private MotionProfile profile;
    private double startTime;

    private boolean game = true;
    @Override
    public void init(){
        Lcontroller = new PIDController(Lp,Li,Ld);
        Acontroller = new PIDController(Ap,Ai,Ad);

        leftIntServo = hardwareMap.get(Servo.class,"leftIntServo");
        rightIntServo = hardwareMap.get(Servo.class,"rightIntServo");

        leftLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        rightLift = hardwareMap.get(DcMotorEx.class,"leftLift");

        leftArm = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        rightArm = hardwareMap.get(DcMotorEx.class,"intakeRight");

        servo = hardwareMap.get(Servo.class, "lturn");
        servotwo = hardwareMap.get(Servo.class, "rturn");

        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        guide = hardwareMap.get(Servo.class, "poleguide");

        backstop = hardwareMap.get(Servo.class, "backstop");

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"leftFront");
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"leftRear");
        motorFrontRight = hardwareMap.get(DcMotorEx.class,"rightFront");
        motorBackRight = hardwareMap.get(DcMotorEx.class,"rightRear");

        limit = hardwareMap.get(RevTouchSensor.class, "limit");
        pgsensor = hardwareMap.get(RevTouchSensor.class, "pg");
        clawsensor = hardwareMap.get(RevTouchSensor.class, "claw");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        starttimer.reset();
        gametimer.reset();
        droptimer.reset();
        timer.reset();
        testimer.reset();
        guide.setPosition(0.3);

        servo.setPosition(0.25);
        servotwo.setPosition(0.75);

        generateMotionProfile(servo.getPosition(), servo.getPosition(), 10, 10);

    }

    @Override
    public void start() {
        // Get the current time
        startTime = getRuntime();
    }

    @Override
    public void loop(){

        int lstickpos = (int) (20 * gamepad2.left_stick_y);
        int rstickpos = (int) (10 * -gamepad2.right_stick_y);


        if(gamepad2.left_stick_y != 0){
            Ltarget = Ltarget - lstickpos;

        }
        if(gamepad2.right_stick_y != 0){
            Atarget = Atarget - rstickpos;
        }


        Lcontroller.setPID(Lp, Li, Ld);
        int liftPos = leftLift.getCurrentPosition();

        double Lpid = Lcontroller.calculate(liftPos, Ltarget);

        double Lff = Lf;

        double Lpower = Lpid + Lff;

        leftLift.setPower(Lpower);
        rightLift.setPower(Lpower);



        Acontroller.setPID(Ap, Ai, Ad);

        int armPos = leftArm.getCurrentPosition();

        double Apid = Acontroller.calculate(armPos, Atarget);


        double Apower = Apid;

        leftArm.setPower(Apower);
        rightArm.setPower(Apower);

        if(testimer.seconds()> 0 && game){
            gametimer.reset();
            game = false;
        }

        double y = -gamepad1.left_stick_y * multiplier * progSlowmode; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * multiplier * progSlowmode; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x * 0.7 * Turnmultiplier * progSlowmode;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);

        //Normal
        if(gamepad1.dpad_up){
            Turnmultiplier = 1;
            multiplier = 1;
        }

        //Slow
        if(gamepad1.dpad_right){
            Turnmultiplier = 0.70;
            multiplier = 1;
        }

        //Super slow
        if(gamepad1.dpad_down){
            Turnmultiplier = 0.4;
            multiplier = 0.5;
        }

        if(gamepad1.dpad_left){
            Turnmultiplier = 0.5;
            multiplier = 0.6;
        }

        double elapsedTime = getRuntime() - startTime;
        double servoPosition = profile.get(elapsedTime).getX();

        servo.setPosition(servoPosition);
        servotwo.setPosition(1 - servoPosition);


        progSlowmode = Range.clip(-0.666*(gamepad1.left_trigger) + 1, 0, 1);


        switch (botState) {
            case pre:
                Atarget = 150;
                wrist.setPosition(0.74);
                leftIntServo.setPosition(0.87);
                rightIntServo.setPosition(0);
                claw.setPosition(0.3);
                timer.reset();
                botState = clawflip;
                break;

            case baseclaw:
                if(timer.seconds() > 0.3){
                    flip = transferflip;
                    timer.reset();
                    botState = base;
                }
                break;

            case base:
                wrist.setPosition(0.74);
                Ltarget = elevatortransfer;
                frontscore = false;
                if(gamepad1.right_trigger > 0){
                    guide.setPosition(0.35);
                }
                else if(starttimer.seconds() < 6){
                    guide.setPosition(0.1);
                }
                else{
                    guide.setPosition(0.05);
                }
                //Default Position
                if(gamepad1.right_bumper){
                    Ltarget = elevatortransfer;
                    Atarget = 530;
                    backstop.setPosition(backstopintaketeleop);
                    leftIntServo.setPosition(0.87);
                    rightIntServo.setPosition(0);
                    claw.setPosition(0.3);
                    wrist.setPosition(0.74);
                    timer.reset();
                    botState = intake;
                }

                if(gamepad2.left_bumper){
                    claw.setPosition(0.3);
                    flip = depositflip;
                    Atarget = 145;
                    wrist.setPosition(0.08);
                    guide.setPosition(0.35);
                    Ltarget = elevatorbackwards;
                    timer.reset();
                    botState = elevatorgrab;
                }

                if(gamepad1.circle){
                    backstop.setPosition(0.);
                    Ltarget = elevatortransfer;
                    Atarget = 453;
                    backstop.setPosition(backstopconeflip);
                    leftIntServo.setPosition(0.87);
                    rightIntServo.setPosition(0);
                    claw.setPosition(0.3);
                    wrist.setPosition(0.74);
                    timer.reset();
                    botState = right;
                }

                if(gamepad1.triangle){
                    Ltarget = elevatortransfer;
                    Atarget = 400;
                    backstop.setPosition(backstopretract);
                    leftIntServo.setPosition(0.2);
                    rightIntServo.setPosition(0.7);
                    claw.setPosition(0.3);
                    wrist.setPosition(0.74);
                    timer.reset();
                    botState = right;
                }
                break;


            case right:
                if(gamepad1.right_bumper){
                    Atarget = 530;
                    timer.reset();
                    botState = rightinter;
                }
                break;

            case rightinter:
                if(timer.seconds() > 0.2){
                    botState = grab;
                }
                break;
            case intake:
                if(timer.seconds()>0.15){
                    Atarget = 530;
                    Turnmultiplier = 0.4;
                    multiplier = 1;
                    flip = transferflip;
                }

                if(timer.seconds() > 0.16){
                    botState = grab;
                }
                break;

            case grab:
                if(((clawsensor.isPressed()) || gamepad1.right_bumper) && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1){
                    leftIntServo.setPosition(0.2);
                    rightIntServo.setPosition(0.7);
                    timer.reset();
                    Turnmultiplier = 1;
                    multiplier = 0.5;
                    botState = transfer;
                }

                if(((clawsensor.isPressed()) || gamepad1.right_bumper) && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1 && gamepad2.left_trigger > 0.1){
                    leftIntServo.setPosition(0.2);
                    rightIntServo.setPosition(0.7);
                    beacon = true;
                    timer.reset();
                    Turnmultiplier = 0.4;
                    multiplier = 1;
                    botState = transfer;
                }


                if(gamepad1.left_bumper){
                    if(Math.abs(leftArm.getCurrentPosition() - Atarget) < 25 && timer.seconds() > 0.1){
                        leftIntServo.setPosition(0.2);
                        rightIntServo.setPosition(0.7);
                        Ltarget = elevatorvomittransfer;
                        timer.reset();

                        vomit = false;
                        botState = vomittransfer;
                    }
                }

                if(((clawsensor.isPressed()) || gamepad1.right_bumper) && gamepad1.left_trigger > 0.1){
                    if(Math.abs(leftArm.getCurrentPosition() - Atarget) < 25){
                        telemetry.addLine("state 3");
                        leftIntServo.setPosition(0.2);
                        rightIntServo.setPosition(0.7);
                        timer.reset();
                        botState = terminal;
                    }
                }
                break;

            case transfer:
                if(timer.seconds() > 0){
                    backstop.setPosition(backstopretract);
                }
                if(timer.seconds() > 0.06){
                    telemetry.addLine("state 4");
                    Atarget = 90;
                    timer.reset();
                    botState = inter;
                }
                break;

            case vomittransfer:
                backstop.setPosition(backstopretract);
                if ((timer.seconds() > 0 && vomit) || (timer.seconds() > 0.25 && !vomit)){
                    telemetry.addLine("state 4");
                    Atarget = 100;
                    grabTest.reset();
                    timer.reset();
                    botState = vomitinter;
                }
                break;

            case terminal:
                if (timer.seconds() > 0.1) {
                    Turnmultiplier = 0.8;
                    multiplier = 1;
                    telemetry.addLine("state 4");
                    Atarget = 200;
                    timer.reset();
                    botState = armlow;
                }
                break;

            case inter:
                if(Math.abs(leftArm.getCurrentPosition() - Atarget) < 25){
                    Atarget = 0;
                    timer.reset();
                    botState = drop;
                }
                break;

            case vomitinter:
                if(Math.abs(leftArm.getCurrentPosition() - Atarget) < 25){
                    timeTest1 = grabTest.seconds();
                    Atarget = 30;
                    timer.reset();
                    botState = vomitdrop;
                }
                break;

            case drop:
                if (Math.abs(leftArm.getCurrentPosition() - Atarget) < 50 || timer.seconds() > 0.35) {
                    telemetry.addLine("state 5");
                    claw.setPosition(0.09);
                    guide.setPosition(0.35);
                    timer.reset();
                    if(beacon){
                        beacon = false;
                        botState = beacongrab2;
                    }else{
                        botState = grab2;
                    }
                }
                //disengage servos
                break;

            case vomitdrop:
                if ((Math.abs(leftArm.getCurrentPosition() - Atarget) < 50) || timer.seconds() > 0.5) {
                    telemetry.addLine("state 5");
                    claw.setPosition(0.09);
                    timer.reset();
                    botState = vomitgrab2;
                }
                break;

            case grab2:
                multiplier = 1;
                if(timer.seconds() > 0.2){
                    leftIntServo.setPosition(0.87);
                    rightIntServo.setPosition(0);
                    gamepad2.rumble(500);
                }

                if(timer.seconds() > 0.3){
                    telemetry.addLine("state 6");
                    Atarget = 300;
                    timer.reset();
                    botState = inter2;
                }
                break;

            case vomitgrab2:
                if(timer.seconds() > 0.1){
                    leftIntServo.setPosition(0.87);
                    rightIntServo.setPosition(0);
                    gamepad2.rumble(500);
                }
                if(timer.seconds() > 0.15){
                    telemetry.addLine("state 6");
                    Atarget = 470;
                    timer.reset();
                    botState = vomitinter2;
                }
                break;

            case inter2:
                if(timer.seconds() > 0.05){
                    flip = outtakeflip;
                    wrist.setPosition(0.74);
                    //FIX
                    botState = mid;
                }
                break;

            case vomitinter2:
                if(timer.seconds() > 0.05){
                    flip = outtakeflip;
                    guide.setPosition(0.35);
                    botState = vomithigh;
                }
                break;

            case low:
                if(timer.seconds() > 0.1 && timer.seconds() < 0.12){
                    flip = outtakeflip;
                    wrist.setPosition(0.08);
                }
                if(timer.seconds() > 0.5){
                    Ltarget = elevatorlow;
                    timer.reset();
                    botState = outtake;
                }
                break;

            case armlow:
                if(gamepad1.left_bumper){
                    timer.reset();
                    botState = terminalOuttake;
                }
                if(gamepad1.right_bumper){
                    timer.reset();
                    botState = specialouttake;
                }

                break;

            case specialouttake:
                if(gamepad1.right_bumper){
                    leftIntServo.setPosition(0.87);
                    rightIntServo.setPosition(0);
                    Turnmultiplier = 1;
                    multiplier = 1;
                    timer.reset();
                    botState = specialtrans;
                }
                break;

            case parkouttake:
                if(Math.abs(leftArm.getCurrentPosition() - Atarget) < 15){
                    leftIntServo.setPosition(0.87);
                    rightIntServo.setPosition(0);
                }
                break;

            case specialtrans:
                if(timer.seconds() > 0.3){
                    botState = baseclaw;
                }
                break;

            case mid:
                if(timer.seconds() > 0){
                    Ltarget = elevatormid;
                    timer.reset();
                    botState = outtake;
                }
                break;

            case high:
                if(timer.seconds() > 0.2){
                    Ltarget = elevatorhigh;
                    timer.reset();
                    botState = outtake;
                }
                break;

            case terminalOuttake:
                if(timer.seconds() > 0.1) {
                    Atarget = 500;
                    timer.reset();
                    botState = terminalDeposit;
                }
                break;

            case vomithigh:
                if(timer.seconds() > 0){
                    Ltarget = elevatorhigh;
//                    Atarget = 550;
                    wrist.setPosition(0.08);
                    timer.reset();
                    botState = vomitouttake;
                }
                break;


            case outtake:
                if(timer.seconds() > 0){
                    flip = outtakeflip;
                    wrist.setPosition(0.08);
                    telemetry.addLine("state 7");
                    timer.reset();
                    botState = deposit;
                    //deposit claw gets ready to place
                }
                break;

            case vomitouttake:
                if(timer.seconds() > 0){
                    flip = outtakeflip;
                    timer.reset();
                    botState = vomitouttakeinter;
                }
                break;

            case vomitouttakeinter:
                if(timer.seconds() > 0.1 && Math.abs(leftLift.getCurrentPosition() - Ltarget) < 30){
                    flip = depositflip;
                    timer.reset();
                    botState = vomitdeposit;
                }
                break;

            case deposit:
                if(timer.seconds() > 0.2){
                    Atarget = 100;
                }
                if(poleHeight == l && timer.seconds() > 0.4){
                    Ltarget = elevatorlowinter;
                }
                if(poleHeight == l && timer.seconds() > 0.8){
                    Ltarget = elevatorlow;
                }

                if(poleHeight == m){
                    Ltarget = elevatormid;
                }
                if(poleHeight == h){
                    Ltarget = elevatorhigh;
                }
                if(!lgrab){
                    if(gamepad1.right_trigger > 0){
                        if(poleHeight == l){
                            guide.setPosition(0.25);
                        }
                        else if(poleHeight == m || poleHeight == h){
                            guide.setPosition(0.35);
                        }
                        if((gamepad1.right_bumper)){
                            Atarget = 100;
                            flip = depositflip;
                            if(leftLift.getCurrentPosition() > 100 && leftLift.getCurrentPosition() < 250){
                                guide.setPosition(0.05);
                            }
                            timer.reset();
                            botState = score;
                        }
                    }
                    else{
                        guide.setPosition(0.05);
                    }
                }

                if(lgrab){
                    if(gamepad1.right_trigger > 0){
                        if(poleHeight == l){
                            guide.setPosition(0.25);
                        }
                        else if(poleHeight == m || poleHeight == h){
                            guide.setPosition(0.35);
                        }
                        if((gamepad1.right_bumper) && timer.seconds() > 0.5){
                            Atarget = 100;
                            flip = elevatorflip;
                            timer.reset();
                            botState = score;
                        }
                        timer.reset();
                        botState = score;
                    }
                    else{
                        guide.setPosition(0.05);
                    }
                }

                break;

            case vomitdeposit:
                Atarget = 550;
                if(timer.seconds() > 0.1 && Math.abs(leftLift.getCurrentPosition() - Ltarget) < 25){
                    guide.setPosition(0.05);
                    claw.setPosition(0.3);
                }
                if(timer.seconds() > 0.15){
                    claw.setPosition(0.3);
                    guide.setPosition(0.05);
                    wrist.setPosition(0.74);
                    flip = transferflip;
                    Ltarget = elevatorvomittransfer;
                    timer.reset();
                    botState = vomitbaseinter;
                }
                break;

            case vomitbaseinter:
                if(timer.seconds() > 0.4){
                    leftIntServo.setPosition(0.16);
                    rightIntServo.setPosition(0.74);
                    botState = vomitbase;
                }
                break;

            case terminalDeposit:
                if(gamepad1.right_bumper && timer.seconds() > 0.4){
                    leftIntServo.setPosition(0.87);
                    rightIntServo.setPosition(0);
                    Turnmultiplier = 1;
                    multiplier = 1;
                    timer.reset();
                    botState = terminalinter;
                }
                break;

            case terminalinter:
                if(timer.seconds() > 0.2){
                    Atarget = 150;
                    botState = baseclaw;
                }
                break;

            case score:
                Atarget = 165;
                if(timer.seconds() > 0.1){
                    claw.setPosition(0.3);
                    guide.setPosition(0.05);
                    if(timer.seconds() > 0.2){
                        flip = transferflip;
                        timer.reset();
                        botState = clawflip;
                    }
                }
                break;

            case vomitbase:
                if(timer.seconds() > 0.1 && gamepad1.left_bumper) {
                    vomit = true;
                    timer.reset();
                    botState = vomittransfer;
                }

                if(gamepad1.right_bumper && timer.seconds() > 0.2){
                    leftIntServo.setPosition(0.2);
                    rightIntServo.setPosition(0.7);

                    timer.reset();
                    Turnmultiplier = 1;
                    multiplier = 1;
                    botState = transfer;
                    vomit = false;
                }
                break;

            case elevatorgrab:
                if(timer.seconds() > 0.15){
                    Ltarget = elevatorbackwardsdown;
                }
                if(timer.seconds() > 0.25){
                    guide.setPosition(0.05);
                }
                if(gamepad1.right_bumper){
                    claw.setPosition(0.09);
                    timer.reset();
                    botState = elevatorgrab2;
                }
                break;

            case elevatorgrab2:
                if(timer.seconds() > 0.25){
                    Ltarget = elevatorhigh;
                    timer.reset();
                    botState = elevatorgrab3;
                }
                break;

            case elevatorgrab3:
                if(timer.seconds() > 0.1 && Math.abs(leftLift.getCurrentPosition() - Ltarget) < 25){
                    lgrab = true;
                    flip = outtakeflip;
                    guide.setPosition(0.35);
                    timer.reset();
                    botState = elevatorgrab4;
                }
                break;
            case elevatorgrab4:
                if(timer.seconds() > 0.75){
                    timer.reset();
                    botState = deposit;
                }
                break;
            case beacongrab2:
                if(timer.seconds() > 0.1){
                    leftIntServo.setPosition(0.87);
                    rightIntServo.setPosition(0);
                    gamepad2.rumble(500);
                }

                if(timer.seconds() > 0.15){
                    Atarget = 530;
                    timer.reset();
                    botState = beaconinter3;
                }
                break;

            case beaconinter3:
                if(timer.seconds() > 0.1){
                    backstop.setPosition(backstopintaketeleop);
                    flip = outtakeflip;
                    timer.reset();
                    botState = beaconinter2;
                }
                break;

            case beaconinter2:
                if(timer.seconds() > 0.05){
                    wrist.setPosition(0.08);
                }
                if(timer.seconds() > 0.4){
                    Ltarget = elevatormid;
                    timer.reset();
                    botState = beacondeposit;
                }
                break;

            case beacondeposit:
                if((clawsensor.isPressed() || gamepad1.right_bumper)){
                    leftIntServo.setPosition(0.2);
                    rightIntServo.setPosition(0.7);
                    timer.reset();
                    Turnmultiplier = 1;
                    multiplier = 1;
                    botState = beacontransfer;
                }
                break;

            case beacontransfer:
                if(poleHeight == l){
                    Ltarget = elevatorlow;
                }
                if(poleHeight == m){
                    Ltarget = elevatormid;
                }
                if(poleHeight == h){
                    Ltarget = elevatorhigh;
                }
                if(timer.seconds() > 0.1){
                    backstop.setPosition(backstopretract);
                    Atarget = 220;
                }
                if(gamepad1.left_bumper){
                    leftIntServo.setPosition(0.87);
                    rightIntServo.setPosition(0);
                    frontscore = true;
                }

                if(gamepad1.right_trigger > 0){
                    if(poleHeight == l){
                        guide.setPosition(0.25);
                    }
                    else if(poleHeight == m || poleHeight == h){
                        guide.setPosition(0.35);
                    }
                    if((gamepad1.right_bumper && timer.seconds() > 0.5)){
                        flip = depositflip;
                        timer.reset();
                        if(frontscore){
                            botState = baseinter;
                        }
                        if(!frontscore){
                            botState = beaconscore;
                        }
                    }
                }else{
                    guide.setPosition(0.05);
                }
                break;

            case baseinter:
                if(timer.seconds() > 0.1){
                    claw.setPosition(0.3);
                }
                if(timer.seconds() > 0.2){
                    timer.reset();
                    flip = transferflip;
                    botState = clawflip;
                }
                break;

            case clawflip:
                if(timer.seconds() > 0.1){
                    botState = baseclaw;
                }
                break;

            case beaconscore:
                if(timer.seconds() > 0.1){
                    claw.setPosition(0.3);
                    if(timer.seconds() > 0.25){
                        flip = transferflip;
                        timer.reset();
                        botState = beaconscoreinter;
                    }
                }
                break;
            case beaconscoreinter:
                claw.setPosition(0.3);
                Ltarget = elevatortransfer;
                if(timer.seconds() > 0.25){
                    wrist.setPosition(0.74);
                }
                if(timer.seconds() > 0.5){
                    botState = transfer;
                }
                break;
            default:
                // should never be reached, as botState should never be null
                botState = base;
        }

        switch (flip) {
            case initializeflip:
                generateMotionProfile(servo.getPosition(), clawinitializepos, 15, 15);
                startTime = getRuntime();
                flip = betweenflip;
                break;

            case transferflip:
                generateMotionProfile(servo.getPosition(), clawtransferpos, 15, 15);
                startTime = getRuntime();
                flip = betweenflip;
                break;

            case outtakeflip:
                generateMotionProfile(servo.getPosition(), clawouttakepos, 15, 15);
                startTime = getRuntime();
                flip = betweenflip;
                break;

            case depositflip:
                generateMotionProfile(servo.getPosition(), clawdepositpos, 15, 15);
                startTime = getRuntime();
                flip = betweenflip;
                break;

            case parkflip:
                generateMotionProfile(servo.getPosition(), clawparkpos, 15, 15);
                startTime = getRuntime();
                flip = betweenflip;
                break;

            case elevatorflip:
                generateMotionProfile(servo.getPosition(), clawelevatorpos, 15, 15);
                startTime = getRuntime();
                flip = betweenflip;
                break;

            case betweenflip:

                break;

        }

        if(gamepad2.dpad_up){
            poleHeight = h;
        }
        if(gamepad2.dpad_down){
            poleHeight = l;
        }
        if(gamepad2.dpad_left){
            poleHeight = m;
        }


        if(gamepad2.cross){
            poleHeight = h;
            Atarget = 370;
        }
        if(gamepad2.square){
            poleHeight = m;
            Atarget = 390;
        }
        if(gamepad2.triangle){
            poleHeight = l;
            Atarget = 410;
        }
        if(gamepad2.circle){
            Atarget = 440;
        }

        if(gamepad1.cross){
            claw.setPosition(0.3);
            guide.setPosition(0.05);
            Turnmultiplier = 1;
            multiplier = 1;
            botState = pre;
        }

        if(gamepad2.left_trigger > 0.1){
            gamepad1.rumble(500);
        }

        if(gamepad2.right_trigger > 0.1){
            gamepad1.rumble(100);
        }
        if(gamepad2.x){
            gametimer.reset();
        }
        telemetry.addData("game timer", gametimer.seconds());

        if(gametimer.seconds() > 80 && endgame){
            gamepad1.rumble(500);
            gamepad2.rumble(500);
            endgame = false;
        }

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        telemetry.addData("time var - replace in vomitinter", timeTest1);


        telemetry.addData("left arm", leftArm.getCurrentPosition());
        telemetry.addData("right arm", rightArm.getCurrentPosition());
        telemetry.addData("left arm amp", leftArm.getCurrentPosition());
        telemetry.addData("right arm amp", rightArm.getCurrentPosition());
        telemetry.addData("A target", Atarget);
        telemetry.addData("L target", Ltarget);
        telemetry.addData("servo1", servo.getPosition());
        telemetry.addData("servo2", servotwo.getPosition());
        telemetry.update();
    }

    private void generateMotionProfile(double initialPosition, double finalPosition,
                                       double maxVelocity, double maxAcceleration) {
        // Generate the motion profile
        double maxJerk = 0;

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(initialPosition, 0, 0),
                new MotionState(finalPosition, 0, 0),
                maxVelocity,
                maxAcceleration,
                maxJerk
        );

        // Reset the start time
        startTime = getRuntime();
    }

}


