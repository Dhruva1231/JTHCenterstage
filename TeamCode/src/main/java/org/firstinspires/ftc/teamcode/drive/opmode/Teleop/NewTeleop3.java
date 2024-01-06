package org.firstinspires.ftc.teamcode.drive.opmode.Teleop;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.barrier;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.barrierinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.base;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.cancel;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.cancelinter1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.cancelinter2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.deposit;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.idle;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.initialization1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.initialization2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.initialize;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.intake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.intakeinter1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.outtake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.outtakeinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.outtakepre;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.pre;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.shoot;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.transfer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name="New Tleeop 3")
public class NewTeleop3 extends OpMode {

    private IMU imu;
    private double slow = 1.0;
    private double turnslow = 1.0;
    private double progSlowmode = 1.0;

    private boolean left = false;
    private boolean right = false;
    boolean onOff = false;
    public DcMotorEx intakeLeftExt;
    public DcMotorEx outtakeMotor;
    public DcMotorEx intakeRightExt;
    private PIDController Lcontroller;
    private PIDController Ocontroller;

    public static double amp1;
    public static double amp2;

    //right claw grab = 0.5, retract = 0
    //left claw grab = 0.44, retract = 0.98

    public static double Lp = 0.006, Li = 0, Ld = 0.0001;

    public static double Lf;

    //Ltarget Max 750, Min -75
    public static int Ltarget;

    //Otarget Max 800, Min 25
    public static int Otarget;

    public static double Op = 0.012, Oi = 0, Od = 0.0002;
    public static double Of = -0.08;

    public DcMotorEx intakeMotor;


    public static double p = 0.7;
    public static double e = 0.52;

    public static double wpos = 0.48;
    public static double ppos = 0.47;
    public static double tpos = 0.32;
    public static double c1pos = 0;
    public static double c2pos = 0.98;
    public static double apos = 0.51;

    public Servo pivotleft;
    public Servo pivotright;
    public Servo elbowleft;
    public Servo elbowright;
    public Servo pan;
    public Servo tilt;
    public Servo wrist;
    public Servo claw1;
    public Servo claw2;

    private int counter = 0;

    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private boolean posLockOut = false;
    private boolean posLockInt = false;
    public enum state {
        initialization1,
        initialization2,
        cancel,
        cancelinter1,
        cancelinter2,
        pre,
        initialize,
        intslides,
        base,
        intake,
        intakeinter1,
        transfer,
        outtake,
        outtakepre,
        outtakeinter,
        barrier,
        barrierinter,
        deposit,
        shoot,
        idle,
        specialsensor
    }

    public static double finalconvert;

    public double targetAngleOut = Math.toRadians(90);
    public double targetAngleInt = Math.toRadians(90);

    state New = initialization1;
    ElapsedTime timer  = new ElapsedTime();
    ElapsedTime holdtimer  = new ElapsedTime();


    public CRServo lift;
    public Servo airplane;
    public AnalogInput distance1;
    public static double Hp = 0.03;

    @Override
    public void init(){

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        Lcontroller = new PIDController(Lp,Li,Ld);
        Ocontroller = new PIDController(Op,Oi,Od);

        distance1 = hardwareMap.get(AnalogInput.class, "one");

        intakeLeftExt = hardwareMap.get(DcMotorEx.class, "lint");
        intakeRightExt = hardwareMap.get(DcMotorEx.class, "rint");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "out");

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeftExt.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRightExt.setDirection(DcMotorSimple.Direction.REVERSE);

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

        lift = hardwareMap.get(CRServo.class, "lift");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
//
        elbowleft = hardwareMap.get(Servo.class, "el");
        elbowright = hardwareMap.get(Servo.class, "er");
        pivotleft = hardwareMap.get(Servo.class, "pl");
        pivotright = hardwareMap.get(Servo.class, "pr");

        pan = hardwareMap.get(Servo.class, "0.");
        wrist = hardwareMap.get(Servo.class, "1.");
        tilt = hardwareMap.get(Servo.class, "5.");
        claw1 = hardwareMap.get(Servo.class, "2.");
        claw2 = hardwareMap.get(Servo.class, "4.");
        airplane = hardwareMap.get(Servo.class, "3.");

        elbowleft.setPosition(0.4);
        elbowright.setPosition(1-0.4);
        pivotleft.setPosition(1-0.7);
        pivotright.setPosition(0.7);

        wrist.setPosition(0.48);
        pan.setPosition(0.47);
//        tilt.setPosition(0.32);
        claw1.setPosition(0);
        claw2.setPosition(0.98);
        airplane.setPosition(0);

    }

    @Override
    public void start() {
        timer.reset();
        // Get the current time
    }

    @Override
    public void loop(){


        double max;
        double axial   = -gamepad1.left_stick_y * slow * progSlowmode;
        double lateral =  gamepad1.left_stick_x * slow * progSlowmode * 1.4;
        double yaw     =  gamepad1.right_stick_x * turnslow * progSlowmode;

        progSlowmode = Range.clip(-0.83*(gamepad1.left_trigger) + 1, 0, 1);

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

        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double modifiedHeading = modify(Math.toDegrees(currentHeading));

        double errorOut =  modifiedHeading - Math.toDegrees(targetAngleOut);
        double errorInt = Math.toDegrees(currentHeading) - Math.toDegrees(targetAngleInt);
        double admissibleError = Math.toRadians(1);

        if(gamepad1.options){
            targetAngleOut = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
        if(gamepad1.share){
            targetAngleInt = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        if((Math.abs(errorInt) > admissibleError) && posLockInt){
            leftFrontPower += errorInt*Hp;
            leftBackPower += errorInt*Hp;
            rightBackPower += -errorInt*Hp;
            rightFrontPower += -errorInt*Hp;
        }

        if((Math.abs(errorOut) > admissibleError) && posLockOut){
            leftFrontPower += errorOut*Hp;
            leftBackPower += errorOut*Hp;
            rightBackPower += -errorOut*Hp;
            rightFrontPower += -errorOut*Hp;
        }

        telemetry.addData("currentheading", Math.toDegrees(currentHeading));
        telemetry.addData("error", errorOut);


        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if(gamepad2.dpad_down){
            posLockInt = true;
            posLockOut = false;
        }else if(gamepad1.dpad_right){
            posLockInt = false;
            posLockOut = true;
        }else if(gamepad1.dpad_left || gamepad2.dpad_left){
            posLockInt = false;
            posLockOut = false;
        }

        elbowleft.setPosition(e);
        elbowright.setPosition(1-e);
        pivotleft.setPosition(1-p);
        pivotright.setPosition(p);

        pan.setPosition(ppos);
        tilt.setPosition(tpos);
        wrist.setPosition(wpos);
        claw1.setPosition(c1pos);
        claw2.setPosition(c2pos);
        airplane.setPosition(apos);


        switch (New) {
            case initialization1:
                Otarget = 200;
                if(Math.abs(outtakeMotor.getCurrentPosition()) > 150){
                    tpos = 0.32;
                    timer.reset();
                    New = initialization2;
                }
                break;

            case initialization2:
                if(timer.seconds() > 0.5){
                    Otarget = 30;
                    timer.reset();
                    New = pre;
                }
                break;
            case cancel:
                if(Ltarget > 100){
                    p = 0.8;
                    intakeMotor.setPower(0.2);
                    timer.reset();
                    New = cancelinter1;
                }else{
                    Otarget = 30;
                    p = 0.8;
                    e = 0.795;
                    timer.reset();
                    New = base;
                }

                break;

            case cancelinter1:
                if(timer.seconds() > 0.25){
                    Ltarget = 50;
                    timer.reset();
                    New = cancelinter2;
                }
                break;

            case cancelinter2:
                if(timer.seconds() > 0.5){
                    Ltarget = -50;
                }
                if(timer.seconds() > 0.75){
                    Otarget = 30;
                    p = 0.8;
                    e = 0.795;
                    timer.reset();
                    New = base;
                }
                break;

            case pre:
                //move to intake
                Otarget = 30;
                if(counter == 0){
                    p = 0.33;
                }else{
                    p = 0.135;
                }
                timer.reset();
                New = initialize;
                break;

            case initialize:
                if(timer.seconds() > 0.25){
                    e = 0.65;
                    timer.reset();
                    New = base;
                }
                break;

            case base:
                //move pivot to intake
                if(timer.seconds() > 0.15){
                    p = 0.135;
                    timer.reset();
                    New = intake;
                }
                break;

            case intake:
                if(gamepad2.right_bumper){
                    slow = 0.5;
                    turnslow = 0.3;
                    e = 0.65;
                    Ltarget = 975;
                }
                if(gamepad2.left_bumper){
                    slow = 0.5;
                    turnslow = 0.3;
                    e = 0.65;
                    Ltarget = 700;
                }
                if(gamepad2.right_trigger > 0.2){
                    slow = 0.75;
                    turnslow = 0.5;
                    e = 0.65;
                    Ltarget = 400;
                }
                if(gamepad2.left_trigger > 0.2){
                    slow = 1;
                    turnslow = 1;
                    e = 0.65;
                    Ltarget = -50;
                }

                int lstickpos1 = (int) (20 * gamepad2.right_stick_y);

                if(gamepad2.right_stick_y != 0){
                    Ltarget = Ltarget - lstickpos1;
                }

                if(gamepad1.right_bumper && !onOff && holdtimer.seconds() > 0.25 && !(intakeLeftExt.getVelocity() > 25)){
                    holdtimer.reset();
                    onOff = true;
                    e = 0.6;
                    intakeMotor.setPower(-1);
                }else if(gamepad1.right_bumper && onOff && holdtimer.seconds() > 0.25 && !(intakeLeftExt.getVelocity() > 25)){
                    holdtimer.reset();
                    onOff = false;
                    intakeMotor.setPower(0);
                    e = 0.65;
                }else if(gamepad1.left_bumper){
                    holdtimer.reset();
                    onOff = false;
                    intakeMotor.setPower(1);
                }

                if(gamepad2.dpad_up || gamepad1.dpad_up){
                    slow = 1;
                    turnslow = 1;
                    e = 0.795;
                    p = 0.8;
                    intakeMotor.setPower(-0.5);
                    timer.reset();
                    New = intakeinter1;
                }
                break;

            case intakeinter1:
                if(timer.seconds() > 0.25){
                    Ltarget = -50;
                    timer.reset();
                    New = transfer;
                }
                break;

            case transfer:
                int lstickpos3 = (int) (20 * gamepad2.right_stick_y);

                if(gamepad2.right_stick_y != 0){
                    Ltarget = Ltarget - lstickpos3;
                }

                if(timer.seconds() > 0.35 && intakeLeftExt.getCurrentPosition() < 25){
                    intakeMotor.setPower(0);
                    p = 0.8;
                    timer.reset();
                    New = outtake;
                }
                break;

            case outtake:
                if(timer.seconds() > 0.35){
                    timer.reset();
                    New = outtakepre;
                }
                break;

            case outtakepre:
                intakeMotor.setPower(1);
//                intakeMotor.setPower((Math.pow((timer.seconds()), 2) * 1.25));
                if(timer.seconds() > 1.25){
                    timer.reset();
                    New = outtakeinter;
                }
                break;

            case outtakeinter:
                if(timer.seconds() > 0.25){
                    Otarget = -50;
                    intakeMotor.setPower(0);
                    timer.reset();
                    New = barrier;
                }
                break;

            case barrier:
                c1pos = 0.5;
                c2pos = 0.44;
                if(timer.seconds() > 0.75 && gamepad2.right_bumper){
                    Otarget = 200;
                    timer.reset();
                    New = barrierinter;
                }
                break;

            case barrierinter:
                int lstickpos2 = (int) (15 * gamepad2.right_stick_y);
                if(gamepad2.right_stick_y != 0){
                    Otarget = Otarget - lstickpos2;
                }
                if(timer.seconds() > 0.1){
                    tpos = 0.7;
                }

                if(gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0){
                    wpos = 0.48;

                }
                else if(gamepad1.right_trigger > 0){
                    wpos = 0.15;
                }
                else if(gamepad1.left_trigger > 0){
                    wpos = 0.86;
                }

                if(gamepad2.right_bumper && timer.seconds() > 1.25){
                    Otarget = 700;
                }
                if(gamepad2.left_bumper && timer.seconds() > 0.5){
                    Otarget = 500;
                }

                if(gamepad2.right_trigger > 0.1 && timer.seconds() > 0.5){
                    Otarget = 350;
                }

                if(gamepad2.left_trigger > 0.1 && timer.seconds() > 0.5){
                    Otarget = 200;
                }

                if(gamepad1.right_bumper && timer.seconds() > 0.5){
                    p = 0.33;
                    left = true;
                    c1pos = 0;
                }
                if(gamepad1.left_bumper && timer.seconds() > 0.5){
                    p = 0.33;
                    right = true;
                    c2pos = 0.98;
                }
                if(left && right){
                    timer.reset();
                    New = deposit;
                }
                break;

            case deposit:
                if(timer.seconds() > 0.5){
                    wpos = 0.48;
                    leftBackPower += -0.3;
                    leftFrontPower += -0.3;
                    rightBackPower += -0.3;
                    rightFrontPower += -0.3;
                }

                if(timer.seconds()> 0.8){
                    tpos = 0.32;
                }

                if(timer.seconds() > 1.25){
                    leftBackPower -= -0.3;
                    leftFrontPower -= -0.3;
                    rightBackPower -= -0.3;
                    rightFrontPower -= -0.3;
                    e = 0.65;
                    left = false;
                    right = false;
                    c1pos = 0;
                    c2pos = 0.98;
                    tpos = 0.32;
                    timer.reset();
                    New = pre;
                }
                break;

            case shoot:
                if(timer.seconds() > 0.5){
                    Ltarget = 650;
                }
                if(timer.seconds() > 2){
                    apos = 0.8;
                    timer.reset();
                    New = idle;
                }
                break;

            case idle:

                break;

            case specialsensor:

                break;
        }

        finalconvert = Range.clip((0.8702699 + (-506713.97027)/(1+(Math.pow(distance1.getVoltage(), 2.126704))/0.00000126998012639)), 0.5, 0.85);

        if(gamepad1.triangle){
            New = cancel;
        }

        Lcontroller.setPID(Lp, Li, Ld);

        int armPos = intakeLeftExt.getCurrentPosition();

        double Lpid = Lcontroller.calculate(armPos, Ltarget);

        double Lpower = Lpid;

        intakeLeftExt.setPower(Lpower);
        intakeRightExt.setPower(Lpower);

        Ocontroller.setPID(Op, Oi, Od);

        int outPos = outtakeMotor.getCurrentPosition();

        double Opid = Ocontroller.calculate(outPos, -Otarget);
        double Off = Of;


        double Opower = Opid + Off;

        outtakeMotor.setPower(Opower);


        if(gamepad2.right_stick_button && gamepad2.left_stick_button){
            timer.reset();
            New = shoot;
        }

        if(gamepad2.cross){
            lift.setPower(1);
        }else if(gamepad2.square){
            lift.setPower(-1);
        }else{
            lift.setPower(0);
        }

        double angleError = 0;

        double errorOuttest = Math.toDegrees(currentHeading) - Math.toDegrees(targetAngleOut);

        errorOuttest -= (360*Math.floor(0.5+((errorOuttest)/360.0)));

        telemetry.addData("testinginging", intakeLeftExt.getCurrentPosition());
        telemetry.addData("egvhgvhgrror", errorOuttest);
        telemetry.addData("axon pos", intakeMotor.getCurrentPosition());

        telemetry.addData("modifiedHeading", modifiedHeading);

        telemetry.addData("left", intakeLeftExt.getCurrentPosition());
        telemetry.addData("right", intakeRightExt.getCurrentPosition());
        telemetry.addData("out", outtakeMotor.getCurrentPosition());

        telemetry.addData("pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));

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


