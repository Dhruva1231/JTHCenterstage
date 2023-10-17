package org.firstinspires.ftc.teamcode.drive.opmode.Teleop;



import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.ScrimmageTeleop.state.base;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.ScrimmageTeleop.state.pre;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="Scrimmage Teleop")
public class ScrimmageTeleop extends OpMode {

    public DcMotorEx intakeLeftExt;
    public DcMotorEx intakeRightExt;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public enum state {
        pre,
        base,
        intake,
        transfer,
        outtake,
        barrier
    }

    private boolean fortnite = false;

    state state = pre;

    @Override
    public void init(){
        intakeLeftExt = hardwareMap.get(DcMotorEx.class, "lint");
        intakeRightExt = hardwareMap.get(DcMotorEx.class, "rint");
    }

    @Override
    public void start() {
        // Get the current time
    }

    @Override
    public void loop(){
        switch (state) {
            case pre:
                if(fortnite){
                    state = base;
                }
                break;

            case base:

                break;
        }


    telemetry.addData("turret-rot-position", intakeLeftExt.getCurrentPosition());
    telemetry.addData("arm-pivot-position", intakeRightExt.getCurrentPosition());
    Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
    telemetry.update();
    }

}


