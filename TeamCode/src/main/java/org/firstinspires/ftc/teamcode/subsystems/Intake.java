package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends Subsystem {

    DcMotor intakeLeftMotor;
    DcMotor intakeRightMotor;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        intakeLeftMotor = hardwareMap.dcMotor.get("intakeleft");
        intakeRightMotor = hardwareMap.dcMotor.get("intakeright");
    };
    public void runIntake()  {
        intakeLeftMotor.setPower(.8);
        intakeRightMotor.setPower(.8);
    };
    public void reverseIntake()  {
        intakeLeftMotor.setPower(-.8);
        intakeRightMotor.setPower(-.8);
    };
    public Action runIntakeAction() {
            return TelemetryPacket -> {
                this.runIntake();
                return false;
            };
    };
    public Action newballAction () {
        return TelemetryPacket -> {
            this.runIntakeAction();
            new SleepAction(.5);
            this.resetAction();
            return false;
        };
    };
    public Action resetAction()  {
        return TelemetryPacket -> { this.reset();return false;};
    };

    public Action stallAction()  {
        return TelemetryPacket -> { this.stall();return false;};
    };

    public void reset() {
        intakeLeftMotor.setPower(0);
        intakeRightMotor.setPower(0);
    };

    public void stall() {
        int rightspeed = intakeRightMotor.getCurrentPosition();
        int leftspeed = intakeLeftMotor.getCurrentPosition();
        intakeRightMotor.setTargetPosition(rightspeed);
        intakeLeftMotor.setTargetPosition(leftspeed);
    }
}
