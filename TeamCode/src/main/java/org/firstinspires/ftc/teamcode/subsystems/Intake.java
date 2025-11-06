package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends Subsystem {
    DcMotor intakeMotor;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        intakeMotor = hardwareMap.dcMotor.get("intake");
    };
    public void runIntake()  {
        intakeMotor.setPower(.8);
    };
    public void reverseIntake()  {
        intakeMotor.setPower(-.8);
    };
    public Action runIntakeAction() {
            return TelemetryPacket -> {
                this.runIntake();
                return false;
            };
    };
    public Action newballAction () {
        return TelemetryPacket -> {
            this.runIntake();
            new SleepAction(.5);
            this.reset();
            return false;
        };
    };
    public Action resetAction()  {
        return TelemetryPacket -> { this.reset();return false;};
    };
    public void reset() {
        intakeMotor.setPower(0);
    };
}
