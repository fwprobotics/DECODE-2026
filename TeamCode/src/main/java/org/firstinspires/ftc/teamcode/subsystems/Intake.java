package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends Subsystem {
    DcMotor intakeMotor;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {

        super(hardwareMap, telemetry);
        intakeMotor = hardwareMap.dcMotor.get("intake");
//        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public Action runIntake()  {
        return TelemetryPacket -> {
        intakeMotor.setPower(1); return false;};
    }
    public Action reset()  {
        return TelemetryPacket -> { intakeMotor.setPower(0);return false;};
    }
}
