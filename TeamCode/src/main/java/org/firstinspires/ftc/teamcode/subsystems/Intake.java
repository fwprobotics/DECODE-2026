package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends Subsystem {
    DcMotor intakeMotor;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {

        super(hardwareMap, telemetry);
        intakeMotor = hardwareMap.dcMotor.get("intake");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void runIntake()  {
        intakeMotor.setPower(.7);
    }
}
