package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FiringArm extends Subsystem{

    public enum FiringState {
        waiting(0),
        firing(1);

        double pos;
        FiringState(double position) {
            this.pos  = position;
        }
    }
    Servo armServo;
    public FiringArm(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        armServo = hardwareMap.servo.get("ArmServo");
    }

    public void setFiringState(FiringState state) {
        this.armServo.setPosition(state.pos);
    }

    public Action setFiringStateAction(FiringState state) {
        return telemetryPacket -> {
            this.setFiringState(state);
            return false;
        };
    }


}
