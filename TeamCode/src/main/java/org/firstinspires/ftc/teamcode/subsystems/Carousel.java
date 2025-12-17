package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Carousel extends Subsystem {

    public double GreenRotationOffset;
    Servo carouselServo;
    public enum CarouselState {
        Igreen(1),
        Ired(.6),
        Iblue(.2),
        Fgreen(.4),
        Fred(0),
        Fblue(.8);

        public final double pos;
        CarouselState(double pos) {
            this.pos = pos;
        }
    }
    public Carousel(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        carouselServo = hardwareMap.servo.get("CarouselServo");

    }
    public void SetCarouselState(CarouselState state) {
        carouselServo.setPosition(state.pos);
        UpdateBallPositions(state.pos);
    }

    public void SetManualCarouselState (double position) {
        carouselServo.setPosition(position);
    }

    protected void UpdateBallPositions(double rotation) {
        this.GreenRotationOffset += rotation;
    }
}
