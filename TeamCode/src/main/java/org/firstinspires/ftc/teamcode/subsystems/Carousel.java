package org.firstinspires.ftc.teamcode.subsystems;

import static java.util.Map.entry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.security.cert.TrustAnchor;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class Carousel extends Subsystem {

    CarouselState carouselState;
    Servo carouselServo;
    ColorSensor sensorColor;
 //   List<Boolean> filled = new ArrayList<>();
   // static final List<CarouselState> firingStates = new ArrayList<>(Arrays.asList(CarouselState.Fgreen, CarouselState.Fblue, CarouselState.Fred));
    Map<CarouselState, Integer> MagazineState = Map.ofEntries(
            entry(CarouselState.Fred, 0),
            entry(CarouselState.Fblue, 0),
            entry(CarouselState.Fgreen, 0)
    );
  //  List<CarouselState> greenFiringPos;
    public enum CarouselState {
      Fred(0),
      Iblue(.2),
      Fgreen(.4),
      Ired(.6),
      Fblue(.8),
      Igreen(1);

        public final double pos;
        CarouselState(double pos) {
            this.pos = pos;
        }
        public CarouselState next() {
          // No bounds checking required here, because the last instance overrides
          return values()[ordinal() + 1];
      }
    }
    public Carousel(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        carouselServo = hardwareMap.servo.get("CarouselServo");
        sensorColor = hardwareMap.get(ColorSensor.class, "colorsensor");
        this.SetCarouselState(CarouselState.Ired);
        carouselState = CarouselState.Ired;

    }
    public void SetCarouselState(CarouselState state) {
        carouselServo.setPosition(state.pos);
    }
    public int isGreen() {
      return 1;
    };
    public void ReadColorSensor() {
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
    }

    public void StoreNewBall() {
        switch (carouselState) {
            case Ired:
                MagazineState.put(CarouselState.Fred, isGreen());break;
            case Iblue:
                MagazineState.put(CarouselState.Fblue, isGreen());break;
            case Igreen:
                MagazineState.put(CarouselState.Fblue, isGreen()); break;}
    }
    public void FireGreen(){
        for (Map.Entry<CarouselState, Integer> entry : MagazineState.entrySet()) {
            if (entry.getValue() == 2) {
                SetCarouselState(entry.getKey());
                carouselState = entry.getKey();
               // MagazineState.put(entry.getKey(), 0);
            }
        }
    }

    public void BallFired() {
        MagazineState.put(carouselState, 0);
    }
    public void FirePurple(){
        for (Map.Entry<CarouselState, Integer> entry : MagazineState.entrySet()) {
            if (entry.getValue() == 1) {
                SetCarouselState(entry.getKey());
                carouselState = entry.getKey();
                // MagazineState.put(entry.getKey(), 0);
            }
        }
    }

    public void rotate() {
        carouselState = carouselState.next();
        SetCarouselState(carouselState);
    }
    public void SetManualCarouselState (double position) {
        carouselServo.setPosition(position);
    }

}
