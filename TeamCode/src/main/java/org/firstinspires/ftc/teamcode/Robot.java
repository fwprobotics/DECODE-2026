package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.FieldTrajectoryPlanner;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.StereoCamera;
//import org.firstinspires.ftc.teamcode.autonomous.FieldTrajectoryPlanner;
//import org.firstinspires.ftc.teamcode.pipelines.HuskySampleDetect;
//import org.firstinspires.ftc.teamcode.subsystems.Arm;
//import org.firstinspires.ftc.teamcode.subsystems.Claw;
//import org.firstinspires.ftc.teamcode.subsystems.Hang;
//import org.firstinspires.ftc.teamcode.subsystems.Lift;
//import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class Robot {
    public enum AutoPos {
        REDBASKET (1, -1),
        REDWALL (1, -1),
        BLUEBASKET ( -1, 1),
        BLUEWALL (-1, 1);
        public int xMult;
        public int yMult;
        AutoPos(int xMult, int yMult) {
            this.xMult = xMult;
            this.yMult = yMult;
        }
    }
    public AutoPos autoPos;
    public MecanumDrive drive;
    public Pose2d startingPos = new Pose2d(12, 68, Math.toRadians(-90 ));
    public Intake intake;
    public Launcher launcher;
    public Camera leftCamera;
    public Camera rightCamera;
    public StereoCamera stereoCamera;
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AutoPos autoPos, boolean teleop) {
        this.intake = new Intake(hardwareMap, telemetry);
        this.launcher = new Launcher(hardwareMap, telemetry);
        this.autoPos = autoPos;
        this.startingPos = new Pose2d(12*autoPos.xMult, 68* autoPos.yMult, Math.toRadians(-90* autoPos.yMult));
        this.drive = new MecanumDrive(hardwareMap, startingPos);
        this.leftCamera = new Camera("WebcamLeft", hardwareMap,telemetry);
        this.rightCamera = new Camera("WebcamRight", hardwareMap,telemetry);
        this.stereoCamera = new StereoCamera(this.leftCamera, this.rightCamera, hardwareMap, telemetry);
    }
    public FieldTrajectoryPlanner createTrajectoryPlanner() {
        return new FieldTrajectoryPlanner(this);
    }
}