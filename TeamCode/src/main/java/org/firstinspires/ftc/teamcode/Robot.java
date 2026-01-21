package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Autonomous.FieldTrajectoryPlanner;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FiringArm;
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
    // i will eat your children
    // i don't like Chase

    public enum AutoPos {
        REDBASKET (1, 1),
        REDWALL (1, -1),
        BLUEBASKET ( -1, -1),
        BLUEWALL (1, 1);
        public int xMult;
        public int yMult;
        AutoPos(int xMult, int yMult) {
            this.xMult = xMult;
            this.yMult = yMult;
        }
    }

    public IMU imu;
    public AutoPos autoPos;
    public MecanumDrive drive;
    public Drivetrain drivetrain;
    public Pose2d startingPos;
    public Intake intake;
    public Launcher launcher;
    public Carousel carousel;
    public FiringArm firingArm;
    public Camera leftCamera;
    public Camera rightCamera;
    public StereoCamera stereoCamera;
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AutoPos autoPos, boolean teleop) {
       // this.intake = new Intake(hardwareMap, telemetry);
        this.launcher = new Launcher(hardwareMap, telemetry);
        this.autoPos = autoPos;
        this.startingPos = new Pose2d(64*autoPos.xMult, 16* autoPos.yMult, Math.toRadians(0* autoPos.yMult));
       // this.drive = new MecanumDrive(hardwareMap, startingPos);
        this.carousel = new Carousel(hardwareMap, telemetry);
        this.firingArm = new FiringArm(hardwareMap, telemetry);
        //this.leftCamera = new Camera("WebcamLeft", hardwareMap,telemetry);
        //this.rightCamera = new Camera(  "WebcamRight", hardwareMap,telemetry);
        //this.stereoCamera = new StereoCamera(this.leftCamera, this.rightCamera, hardwareMap, telemetry);
    }


    public FieldTrajectoryPlanner createTrajectoryPlanner() {
        return new FieldTrajectoryPlanner(this);
    }
}