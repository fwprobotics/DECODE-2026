package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.subsystems.Arm;
//import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.TeleopActionRunner;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.StereoCamera;

import java.util.Arrays;
//import org.firstinspires.ftc.teamcode.subsystems.Lift;
//import org.firstinspires.ftc.teamcode.subsystems.Wrist;
//import org.firstinspires.ftc.teamcode.util.TeleopActionRunner;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this, hardwareMap, telemetry);
        Launcher launcher = new Launcher(hardwareMap, telemetry);
        HuskyLens leftcamera = hardwareMap.get(HuskyLens.class, "WebcamLeft");

        TeleopActionRunner actionRunner = new TeleopActionRunner();
//        Camera leftcamera = new Camera("WebcamLeft", hardwareMap, telemetry);
        Camera rightcamera = new Camera("WebcamRight", hardwareMap, telemetry);
//        StereoCamera stereoCamera = new StereoCamera(leftcamera, rightcamera, hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        boolean firing = false;
        byte firing_pattern = 0;
        waitForStart();
        while (!isStopRequested()) {
            drivetrain.joystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.right_bumper, false, gamepad1.left_bumper);
            if (gamepad1.a) {
                double [] data = launcher.FireAtY(36,72);
                telemetry.addData("Launch Motor Power", data[0]);
                telemetry.addData("Target Velocity", (data[1]));
            }
            if (gamepad1.b) {
                launcher.reset();
            }
            if (gamepad1.y) {
                intake.runIntake();
            }
            if (gamepad1.dpad_down) {
//                telemetry.addData("X DISTANCE", leftcamera.find_april_tag());
                HuskyLens.Block[] blocks = rightcamera.webcam.blocks();
                telemetry.addData("RIGHT BLOCKS LENGTH:", (blocks.length));
                telemetry.addData("RIGHT BLOCKS :", (Arrays.toString(blocks)));

                HuskyLens.Block[] newblocks = leftcamera.blocks();
                telemetry.addData("LEFT BLOCKS LENGTH:", (newblocks.length));
                telemetry.addData("BLOCKS:", Arrays.toString(newblocks));
            }


            telemetry.update();
        }
    }
}