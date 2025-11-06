package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

@Autonomous
public class MeetOneAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.AutoPos autoPos = Robot.AutoPos.BLUEWALL;
        while (!gamepad1.a) {
            if (gamepad1.dpad_down) {
                autoPos = Robot.AutoPos.REDWALL;
            } else if (gamepad1.dpad_up) {
                autoPos = Robot.AutoPos.BLUEWALL;
            }
            telemetry.addData("starting pos", autoPos);
            telemetry.update();
        }
        Robot robot = new Robot(hardwareMap, telemetry, autoPos, false);

        Action autoAction = robot.createTrajectoryPlanner()
                .stepToShot()
                .fireWholeMagazine()
                .returnToPark()
                .builder.build();

        waitForStart();

        Actions.runBlocking(autoAction);
    }
}
