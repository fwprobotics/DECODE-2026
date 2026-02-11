package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.FiringArm;


public class FieldTrajectoryPlanner {

    public TrajectoryActionBuilder builder;
    Robot robot;

    public FieldTrajectoryPlanner(Robot robot) {
        this.builder = robot.drive.actionBuilder(robot.startingPos);
        this.robot = robot;

    };
    public FieldTrajectoryPlanner stepToShot() {
        builder = builder
                .strafeToLinearHeading(
                        new Vector2d(-27*robot.autoPos.xMult,
                                     16*robot.autoPos.yMult),
                        Math.toRadians(175*robot.autoPos.yMult));
        return this;
    };


    public FieldTrajectoryPlanner lineWithBackWall() {
        builder = builder
                .strafeToLinearHeading(
                        new Vector2d(12*robot.autoPos.xMult,
                                     63*robot.autoPos.yMult),
                        Math.toRadians(90*robot.autoPos.yMult));
        return this;
    };

    public FieldTrajectoryPlanner returnToPark() {
        builder = builder
                .strafeToLinearHeading(
                        new Vector2d(18*robot.autoPos.xMult,
                                     42*robot.autoPos.yMult),
                        Math.toRadians(-90*robot.autoPos.yMult)
                );
        return this;
    };

    public FieldTrajectoryPlanner fireWholeMagazineInSequence(int sequence) {
        builder = builder.afterTime(.1, new SequentialAction())
                .stopAndAdd(FiringInSeqenceAction(sequence));
        return this;
    };
    public FieldTrajectoryPlanner fireAtBasket() {
        builder = builder.afterTime(.1, new SequentialAction())
                .stopAndAdd(FireAtBasketAction());
        return this;
    };

    public Action FiringInSeqenceAction(int sequence) {
        if (sequence == 1){
        return new SequentialAction(
                robot.carousel.FireGreenAction(),
                FireAtBasketAction(),
                robot.carousel.BallFiredAction(),
                new SleepAction(.5),
                robot.carousel.FirePurpleAction(),
                FireAtBasketAction(),
                robot.carousel.BallFiredAction(),
                new SleepAction(.5),
                robot.carousel.FirePurpleAction(),
                FireAtBasketAction(),
                robot.carousel.BallFiredAction()
                );}
        if (sequence == 2){
            return new SequentialAction(
                    robot.carousel.FirePurpleAction(),
                    FireAtBasketAction(),
                    new SleepAction(.5),
                    robot.carousel.FireGreenAction(),
                    FireAtBasketAction(),
                    new SleepAction(.5),
                    robot.carousel.FirePurpleAction(),
                    FireAtBasketAction()
            );}
        if (sequence == 3){
            return new SequentialAction(
                    robot.carousel.FirePurpleAction(),
                    FireAtBasketAction(),
                    new SleepAction(.5),
                    robot.carousel.FirePurpleAction(),
                    FireAtBasketAction(),
                    new SleepAction(.5),
                    robot.carousel.FireGreenAction(),
                    FireAtBasketAction()
            );}

        return telemetryPacket -> false;
    };
    public Action FireAtBasketAction() {
        return new SequentialAction(
                robot.launcher.FireAtYAction(32, this.robot.stereoCamera.compute_x_distance() ),
                new SleepAction(2),
                robot.firingArm.setFiringStateAction(FiringArm.FiringState.firing),
                new SleepAction(.2),
                robot.firingArm.setFiringStateAction(FiringArm.FiringState.waiting),
                robot.launcher.reset()
        );
    };

    public FieldTrajectoryPlanner determineThenFire() {
        builder = builder.afterTime(.1, new SequentialAction())
                .stopAndAdd(FiringInSeqenceAction(robot.stereoCamera.getSequence() % 10));
        return this;
    }

}