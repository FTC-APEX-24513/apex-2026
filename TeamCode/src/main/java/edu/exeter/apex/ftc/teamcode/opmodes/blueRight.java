package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.claw;
import org.firstinspires.ftc.teamcode.subsystems.hslides;
import org.firstinspires.ftc.teamcode.subsystems.slides;

import org.firstinspires.ftc.teamcode.OpModes.Auto.AConstants.BlueRight;
import org.firstinspires.ftc.teamcode.OpModes.Auto.AConstants.BLUE_RIGHT;
import org.firstinspires.ftc.teamcode.utility.constants;

import java.util.Optional;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Config
@Autonomous(name = "BlueRight", group = "Qual-1")
public class blueRight extends OpMode{

    private long timeout = 100;

    public static int s = 0;

    //Pedro Pathing Business
    private Follower follower;
    private Timer pathTimer = new Timer();
    private Timer actionTimer = new Timer();
    private Timer opModeTimer = new Timer();

    private BLUE_RIGHT state;

    private PathChain navigateToPreload, scorePreload, pushSample1, pushSample2, navigateToPickup, navigateToScore, score, park;

    //All Subsystems
    private slides slides;
    private claw claw;
    private hslides hslides;
    private arm arm;

    private String alliance = "BLUE";

    public void buildPaths() {
        navigateToPreload = follower.pathBuilder()
            .addPath(new Path(new BezierCurve(BlueRight.startPoint, BlueRight.navigateToPreloadControl1, BlueRight.navigateToPreload)))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

        scorePreload = follower.pathBuilder()
                .addPath(new Path(new BezierLine(BlueRight.navigateToPreload, BlueRight.scorePreload)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pushSample1 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(BlueRight.scorePreload, BlueRight.navigateToSample1Control1, BlueRight.NavigateToSample1Control2,BlueRight.navigateToSample1)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
//                .addPath(new Path(new BezierLine(BlueRight.navigateToSample1, BlueRight.pushSample1)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();

        pushSample2 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(BlueRight.pushSample1, BlueRight.navigateToSample2Control1, BlueRight.navigateToSample2)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new Path(new BezierLine(BlueRight.navigateToSample2, BlueRight.pushSample2)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        navigateToPickup = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(BlueRight.scorePreload, BlueRight.navigateToPickupControl1, BlueRight.navigateToPickup)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        navigateToScore = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(BlueRight.navigateToPickup, BlueRight.navigateToScoreControl1, BlueRight.navigateToScore)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(0.75)
                .setPathEndVelocityConstraint(0)
                .build();

        score = follower.pathBuilder()
                .addPath(new Path(new BezierLine(BlueRight.navigateToScore, BlueRight.score)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        park = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(BlueRight.score, BlueRight.parkControl1, BlueRight.park)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }


    public void pathUpdater() {
        switch (state) {
            case INIT:
                claw.preset(constants.CLAW.CLOSED);
                claw.preset(constants.CLAW.ROTATED);
                break;

            case START:
                state = BLUE_RIGHT.PRELOAD;
                claw.preset(constants.CLAW.ROTATED);
                slides.setHeight(1300);
                arm.preset(constants.ARM.BACK);
                break;

            case PRELOAD:
                if (!follower.isBusy()) {
                    follower.followPath(navigateToPreload, true);
                    setNextPath(BLUE_RIGHT.PRE_SCORE);
                }
                break;

            case PRE_SCORE:
                if (!follower.isBusy()) {
                    slides.setHeight(1780);
                    setNextPath(BLUE_RIGHT.SCORE_PRELOAD);
                }
                break;

            case SCORE_PRELOAD:
                if (!follower.isBusy() && !slides.isBusy()) {
                    follower.followPath(scorePreload, true);
                    setNextPath(BLUE_RIGHT.PICK_1);
                }
                break;
//
//            case PUSH_1:
//                if (!follower.isBusy()) {
//                    follower.followPath(pushSample1, true);
//                    setNextPath(BLUE_RIGHT.PUSH_2);
//                }
//                break;
//
//            case PUSH_2:
//                if (!follower.isBusy()) {
//                    follower.followPath(pushSample2, true);
//                    setNextPath(BLUE_RIGHT.PICK_1);
//                }
//                break;

            case PICK_1:
                if (!follower.isBusy() && !slides.isBusy()) {
                    claw.preset(constants.CLAW.OPEN);
                    follower.followPath(navigateToPickup, true);
                    slides.setHeight(1300);
                    arm.preset(constants.ARM.PRE_PICKUP);
                    setNextPath(BLUE_RIGHT.RESET_TIMER);
                }
                break;

            case RESET_TIMER:
                 actionTimer.resetTimer();
                if (!follower.isBusy()) {
                    setNextPath(BLUE_RIGHT.UNLATCH);
                }
                break;

            case UNLATCH:
                if (!follower.isBusy()){
                    slides.setHeight(1180);
                    if (actionTimer.getElapsedTimeSeconds() > 4.5) {
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 6.5) {
                        claw.preset(constants.CLAW.CLOSED);
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 10.5) {
                        hslides.unlatch();
                        arm.preset(constants.ARM.PRE_PICKUP);
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 12.5) {
                        setNextPath(BLUE_RIGHT.READY);
                    }

                }
                break;

            case READY:
                if (!follower.isBusy() && !slides.isBusy()) {
                    arm.preset(constants.ARM.BACK);
                    follower.followPath(navigateToScore, true);
                    hslides.preset(constants.HSLIDES.RETRACTED);
                    slides.setHeight(1300);
                    setNextPath(BLUE_RIGHT.PRE);
                }
                break;

            case PRE:
                if (!follower.isBusy() && !slides.isBusy()) {
                    slides.setHeight(1780);
                    setNextPath(BLUE_RIGHT.SCORE);
                }
                break;

            case SCORE:
                if (!follower.isBusy() && !slides.isBusy()) {
                    follower.followPath(score, true);
                    setNextPath(BLUE_RIGHT.PARK);
                }
                break;

            case PARK:
                if (!follower.isBusy() && !slides.isBusy()) {
                    claw.preset(constants.CLAW.OPEN);
                    slides.preset(constants.VSLIDES.BASE);
                    follower.followPath(park, true);
                    setNextPath(BLUE_RIGHT.IDLE);
                }
                break;
        }
    }

    public void setNextPath(BLUE_RIGHT next) {
        state = next;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(BlueRight.startPose);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = new arm();
        claw = new claw();
        slides = new slides();
        hslides = new hslides();

        slides.init(hardwareMap);
        claw.init(hardwareMap);
        hslides.init(hardwareMap);
        arm.init(hardwareMap);

        telemetry.addLine("blueRight Autonomous Initiated... Waiting For Start...");
        telemetry.addData("blueRight State: ", state);
        telemetry.update();

        buildPaths();

        setNextPath(BLUE_RIGHT.INIT);
        pathUpdater();
    }

    @Override
    public void init_loop() {
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setNextPath(BLUE_RIGHT.START);
    }

    @Override
    public void loop() {
        follower.update();
        pathUpdater();
        infoUpdater();
    }

    @Override
    public void stop() {};

    private void infoUpdater() {
        telemetry.addData("Path State: ", state);
        telemetry.addData("Position: ", follower.getPose().toString());
        telemetry.addLine(claw.getClawState());
        telemetry.addData("Slide Height: ", slides.getHeight());
        telemetry.addData("Action Timer: ", actionTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

}
