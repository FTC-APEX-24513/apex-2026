package edu.exeter.apex.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.Constants;

@Autonomous(name = "redBottom_3", group = "Examples")
public class redBottom_3 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPoseRedBottom = new Pose(88, 16, Math.toRadians(180)); //Aligned with left right of robot on Y1
    private final Pose redTopRowStart = new Pose(104, 84, Math.toRadians(180));
    private final Pose redTopRowEnd = new Pose(124, 84, Math.toRadians(180));
    private final Pose redMiddleRowStart = new Pose(104, 60, Math.toRadians(180));
    private final Pose redMiddleRowEnd = new Pose(124, 60, Math.toRadians(180));
    private final Pose redBottomRowStart = new Pose(104, 36, Math.toRadians(180));
    private final Pose redBottomRowEnd = new Pose(124, 36, Math.toRadians(180));
    private final Pose redScore = new Pose();
    private PathChain redStartToScore, redScoreToTop, redTopIntake, redTopToScore, redScoreToMiddle, redMiddleIntake, redMiddleToScore, redScoreToBottom, redBottomIntake, redBottomToScore;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPoseRedBottom);
    }

    @Override
    public void start(){
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void buildPaths() {

        redStartToScore = follower.pathBuilder()
                .addPath(new Path(new BezierLine(startPoseRedBottom, redScore)))
                .setLinearHeadingInterpolation(startPoseRedBottom.getHeading(), redScore.getHeading())
                .build();

        //Limelight localization
        //Shoot here

        redScoreToTop = follower.pathBuilder()
                .addPath(new Path(new BezierLine(redScore, redTopRowStart)))
                .setLinearHeadingInterpolation(redScore.getHeading(), redTopRowStart.getHeading())
                .build();

        redTopIntake = follower.pathBuilder()
                .addPath(new Path(new BezierLine(redTopRowStart, redTopRowEnd)))
                .setLinearHeadingInterpolation(redTopRowStart.getHeading(), redTopRowEnd.getHeading())
                .build();

        redTopToScore = follower.pathBuilder()
                .addPath(new Path(new BezierLine(redTopRowEnd, redScore)))
                .setLinearHeadingInterpolation(redTopRowEnd.getHeading(), redScore.getHeading())
                .build();

        //Limelight localization
        //Shoot here

        redScoreToMiddle = follower.pathBuilder()
                .addPath(new Path(new BezierLine(redScore, redMiddleRowStart)))
                .setLinearHeadingInterpolation(redScore.getHeading(), redMiddleRowStart.getHeading())
                .build();

        redMiddleIntake = follower.pathBuilder()
                .addPath(new Path(new BezierLine(redMiddleRowStart, redMiddleRowEnd)))
                .setLinearHeadingInterpolation(redMiddleRowStart.getHeading(), redMiddleRowEnd.getHeading())
                .build();

        redMiddleToScore = follower.pathBuilder()
                .addPath(new Path(new BezierLine(redMiddleRowEnd, redScore)))
                .setLinearHeadingInterpolation(redMiddleRowEnd.getHeading(), redScore.getHeading())
                .build();

        //Limelight localization
        //Shoot here

        redScoreToBottom = follower.pathBuilder()
                .addPath(new Path(new BezierLine(redScore, redBottomRowStart)))
                .setLinearHeadingInterpolation(redScore.getHeading(), redBottomRowStart.getHeading())
                .build();

        redBottomIntake = follower.pathBuilder()
                .addPath(new Path(new BezierLine(redBottomRowStart, redBottomRowEnd)))
                .setLinearHeadingInterpolation(redBottomRowStart.getHeading(), redBottomRowEnd.getHeading())
                .build();

        redBottomToScore = follower.pathBuilder()
                .addPath(new Path(new BezierLine(redBottomRowEnd, redScore)))
                .setLinearHeadingInterpolation(redBottomRowEnd.getHeading(), redScore.getHeading())
                .build();

        //Limelight localization
        //Shoot here

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start → first scoring position
                follower.followPath(redStartToScore);
                setPathState(1);
                break;

            case 1:
                // Wait until start-to-score path is done
                if (!follower.isBusy()) {
                    follower.followPath(redScoreToTop);
                    setPathState(2);
                }
                break;

            case 2:
                // Move to top row for intake
                if (!follower.isBusy()) {
                    follower.followPath(redTopIntake);
                    setPathState(3);
                }
                break;

            case 3:
                // After finishing top intake path
                if (!follower.isBusy()) {
                    follower.followPath(redTopToScore);
                    setPathState(4);
                }
                break;

            case 4:
                // Return from top row to scoring
                if (!follower.isBusy()) {
                    follower.followPath(redScoreToMiddle);
                    setPathState(5);
                }
                break;

            case 5:
                // Move to middle row for intake
                if (!follower.isBusy()) {
                    follower.followPath(redMiddleIntake);
                    setPathState(6);
                }
                break;

            case 6:
                // Finish middle intake
                if (!follower.isBusy()) {
                    follower.followPath(redMiddleToScore);
                    setPathState(7);
                }
                break;

            case 7:
                // Return to score after middle row
                if (!follower.isBusy()) {
                    follower.followPath(redScoreToBottom);
                    setPathState(8);
                }
                break;

            case 8:
                // Move to bottom row for intake
                if (!follower.isBusy()) {
                    follower.followPath(redBottomIntake);
                    setPathState(9);
                }
                break;

            case 9:
                // Finish bottom intake
                if (!follower.isBusy()) {
                    follower.followPath(redBottomToScore);
                    setPathState(10);
                }
                break;

            case 10:
                // Final score or park
                if (!follower.isBusy()) {
                    setPathState(11); // End of routine
                }
                break;

            case 11:
                // Autonomous routine finished
                break;
        }
    }
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }
}