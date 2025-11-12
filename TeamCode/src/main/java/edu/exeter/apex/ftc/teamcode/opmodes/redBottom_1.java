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

@Autonomous(name = "redBottom_1", group = "Examples")
public class redBottom_1 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    //Aligned with top right of robot on W1
    private final Pose redBottomRowStart = new Pose(104, 36, Math.toRadians(180));
    private final Pose redBottomRowEnd = new Pose(124, 36, Math.toRadians(180));
    private final Pose redScore = new Pose();
    private final Pose startPoseRedBottom = new Pose(88, 16, Math.toRadians(180)); //Aligned with left right of robot on Y1

    private PathChain redStartToScore, redScoreToBottom, redBottomIntake, redBottomToScore;

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
                // Return to score after middle row
                if (!follower.isBusy()) {
                    follower.followPath(redScoreToBottom);
                    setPathState(2);
                }
                break;

            case 2:
                // Move to bottom row for intake
                if (!follower.isBusy()) {
                    follower.followPath(redBottomIntake);
                    setPathState(3);
                }
                break;

            case 3:
                // Finish bottom intake
                if (!follower.isBusy()) {
                    follower.followPath(redBottomToScore);
                    setPathState(4);
                }
                break;

            case 4:
                // Final score or park
                if (!follower.isBusy()) {
                    setPathState(5); // End of routine
                }
                break;

            case 5:
                // Autonomous routine finished
                break;
        }
    }
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }
}