package edu.exeter.apex.ftc.teamcode.opmodes

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem

@Autonomous(name = "redBottom_3", group = "Examples")
abstract class RedBottom3 : OpMode() {

    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer

    private var pathState = 0

    // Subsystems
    private lateinit var intake: IntakeSubsystem
    private lateinit var outtake: OuttakeSubsystem
    private lateinit var transfer: TransferSubsystem
    private lateinit var spindexer: SpindexerSubsystem
    private lateinit var limelight: LimelightSubsystem

    // Paths
    private lateinit var redStartToScore: Path
    private lateinit var redScoreToTop: PathChain
    private lateinit var redTopIntake: PathChain
    private lateinit var redTopToScore: PathChain
    private lateinit var redScoreToMiddle: PathChain
    private lateinit var redMiddleIntake: PathChain
    private lateinit var redMiddleToScore: PathChain
    private lateinit var redScoreToBottom: PathChain
    private lateinit var redBottomIntake: PathChain
    private lateinit var redBottomToScore: PathChain

    // Poses
    private val startPoseRedBottom = Pose(88.0, 16.0, Math.toRadians(180.0))
    private val redTopRowStart = Pose(104.0, 84.0, Math.toRadians(180.0))
    private val redTopRowEnd = Pose(124.0, 84.0, Math.toRadians(180.0))
    private val redMiddleRowStart = Pose(104.0, 60.0, Math.toRadians(180.0))
    private val redMiddleRowEnd = Pose(124.0, 60.0, Math.toRadians(180.0))
    private val redBottomRowStart = Pose(104.0, 36.0, Math.toRadians(180.0))
    private val redBottomRowEnd = Pose(124.0, 36.0, Math.toRadians(180.0))
    private val redScore = Pose(112.0, 48.0, Math.toRadians(180.0)) // tune after testing

    override fun init() {
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()

        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        follower.setStartingPose(startPoseRedBottom)

        intake = IntakeSubsystem(hardwareMap)
        outtake = OuttakeSubsystem(hardwareMap)
        transfer = TransferSubsystem(hardwareMap)
        spindexer = SpindexerSubsystem(hardwareMap)
        limelight = LimelightSubsystem(hardwareMap)
    }

    override fun start() {
        opmodeTimer.resetTimer()
        setPathState(0)
    }

    override fun loop() {
        follower.update()
        autonomousPathUpdate()

        telemetry.addData("Path State", pathState)
        telemetry.addData("X", follower.pose.x)
        telemetry.addData("Y", follower.pose.y)
        telemetry.addData("Heading", follower.pose.heading)
        telemetry.update()
    }

    private fun buildPaths() {
        redStartToScore = Path(BezierLine(startPoseRedBottom, redScore))
        redStartToScore.setLinearHeadingInterpolation(startPoseRedBottom.heading, redScore.heading)

        redScoreToTop = follower.pathBuilder()
            .addPath(BezierLine(redScore, redTopRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redTopRowStart.heading)
            .build()

        redTopIntake = follower.pathBuilder()
            .addPath(BezierLine(redTopRowStart, redTopRowEnd))
            .setLinearHeadingInterpolation(redTopRowStart.heading, redTopRowEnd.heading)
            .build()

        redTopToScore = follower.pathBuilder()
            .addPath(BezierLine(redTopRowEnd, redScore))
            .setLinearHeadingInterpolation(redTopRowEnd.heading, redScore.heading)
            .build()

        redScoreToMiddle = follower.pathBuilder()
            .addPath(BezierLine(redScore, redMiddleRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redMiddleRowStart.heading)
            .build()

        redMiddleIntake = follower.pathBuilder()
            .addPath(BezierLine(redMiddleRowStart, redMiddleRowEnd))
            .setLinearHeadingInterpolation(redMiddleRowStart.heading, redMiddleRowEnd.heading)
            .build()

        redMiddleToScore = follower.pathBuilder()
            .addPath(BezierLine(redMiddleRowEnd, redScore))
            .setLinearHeadingInterpolation(redMiddleRowEnd.heading, redScore.heading)
            .build()

        redScoreToBottom = follower.pathBuilder()
            .addPath(BezierLine(redScore, redBottomRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redBottomRowStart.heading)
            .build()

        redBottomIntake = follower.pathBuilder()
            .addPath(BezierLine(redBottomRowStart, redBottomRowEnd))
            .setLinearHeadingInterpolation(redBottomRowStart.heading, redBottomRowEnd.heading)
            .build()

        redBottomToScore = follower.pathBuilder()
            .addPath(BezierLine(redBottomRowEnd, redScore))
            .setLinearHeadingInterpolation(redBottomRowEnd.heading, redScore.heading)
            .build()
    }

    private fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                follower.followPath(redStartToScore)
                limelight.update()
                limelight.localize()
                outtake.launch()
                setPathState(1)
            }
            1 -> if (!follower.isBusy) {
                follower.followPath(redScoreToTop)
                setPathState(2)
            }
            2 -> if (!follower.isBusy) {
                intake.collect()
                follower.followPath(redTopIntake)
                setPathState(3)
            }
            3 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(redTopToScore)
                limelight.update()
                limelight.localize()
                outtake.launch()
                setPathState(4)
            }
            4 -> if (!follower.isBusy) {
                follower.followPath(redScoreToMiddle)
                setPathState(5)
            }
            5 -> if (!follower.isBusy) {
                intake.collect()
                follower.followPath(redMiddleIntake)
                setPathState(6)
            }
            6 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(redMiddleToScore)
                limelight.update()
                limelight.localize()
                outtake.launch()
                setPathState(7)
            }
            7 -> if (!follower.isBusy) {
                follower.followPath(redScoreToBottom)
                setPathState(8)
            }
            8 -> if (!follower.isBusy) {
                intake.collect()
                follower.followPath(redBottomIntake)
                setPathState(9)
            }
            9 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(redBottomToScore)
                limelight.update()
                limelight.localize()
                outtake.launch()
                setPathState(10)
            }
            10 -> if (!follower.isBusy) {
                setPathState(11)
            }
            11 -> { /* Auto finished */ }
        }
    }

    private fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }
}
