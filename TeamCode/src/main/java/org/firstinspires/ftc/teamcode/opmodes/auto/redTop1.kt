package org.firstinspires.ftc.teamcode.opmodes.auto

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
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem

@Autonomous(name = "redTop1", group = "Examples")
class RedBottom1OpMode : OpMode() {

    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private var pathState = 0

    private val startPoseRedTop =
        Pose(88.0, 88.0, Math.toRadians(180.0)) //Aligned with top left of robot on Y4
    private val startPoseRedBottom =
        Pose(88.0, 16.0, Math.toRadians(180.0)) //Aligned with left right of robot on Y1
    private val redTopRowStart = Pose(104.0, 84.0, Math.toRadians(90.0))
    private val redTopRowEnd = Pose(124.0, 84.0, Math.toRadians(90.0))
    private val redMiddleRowStart = Pose(104.0, 60.0, Math.toRadians(180.0))
    private val redMiddleRowEnd = Pose(124.0, 60.0, Math.toRadians(180.0))
    private val redBottomRowStart = Pose(104.0, 36.0, Math.toRadians(180.0))
    private val redBottomRowEnd = Pose(124.0, 36.0, Math.toRadians(180.0))
    private val redScore = Pose()

    private lateinit var scorePreload: Path
    private lateinit var redScoreToTop: PathChain
    private lateinit var redTopIntake: PathChain
    private lateinit var redTopToScore: PathChain

    private lateinit var intake: IntakeSubsystem
    private lateinit var outtake: OuttakeSubsystem
    private lateinit var limelight: LimelightSubsystem
    private lateinit var transfer: TransferSubsystem
    private lateinit var spindexer: SpindexerSubsystem

    override fun init() {
        pathTimer = Timer()
        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        follower.setStartingPose(startPoseRedTop)

        intake = IntakeSubsystem(hardwareMap)
        outtake = OuttakeSubsystem(hardwareMap)
        limelight = LimelightSubsystem()
        limelight.init(hardwareMap, telemetry)
        transfer = TransferSubsystem(hardwareMap)
        spindexer = SpindexerSubsystem(hardwareMap)
    }

    private fun buildPaths() {
        scorePreload = Path(BezierLine(startPoseRedTop, redScore)).apply {
            setLinearHeadingInterpolation(startPoseRedTop.heading, redScore.heading)
        }

        redScoreToTop = follower.pathBuilder()
            .addPath(BezierLine(redScore, redTopRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redTopRowStart.heading)
            .build()

        redTopIntake = follower.pathBuilder()
            .addPath(BezierLine(redTopRowStart, redTopRowEnd))
            .setLinearHeadingInterpolation(redTopRowStart.heading, redTopRowEnd.heading)
            .build()

        redTopIntake = follower.pathBuilder()
            .addPath(BezierLine(redTopRowEnd, redScore))
            .setLinearHeadingInterpolation(redTopRowEnd.heading, redScore.heading)
            .build()
    }

    override fun start() {
        pathState = 0
    }

    override fun loop() {
        follower.update()
        when (pathState) {
            0 -> {
                follower.followPath(scorePreload)
                outtake.outtake()
                pathState = 1
            }
            1 -> if (!follower.isBusy) {
                follower.followPath(redScoreToTop)
                pathState = 2
            }
            2 -> if (!follower.isBusy) {
                intake.intake()
                follower.followPath(redTopIntake)
                pathState = 3
            }
            3 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(redTopToScore)
                outtake.outtake()
                pathState = 4
            }
            4 -> if (!follower.isBusy) pathState = 5
            5 -> { /* Finished */ }
        }
    }
}
