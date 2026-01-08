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

@Autonomous(name = "blueTop1", group = "Examples")
abstract class BlueTop2 : OpMode() {

    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer

    private var pathState = 0

    // Subsystems
    private lateinit var intake: IntakeSubsystem
    private lateinit var outtake: OuttakeSubsystem
    private lateinit var transfer: TransferSubsystem
    private lateinit var spindexer: SpindexerSubsystem
    private lateinit var limelight: LimelightSubsystem

    // Paths
    private lateinit var scorePreload: Path
    private lateinit var blueScoreToTopStart: PathChain
    private lateinit var blueTopStartToTopEnd: PathChain
    private lateinit var blueTopEndToScore: PathChain

    // Poses
    private val startPoseBlueTop = Pose(56.0, 88.0, Math.toRadians(180.0))
    private val blueTopRowStart = Pose(40.0, 84.0, Math.toRadians(180.0))
    private val blueTopRowEnd = Pose(20.0, 84.0, Math.toRadians(180.0))
    private val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0))

    override fun init() {
        pathTimer = Timer()
        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        follower.setStartingPose(startPoseBlueTop)

        intake = IntakeSubsystem(hardwareMap)
        outtake = OuttakeSubsystem(hardwareMap)
        transfer = TransferSubsystem(hardwareMap)
        spindexer = SpindexerSubsystem(hardwareMap)
        limelight = LimelightSubsystem(hardwareMap)
    }

    override fun start() {
        setPathState(0)
    }

    override fun loop() {
        follower.update()
        autonomousPathUpdate()
    }

    private fun buildPaths() {
        scorePreload = Path(BezierLine(startPoseBlueTop, blueScore))
        scorePreload.setLinearHeadingInterpolation(startPoseBlueTop.heading, blueScore.heading)

        blueScoreToTopStart = follower.pathBuilder()
            .addPath(BezierLine(blueScore, blueTopRowStart))
            .setLinearHeadingInterpolation(blueScore.heading, blueTopRowStart.heading)
            .build()

        blueTopStartToTopEnd = follower.pathBuilder()
            .addPath(BezierLine(blueTopRowStart, blueTopRowEnd))
            .setLinearHeadingInterpolation(blueTopRowStart.heading, blueTopRowEnd.heading)
            .build()

        blueTopEndToScore = follower.pathBuilder()
            .addPath(BezierLine(blueTopRowEnd, blueScore))
            .setLinearHeadingInterpolation(blueTopRowEnd.heading, blueScore.heading)
            .build()
    }

    private fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                follower.followPath(scorePreload)
                outtake.launch()
                pathState = 1
            }
            1 -> if (!follower.isBusy) {
                follower.followPath(blueScoreToTopStart)
                pathState = 2
            }
            2 -> if (!follower.isBusy) {
                intake.collect()
                follower.followPath(blueTopStartToTopEnd)
                pathState = 3
            }
            3 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(blueTopEndToScore)
                outtake.launch()
                pathState = 4
            }
            4 -> if (!follower.isBusy) {
                pathState = 5
            }
            5 -> { /* Finished */ }
        }
    }

    private fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }
}
