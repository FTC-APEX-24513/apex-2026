package org.firstinspires.ftc.teamcode.opmodes.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem

@Autonomous(name = "blueTop0", group = "Examples")
abstract class BlueTop0 : OpMode() {

    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer

    private var pathState = 0

    // Subsystems
    private lateinit var intake: IntakeSubsystem
    private lateinit var outtake: OuttakeSubsystem
    private lateinit var transfer: TransferSubsystem
    private lateinit var spindexer: SpindexerSubsystem
    private lateinit var limelight: LimelightSubsystem

    // Path
    private lateinit var scorePreload: Path

    // Poses
    private val startPoseBlueTop = Pose(56.0, 88.0, Math.toRadians(180.0))
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
    }

    private fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                follower.followPath(scorePreload)
                outtake.launch()
                setPathState(1)
            }
            1 -> if (!follower.isBusy) {
                pathState = 2
            }
            2 -> { /* Auto finished */ }
        }
    }

    private fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }
}
