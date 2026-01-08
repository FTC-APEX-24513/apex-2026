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
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem

@Autonomous(name = "blueBottom_0", group = "Examples")
class BlueBottom0 : OpMode() {

    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer

    private var pathState = 0
    private var actionState = 0

    private lateinit var intake: IntakeSubsystem
    private lateinit var transfer: TransferSubsystem
    private lateinit var outtake: OuttakeSubsystem
    private lateinit var spindexer: SpindexerSubsystem
    private lateinit var limelight: LimelightSubsystem

    private val startPoseBlueBottom = Pose(56.0, 16.0, Math.toRadians(180.0))
    private val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0))

    private lateinit var blueStartToScore: PathChain

    override fun init() {
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()

        follower = Constants.createFollower(hardwareMap)

        intake = IntakeSubsystem(hardwareMap)
        transfer = TransferSubsystem(hardwareMap)
        outtake = OuttakeSubsystem(hardwareMap)
        spindexer = SpindexerSubsystem(hardwareMap)
        limelight = LimelightSubsystem(hardwareMap)

        buildPaths()
        follower.setStartingPose(startPoseBlueBottom)
    }

    override fun start() {
        setPathState(0)
    }

    override fun loop() {
        follower.update()
        autonomousPathUpdate()
    }

    private fun buildPaths() {
        blueStartToScore = follower.pathBuilder()
            .addPath(Path(BezierLine(startPoseBlueBottom, blueScore)))
            .setLinearHeadingInterpolation(
                startPoseBlueBottom.heading,
                blueScore.heading
            )
            .build()
    }

    private fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                intake.collect()
                follower.followPath(blueStartToScore)
                setPathState(1)
            }
            1 -> if (!follower.isBusy) {
                intake.stop()
                limelight.useAprilTagPipeline()
                actionTimer.resetTimer()
                actionState = 0
                setPathState(2)
            }
            2 -> shootingSequence()
            3 -> { /* Finished */ }
        }
    }

    private fun shootingSequence() {
        when (actionState) {
            0 -> if (actionTimer.elapsedTimeSeconds > 0.5) { actionTimer.resetTimer(); actionState = 1 }
            1 -> if (actionTimer.elapsedTimeSeconds > 0.35) { transfer.transfer(); actionTimer.resetTimer(); actionState = 2 }
            2 -> if (actionTimer.elapsedTimeSeconds > 0.6) { outtake.launch(); actionTimer.resetTimer(); actionState = 3 }
            3 -> if (actionTimer.elapsedTimeSeconds > 0.3) { spindexer.switch(); actionTimer.resetTimer(); actionState = 4 }
            4 -> if (actionTimer.elapsedTimeSeconds > 0.35) { transfer.transfer(); actionTimer.resetTimer(); actionState = 5 }
            5 -> if (actionTimer.elapsedTimeSeconds > 0.6) { outtake.launch(); actionTimer.resetTimer(); actionState = 6 }
            6 -> if (actionTimer.elapsedTimeSeconds > 0.3) { spindexer.switch(); actionTimer.resetTimer(); actionState = 7 }
            7 -> if (actionTimer.elapsedTimeSeconds > 0.35) { transfer.transfer(); actionTimer.resetTimer(); actionState = 8 }
            8 -> if (actionTimer.elapsedTimeSeconds > 0.6) { outtake.launch(); setPathState(3) }
        }
    }

    private fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }
}
