package org.firstinspires.ftc.teamcode.opmodes.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.util.Timer
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem

@Suppress("UNUSED")
val blueTop0 = Mercurial.autonomous {

    lateinit var follower: Follower
    lateinit var pathTimer: Timer

    var pathState = 0

    lateinit var intake: IntakeSubsystem
    lateinit var outtake: OuttakeSubsystem
    lateinit var transfer: TransferSubsystem
    lateinit var spindexer: SpindexerSubsystem
    lateinit var limelight: LimelightSubsystem

    lateinit var scorePreload: Path

    val startPoseBlueTop = Pose(56.0, 88.0, Math.toRadians(180.0))
    val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0))

    fun buildPaths() {
        scorePreload = Path(BezierLine(startPoseBlueTop, blueScore))
        scorePreload.setLinearHeadingInterpolation(
            startPoseBlueTop.heading,
            blueScore.heading
        )
    }

    fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }

    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                follower.followPath(scorePreload)
                outtake.launch()
                setPathState(1)
            }
            1 -> if (!follower.isBusy) {
                pathState = 2
            }
            2 -> { }
        }
    }

    sequence(
        exec {
            pathTimer = Timer()
            follower = Constants.createFollower(hardwareMap)

            buildPaths()
            follower.setStartingPose(startPoseBlueTop)

            intake = IntakeSubsystem(hardwareMap)
            outtake = OuttakeSubsystem(hardwareMap)
            transfer = TransferSubsystem(hardwareMap)
            spindexer = SpindexerSubsystem(hardwareMap)
            limelight = LimelightSubsystem(hardwareMap)
        },
        wait { inLoop },
        exec { setPathState(0) },
        loop(exec {
            follower.update()
            autonomousPathUpdate()
        })
    )

    dropToScheduler()
}
