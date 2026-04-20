package sigmacorns.io

import sigmacorns.math.Pose2d
import sigmacorns.vision.tracker.PixelDetection
import kotlin.time.Duration

interface SigmaIO {

    // drive motors
    var driveFL: Double
    var driveBL: Double
    var driveFR: Double
    var driveBR: Double

    var flywheel: Double
    //intake
    var intake: Double
    //turret motor power (legacy, used by tuning opmodes)
    var turret: Double
    // turret servos (dual-servo geared turret)
    var turretLeft: Double
    var turretRight: Double
    // hood servo position (controls launch angle)
    var hood: Double
    // blocker servo position (0.0 = engaged/blocking, 1.0 = disengaged/open)
    var blocker: Double

    // beam break sensors (true = beam broken = ball present)
    fun beamBreak1(): Boolean
    fun beamBreak2(): Boolean
    fun beamBreak3(): Boolean

    fun position(): Pose2d
    fun velocity(): Pose2d
    fun flywheelVelocity(): Double
    fun intake1RPM(): Double
    fun turretPosition(): Double
    fun setPosition(p: Pose2d)
    fun time(): Duration
    fun configurePinpoint()
    fun voltage(): Double
    fun update()

    /**
     * Ball detections visible this tick. Default: empty (no vision pipeline).
     * JoltSimIO returns detections from the SimulatedCamera; HardwareIO will
     * return detections from the Limelight pipeline once the hardware side
     * is wired. Implementations should attach the CAPTURE timestamp to each
     * detection so the tracker can pose-interpolate correctly.
     */
    fun getBallDetections(t: Double, robotPose: Pose2d): List<PixelDetection> = emptyList()
}
