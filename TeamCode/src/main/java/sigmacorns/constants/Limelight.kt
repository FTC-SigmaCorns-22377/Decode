package sigmacorns.constants

object Limelight {
    val APRILTAG_PIPELINE = 0

    /**
     * Color-detector pipeline dedicated to DECODE artifacts (balls). Configure
     * the Limelight web UI so pipeline index [BALL_PIPELINE] runs a color
     * detector tuned for the artifact's HSV range and outputs `detectorResults`
     * with per-target `targetXPixels` / `targetYPixels` populated.
     *
     * The opmode that wants ball tracking must call
     *   `limelight.pipelineSwitch(Limelight.BALL_PIPELINE)`
     * before asking the tracker for detections — AprilTag localization and
     * ball tracking cannot run in the same pipeline.
     */
    val BALL_PIPELINE = 1

    val IDLE_PIPELINE = 3
}