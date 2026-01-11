package sigmacorns.control.aim

data class AimConfig(
    var cameraMountingOffsetYaw: Double = 0.0,
    var txSignMultiplier: Double = -1.0,
    var txDeadband: Double = 0.01,
    var maxAcceptableUncertainty: Double = Double.MAX_VALUE,
    var predictionTimeoutMs: Long = 2000L,
    var visionPixelSigma: Double = 1.0,
    var maxPoseDivergence: Double = 2.0,
    var turretRotationUncertaintyGain: Double = 1.5,
    var uncertaintyDecayRate: Double = 0.01
)
