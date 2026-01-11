package sigmacorns.control.aim

import android.util.Log
import java.util.concurrent.atomic.AtomicLong

enum class LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR
}

interface Logger {
    fun log(level: LogLevel, message: String)
}

class LogcatLogger(private val tag: String) : Logger {
    override fun log(level: LogLevel, message: String) {
        when (level) {
            LogLevel.DEBUG -> Log.d(tag, message)
            LogLevel.INFO -> Log.i(tag, message)
            LogLevel.WARN -> Log.w(tag, message)
            LogLevel.ERROR -> Log.e(tag, message)
        }
    }
}

class ThrottledLogger(
    private val base: Logger,
    private val minIntervalMs: Long = 1000L
) : Logger {
    private val lastLogMs = AtomicLong(0L)

    override fun log(level: LogLevel, message: String) {
        val now = System.currentTimeMillis()
        val elapsed = now - lastLogMs.get()
        if (level.ordinal >= LogLevel.WARN.ordinal || elapsed >= minIntervalMs) {
            lastLogMs.set(now)
            base.log(level, message)
        }
    }
}
