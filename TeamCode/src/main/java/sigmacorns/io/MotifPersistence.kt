package sigmacorns.io

import java.io.File
import kotlin.time.Duration.Companion.seconds

object MotifPersistence {
    private const val MOTIF_FILE_NAME = "last_auto_motif.txt"
    private val MAX_AGE = 120.seconds

    fun saveMotif(motifId: Int, storageDir: File) {
        try {
            val file = File(storageDir, MOTIF_FILE_NAME)
            val timestamp = System.currentTimeMillis()
            file.writeText("$timestamp,$motifId")
            println("MotifPersistence: Saved motif $motifId to ${file.absolutePath}")
        } catch (e: Exception) {
            println("MotifPersistence: Failed to save motif: ${e.message}")
        }
    }

    fun loadMotif(storageDir: File): Int? {
        try {
            val file = File(storageDir, MOTIF_FILE_NAME)
            if (!file.exists()) {
                println("MotifPersistence: No saved motif file found")
                return null
            }

            val parts = file.readText().trim().split(",")
            if (parts.size != 2) {
                println("MotifPersistence: Invalid motif file format")
                return null
            }

            val timestamp = parts[0].toLongOrNull() ?: return null
            val motif = parts[1].toIntOrNull() ?: return null

            val ageMs = System.currentTimeMillis() - timestamp
            if (ageMs > MAX_AGE.inWholeMilliseconds) {
                println("MotifPersistence: Motif too old (${ageMs}ms)")
                return null
            }

            println("MotifPersistence: Loaded motif $motif (age: ${ageMs}ms)")
            return motif
        } catch (e: Exception) {
            println("MotifPersistence: Failed to load motif: ${e.message}")
            return null
        }
    }
}
