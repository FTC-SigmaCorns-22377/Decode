package sigmacorns.control.aim.tune

import com.google.gson.Gson
import com.google.gson.GsonBuilder
import com.google.gson.reflect.TypeToken
import java.io.File

/**
 * Persists bisection tuner state: grid cell brackets + all shot samples.
 * Two JSON files so sample log is append-only-friendly and cell state can be
 * overwritten atomically.
 */
class BisectionStore(
    private val cellFile: String = "/sdcard/FIRST/omega_bisect_cells.json",
    private val sampleFile: String = "/sdcard/FIRST/omega_bisect_samples.json"
) {
    private val gson: Gson = GsonBuilder().setPrettyPrinting().create()
    private val cells = mutableMapOf<String, CellState>()
    private val samples = mutableListOf<BisectionSample>()

    @Synchronized fun getCells(): List<CellState> = cells.values.sortedWith(
        compareBy({ it.targetDistance }, { it.targetHoodDeg })
    )

    @Synchronized fun getCell(id: String): CellState? = cells[id]

    @Synchronized fun putCell(c: CellState) { cells[c.cellId] = c }

    @Synchronized fun getSamples(): List<BisectionSample> = samples.toList()

    @Synchronized fun addSample(s: BisectionSample) { samples.add(s) }

    @Synchronized fun madeSamples(): List<BisectionSample> =
        samples.filter { it.outcome == ShotOutcome.MADE }

    @Synchronized
    fun save() {
        try {
            File(cellFile).writeText(gson.toJson(cells.values.toList()))
            File(sampleFile).writeText(gson.toJson(samples.toList()))
        } catch (e: Exception) { e.printStackTrace() }
    }

    @Synchronized
    fun load() {
        try {
            File(cellFile).takeIf { it.exists() }?.let { f ->
                val t = object : TypeToken<List<CellState>>() {}.type
                val loaded: List<CellState>? = gson.fromJson(f.readText(), t)
                cells.clear()
                loaded?.forEach { cells[it.cellId] = it }
            }
            File(sampleFile).takeIf { it.exists() }?.let { f ->
                val t = object : TypeToken<List<BisectionSample>>() {}.type
                val loaded: List<BisectionSample>? = gson.fromJson(f.readText(), t)
                samples.clear()
                loaded?.let { samples.addAll(it) }
            }
        } catch (e: Exception) { e.printStackTrace() }
    }

    @Synchronized fun clear() { cells.clear(); samples.clear() }

    @Synchronized fun removeLastSample() {
        if (samples.isNotEmpty()) samples.removeAt(samples.size - 1)
    }
}
