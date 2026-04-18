package sigmacorns.test

import com.google.gson.GsonBuilder
import com.google.gson.reflect.TypeToken
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.io.TempDir
import sigmacorns.control.aim.tune.ShotDataStore
import sigmacorns.control.aim.tune.SpeedPoint
import java.io.File

class ShotDataStoreFilterTest {

    @Test
    fun filterExcludesPhysicsEstimates(@TempDir tempDir: File) {
        val file = File(tempDir, "test_data.json").absolutePath
        val store = ShotDataStore(file)

        store.addPoint(SpeedPoint(1.0, 300.0, 45.0, physicsEstimate = true))
        store.addPoint(SpeedPoint(2.0, 350.0, 42.0, physicsEstimate = false))
        store.addPoint(SpeedPoint(3.0, 400.0, 40.0, physicsEstimate = true))
        store.addPoint(SpeedPoint(4.0, 450.0, 38.0, physicsEstimate = false))

        val filtered = store.getPoints(includePhysicsEstimates = false)
        assertEquals(2, filtered.size)
        assertTrue(filtered.all { !it.physicsEstimate })
        assertEquals(2.0, filtered[0].distance)
        assertEquals(4.0, filtered[1].distance)
    }

    @Test
    fun filterIncludesAll(@TempDir tempDir: File) {
        val file = File(tempDir, "test_data.json").absolutePath
        val store = ShotDataStore(file)

        store.addPoint(SpeedPoint(1.0, 300.0, 45.0, physicsEstimate = true))
        store.addPoint(SpeedPoint(2.0, 350.0, 42.0, physicsEstimate = false))

        val all = store.getPoints(includePhysicsEstimates = true)
        assertEquals(2, all.size)
    }

    @Test
    fun countMethods(@TempDir tempDir: File) {
        val file = File(tempDir, "test_data.json").absolutePath
        val store = ShotDataStore(file)

        store.addPoint(SpeedPoint(1.0, 300.0, 45.0, physicsEstimate = true))
        store.addPoint(SpeedPoint(2.0, 350.0, 42.0, physicsEstimate = true))
        store.addPoint(SpeedPoint(3.0, 400.0, 40.0, physicsEstimate = false))

        assertEquals(2, store.getPhysicsEstimateCount())
        assertEquals(1, store.getEmpiricalCount())
    }

    @Test
    fun backwardCompatibleDeserialization(@TempDir tempDir: File) {
        val file = File(tempDir, "test_data.json")

        // Simulate old JSON format without physicsEstimate field
        val oldJson = """
            [
              { "distance": 1.5, "speed": 320.0, "hoodAngle": 44.0 },
              { "distance": 2.5, "speed": 380.0, "hoodAngle": 41.0 }
            ]
        """.trimIndent()
        file.writeText(oldJson)

        val store = ShotDataStore(file.absolutePath)
        store.load()

        val points = store.getPoints()
        assertEquals(2, points.size)
        // Gson defaults missing Boolean to false
        assertFalse(points[0].physicsEstimate)
        assertFalse(points[1].physicsEstimate)
        assertEquals(0, store.getPhysicsEstimateCount())
        assertEquals(2, store.getEmpiricalCount())
    }

    @Test
    fun saveAndReloadPreservesPhysicsFlag(@TempDir tempDir: File) {
        val file = File(tempDir, "test_data.json").absolutePath
        val store = ShotDataStore(file)

        store.addPoint(SpeedPoint(1.0, 300.0, 45.0, physicsEstimate = true))
        store.addPoint(SpeedPoint(2.0, 350.0, 42.0, physicsEstimate = false))
        store.save()

        val store2 = ShotDataStore(file)
        store2.load()

        val points = store2.getPoints()
        assertEquals(2, points.size)
        assertTrue(points[0].physicsEstimate)
        assertFalse(points[1].physicsEstimate)
    }
}
