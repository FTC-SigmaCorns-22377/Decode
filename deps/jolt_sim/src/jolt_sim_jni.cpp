#include <jni.h>
#include <string>
#include "jolt_world.h"

// ---------------------------------------------------------------------------
// JNI helpers
// ---------------------------------------------------------------------------
namespace {

void throw_exception(JNIEnv* env, const char* class_name, const std::string& msg) {
    if (env->ExceptionCheck()) return;
    jclass cls = env->FindClass(class_name);
    if (!cls) return;
    env->ThrowNew(cls, msg.c_str());
    env->DeleteLocalRef(cls);
}

void throw_runtime(JNIEnv* env, const std::string& msg) {
    throw_exception(env, "java/lang/RuntimeException", msg);
}

void throw_illegal_state(JNIEnv* env, const std::string& msg) {
    throw_exception(env, "java/lang/IllegalStateException", msg);
}

JoltWorld* from_handle(jlong handle) {
    return reinterpret_cast<JoltWorld*>(handle);
}

bool check_handle(JNIEnv* env, jlong handle) {
    if (handle == 0) {
        throw_illegal_state(env, "JoltWorld handle is null (destroyed or never created)");
        return false;
    }
    return true;
}

// RAII wrapper for JNI float array access
class ScopedFloatArray {
public:
    ScopedFloatArray(JNIEnv* env, jfloatArray arr, jint release_mode)
        : env_(env), arr_(arr), mode_(release_mode), ptr_(nullptr) {
        if (!env_->ExceptionCheck() && arr_)
            ptr_ = env_->GetFloatArrayElements(arr_, nullptr);
    }
    ~ScopedFloatArray() {
        if (ptr_) env_->ReleaseFloatArrayElements(arr_, ptr_, mode_);
    }
    float* data() const { return ptr_; }
    bool valid() const { return ptr_ != nullptr; }
private:
    JNIEnv* env_;
    jfloatArray arr_;
    jint mode_;
    float* ptr_;
};

class ScopedIntArray {
public:
    ScopedIntArray(JNIEnv* env, jintArray arr, jint release_mode)
        : env_(env), arr_(arr), mode_(release_mode), ptr_(nullptr) {
        if (!env_->ExceptionCheck() && arr_)
            ptr_ = env_->GetIntArrayElements(arr_, nullptr);
    }
    ~ScopedIntArray() {
        if (ptr_) env_->ReleaseIntArrayElements(arr_, ptr_, mode_);
    }
    jint* data() const { return ptr_; }
    bool valid() const { return ptr_ != nullptr; }
private:
    JNIEnv* env_;
    jintArray arr_;
    jint mode_;
    jint* ptr_;
};

} // namespace

// ---------------------------------------------------------------------------
// JNI exports — Java class: sigmacorns.sim.JoltNative
// ---------------------------------------------------------------------------

extern "C" {

JNIEXPORT jlong JNICALL
Java_sigmacorns_sim_JoltNative_nativeCreate(JNIEnv* env, jclass) {
    try {
        auto* world = new JoltWorld();
        return reinterpret_cast<jlong>(world);
    } catch (const std::exception& e) {
        throw_runtime(env, std::string("Failed to create JoltWorld: ") + e.what());
    } catch (...) {
        throw_runtime(env, "Failed to create JoltWorld: unknown error");
    }
    return 0;
}

JNIEXPORT void JNICALL
Java_sigmacorns_sim_JoltNative_nativeDestroy(JNIEnv* env, jclass, jlong handle) {
    if (!check_handle(env, handle)) return;
    delete from_handle(handle);
}

JNIEXPORT void JNICALL
Java_sigmacorns_sim_JoltNative_nativeStep(JNIEnv* env, jclass, jlong handle, jfloat dt) {
    if (!check_handle(env, handle)) return;
    from_handle(handle)->step(dt);
}

JNIEXPORT void JNICALL
Java_sigmacorns_sim_JoltNative_nativeApplyRobotForce(JNIEnv* env, jclass, jlong handle,
                                                      jfloat fx, jfloat fy, jfloat tz) {
    if (!check_handle(env, handle)) return;
    from_handle(handle)->applyRobotForce(fx, fy, tz);
}

JNIEXPORT void JNICALL
Java_sigmacorns_sim_JoltNative_nativeSetRobotPose(JNIEnv* env, jclass, jlong handle,
                                                    jfloat x, jfloat y, jfloat theta) {
    if (!check_handle(env, handle)) return;
    from_handle(handle)->setRobotPose(x, y, theta);
}

JNIEXPORT void JNICALL
Java_sigmacorns_sim_JoltNative_nativeGetRobotState(JNIEnv* env, jclass, jlong handle,
                                                     jfloatArray out) {
    if (!check_handle(env, handle)) return;
    ScopedFloatArray arr(env, out, 0);
    if (!arr.valid()) return;
    from_handle(handle)->getRobotState(arr.data());
}

JNIEXPORT jint JNICALL
Java_sigmacorns_sim_JoltNative_nativeSpawnBall(JNIEnv* env, jclass, jlong handle,
                                                jfloat x, jfloat y, jfloat z,
                                                jfloat vx, jfloat vy, jfloat vz,
                                                jint color) {
    if (!check_handle(env, handle)) return -1;
    return from_handle(handle)->spawnBall(x, y, z, vx, vy, vz, color);
}

JNIEXPORT void JNICALL
Java_sigmacorns_sim_JoltNative_nativeRemoveBall(JNIEnv* env, jclass, jlong handle, jint index) {
    if (!check_handle(env, handle)) return;
    from_handle(handle)->removeBall(index);
}

JNIEXPORT jint JNICALL
Java_sigmacorns_sim_JoltNative_nativeGetBallCount(JNIEnv* env, jclass, jlong handle) {
    if (!check_handle(env, handle)) return 0;
    return from_handle(handle)->getBallCount();
}

JNIEXPORT void JNICALL
Java_sigmacorns_sim_JoltNative_nativeGetBallStates(JNIEnv* env, jclass, jlong handle,
                                                     jfloatArray out) {
    if (!check_handle(env, handle)) return;
    ScopedFloatArray arr(env, out, 0);
    if (!arr.valid()) return;
    from_handle(handle)->getBallStates(arr.data());
}

JNIEXPORT void JNICALL
Java_sigmacorns_sim_JoltNative_nativeGetBallColors(JNIEnv* env, jclass, jlong handle,
                                                     jintArray out) {
    if (!check_handle(env, handle)) return;
    ScopedIntArray arr(env, out, 0);
    if (!arr.valid()) return;
    from_handle(handle)->getBallColors(arr.data());
}

JNIEXPORT jint JNICALL
Java_sigmacorns_sim_JoltNative_nativeGetIntakeOverlaps(JNIEnv* env, jclass, jlong handle,
                                                        jintArray out, jint max) {
    if (!check_handle(env, handle)) return 0;
    ScopedIntArray arr(env, out, 0);
    if (!arr.valid()) return 0;
    return from_handle(handle)->getIntakeOverlaps(arr.data(), max);
}

JNIEXPORT void JNICALL
Java_sigmacorns_sim_JoltNative_nativeGetGoalStates(JNIEnv* env, jclass, jlong handle,
                                                    jfloatArray out) {
    if (!check_handle(env, handle)) return;
    ScopedFloatArray arr(env, out, 0);
    if (!arr.valid()) return;
    from_handle(handle)->getGoalStates(arr.data());
}

} // extern "C"
