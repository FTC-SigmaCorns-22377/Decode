#include <jni.h>
#include <string>
#include "DrakeSim.hpp"

extern "C" JNIEXPORT jlong JNICALL
Java_sigmacorns_sim_DrakeNative_createSim(
        JNIEnv* env,
        jobject /* this */,
        jstring urdfPath) {
    const char* path = env->GetStringUTFChars(urdfPath, 0);
    DrakeSim* sim = new DrakeSim(std::string(path));
    env->ReleaseStringUTFChars(urdfPath, path);
    return reinterpret_cast<jlong>(sim);
}

extern "C" JNIEXPORT void JNICALL
Java_sigmacorns_sim_DrakeNative_step(
        JNIEnv* env,
        jobject /* this */,
        jlong simPtr,
        jdouble dt,
        jdoubleArray inputs) {
    DrakeSim* sim = reinterpret_cast<DrakeSim*>(simPtr);
    if (!sim) return;

    jsize len = env->GetArrayLength(inputs);
    jdouble* body = env->GetDoubleArrayElements(inputs, 0);
    std::vector<double> inputVec(body, body + len);
    env->ReleaseDoubleArrayElements(inputs, body, 0);

    sim->Step(dt, inputVec);
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_sigmacorns_sim_DrakeNative_getState(
        JNIEnv* env,
        jobject /* this */,
        jlong simPtr) {
    DrakeSim* sim = reinterpret_cast<DrakeSim*>(simPtr);
    if (!sim) return nullptr;

    std::vector<double> state = sim->GetState();
    
    jdoubleArray result = env->NewDoubleArray(state.size());
    env->SetDoubleArrayRegion(result, 0, state.size(), state.data());
    return result;
}

extern "C" JNIEXPORT void JNICALL
Java_sigmacorns_sim_DrakeNative_spawnBall(
        JNIEnv* env,
        jobject /* this */,
        jlong simPtr,
        jdouble x,
        jdouble y,
        jdouble z) {
    DrakeSim* sim = reinterpret_cast<DrakeSim*>(simPtr);
    if (sim) {
        sim->SpawnBall(x, y, z);
    }
}

extern "C" JNIEXPORT void JNICALL
Java_sigmacorns_sim_DrakeNative_setPosition(
        JNIEnv* env,
        jobject /* this */,
        jlong simPtr,
        jdouble x,
        jdouble y,
        jdouble yaw) {
    DrakeSim* sim = reinterpret_cast<DrakeSim*>(simPtr);
    if (sim) {
        sim->SetPosition(x, y, yaw);
    }
}

extern "C" JNIEXPORT void JNICALL
Java_sigmacorns_sim_DrakeNative_destroySim(
        JNIEnv* env,
        jobject /* this */,
        jlong simPtr) {
    DrakeSim* sim = reinterpret_cast<DrakeSim*>(simPtr);
    if (sim) {
        delete sim;
    }
}
