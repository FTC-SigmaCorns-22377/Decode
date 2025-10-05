use jni::{
    objects::{JClass, JFloatArray, JString}, sys::jlong, JNIEnv
};
use rerun::{Arrows3D, LineStrip3D, LineStrips3D, RecordingStream, RecordingStreamBuilder, Vec3D};

pub fn string_from_jni(env: &mut JNIEnv, v: &JString) -> Result<String, jni::errors::Error> {
    Ok(env.get_string(v)?.into())
}

pub fn connection_from_ptr(ptr: jlong) -> &'static mut RecordingStream {
    unsafe { (ptr as *mut RecordingStream).as_mut().unwrap() }
}

pub fn vec_from_java_float_arr(env: &JNIEnv, arr: JFloatArray) -> jni::errors::Result<Vec<f32>> {
    let len = env.get_array_length(&arr)? as usize;
    let mut buf = Vec::with_capacity(len);
    let ptr = buf.as_mut_ptr();

    unsafe {
        let slice = std::slice::from_raw_parts_mut(ptr, len);
        env.get_float_array_region(&arr, 0, slice)?;
        buf.set_len(buf.capacity());
    }

    Ok(buf)
}

#[unsafe(no_mangle)]
#[allow(non_snake_case)]
pub extern "C" fn Java_sigmacorns_io_RerunLogging_connect(
    mut env: JNIEnv,
    _class: JClass,
    name: JString,
    url: JString,
) -> jlong {
    let name = string_from_jni(&mut env, &name).unwrap();
    let url = string_from_jni(&mut env, &url).unwrap();

    RecordingStreamBuilder::new(name)
        .connect_grpc_opts(url)
        .map(|it| Box::into_raw(Box::new(it)) as jlong)
        .unwrap()
}

#[unsafe(no_mangle)]
#[allow(non_snake_case)]
pub extern "C" fn Java_sigmacorns_io_RerunLogging_save(
    mut env: JNIEnv,
    _class: JClass,
    name: JString,
    path: JString,
) -> jlong {
    let name = string_from_jni(&mut env, &name).unwrap();
    let path = string_from_jni(&mut env, &path).unwrap();
    let rec = Box::new(RecordingStreamBuilder::new(name).save(path).unwrap());

    Box::into_raw(rec) as jlong
}

#[unsafe(no_mangle)]
#[allow(non_snake_case)]
pub extern "C" fn Java_sigmacorns_io_RerunLogging_destroy(
    _env: JNIEnv,
    _class: JClass,
    rec: jlong,
) {
    let rec = unsafe { Box::from_raw(rec as *mut RecordingStream) };
    rec.memory();
    drop(rec);
}

#[unsafe(no_mangle)]
#[allow(non_snake_case)]
pub extern "C" fn Java_sigmacorns_io_RerunLogging_logState(
    env: JNIEnv,
    _class: JClass,
    connection: jlong,
    data: JFloatArray,
) {
    let rec = connection_from_ptr(connection);

    // Copy Java double[] into a Rust Vec<f64>
    let values: Vec<f32> = vec_from_java_float_arr(&env, data).unwrap();

    // Index mapping must match the Kotlin serializer
    let flywheel_speed = values[0];
    let px = values[1];
    let py = values[2];
    let prot = values[3];
    let vx = values[4];
    let vy = values[5];
    let vrot = values[6];
    let ax = values[7];
    let ay = values[8];
    // let arot = values[9]; // unused
    let intake_flap = values[10];
    let intake_roller = values[11];

    let timestamp = values[12];
    
    rec.set_time("state time", std::time::Duration::from_secs_f32(timestamp));

    // --- Log values to Rerun ---

    rec.log("robot/flywheel_speed", &rerun::Scalars::single(flywheel_speed)).unwrap();
    rec.log("robot/intake_flap", &rerun::Scalars::single(intake_flap)).unwrap();
    rec.log("robot/intake_roller", &rerun::Scalars::single(intake_roller)).unwrap();

    rec.log("robot/px", &rerun::Scalars::single(px)).unwrap();
    rec.log("robot/py", &rerun::Scalars::single(py)).unwrap();
    rec.log("robot/theta", &rerun::Scalars::single(prot)).unwrap();

    rec.log("robot/vx", &rerun::Scalars::single(vx)).unwrap();
    rec.log("robot/vy", &rerun::Scalars::single(vy)).unwrap();
    rec.log("robot/omega", &rerun::Scalars::single(vrot)).unwrap();

    // Pose arrow (direction from rotation)
    let pose_vectors = vec![Vec3D::new(prot.cos(), prot.sin(), 0.0)];
    let pose_origins = vec![Vec3D::new(px, py, 0.0)];
    let pose_arrows = Arrows3D::from_vectors(pose_vectors).with_origins(pose_origins);
    rec.log("robot/pose", &pose_arrows).expect("log pose");

    // Velocity arrow (direction = velocity vector)
    let vel_vectors = vec![Vec3D::new(vx, vy, 0.0)];
    let vel_origins = vec![Vec3D::new(px, py, 0.0)];
    let vel_arrows = Arrows3D::from_vectors(vel_vectors).with_origins(vel_origins);
    rec.log("robot/velocity", &vel_arrows).expect("log velocity");

    // Acceleration arrow
    let acc_vectors = vec![Vec3D::new(ax, ay, 0.0)];
    let acc_origins = vec![Vec3D::new(px, py, 0.0)];
    let acc_arrows = Arrows3D::from_vectors(acc_vectors).with_origins(acc_origins);
    rec.log("robot/acceleration", &acc_arrows).expect("log acceleration");
}


#[unsafe(no_mangle)]
#[allow(non_snake_case)]
pub extern "C" fn Java_sigmacorns_io_RerunLogging_logInputs(
    env: JNIEnv,
    _class: JClass,
    connection: jlong,
    data: JFloatArray,
) {
    let rec = connection_from_ptr(connection);

    // Copy Java double[] into a Rust Vec<f64>
    let values: Vec<f32> = vec_from_java_float_arr(&env, data).unwrap();

    rec.log("inputs/FL", &rerun::Scalars::single(values[0])).unwrap();
    rec.log("inputs/BL", &rerun::Scalars::single(values[1])).unwrap();
    rec.log("inputs/BR", &rerun::Scalars::single(values[2])).unwrap();
    rec.log("inputs/FR", &rerun::Scalars::single(values[3])).unwrap();
}

#[unsafe(no_mangle)]
#[allow(non_snake_case)]
pub extern "C" fn Java_sigmacorns_io_RerunLogging_logLineStrip3D(
    mut env: JNIEnv,
    _class: JClass,
    connection: jlong,
    name: JString,
    data: JFloatArray,
) {
    let rec = connection_from_ptr(connection);
    let name = string_from_jni(&mut env, &name).unwrap();

    // Copy Java double[] into a Rust Vec<f64>
    let values: Vec<f32> = vec_from_java_float_arr(&env, data).unwrap();

    let strip = LineStrips3D::new([values.chunks(3).map(|it| [it[0], it[1], it[2]]).collect::<Vec<[f32; 3]>>()]);

    rec.log(name.as_str(), &strip).unwrap();
}
