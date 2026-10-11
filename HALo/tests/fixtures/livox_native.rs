//! Test ABI provider: no hardware, SDK, or C++ compiler required.
use std::ffi::{c_char, c_void, CStr};
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

type Callback = unsafe extern "C" fn(*const u8, usize, *mut c_void);
static SESSION: Mutex<Option<(Callback, usize, String)>> = Mutex::new(None);
static STOPS: AtomicUsize = AtomicUsize::new(0);

fn emit(callback: Callback, context: usize, kind: u8, id: u8, values: &[f32]) {
    let mut packet = b"LVXR".to_vec();
    packet.extend([1, kind, id, if kind == 1 { 1 } else { 0 }]);
    packet.extend((if kind == 1 { (values.len()/3) as u16 } else { 1 }).to_le_bytes());
    packet.extend([0, 0]);
    packet.extend(123_u64.to_le_bytes());
    for v in values {
        packet.extend(v.to_le_bytes());
    }
    unsafe { callback(packet.as_ptr(), packet.len(), context as *mut c_void) };
}

#[no_mangle]
pub unsafe extern "C" fn halo_livox_start(
    path: *const c_char,
    cb: Callback,
    ctx: *mut c_void,
) -> i32 {
    let path = unsafe { CStr::from_ptr(path) }.to_str().unwrap().to_owned();
    if std::fs::read_to_string(&path).unwrap().contains("192.0.2.254") {
        return 1;
    }
    let mut session = SESSION.lock().unwrap();
    if session.is_some() {
        return 2;
    }
    *session = Some((cb, ctx as usize, path));
    emit(cb, ctx as usize, 1, 40, &[1.0, 2.0, 3.0]);
    emit(cb, ctx as usize, 1, 41, &[4.0, 5.0, 6.0]);
    emit(cb, ctx as usize, 2, 40, &[0.1, 0.2, 0.3, 0.0, 0.0, 1.0]);
    // Neither corrupt packets nor unconfigured devices may reach consumers.
    emit(cb, ctx as usize, 1, 40, &[f32::NAN, 0.0, 0.0]);
    emit(cb, ctx as usize, 1, 42, &[7.0, 8.0, 9.0]);
    0
}

#[no_mangle]
pub extern "C" fn halo_livox_stop() {
    if let Some((cb, ctx, path)) = SESSION.lock().unwrap().take() {
        // The Rust context must still be alive during SDK shutdown.
        emit(cb, ctx, 1, 40, &[10.0, 11.0, 12.0]);
        assert!(std::path::Path::new(&path).is_file());
        STOPS.fetch_add(1, Ordering::SeqCst);
    }
}

#[no_mangle]
pub extern "C" fn halo_livox_stop_count() -> usize { STOPS.load(Ordering::SeqCst) }

#[no_mangle]
pub unsafe extern "C" fn halo_livox_config_path(buffer: *mut u8, capacity: usize) -> usize {
    if let Some((_, _, path)) = &*SESSION.lock().unwrap() {
        if capacity < path.len() { return 0; }
        unsafe { std::ptr::copy_nonoverlapping(path.as_ptr(), buffer, path.len()); }
        path.len()
    } else { 0 }
}

#[no_mangle]
pub extern "C" fn halo_livox_emit() {
    if let Some((cb, ctx, _)) = &*SESSION.lock().unwrap() {
        let values: Vec<f32> = (0..512).flat_map(|i| [i as f32, 2.0, 3.0]).collect();
        emit(*cb, *ctx, 1, 40, &values);
        emit(*cb, *ctx, 2, 40, &[0.1, 0.2, 0.3, 0.0, 0.0, 1.0]);
    }
}
