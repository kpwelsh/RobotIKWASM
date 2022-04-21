extern crate serde_json;
extern crate serde_derive;
extern crate wasm_bindgen;
use k::{Translation3, Isometry3};
use wasm_bindgen::prelude::*;
use robot_ik_base::IKSolver;
extern crate console_error_panic_hook;

extern crate nalgebra;
use nalgebra as na;
use na::{Quaternion, UnitQuaternion};

extern crate js_sys;

// When the `wee_alloc` feature is enabled, use `wee_alloc` as the global
// allocator.
#[cfg(feature = "wee_alloc")]
#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

#[wasm_bindgen]
pub fn test(v : &JsValue) -> js_sys::Float64Array {
    let a : [f64; 3] = v.into_serde().unwrap();
    js_sys::Float64Array::from(&a[..])
}

#[wasm_bindgen]
pub fn new_solver(urdf: &str, ee_frame: &str) -> *mut IKSolver {
    console_error_panic_hook::set_once();
    let mut solver = robot_ik_base::IKSolver::from_urdf_str(urdf);
    solver.set_ee(ee_frame);
    Box::into_raw(Box::new(solver))
}

#[wasm_bindgen]
pub fn set_self_collision(iksolver: *mut IKSolver, self_collision: bool) {
    unsafe { 
        match iksolver.as_mut() {
            Some(solver) => solver.self_collision = self_collision,
            None => { }
        }
    };
}

#[wasm_bindgen]
pub fn dof(iksolver: *mut IKSolver) -> i32 {
    unsafe { 
        match iksolver.as_mut() {
            Some(solver) => solver.dof(),
            None => { -1 }
        }
    }
}

#[wasm_bindgen]
pub fn solve(iksolver: *mut IKSolver, current_q: &JsValue, trans: &JsValue) -> js_sys::Float32Array {
    let current_q : Vec<f32> = current_q.into_serde().unwrap();
    let current_q : Vec<f64> = current_q.into_iter().map(|v| v as f64).collect();
    let trans : [f32; 7] = trans.into_serde().unwrap();
    let mut result : Vec<f32> = Vec::new();
    for _i in 0..current_q.len() {
        result.push(0.0);
    }
    result.push(1.);

    let iksolver = unsafe { 
        match iksolver.as_mut() {
            Some(solver) => solver,
            None => { 
                return js_sys::Float32Array::from(&result[..]); 
            }
        }
    };
    
    let x = Translation3::new(trans[0] as f64, trans[1] as f64, trans[2] as f64);
    let rot = UnitQuaternion::from_quaternion(Quaternion::new(trans[3] as f64, trans[4] as f64, trans[5] as f64, trans[6] as f64));
    let pose = Isometry3::from_parts(x, rot);
    match iksolver.solve(&current_q, &pose) {
        Ok(q) => {
            for i in 0..q.len() {
                result[i] = q[i] as f32;
            }
            // Set the error flag to 0 to indicate a successful ik solving.
            result[current_q.len()] = 0.;
        },
        Err(_) => { /* result already claims its an error by having result[-1] == -1 */}
    };
    return js_sys::Float32Array::from(&result[..]);
}