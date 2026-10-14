#![no_std]
#![feature(core_float_math)]
#[unsafe(no_mangle)]
pub fn core_sqrt(x: f32) -> f32 {
    core::f32::math::sqrt(x)
}
#[unsafe(no_mangle)]
pub fn asm_sqrt(x: f32) -> f32 {
    let r;
    unsafe {
        core::arch::asm!("vsqrt.f32 {o}, {i}", o = out(sreg) r, i = in(sreg) x, options(pure, nomem, nostack));
    }
    r
}
#[unsafe(no_mangle)]
pub fn core_floor(x: f32) -> f32 {
    core::f32::math::floor(x)
}
#[unsafe(no_mangle)]
pub fn core_round(x: f32) -> f32 {
    core::f32::math::round(x)
}
#[unsafe(no_mangle)]
pub fn core_mod(x: f32, y: f32) -> f32 {
    x % y
}
#[unsafe(no_mangle)]
pub fn rust_q31_to_f32(x: i32) -> f32 {
    x as f32 * (1.0 / 2147483648.0)
}
#[unsafe(no_mangle)]
pub fn rust_f32_to_q31(x: f32) -> i32 {
    (x * 2147483648.0) as i32
}
#[unsafe(no_mangle)]
pub fn asm_q31_to_f32(x: i32) -> f32 {
    let r;
    unsafe {
        core::arch::asm!("vmov {tmp}, {val}", "vcvt.f32.s32 {tmp}, {tmp}, #31", val=in(reg)x, tmp=out(sreg)r);
    }
    r
}
#[unsafe(no_mangle)]
pub fn asm_f32_to_q31(x: f32) -> i32 {
    let r;
    unsafe {
        core::arch::asm!("vcvt.s32.f32 {tmp}, {tmp}, #31", "vmov {res}, {tmp}", tmp=inout(sreg)x=>_, res=out(reg)r);
    }
    r
}
