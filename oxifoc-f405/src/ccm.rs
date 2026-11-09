//! Initialize the CCM load image before any Rust code can access its statics.
//! Use assembly: cortex-m-rt calls __pre_init before ordinary RAM/FPU init.
//! Only CPU-owned data is placed here; no code, DMA buffers or stack.

core::arch::global_asm!(
    ".section .text.__pre_init,\"ax\"",
    ".global __pre_init",
    ".type __pre_init,%function",
    ".thumb_func",
    "__pre_init:",
    // STM32F405 RCC_AHB1ENR.CCMDATARAMEN, enable before the first CCM write.
    "ldr r0, =0x40023830",
    "ldr r1, [r0]",
    "orr r1, r1, #0x100000",
    "str r1, [r0]",
    "dsb",
    "ldr r0, =__siccmdata",
    "ldr r1, =__sccmdata",
    "ldr r2, =__eccmdata",
    "2:",
    "cmp r1, r2",
    "bhs 3f",
    "ldr r3, [r0], #4",
    "str r3, [r1], #4",
    "b 2b",
    "3:",
    "bx lr",
    ".size __pre_init, . - __pre_init",
);
