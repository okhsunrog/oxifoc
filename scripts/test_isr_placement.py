import unittest

import isr_placement as p

# Excerpt in lld -Map layout: output section, input sections, Thumb symbols
# (odd addresses), mapping symbols and a script assignment.
MAP = """\
     VMA      LMA     Size Align Out     In      Symbol
 8000188  8000188      15c     8 .text
 8000188  8000188        0     1         __stext = .
 8000188  8000188       58     4         out/oxifoc_f405.o:(.Reset)
 8000188  8000188        0     1                 $t
 8000189  8000189       3e     1                 Reset
 80001e0  80001e0       4c     4         out/oxifoc_f405.o:(.text.__pre_init)
 80001e1  80001e1       34     1                 __pre_init
 8000232  8000232       44     2         out/oxifoc_f405.o:(.text._RNvNtCs3_11oxifoc_core3foc18wrap_angle_general)
 8000232  8000232        0     1                 $t
 8000233  8000233       44     1                 oxifoc_core::foc::wrap_angle_general
 8000280  8000280       60    16         lib.rlib(memcpy.o):(.text.memcpy)
 80002e0  80002e0        4     1         <internal>:(.text.unnamed)
 805f55c  805f55c       10     1 .foc_hot
 805f55c  805f55c        0     1         __foc_svpwm_start = .
"""

FOC_HOT = """\
SECTIONS
{
  .foc_hot : ALIGN(16)
  {
    . = ALIGN(16) + 8;
    __foc_cycle_start = .;
    KEEP(*(.text.foc_cycle));
    __foc_cycle_end = .;
    . = ALIGN(16);
    __foc_grant_start = .;
    KEEP(*(.text.foc_grant));
    __foc_grant_end = .;
  } > FLASH
}
INSERT BEFORE .rodata;
"""


class MapTests(unittest.TestCase):
    def test_input_sections_and_first_symbols(self):
        text = p.parse_map(MAP)[".text"]
        self.assertEqual([s.name for s in text], [
            ".Reset", ".text.__pre_init", ".text._RNvNtCs3_11oxifoc_core3foc18wrap_angle_general",
            ".text.memcpy", ".text.unnamed"])
        reset, pre, wrap, memcpy, unnamed = text
        self.assertEqual((reset.vma, reset.size, reset.align, reset.symbol), (0x8000188, 0x58, 4, "Reset"))
        self.assertEqual(pre.symbol, "__pre_init")
        self.assertEqual(wrap.symbol, "oxifoc_core::foc::wrap_angle_general")
        self.assertEqual(memcpy.file, "lib.rlib(memcpy.o)")
        self.assertIsNone(memcpy.symbol)
        self.assertEqual(unnamed.file, "<internal>")
        self.assertEqual(p.parse_map(MAP)[".foc_hot"], [])

    def test_reference_residues_strip_hashes(self):
        a = p.InSec(0x804, 4, 2, "f", ".text.x", "core::fmt::write::h0123456789abcdef")
        b = p.InSec(0x90a, 4, 2, "f", ".text.y", "core::fmt::write::hfedcba9876543210")
        self.assertEqual(p.residues_from([a, b]), {"core::fmt::write": [4, 0xa]})


class ResidueTests(unittest.TestCase):
    def test_entry_moves_forward_to_the_residue(self):
        self.assertEqual(p.entry(0x1000, 8, 4), 0x1008)
        self.assertEqual(p.entry(0x1008, 8, 4), 0x1008)
        self.assertEqual(p.entry(0x100a, 8, 2), 0x1018)
        self.assertEqual(p.entry(0x100c, 0, 4), 0x1010)

    def test_entry_honours_wide_alignment(self):
        self.assertEqual(p.entry(0x1004, 0, 32), 0x1020)
        with self.assertRaises(ValueError):
            p.entry(0x1000, 2, 4)

    def test_plan_keeps_packing_after_a_forced_residue(self):
        secs = [p.InSec(0x100, 6, 2, "", "a", None), p.InSec(0x106, 8, 4, "", "b", None),
                p.InSec(0x110, 2, 2, "", "c", None)]
        placed, conflicts, end = p.plan_text(secs, [0, 1, 2], {1: 0xc})
        self.assertEqual(placed, [(0, 0x100), (1, 0x10c), (2, 0x114)])
        self.assertEqual((conflicts, end), (0, 0x116))
        # Residue 2 is impossible for a 4-aligned section: natural packing.
        placed, conflicts, _ = p.plan_text(secs, [0, 2, 1], {1: 2})
        self.assertEqual(placed, [(0, 0x100), (2, 0x106), (1, 0x108)])
        self.assertEqual(conflicts, 1)

    def test_plan_reproduces_its_own_residues(self):
        secs = p.parse_map(MAP)[".text"]
        placed, _, _ = p.plan_text(secs, range(len(secs)), {i: s.vma % 16 for i, s in enumerate(secs)})
        self.assertEqual([a for _, a in placed], [s.vma for s in secs])


class ScriptTests(unittest.TestCase):
    def test_set_pin_residue(self):
        out = p.set_pin_residue(FOC_HOT, "grant", 4)
        self.assertIn(". = ALIGN(16) + 4;\n    __foc_grant_start", out)
        self.assertIn(". = ALIGN(16) + 8;\n    __foc_cycle_start", out)
        self.assertIn(". = ALIGN(16);\n    __foc_cycle_start", p.set_pin_residue(FOC_HOT, "cycle", 0))
        with self.assertRaises(ValueError):
            p.set_pin_residue(FOC_HOT, "missing", 0)

    def test_add_pin_appends_a_stanza_and_its_check(self):
        out = p.add_pin(FOC_HOT, "wrap", "*wrap_angle_general*", 2)
        stanza = out.index("KEEP(*(*wrap_angle_general*))")
        self.assertLess(out.index("__foc_grant_end"), stanza)
        self.assertLess(stanza, out.index("} > FLASH"))
        self.assertIn('ASSERT(__foc_wrap_end > __foc_wrap_start', out)

    def test_replace_text_body(self):
        link_x = "  .text _stext :\n  {\n    *(.text .text.*);\n  } > FLASH\n"
        out = p.replace_text_body(link_x, ['    . = 0x8000188; *(".Reset");'])
        self.assertNotIn("*(.text .text.*)", out)
        self.assertIn('__stext = .;\n    . = 0x8000188; *(".Reset");\n    . = ALIGN(4);', out)


def objdump(base):
    """rust-objdump -d excerpt with ADC at `base`, the pinned cycle function
    at base+0x100 (objdump labels it with the __foc_cycle_start alias) and a
    generic callee at base+0x200."""
    adc, cyc, fmt = base, base + 0x100, base + 0x200
    return f"""\
{adc:08x} <ADC>:
 {adc:x}:      \tpush\t{{r7, lr}}
 {adc + 2:x}:      \tmovw\tr0, #0x1370
 {adc + 6:x}:      \tmovt\tr0, #0x2001
 {adc + 10:x}:      \tbl\t{cyc:#x} <__foc_cycle_start> @ imm = #0xf4
 {adc + 14:x}:      \tbeq\t{adc + 2:#x} <ADC+0x2>
 {adc + 16:x}:      \tb.w\t{fmt:#x} <_RINvNtCs1_4core3fmt5writeE>
 {adc + 20:x}: 00 c0 01 20  \t.word\t0x2001c000

{cyc:08x} <__foc_cycle_start>:
 {cyc:x}:      \tbx\tlr
"""


CYCLE = ("<oxifoc_core::motor::FocDriver<oxifoc_f405::motor::MotorPwm> as oxifoc_core::Step<u8>>"
         "::step::<f32>")


def syms(base):
    return [(base + 1, 0x18, "T", "ADC", "ADC"),
            (base + 0x100, 0, "T", "__foc_cycle_start", "__foc_cycle_start"),
            (base + 0x101, 2, "t", "_RNv5cycle", CYCLE),
            (base + 0x201, 2, "t", "_RINvNtCs1_4core3fmt5writeE", "core::fmt::write")]


class DisassemblyTests(unittest.TestCase):
    def funcs(self, base=0x8000400):
        return p.functions(syms(base), p.parse_objdump(objdump(base)))

    def test_functions_split_by_symbol_address(self):
        f = self.funcs()
        self.assertEqual(sorted(f), [0x8000400, 0x8000500, 0x8000600])
        self.assertEqual(len(f[0x8000400].insns), 7)
        self.assertEqual(f[0x8000400].insns[-1], ".word\t0x2001c000")

    def test_calls_resolve_by_address_whatever_the_label(self):
        f = self.funcs()
        # The pinned function is labelled __foc_cycle_start by objdump and
        # has a generic demangled name: both used to hide it.
        self.assertEqual(p.callees(f[0x8000400], f), {0x8000500: 1, 0x8000600: 1})
        self.assertEqual(set(p.reachable(f)), {0x8000400, 0x8000500, 0x8000600})
        self.assertEqual(p.short(f[0x8000500].name), "FocDriver as Step::step")

    def test_ram_references(self):
        f = self.funcs()
        self.assertEqual(p.ram_refs(f[0x8000400].insns), [0x20011370, 0x2001c000])
        self.assertEqual([p.region(x) for x in (0x10000000, 0x2001bfff, 0x2001c000)], ["CCM", "SRAM1", "SRAM2"])

    def test_same_code_elsewhere_normalises_equal(self):
        a, b = self.funcs(), self.funcs(base=0x8010402)
        self.assertEqual(p.normalise(a[0x8000400], a), p.normalise(b[0x8010402], b))
        self.assertIn("bl\t<" + a[0x8000500].name + ">", p.normalise(a[0x8000400], a))
        self.assertIn("beq\t<.>", p.normalise(a[0x8000400], a))


class NameTests(unittest.TestCase):
    def test_short(self):
        self.assertEqual(p.short("oxifoc_core::state::run_foc_cycle::<a::B, c::D<e::F>>"), "state::run_foc_cycle")
        self.assertEqual(p.short("<oxifoc_core::foc::phase::manager::PhaseManager<a::B, c::D>>::blend_with_observer"),
                         "PhaseManager::blend_with_observer")
        self.assertEqual(p.short("<a::PhaseManager<x::Y> as b::PhaseProvider>::injection"),
                         "PhaseManager as PhaseProvider::injection")
        self.assertEqual(p.short("ADC"), "ADC")


if __name__ == "__main__":
    unittest.main()
