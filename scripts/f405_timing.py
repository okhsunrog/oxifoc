"""Read back the complete F405 PWM/injected-ADC timing contract while Stopped."""
import bench_overclock as probe

SAMPLE_CYCLES = [3, 15, 28, 56, 84, 112, 144, 480]


def snapshot():
    clocks = probe.clocks()
    tim = probe.read(0x40010000, 18)
    common = probe.read(0x40012300, 2)
    assert (tim[0] >> 5) & 3 == 3 and tim[0] & 1, 'TIM1 center-aligned mode changed'
    assert tim[10] == 0 and tim[12] == 1, 'TIM1 prescaler/repetition changed'
    assert tim[11] == clocks['tim1_hz'] // 40_000, 'TIM1 is not configured for nominal 20 kHz'
    assert (tim[7] >> 12) & 7 == 6 and tim[7] & (1 << 11), 'CH4 PWM/preload changed'
    assert tim[8] & 0x1000, 'ADC trigger channel disabled'
    assert tim[16] == tim[11] - tim[11] // 50, 'CCR4 sampling location changed'
    assert common[1] & 31 == 0, 'ADC independent mode changed'
    ckd = (tim[0] >> 8) & 3
    assert ckd < 3
    dtg = tim[17] & 255
    ticks = dtg if dtg < 128 else ((64+(dtg&63))*2 if dtg < 192 else
        (32+(dtg&31))*(8 if dtg < 224 else 16))
    deadtime_ns = ticks * (1 << ckd) * 1e9 / clocks['tim1_hz']
    assert 345 <= deadtime_ns <= 375, 'hardware deadtime changed'
    result = dict(clocks=clocks, tim1_raw=tim, adc_common_raw=common,
        timer_arr=tim[11], timer_psc=tim[10], timer_rcr=tim[12], ccr4=tim[16],
        hardware_deadtime_ns=deadtime_ns, ccr4_distance_from_peak_us=(tim[11]-tim[16])*1e6/clocks['tim1_hz'], adcs=[])
    for i, expected in enumerate([[10,3,0], [11,14,1], [12,13,2]]):
        regs = probe.read(0x40012000 + i*0x100, 15)
        cr1, cr2, smpr1, smpr2, jsqr = regs[1],regs[2],regs[3],regs[4],regs[14]
        length = ((jsqr >> 20) & 3) + 1
        channels = [(jsqr >> (5*j)) & 31 for j in range(4-length,4)]
        assert length == 3 and channels == expected, 'injected ranks/channels changed'
        assert (cr1 >> 24) & 3 == 0 and cr1 & (1<<8), 'ADC resolution/scan changed'
        assert cr1 & ((1<<10)|(1<<12)) == 0, 'ADC automatic/discontinuous injection enabled'
        assert (cr2 >> 16) & 15 == 0 and (cr2 >> 20) & 3 == 1, 'TIM1_CC4 rising trigger changed'
        assert cr2 & 1 and not cr2 & 2, 'ADC disabled or continuous mode enabled'
        assert cr2 & (1<<8) == 0, 'ADC DMA unexpectedly enabled'
        assert bool(cr1 & (1<<7)) == (i == 2), 'JEOC interrupt ownership changed'
        samples = [SAMPLE_CYCLES[((smpr2 if ch < 10 else smpr1) >> (3*(ch%10))) & 7] for ch in channels]
        assert samples == [15]*3, 'ADC sample-time setting changed'
        result['adcs'].append(dict(adc=i+1, raw=regs, channels=channels, sample_cycles=samples,
            sample_us=[s*1e6/clocks['adc_hz'] for s in samples],
            sequence_conversion_us=sum(s+12 for s in samples)*1e6/clocks['adc_hz']))
    return result
