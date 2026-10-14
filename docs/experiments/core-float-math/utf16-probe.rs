#![no_std]
#[unsafe(no_mangle)]
pub fn utf16_count(s: &str) -> usize {
    s.encode_utf16().count()
}
#[unsafe(no_mangle)]
pub fn utf16_count_next(s: &str) -> usize {
    let mut it = s.encode_utf16();
    let mut count = 0;
    while it.next().is_some() {
        count += 1;
    }
    count
}
#[unsafe(no_mangle)]
pub fn utf16_count_fold(s: &str) -> usize {
    s.encode_utf16().fold(0usize, |n, _| n + 1)
}
#[cfg(test)]
mod tests {
    #[test]
    fn every_unicode_scalar() {
        let mut bytes = [0; 4];
        for code in 0..=0x10ffff {
            if let Some(c) = char::from_u32(code) {
                let s = c.encode_utf8(&mut bytes);
                assert_eq!(super::utf16_count(s), super::utf16_count_next(s));
                assert_eq!(super::utf16_count(s), super::utf16_count_fold(s));
            }
        }
        for s in ["", "ASCII", "Привет", "a😀б𐐀", "\0\u{ffff}\u{10000}"] {
            assert_eq!(super::utf16_count(s), super::utf16_count_next(s));
            assert_eq!(super::utf16_count(s), super::utf16_count_fold(s));
        }
    }
}
