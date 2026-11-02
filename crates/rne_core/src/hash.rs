//! A fast, fixed hasher for maps keyed by entities, handles and other small
//! integers.
//!
//! The standard library's `RandomState` runs SipHash with per-process random
//! keys: it resists hash flooding by untrusted input, which simulation keys
//! never are, and costs a noticeable share of a physics step that looks up a
//! few maps per body. [`FxHasher`] is rustc's hash, one rotate, xor and
//! multiply per word. Its output is the same in every process, so a map's
//! iteration order no longer varies between runs either; code that must not
//! depend on that order still sorts.

use std::hash::{BuildHasherDefault, Hasher};

const SEED: u64 = 0x51_7c_c1_b7_27_22_0a_95;

/// rustc's FxHash.
#[derive(Clone, Copy, Debug, Default)]
pub struct FxHasher {
    hash: u64,
}

impl FxHasher {
    #[inline]
    fn add(&mut self, word: u64) {
        self.hash = (self.hash.rotate_left(5) ^ word).wrapping_mul(SEED);
    }
}

impl Hasher for FxHasher {
    #[inline]
    fn write(&mut self, bytes: &[u8]) {
        let mut chunks = bytes.chunks_exact(8);
        for chunk in &mut chunks {
            let mut word = [0; 8];
            word.copy_from_slice(chunk);
            self.add(u64::from_le_bytes(word));
        }
        let rest = chunks.remainder();
        if !rest.is_empty() {
            let mut word = [0; 8];
            word[..rest.len()].copy_from_slice(rest);
            self.add(u64::from_le_bytes(word));
        }
    }

    #[inline]
    fn write_u8(&mut self, value: u8) {
        self.add(u64::from(value));
    }

    #[inline]
    fn write_u16(&mut self, value: u16) {
        self.add(u64::from(value));
    }

    #[inline]
    fn write_u32(&mut self, value: u32) {
        self.add(u64::from(value));
    }

    #[inline]
    fn write_u64(&mut self, value: u64) {
        self.add(value);
    }

    #[inline]
    fn write_usize(&mut self, value: usize) {
        self.add(value as u64);
    }

    #[inline]
    fn finish(&self) -> u64 {
        self.hash
    }
}

/// Builds [`FxHasher`]s, for `HashMap<K, V, FxBuildHasher>`.
pub type FxBuildHasher = BuildHasherDefault<FxHasher>;

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::{HashMap, HashSet};
    use std::hash::{BuildHasher, Hash};

    #[test]
    fn hashes_are_fixed_and_spread_small_integers() {
        let hash = |value: u64| FxBuildHasher::default().hash_one(value);
        assert_eq!(hash(1), hash(1));
        assert_ne!(hash(1), hash(2));
        // Consecutive keys must not share the bucket bits hashbrown uses.
        let low: HashSet<u64> = (0..1024u64).map(|v| hash(v) & 0x3ff).collect();
        assert!(low.len() > 600, "{} distinct low-bit patterns", low.len());
        let mut map: HashMap<u32, u32, FxBuildHasher> = HashMap::default();
        for key in 0..10_000 {
            map.insert(key, key * 2);
        }
        assert!((0..10_000).all(|key| map[&key] == key * 2));
    }

    #[test]
    fn byte_slices_hash_their_tail() {
        let hash = |bytes: &[u8]| {
            let mut hasher = FxHasher::default();
            bytes.hash(&mut hasher);
            hasher.finish()
        };
        assert_ne!(
            hash(&[1, 2, 3, 4, 5, 6, 7, 8, 9]),
            hash(&[1, 2, 3, 4, 5, 6, 7, 8, 10])
        );
    }
}
