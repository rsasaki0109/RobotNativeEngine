//! Fail-closed verification for an extracted native release bundle.

use anyhow::{bail, Context, Result};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::fs;
use std::io::Read;
use std::path::{Component, Path};

use crate::{
    INSTALLED_BUNDLE_VERIFICATION_REPORT_KIND, INSTALLED_BUNDLE_VERIFICATION_REPORT_SCHEMA_VERSION,
};

const CHECKSUM_MANIFEST: &str = "SHA256SUMS";
const RELEASE_REPORT: &str = "release-report.json";

/// Content identity of one file used to verify an installed bundle.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct InstalledBundleArtifact {
    /// Forward-slash path relative to the extracted bundle root.
    pub path: String,
    /// Exact file size.
    pub size_bytes: u64,
    /// Lowercase SHA-256 digest without a prefix.
    pub sha256: String,
}

/// Evidence that every declared member of an extracted release bundle matched.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct InstalledBundleVerificationReport {
    /// Stable report kind.
    pub kind: String,
    /// Report schema version.
    pub schema_version: u32,
    /// `passed` only after exact member-set and digest verification.
    pub status: String,
    /// Extracted bundle directory name.
    pub bundle_root: String,
    /// Number of payload members covered by `SHA256SUMS`.
    pub verified_member_count: usize,
    /// Sum of exact payload file sizes.
    pub verified_payload_bytes: u64,
    /// Identity of the manifest that covers every payload member.
    pub checksum_manifest: InstalledBundleArtifact,
    /// Identity of the release report covered by that manifest.
    pub release_report: InstalledBundleArtifact,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct ActualMember {
    size_bytes: u64,
    sha256: String,
}

/// Verifies the exact regular-file graph declared by an extracted bundle.
///
/// The root and every descendant must be free of symbolic links. Extra,
/// missing, duplicate, malformed, or digest-mismatched members fail closed.
/// Relative roots such as `.` and `..` use their resolved directory name.
pub fn verify(root: &Path) -> Result<InstalledBundleVerificationReport> {
    let metadata = fs::symlink_metadata(root)
        .with_context(|| format!("inspect installed bundle root {}", root.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_dir() && !metadata.file_type().is_symlink(),
        "installed bundle root must be a non-symlink directory: {}",
        root.display()
    );
    // Reject symbolic-link roots before resolving relative directory names.
    let resolved_root = fs::canonicalize(root)
        .with_context(|| format!("resolve installed bundle root {}", root.display()))?;
    let root = resolved_root.as_path();
    let manifest_path = root.join(CHECKSUM_MANIFEST);
    let manifest_bytes = read_regular_file(&manifest_path, "checksum manifest")?;
    let declared = parse_manifest(&manifest_bytes)?;
    let actual = collect_members(root)?;
    anyhow::ensure!(
        declared.len() == actual.len()
            && declared.iter().all(|(path, digest)| {
                actual
                    .get(path)
                    .is_some_and(|member| member.sha256 == *digest)
            }),
        "installed bundle SHA256SUMS does not match its exact member graph"
    );
    let release = actual
        .get(RELEASE_REPORT)
        .context("installed bundle omitted release-report.json")?;
    let bundle_root = root
        .file_name()
        .and_then(|name| name.to_str())
        .context("installed bundle root has no Unicode directory name")?;
    anyhow::ensure!(
        !bundle_root.is_empty(),
        "installed bundle root directory name is empty"
    );
    Ok(InstalledBundleVerificationReport {
        kind: INSTALLED_BUNDLE_VERIFICATION_REPORT_KIND.to_string(),
        schema_version: INSTALLED_BUNDLE_VERIFICATION_REPORT_SCHEMA_VERSION,
        status: "passed".to_string(),
        bundle_root: bundle_root.to_string(),
        verified_member_count: actual.len(),
        verified_payload_bytes: actual.values().try_fold(0_u64, |total, member| {
            total
                .checked_add(member.size_bytes)
                .context("installed bundle payload size overflowed u64")
        })?,
        checksum_manifest: artifact(CHECKSUM_MANIFEST, &manifest_bytes),
        release_report: InstalledBundleArtifact {
            path: RELEASE_REPORT.to_string(),
            size_bytes: release.size_bytes,
            sha256: release.sha256.clone(),
        },
    })
}

fn artifact(path: &str, bytes: &[u8]) -> InstalledBundleArtifact {
    InstalledBundleArtifact {
        path: path.to_string(),
        size_bytes: u64::try_from(bytes.len()).unwrap_or(u64::MAX),
        sha256: format!("{:x}", Sha256::digest(bytes)),
    }
}

fn collect_members(root: &Path) -> Result<BTreeMap<String, ActualMember>> {
    fn visit(
        root: &Path,
        directory: &Path,
        members: &mut BTreeMap<String, ActualMember>,
    ) -> Result<()> {
        let mut entries = fs::read_dir(directory)
            .with_context(|| format!("read installed bundle directory {}", directory.display()))?
            .collect::<std::result::Result<Vec<_>, _>>()?;
        entries.sort_by_key(fs::DirEntry::file_name);
        for entry in entries {
            let path = entry.path();
            let file_type = entry.file_type()?;
            anyhow::ensure!(
                !file_type.is_symlink(),
                "installed bundle contains symbolic link {}",
                path.display()
            );
            if file_type.is_dir() {
                visit(root, &path, members)?;
            } else if file_type.is_file() {
                let relative = path.strip_prefix(root)?;
                validate_relative_path(relative)?;
                let relative = relative.to_string_lossy().replace('\\', "/");
                if relative == CHECKSUM_MANIFEST {
                    continue;
                }
                let member = hash_member(&path)?;
                anyhow::ensure!(
                    members.insert(relative.clone(), member).is_none(),
                    "duplicate installed bundle member {relative}"
                );
            } else {
                bail!(
                    "installed bundle contains unsupported member {}",
                    path.display()
                );
            }
        }
        Ok(())
    }

    let mut members = BTreeMap::new();
    visit(root, root, &mut members)?;
    Ok(members)
}

fn parse_manifest(bytes: &[u8]) -> Result<BTreeMap<String, String>> {
    let text = std::str::from_utf8(bytes).context("SHA256SUMS is not UTF-8")?;
    anyhow::ensure!(
        !text.is_empty() && text.ends_with('\n') && !text.contains('\r'),
        "SHA256SUMS must be non-empty canonical LF-terminated text"
    );
    let mut members = BTreeMap::new();
    for line in text.lines() {
        let (digest, path) = line
            .split_once("  ")
            .context("SHA256SUMS entries must use `<digest>  <path>`")?;
        anyhow::ensure!(
            digest.len() == 64
                && digest
                    .bytes()
                    .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)),
            "SHA256SUMS member {path} has an invalid lowercase SHA-256 digest"
        );
        anyhow::ensure!(
            !path.contains('\\'),
            "SHA256SUMS paths must use forward slashes"
        );
        validate_relative_path(Path::new(path))?;
        anyhow::ensure!(
            path != CHECKSUM_MANIFEST,
            "SHA256SUMS must not contain itself"
        );
        anyhow::ensure!(
            members
                .insert(path.to_string(), digest.to_string())
                .is_none(),
            "duplicate SHA256SUMS member {path}"
        );
    }
    Ok(members)
}

fn validate_relative_path(path: &Path) -> Result<()> {
    anyhow::ensure!(
        !path.as_os_str().is_empty()
            && !path.is_absolute()
            && path
                .components()
                .all(|component| matches!(component, Component::Normal(_))),
        "invalid installed bundle member path {}",
        path.display()
    );
    Ok(())
}

fn read_regular_file(path: &Path, label: &str) -> Result<Vec<u8>> {
    let metadata = fs::symlink_metadata(path)
        .with_context(|| format!("inspect {label} {}", path.display()))?;
    anyhow::ensure!(
        metadata.file_type().is_file() && !metadata.file_type().is_symlink(),
        "{label} must be a regular non-symlink file: {}",
        path.display()
    );
    fs::read(path).with_context(|| format!("read {label} {}", path.display()))
}

fn hash_member(path: &Path) -> Result<ActualMember> {
    let mut file = fs::File::open(path)
        .with_context(|| format!("open installed bundle member {}", path.display()))?;
    let size_bytes = file.metadata()?.len();
    let mut digest = Sha256::new();
    // Keep the hashing buffer off the main thread's comparatively small Windows stack.
    let mut buffer = vec![0_u8; 1024 * 1024];
    let mut hashed_bytes = 0_u64;
    loop {
        let read = file
            .read(&mut buffer)
            .with_context(|| format!("hash installed bundle member {}", path.display()))?;
        if read == 0 {
            break;
        }
        hashed_bytes = hashed_bytes
            .checked_add(u64::try_from(read).unwrap_or(u64::MAX))
            .context("installed bundle member size overflowed u64")?;
        digest.update(&buffer[..read]);
    }
    anyhow::ensure!(
        hashed_bytes == size_bytes,
        "installed bundle member changed while hashing: {}",
        path.display()
    );
    Ok(ActualMember {
        size_bytes,
        sha256: format!("{:x}", digest.finalize()),
    })
}

#[cfg(test)]
mod tests {
    use super::{verify, CHECKSUM_MANIFEST, RELEASE_REPORT};
    use sha2::{Digest, Sha256};
    use std::fs;

    fn fixture() -> tempfile::TempDir {
        let directory = tempfile::tempdir().expect("temporary installed bundle");
        fs::create_dir(directory.path().join("bin")).expect("bin directory");
        fs::write(directory.path().join(RELEASE_REPORT), b"release\n").expect("release report");
        fs::write(directory.path().join("bin/rne-tool"), b"binary\n").expect("binary");
        let digest = |bytes: &[u8]| format!("{:x}", Sha256::digest(bytes));
        fs::write(
            directory.path().join(CHECKSUM_MANIFEST),
            format!(
                "{}  bin/rne-tool\n{}  release-report.json\n",
                digest(b"binary\n"),
                digest(b"release\n")
            ),
        )
        .expect("manifest");
        directory
    }

    #[test]
    fn verifies_exact_installed_bundle_member_graph() {
        let directory = fixture();
        let report = verify(directory.path()).expect("valid installed bundle");
        assert_eq!(report.status, "passed");
        assert_eq!(report.verified_member_count, 2);
        assert_eq!(report.release_report.path, RELEASE_REPORT);
    }

    #[test]
    fn verifies_current_directory_without_changing_the_parent_process_directory() {
        const EXPECTED_ROOT: &str = "RNE_ASSET_CLI_TEST_BUNDLE_ROOT_NAME";
        if let Some(expected) = std::env::var_os(EXPECTED_ROOT) {
            let report = verify(std::path::Path::new(".")).expect("current-directory bundle");
            assert_eq!(report.bundle_root, expected.to_string_lossy());
            assert_eq!(report.verified_member_count, 2);
            return;
        }

        let directory = fixture();
        let module = module_path!().split_once("::").expect("crate module").1;
        let output = std::process::Command::new(std::env::current_exe().expect("test executable"))
            .arg("--exact")
            .arg(format!("{module}::verifies_current_directory_without_changing_the_parent_process_directory"))
            .current_dir(directory.path())
            .env(EXPECTED_ROOT, directory.path().file_name().expect("fixture name"))
            .output()
            .expect("isolated current-directory verification");
        assert!(
            output.status.success(),
            "{}\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        assert!(String::from_utf8_lossy(&output.stdout).contains("1 passed"));
    }

    #[test]
    fn parent_directory_input_has_the_same_verified_identity() {
        let directory = fixture();
        let child = directory.path().join("empty-child");
        fs::create_dir(&child).expect("empty child directory");
        let expected = verify(directory.path()).expect("absolute bundle root");
        let actual = verify(&child.join("..")).expect("parent-directory bundle root");
        assert_eq!(actual, expected);
    }

    #[cfg(unix)]
    #[test]
    fn rejects_symlink_roots_before_resolving_the_directory_name() {
        let directory = fixture();
        let aliases = tempfile::tempdir().expect("alias directory");
        let alias = aliases.path().join("bundle");
        std::os::unix::fs::symlink(directory.path(), &alias).expect("root symlink");
        assert!(verify(&alias).is_err());
    }

    #[test]
    fn rejects_tamper_extra_member_and_manifest_path_escape() {
        let directory = fixture();
        fs::write(directory.path().join("bin/rne-tool"), b"tampered\n").expect("tamper");
        assert!(verify(directory.path()).is_err());

        let directory = fixture();
        fs::write(directory.path().join("extra"), b"extra\n").expect("extra");
        assert!(verify(directory.path()).is_err());

        let directory = fixture();
        fs::remove_file(directory.path().join("bin/rne-tool")).expect("remove member");
        assert!(verify(directory.path()).is_err());

        let directory = fixture();
        fs::write(
            directory.path().join(CHECKSUM_MANIFEST),
            format!("{}  ../escape\n", "0".repeat(64)),
        )
        .expect("escaping manifest");
        assert!(verify(directory.path()).is_err());

        let directory = fixture();
        let manifest = fs::read(directory.path().join(CHECKSUM_MANIFEST)).expect("manifest");
        let first = manifest
            .split(|byte| *byte == b'\n')
            .next()
            .expect("entry")
            .to_vec();
        let mut duplicate = manifest;
        duplicate.extend_from_slice(&first);
        duplicate.push(b'\n');
        fs::write(directory.path().join(CHECKSUM_MANIFEST), duplicate).expect("duplicate manifest");
        assert!(verify(directory.path()).is_err());

        #[cfg(unix)]
        {
            use std::os::unix::fs::symlink;

            let directory = fixture();
            symlink("rne-tool", directory.path().join("bin/alias")).expect("bundle symlink");
            assert!(verify(directory.path()).is_err());
        }
    }
}
