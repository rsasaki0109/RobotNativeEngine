//! Changelog fragments: one file per change under `changelog.d/<section>/`, folded
//! into `CHANGELOG.md` at release time.
//!
//! Concurrent branches that each added a bullet to `CHANGELOG.md`'s `[Unreleased]`
//! section conflicted with one another on every merge; a fragment is a new file, so
//! two branches never touch the same lines. `cargo run -p xtask -- changelog`
//! previews the fragments, `--check` validates them (part of `ci-lint`), `--fold`
//! moves them into `[Unreleased]`, and `--release VERSION --date YYYY-MM-DD` moves
//! them and turns `[Unreleased]` into that release.

use anyhow::Context;
use std::fs;
use std::path::{Path, PathBuf};

/// Directory, relative to the workspace root, that holds the fragments.
pub(crate) const FRAGMENT_DIR: &str = "changelog.d";

/// Sections a fragment may belong to, in the order they are written out
/// (Keep a Changelog's order). The directory name is the lowercase heading.
const SECTIONS: [&str; 6] = [
    "Added",
    "Changed",
    "Deprecated",
    "Removed",
    "Fixed",
    "Security",
];

const UNRELEASED: &str = "## [Unreleased]";

/// One changelog entry: the section it goes under and its Markdown bullets.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Fragment {
    /// Section heading, one of [`SECTIONS`].
    pub(crate) section: &'static str,
    /// File the entry came from, for messages and deletion.
    pub(crate) path: PathBuf,
    /// The entry's text, trimmed, starting with `- `.
    pub(crate) body: String,
}

/// `cargo run -p xtask -- changelog [--check | --fold | --release VERSION --date DATE]`.
pub(crate) fn run(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let mut mode = "preview";
    let mut version = None;
    let mut date = None;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--check" => mode = "check",
            "--fold" => mode = "fold",
            "--release" => {
                mode = "release";
                version = Some(args.next().context("--release needs a version")?);
            }
            "--date" => date = Some(args.next().context("--date needs YYYY-MM-DD")?),
            other => anyhow::bail!("unknown changelog argument: {other}"),
        }
    }

    let root = crate::workspace_root()?;
    let fragments = collect_fragments(&root.join(FRAGMENT_DIR))?;
    match mode {
        "check" => {
            println!("changelog: {} fragment(s) valid", fragments.len());
            Ok(())
        }
        "preview" => {
            print!("{}", render_sections(&fragments));
            Ok(())
        }
        _ => {
            let release = match mode {
                "release" => {
                    let version = version.context("--release needs a version")?;
                    let date = date.context("--release needs --date YYYY-MM-DD")?;
                    validate_date(&date)?;
                    Some((version, date))
                }
                _ => None,
            };
            let changelog_path = root.join("CHANGELOG.md");
            let changelog = fs::read_to_string(&changelog_path)
                .with_context(|| format!("reading {}", changelog_path.display()))?;
            let folded = fold(
                &changelog,
                &fragments,
                release.as_ref().map(|(v, d)| (v.as_str(), d.as_str())),
            )?;
            fs::write(&changelog_path, folded)?;
            for fragment in &fragments {
                fs::remove_file(&fragment.path)
                    .with_context(|| format!("removing {}", fragment.path.display()))?;
            }
            println!(
                "changelog: folded {} fragment(s) into CHANGELOG.md",
                fragments.len()
            );
            Ok(())
        }
    }
}

/// Reads and validates every fragment under `dir`, sorted by section and file name.
///
/// `dir/README.md` is documentation and is skipped. Anything else must be
/// `dir/<section>/<name>.md` with a known section and a body of Markdown bullets.
pub(crate) fn collect_fragments(dir: &Path) -> anyhow::Result<Vec<Fragment>> {
    let mut fragments = Vec::new();
    if !dir.exists() {
        return Ok(fragments);
    }
    let mut entries = read_dir_sorted(dir)?;
    entries.retain(|path| path.file_name().is_some_and(|name| name != "README.md"));
    for entry in entries {
        let name = entry
            .file_name()
            .and_then(|name| name.to_str())
            .unwrap_or_default()
            .to_string();
        anyhow::ensure!(
            entry.is_dir(),
            "{}: fragments go in {FRAGMENT_DIR}/<section>/, not directly in {FRAGMENT_DIR}/",
            entry.display()
        );
        let section = SECTIONS
            .iter()
            .find(|section| section.to_lowercase() == name)
            .with_context(|| {
                format!(
                    "{}: unknown section `{name}`; expected one of {}",
                    entry.display(),
                    SECTIONS
                        .iter()
                        .map(|s| s.to_lowercase())
                        .collect::<Vec<_>>()
                        .join(", ")
                )
            })?;
        for path in read_dir_sorted(&entry)? {
            anyhow::ensure!(
                path.is_file() && path.extension().is_some_and(|ext| ext == "md"),
                "{}: a fragment must be a .md file",
                path.display()
            );
            let text =
                fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
            let body = validate_body(&text).with_context(|| path.display().to_string())?;
            fragments.push(Fragment {
                section,
                path,
                body,
            });
        }
    }
    fragments.sort_by(|a, b| {
        section_index(a.section)
            .cmp(&section_index(b.section))
            .then_with(|| a.path.cmp(&b.path))
    });
    Ok(fragments)
}

fn read_dir_sorted(dir: &Path) -> anyhow::Result<Vec<PathBuf>> {
    let mut paths = fs::read_dir(dir)
        .with_context(|| format!("reading {}", dir.display()))?
        .map(|entry| entry.map(|entry| entry.path()))
        .collect::<Result<Vec<_>, _>>()?;
    paths.sort();
    Ok(paths)
}

fn section_index(section: &str) -> usize {
    SECTIONS
        .iter()
        .position(|candidate| *candidate == section)
        .unwrap_or(SECTIONS.len())
}

/// A fragment's body must be one or more Markdown bullets, without headings.
fn validate_body(text: &str) -> anyhow::Result<String> {
    anyhow::ensure!(!text.contains('\r'), "fragment must use Unix line endings");
    let body = text.trim();
    anyhow::ensure!(!body.is_empty(), "fragment is empty");
    anyhow::ensure!(
        body.starts_with("- "),
        "fragment must start with a `- ` bullet"
    );
    anyhow::ensure!(
        !body.lines().any(|line| line.trim_start().starts_with('#')),
        "fragment must not contain headings; its directory names the section"
    );
    Ok(body.to_string())
}

fn validate_date(date: &str) -> anyhow::Result<()> {
    let parts: Vec<&str> = date.split('-').collect();
    anyhow::ensure!(
        parts.len() == 3
            && [4, 2, 2]
                .iter()
                .zip(&parts)
                .all(|(len, part)| part.len() == *len && part.chars().all(|c| c.is_ascii_digit())),
        "--date must be YYYY-MM-DD, got `{date}`"
    );
    Ok(())
}

/// The fragments as `### Section` blocks, in section order.
pub(crate) fn render_sections(fragments: &[Fragment]) -> String {
    let mut out = String::new();
    for section in SECTIONS {
        let bodies: Vec<&str> = fragments
            .iter()
            .filter(|fragment| fragment.section == section)
            .map(|fragment| fragment.body.as_str())
            .collect();
        if bodies.is_empty() {
            continue;
        }
        out.push_str(&format!("### {section}\n\n"));
        for body in bodies {
            out.push_str(body);
            out.push('\n');
        }
        out.push('\n');
    }
    out
}

/// Inserts the fragments at the top of `changelog`'s `[Unreleased]` section and,
/// with `release`, renames that section to `## [VERSION] - DATE` under a fresh,
/// empty `[Unreleased]`.
pub(crate) fn fold(
    changelog: &str,
    fragments: &[Fragment],
    release: Option<(&str, &str)>,
) -> anyhow::Result<String> {
    let start = changelog
        .lines()
        .position(|line| line.trim_end() == UNRELEASED)
        .with_context(|| format!("CHANGELOG.md has no `{UNRELEASED}` heading"))?;
    let lines: Vec<&str> = changelog.lines().collect();
    let mut out: Vec<String> = lines[..start].iter().map(|l| l.to_string()).collect();
    match release {
        Some((version, date)) => {
            out.push(UNRELEASED.to_string());
            out.push(String::new());
            out.push(format!("## [{version}] - {date}"));
        }
        None => out.push(UNRELEASED.to_string()),
    }
    out.push(String::new());
    let rendered = render_sections(fragments);
    out.extend(rendered.lines().map(str::to_string));
    let rest = &lines[start + 1..];
    let first_content = rest
        .iter()
        .position(|line| !line.trim().is_empty())
        .unwrap_or(rest.len());
    out.extend(rest[first_content..].iter().map(|l| l.to_string()));
    let mut text = out.join("\n");
    text.push('\n');
    Ok(text)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fragment(section: &'static str, name: &str, body: &str) -> Fragment {
        Fragment {
            section,
            path: PathBuf::from(name),
            body: body.to_string(),
        }
    }

    const CHANGELOG: &str = "# Changelog\n\nIntro.\n\n## [Unreleased]\n\n### Added\n\n- Old entry.\n\n## [0.4.0] - 2026-09-26\n\n- Released.\n";

    #[test]
    fn fold_puts_fragments_on_top_of_unreleased() {
        let fragments = [
            fragment("Added", "a.md", "- New thing."),
            fragment("Fixed", "b.md", "- A fix,\n  wrapped."),
        ];
        let folded = fold(CHANGELOG, &fragments, None).unwrap();
        assert_eq!(
            folded,
            "# Changelog\n\nIntro.\n\n## [Unreleased]\n\n### Added\n\n- New thing.\n\n### Fixed\n\n- A fix,\n  wrapped.\n\n### Added\n\n- Old entry.\n\n## [0.4.0] - 2026-09-26\n\n- Released.\n"
        );
    }

    #[test]
    fn release_renames_unreleased_under_a_fresh_one() {
        let fragments = [fragment("Changed", "c.md", "- Changed.")];
        let folded = fold(CHANGELOG, &fragments, Some(("0.5.0", "2026-12-01"))).unwrap();
        assert!(folded.starts_with(
            "# Changelog\n\nIntro.\n\n## [Unreleased]\n\n## [0.5.0] - 2026-12-01\n\n### Changed\n\n- Changed.\n\n### Added\n\n- Old entry.\n"
        ));
        assert!(folded.ends_with("## [0.4.0] - 2026-09-26\n\n- Released.\n"));
    }

    #[test]
    fn fold_without_fragments_keeps_the_changelog() {
        assert_eq!(fold(CHANGELOG, &[], None).unwrap(), CHANGELOG);
    }

    #[test]
    fn fold_needs_an_unreleased_heading() {
        assert!(fold("# Changelog\n", &[], None).is_err());
    }

    #[test]
    fn bodies_must_be_bullets_without_headings() {
        assert_eq!(validate_body("- Entry.\n\n").unwrap(), "- Entry.");
        assert!(validate_body("").is_err());
        assert!(validate_body("Entry.").is_err());
        assert!(validate_body("- Entry.\n### Added").is_err());
        assert!(validate_body("- Entry.\r\n").is_err());
    }

    #[test]
    fn dates_must_be_iso() {
        assert!(validate_date("2026-12-01").is_ok());
        assert!(validate_date("2026-12-1").is_err());
        assert!(validate_date("Dec 1").is_err());
    }

    #[test]
    fn collect_reads_sections_in_order_and_rejects_strays() {
        let dir = std::env::temp_dir().join(format!("rne-changelog-test-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(dir.join("fixed")).unwrap();
        fs::create_dir_all(dir.join("added")).unwrap();
        fs::write(dir.join("README.md"), "# How to\n").unwrap();
        fs::write(dir.join("fixed").join("a.md"), "- Fix.\n").unwrap();
        fs::write(dir.join("added").join("z.md"), "- Z.\n").unwrap();
        fs::write(dir.join("added").join("b.md"), "- B.\n").unwrap();
        let fragments = collect_fragments(&dir).unwrap();
        let bodies: Vec<&str> = fragments.iter().map(|f| f.body.as_str()).collect();
        assert_eq!(bodies, ["- B.", "- Z.", "- Fix."]);

        fs::create_dir_all(dir.join("misc")).unwrap();
        assert!(collect_fragments(&dir).is_err());
        fs::remove_dir_all(dir.join("misc")).unwrap();
        fs::write(dir.join("stray.md"), "- Stray.\n").unwrap();
        assert!(collect_fragments(&dir).is_err());
        fs::remove_dir_all(&dir).unwrap();
    }
}
