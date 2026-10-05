//! Keeps time reads and waits in `src/` to ones that go through libc, where a
//! simulator interposing the OS can see them. A cycle counter, inline
//! assembly, a raw clock or futex syscall, or a TSC-backed clock crate would
//! read or wait on time behind its back.

use std::path::Path;

const FORBIDDEN: &[&str] = &[
    "rdtsc",
    "asm!",
    "global_asm!",
    "std::arch::",
    "core::arch::",
    "quanta",
    "minstant",
];

const RAW_SYSCALLS: &[&str] = &[
    "SYS_clock_gettime",
    "SYS_clock_nanosleep",
    "SYS_gettimeofday",
    "SYS_nanosleep",
    "SYS_futex",
];

fn visit(dir: &Path, violations: &mut Vec<String>) {
    for entry in std::fs::read_dir(dir).unwrap().flatten() {
        let path = entry.path();
        if path.is_dir() {
            visit(&path, violations);
            continue;
        }
        if path.extension().is_none_or(|e| e != "rs") {
            continue;
        }
        let text = std::fs::read_to_string(&path).unwrap();
        for (n, line) in text.lines().enumerate() {
            let code = line.split("//").next().unwrap_or("");
            let raw_syscall =
                code.contains("syscall(") && RAW_SYSCALLS.iter().any(|s| code.contains(s));
            if raw_syscall || FORBIDDEN.iter().any(|p| code.contains(p)) {
                violations.push(format!("{}:{}: {}", path.display(), n + 1, line.trim()));
            }
        }
    }
}

#[test]
fn src_reads_and_waits_on_time_only_through_libc() {
    let mut violations = Vec::new();
    visit(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("src"),
        &mut violations,
    );
    assert!(
        violations.is_empty(),
        "time read or waited on outside libc:\n{}",
        violations.join("\n")
    );
}
