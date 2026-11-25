#!/usr/bin/env python3
"""Cache of the maps built by scripts/smoke_os1_128.sh (map_full and map_prefix).

A cache entry is a directory with the mapper output (dump/, traj_glim.txt, mapper_summary.json,
mapper.log), the compiled map (compiled/) and cache_entry.json, which holds the cache key, the
components the key was made from, and the size and SHA-256 of every other file in the entry.

Subcommands:
  key      hash the inputs of a map build into a key file (JSON)
  check    is an entry valid and built with this key? Exit 0 if it is; exit 1 and print the
           reason (one line) if it is not
  seal     validate a freshly built entry and write its cache_entry.json (the manifest)
  install  replace an entry with a sealed build directory by renaming it (same parent directory)
  cleanup  at the start of a run: remove leftover builds (.tmp-map_*), and put a displaced entry
           (.old-map_<label>-<pid>, left by an install that was interrupted between its two renames)
           back when its slot is empty, else remove it. A restored entry is checked like any other

The key components are named, so `check` can say which input changed. Inputs:
  --input NAME=PATH   a file or a directory (hashed recursively: relative names, sizes, contents)
  --value NAME=TEXT   a literal, for example the split arguments
  --binary PATH       an executable or library: its contents and those of every library ldd
                      resolves for it (unresolved ones become "missing:<name>" components)
  --modules CONFIG_DIR
                      the GLIM modules a config loads at run time (every "so_name" in its JSON
                      files, // comments skipped), found on LD_LIBRARY_PATH (an error if one is
                      not) and hashed like --binary
  Only these inputs are hashed: a file named in a --value (for example in MAPPER_ARGS) is not.

Exit codes: 0 ok, 1 invalid or mismatched entry (check) or failed validation (seal, install),
2 bad arguments.
"""

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys

FORMAT = 1
ENTRY_FILE = "cache_entry.json"
# The mapper's trajectory_coverage gate in smoke_os1_128.sh; a cached map must meet it too
MIN_TRAJECTORY_COVERAGE = 0.95
REQUIRED_FILES = ("mapper_summary.json", "traj_glim.txt")
REQUIRED_DIRS = ("dump", "compiled")


class CacheError(Exception):
    pass


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def hash_tree(path):
    """SHA-256 of a file, or of a directory: every entry's relative name, type, size and contents.

    Symlinks are followed, so a dataset whose points/ is a link to another directory is hashed by what the
    tools read. A link back to a directory that is already being walked (a loop) and a dangling link are
    errors."""
    if os.path.isfile(path):
        return sha256_file(path)
    if not os.path.isdir(path):
        raise CacheError(f"input does not exist: {path}")
    digest = hashlib.sha256()

    def walk(directory, rel, active):
        real = os.path.realpath(directory)
        if real in active:
            raise CacheError(f"symlink loop in the input at {os.path.join(path, rel)}")
        active = active | {real}
        for name in sorted(os.listdir(directory)):
            full = os.path.join(directory, name)
            rel_name = os.path.join(rel, name)
            if os.path.isdir(full):
                digest.update(f"D {rel_name}\n".encode())
                walk(full, rel_name, active)
            elif os.path.isfile(full):
                digest.update(f"F {rel_name} {os.path.getsize(full)} {sha256_file(full)}\n".encode())
            else:
                raise CacheError(f"not a file or directory (a dangling link?): {full}")

    walk(path, ".", frozenset())
    return digest.hexdigest()


def parse_ldd(text):
    """(resolved library paths, unresolved names) from ldd output."""
    resolved, missing = [], []
    for line in text.splitlines():
        line = line.strip()
        match = re.match(r"^(\S+)\s+=>\s+(.*)$", line)
        if match:
            target = match.group(2)
            if target.startswith("not found"):
                missing.append(match.group(1))
            else:
                path = target.split(" (")[0].strip()
                if path.startswith("/"):
                    resolved.append(path)
        elif line.startswith("/"):  # the dynamic loader: /lib/ld-linux-aarch64.so.1 (0x...)
            resolved.append(line.split(" (")[0].strip())
        # linux-vdso.so.1 (0x...) and "statically linked" have no file
    return resolved, missing


def ldd(path):
    """ldd of a binary; a file that is not a dynamic ELF (a script) has no libraries."""
    try:
        result = subprocess.run(["ldd", path], capture_output=True, text=True, check=False)
    except OSError:
        return [], []
    if result.returncode != 0:
        return [], []
    return parse_ldd(result.stdout)


def so_names(config_dir):
    """Every "so_name" in the config's JSON files; GLIM's configs are JSONC, so // comments are skipped."""
    names = set()
    for name in sorted(os.listdir(config_dir)):
        if name.endswith(".json"):
            with open(os.path.join(config_dir, name)) as handle:
                for line in handle:
                    code = line.split("//", 1)[0]  # a so_name never contains //
                    names.update(re.findall(r'"so_name"\s*:\s*"([^"]+)"', code))
    return sorted(names)


def find_library(name, search_path):
    if os.path.isabs(name):
        return name if os.path.isfile(name) else None
    for directory in search_path:
        candidate = os.path.join(directory, name)
        if directory and os.path.isfile(candidate):
            return candidate
    return None


def binary_components(paths):
    components = {}
    for path in paths:
        if not os.path.isfile(path):
            raise CacheError(f"binary does not exist: {path}")
        real = os.path.realpath(path)
        components[f"binary:{real}"] = sha256_file(real)
        resolved, missing = ldd(real)
        for library in resolved:
            library = os.path.realpath(library)
            if f"binary:{library}" not in components:
                components[f"binary:{library}"] = sha256_file(library)
        for name in missing:
            components[f"missing:{name}"] = "not found"
    return components


def split_assignment(text, option):
    name, sep, value = text.partition("=")
    if not sep or not name:
        raise CacheError(f"{option} expects NAME=VALUE, got {text!r}")
    return name, value


def make_key(inputs=(), values=(), binaries=(), module_configs=(), search_path=None):
    """{"format", "key", "components"} for the given inputs."""
    if search_path is None:
        search_path = [d for d in os.environ.get("LD_LIBRARY_PATH", "").split(":") if d]
    components = {}
    for text in inputs:
        name, path = split_assignment(text, "--input")
        components[f"input:{name}"] = hash_tree(path)
    for text in values:
        name, value = split_assignment(text, "--value")
        components[f"value:{name}"] = value
    paths = list(binaries)
    for config_dir in module_configs:
        for name in so_names(config_dir):
            found = find_library(name, search_path)
            if found is None:  # the mapper could not load it either
                raise CacheError(f"the module {name} of {config_dir} is not on LD_LIBRARY_PATH")
            paths.append(found)
    components.update(binary_components(paths))
    components = dict(sorted(components.items()))
    key = hashlib.sha256(json.dumps({"format": FORMAT, "components": components}, sort_keys=True).encode()).hexdigest()
    return {"format": FORMAT, "key": key, "components": components}


def changed_components(old, new):
    """Names of the components that differ, are new or are gone, for the log."""
    names = sorted(set(old) | set(new))
    changed = [n for n in names if old.get(n) != new.get(n)]

    def short(name):  # binary:/opt/glim/lib/libglim.so -> binary libglim.so
        kind, _, rest = name.partition(":")
        return f"{kind} {os.path.basename(rest) if kind == 'binary' else rest}"

    return [short(n) for n in changed]


def list_files(entry):
    """Relative paths of the files in an entry (without cache_entry.json); a symlink is an error."""
    files = []
    for root, dirs, names in os.walk(entry):
        for name in dirs + names:
            if os.path.islink(os.path.join(root, name)):
                raise CacheError(f"unexpected symlink {os.path.relpath(os.path.join(root, name), entry)}")
        for name in names:
            rel = os.path.relpath(os.path.join(root, name), entry)
            if rel != ENTRY_FILE:
                files.append(rel)
    return sorted(files)


def validate_content(entry):
    """Checks of the map and its reference trajectory that do not need the manifest."""
    for name in REQUIRED_FILES:
        if not os.path.isfile(os.path.join(entry, name)):
            raise CacheError(f"{name} is missing")
    for name in REQUIRED_DIRS:
        path = os.path.join(entry, name)
        if not os.path.isdir(path) or not any(files for _, _, files in os.walk(path)):
            raise CacheError(f"{name}/ is missing or empty")

    try:
        with open(os.path.join(entry, "mapper_summary.json")) as handle:
            summary = json.load(handle)
    except (OSError, ValueError) as error:
        raise CacheError(f"mapper_summary.json cannot be read: {error}") from error
    if not isinstance(summary, dict):
        raise CacheError("mapper_summary.json is not an object")
    if summary.get("failed") is not False:
        raise CacheError("mapper_summary.json does not say failed: false")
    coverage = summary.get("trajectory_coverage")
    if isinstance(coverage, bool) or not isinstance(coverage, (int, float)) or not math.isfinite(coverage):
        raise CacheError(f"trajectory_coverage is not a finite number: {coverage!r}")
    if coverage < MIN_TRAJECTORY_COVERAGE:
        raise CacheError(f"trajectory_coverage {coverage} is below {MIN_TRAJECTORY_COVERAGE}")

    rows = []
    with open(os.path.join(entry, "traj_glim.txt")) as handle:
        for number, line in enumerate(handle, 1):
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            fields = line.split()
            try:
                row = [float(v) for v in fields]
            except ValueError as error:
                raise CacheError(f"traj_glim.txt line {number} is not numeric") from error
            if len(row) != 8 or not all(math.isfinite(v) for v in row):
                raise CacheError(f"traj_glim.txt line {number} is not 8 finite numbers")
            norm = math.sqrt(sum(v * v for v in row[4:8]))
            if abs(norm - 1.0) > 1e-3:
                raise CacheError(f"traj_glim.txt line {number} has a quaternion of norm {norm:.6f}")
            if rows and row[0] <= rows[-1][0]:
                raise CacheError(f"traj_glim.txt line {number}: stamps are not increasing")
            rows.append(row)
    if len(rows) < 2:
        raise CacheError("traj_glim.txt has fewer than 2 poses")
    poses = summary.get("num_trajectory_poses")
    if isinstance(poses, int) and not isinstance(poses, bool) and poses != len(rows):
        raise CacheError(f"traj_glim.txt has {len(rows)} poses but mapper_summary.json says {poses}")


def manifest(entry):
    return {rel: {"size": os.path.getsize(os.path.join(entry, rel)), "sha256": sha256_file(os.path.join(entry, rel))}
            for rel in list_files(entry)}


def read_json(path, what):
    try:
        with open(path) as handle:
            data = json.load(handle)
    except (OSError, ValueError) as error:
        raise CacheError(f"{what} cannot be read: {error}") from error
    if not isinstance(data, dict):
        raise CacheError(f"{what} is not an object")
    return data


def check(entry, key):
    """None when the entry is valid and has this key, else the reason."""
    if os.path.islink(entry) or (os.path.exists(entry) and not os.path.isdir(entry)):
        return "the cache entry is not a directory"
    if not os.path.isdir(entry):
        return "no cached map"
    try:
        meta = read_json(os.path.join(entry, ENTRY_FILE), ENTRY_FILE)
    except CacheError as error:
        return f"the cached map is incomplete or corrupt ({error})"
    if meta.get("format") != FORMAT:
        return f"the cache format changed ({meta.get('format')!r} -> {FORMAT})"
    if meta.get("key") != key["key"]:
        old = meta.get("components") if isinstance(meta.get("components"), dict) else {}
        changed = changed_components(old, key["components"]) or ["(unknown)"]
        return "the key changed: " + ", ".join(changed)
    files = meta.get("files")
    if not isinstance(files, dict):
        return "the cached map is corrupt (no file list)"
    try:
        present = list_files(entry)
        if present != sorted(files):
            extra = sorted(set(present) - set(files))
            gone = sorted(set(files) - set(present))
            return f"the cached map is corrupt (missing files {gone[:3]}, unexpected files {extra[:3]})"
        for rel in present:
            path = os.path.join(entry, rel)
            expected = files[rel]
            if not isinstance(expected, dict) or os.path.getsize(path) != expected.get("size"):
                return f"the cached map is corrupt ({rel}: size differs)"
            if sha256_file(path) != expected.get("sha256"):
                return f"the cached map is corrupt ({rel}: contents differ)"
        validate_content(entry)
    except (CacheError, OSError) as error:
        return f"the cached map is invalid ({error})"
    return None


def seal(entry, key, extra=None):
    validate_content(entry)
    meta = {"format": FORMAT, "key": key["key"], "components": key["components"], "files": manifest(entry)}
    if extra:
        meta.update(extra)
    tmp = os.path.join(entry, ENTRY_FILE + ".tmp")
    with open(tmp, "w") as handle:
        json.dump(meta, handle, indent=2, sort_keys=True)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, os.path.join(entry, ENTRY_FILE))


def install(build, entry):
    """Rename a sealed build over the entry. A reader sees the old entry, no entry, or the new one."""
    build, entry = os.path.abspath(build), os.path.abspath(entry)
    if os.path.dirname(build) != os.path.dirname(entry):
        raise CacheError("the build and the entry must be in the same directory")
    if not os.path.isfile(os.path.join(build, ENTRY_FILE)):
        raise CacheError(f"{build} is not sealed")
    old = None
    if os.path.lexists(entry):
        old = os.path.join(os.path.dirname(entry), f".old-{os.path.basename(entry)}-{os.getpid()}")
        os.rename(entry, old)
    try:
        os.rename(build, entry)
    except OSError:
        if old is not None:
            os.rename(old, entry)  # the previous entry goes back; cleanup() does the same after a crash here
        raise
    if old is not None:
        remove(old)


def remove(path):
    if os.path.isdir(path) and not os.path.islink(path):
        shutil.rmtree(path)
    else:
        os.remove(path)


OLD_PATTERN = re.compile(r"^\.old-(map_[A-Za-z0-9_]+)-\d+$")


def cleanup(cache_dir):
    """Messages for the log about what was removed or restored."""
    messages = []
    for name in sorted(os.listdir(cache_dir)):
        full = os.path.join(cache_dir, name)
        if name.startswith(".tmp-map_"):
            remove(full)
            messages.append(f"removed the unfinished build {name}")
            continue
        match = OLD_PATTERN.match(name)
        if match is None:
            continue
        entry = os.path.join(cache_dir, match.group(1))
        if os.path.lexists(entry):
            remove(full)
            messages.append(f"removed the replaced entry {name}")
        else:
            os.rename(full, entry)
            messages.append(f"restored {name} as {match.group(1)} (an install was interrupted)")
    return messages


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    p_key = sub.add_parser("key")
    p_key.add_argument("--out", required=True)
    p_key.add_argument("--input", action="append", default=[])
    p_key.add_argument("--value", action="append", default=[])
    p_key.add_argument("--binary", action="append", default=[])
    p_key.add_argument("--modules", action="append", default=[])
    p_check = sub.add_parser("check")
    p_check.add_argument("--entry", required=True)
    p_check.add_argument("--key", required=True)
    p_seal = sub.add_parser("seal")
    p_seal.add_argument("--entry", required=True)
    p_seal.add_argument("--key", required=True)
    p_seal.add_argument("--note", action="append", default=[], help="NAME=VALUE stored in cache_entry.json")
    p_install = sub.add_parser("install")
    p_install.add_argument("--build", required=True)
    p_install.add_argument("--entry", required=True)
    p_cleanup = sub.add_parser("cleanup")
    p_cleanup.add_argument("--cache", required=True)
    args = parser.parse_args(argv)

    try:
        if args.command == "key":
            key = make_key(args.input, args.value, args.binary, args.modules)
            tmp = args.out + ".tmp"
            with open(tmp, "w") as handle:
                json.dump(key, handle, indent=2, sort_keys=True)
                handle.write("\n")
            os.replace(tmp, args.out)
            print(key["key"])
            return 0
        if args.command == "check":
            key = read_json(args.key, "the key file")
            reason = check(args.entry, key)
            if reason:
                print(reason)
                return 1
            print("valid")
            return 0
        if args.command == "seal":
            key = read_json(args.key, "the key file")
            notes = dict(split_assignment(text, "--note") for text in args.note)
            seal(args.entry, key, {"notes": notes} if notes else None)
            return 0
        if args.command == "cleanup":
            for message in cleanup(args.cache):
                print(f"cache: {message}")
            return 0
        install(args.build, args.entry)
        return 0
    except CacheError as error:
        print(f"smoke_map_cache: {error}", file=sys.stderr)
        return 2 if args.command == "key" else 1
    except OSError as error:
        print(f"smoke_map_cache: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
