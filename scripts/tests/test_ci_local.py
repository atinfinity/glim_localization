"""Tests for scripts/ci_local.sh (snapshot, lock, pass reuse, docs-only skip) and ci.sh --skip-deps.

They use temporary git repos with a stubbed docker command and a stubbed scripts/docker_run.sh, so
no real Docker is needed. The stub "build" is a tiny Makefile that trusts mtimes in a per-project
directory standing in for the build volume, which reproduces the stale-object false pass.
"""
import os
import re
import shutil
import subprocess
import threading
import time
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1]
DOCKERFILE = "FROM scratch\n"
# Repository paths the image bakes in; they must match image_inputs in scripts/image_inputs_hash.sh.
INPUTS = [
    "docker/Dockerfile",
    "docker/entrypoint.sh",
    "scripts/requirements.txt",
    "third_party/patches/glim",
    "third_party/patches/glim_ros2",
]

# Builds with make (mtime based) in the stub volume, then optionally misbehaves as the test asks.
STUB_DOCKER_RUN = """#!/bin/bash
root="$(cd "$(dirname "$0")/.." && pwd)"
project="${COMPOSE_PROJECT_NAME:-default}"
echo "$* @ ${project} @ ${root}" >> "${STUB_CALLS}"
# Which image the container would start: compose uses GLIM_LOC_IMAGE, else the moving dev tag.
ref="${GLIM_LOC_IMAGE:-glim_localization:dev}"
if [[ -f "${STUB_TAGS}/${ref}" ]]; then ran="$(cat "${STUB_TAGS}/${ref}")"; else ran="$(cat "${STUB_IMAGE_FILE}")"; fi
echo "${ran} ${ref}" >> "${STUB_RAN}"
[[ -z "${STUB_KILL_PARENT:-}" ]] || kill -KILL "${PPID}"
if [[ -n "${STUB_TERM_PARENT:-}" ]]; then kill -TERM "${PPID}"; sleep 1; exit 1; fi
export ROOT="${root}"
[[ -z "${STUB_HOOK:-}" ]] || bash -c "${STUB_HOOK}"
if [[ -f "${root}/Makefile" ]]; then
  make -s -C "${root}" B="${STUB_VOLUMES}/${project}_build" check || exit 1
fi
exit "${STUB_RC:-0}"
"""

# Handles the few docker subcommands ci_local.sh uses.
STUB_DOCKER = """#!/bin/bash
echo "docker $*" >> "${STUB_DOCKER_LOG}"
case "$1 $2" in
  "image inspect")
    ref="${@: -1}"
    if [[ -f "${STUB_TAGS}/${ref}" ]]; then cat "${STUB_TAGS}/${ref}"; else cat "${STUB_IMAGE_FILE}"; fi ;;
  "tag "*) printf '%s\\n' "$2" > "${STUB_TAGS}/$3" ;;
  "rmi "*) rm -f "${STUB_TAGS}/$2" ;;
  "ps"*) [[ -z "${STUB_PS_HOOK:-}" ]] || bash -c "${STUB_PS_HOOK}" ;;
  "volume inspect") [[ -d "${STUB_VOLUMES}/$3" ]] ;;
  "volume rm") rm -rf "${STUB_VOLUMES}/$3" ;;
  "run --rm")
    case "$*" in
      *"--entrypoint date"*) echo "${STUB_CONTAINER_DATE:-$(date +%s)}" ;;
      *"--entrypoint sh"*)
        # The listing of /etc/glim_localization/inputs in the image.
        # $5 is the image: an ID is stable, a tag may have moved (STUB_TAG_INPUTS = the new image).
        dir="${STUB_IMAGE_INPUTS}"
        case "$5" in sha256:*) ;; *) dir="${STUB_TAG_INPUTS:-${dir}}" ;; esac
        if [[ -d "${dir}" ]]; then
          cd "${dir}" && find . -type f | LC_ALL=C sort | while read -r f; do
            printf '%s  %s\\n' "$( { sha256sum "$f" 2>/dev/null || shasum -a 256 "$f"; } | cut -d' ' -f1)" "$f"
          done
        fi ;;
      *"--entrypoint chown"*)
        # run --rm -v SRC:/s --entrypoint chown IMAGE -R UID:GID /s; the stub makes everything writable.
        [[ -z "${STUB_CHOWN_FAIL:-}" ]] || exit 1
        chmod -R u+rwX "${4%:/s}" ;;
    esac ;;
esac
"""

MAKEFILE = """B ?= build
$(B)/built.txt: value.txt
\\tmkdir -p $(B)
\\tcp value.txt $@
\t@perl -e 'my $$t = time - 5; utime $$t, $$t, $$ARGV[0]' $@
# Like a CMake cache: filled once from CMakeLists.txt and never refreshed.
$(B)/cache.txt:
\\tmkdir -p $(B)
\\tcp CMakeLists.txt $@
check: $(B)/built.txt $(B)/cache.txt
\\ttest "$$(cat $(B)/built.txt)" = pass
\\ttest "$$(cat $(B)/cache.txt)" = pass
"""


def git(repo, *args):
    return subprocess.run(
        ["git", "-c", "user.name=test", "-c", "user.email=test@example.invalid", *args],
        cwd=repo,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def commit_file(repo, name, text, message="change"):
    path = repo / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    git(repo, "add", "-A")
    git(repo, "commit", "-q", "-m", message)


@pytest.fixture(scope="module")
def template(tmp_path_factory):
    repo = tmp_path_factory.mktemp("template") / "repo"
    scripts = repo / "scripts"
    (scripts / "check_deps").mkdir(parents=True)
    (repo / "docker").mkdir()
    for name in ("ci_local.sh", "is_docs_only.sh", "image_inputs_hash.sh"):
        shutil.copy(SCRIPTS / name, scripts / name)
    (scripts / "docker_run.sh").write_text(STUB_DOCKER_RUN)
    (scripts / "check_deps.sh").write_text("# deps v1\n")
    (scripts / "requirements.in").write_text("pytest==1\n")
    (scripts / "check_deps" / "main.cpp").write_text("int main() {}\n")
    (repo / "docker" / "Dockerfile").write_text(DOCKERFILE)
    (repo / "docker" / "entrypoint.sh").write_text("#!/bin/bash\nexec \"$@\"\n")
    (scripts / "requirements.txt").write_text("pytest==1\n")
    for sub in ("glim", "glim_ros2"):
        (repo / "third_party" / "patches" / sub).mkdir(parents=True)
        (repo / "third_party" / "patches" / sub / "fix.patch").write_text(f"patch {sub}\n")
    (repo / "CMakeLists.txt").write_text("pass\n")
    (repo / ".gitignore").write_text("out/\n.cache/\n")
    (repo / "code.cpp").write_text("int main() { return 0; }\n")
    (repo / "value.txt").write_text("pass\n")
    (repo / "Makefile").write_text(MAKEFILE.replace("\\t", "\t"))
    for path in (scripts / "ci_local.sh", scripts / "is_docs_only.sh", scripts / "image_inputs_hash.sh", scripts / "docker_run.sh"):
        path.chmod(0o755)
    git(repo.parent, "init", "-q", "-b", "main", str(repo))
    git(repo, "add", "-A")
    git(repo, "commit", "-q", "-m", "base")
    git(repo, "update-ref", "refs/remotes/origin/main", "HEAD")
    return repo


class Env:
    def __init__(self, tmp_path, template):
        self.root = tmp_path
        self.repo = tmp_path / "repo"
        shutil.copytree(template, self.repo, symlinks=True)
        self.calls = tmp_path / "calls.log"
        self.docker_log = tmp_path / "docker.log"
        self.volumes = tmp_path / "volumes"
        self.volumes.mkdir()
        self.tags = tmp_path / "tags"
        self.tags.mkdir()
        self.ran = tmp_path / "ran.log"
        self.image_file = tmp_path / "image_id"
        self.image_file.write_text("sha256:image-one\n")
        self.image_inputs = tmp_path / "image_inputs"
        self.rebuild_image_inputs()
        (tmp_path / "docker").write_text(STUB_DOCKER)
        (tmp_path / "docker").chmod(0o755)
        self.env = {
            **os.environ,
            "STUB_CALLS": str(self.calls),
            "STUB_DOCKER_LOG": str(self.docker_log),
            "STUB_VOLUMES": str(self.volumes),
            "STUB_IMAGE_FILE": str(self.image_file),
            "STUB_TAGS": str(self.tags),
            "STUB_RAN": str(self.ran),
            "STUB_IMAGE_INPUTS": str(self.image_inputs),
            "CI_LOCAL_DOCKER": str(tmp_path / "docker"),
            "CI_LOCAL_POLL": "0.1",
        }
        for key in ("STUB_RC", "STUB_HOOK", "STUB_KILL_PARENT", "STUB_CONTAINER_DATE", "STUB_CHOWN_FAIL", "STUB_TAG_INPUTS",
                    "STUB_PS_HOOK", "STUB_TERM_PARENT"):
            self.env.pop(key, None)

    def rebuild_image_inputs(self):
        """What `docker compose build` bakes into the image from the current repository files."""
        shutil.rmtree(self.image_inputs, ignore_errors=True)
        for path in INPUTS:
            source = self.repo / path
            if not source.exists():
                continue
            target = self.image_inputs / path
            target.parent.mkdir(parents=True, exist_ok=True)
            if source.is_dir():
                shutil.copytree(source, target)
            else:
                shutil.copy(source, target)

    def cmd(self, repo, *args, **extra):
        return (["bash", str(repo / "scripts" / "ci_local.sh"), *args], {**self.env, **extra})

    def run(self, *args, repo=None, **extra):
        repo = repo or self.repo
        command, env = self.cmd(repo, *args, **extra)
        return subprocess.run(command, cwd=repo, env=env, capture_output=True, text=True, check=False)

    def popen(self, *args, repo=None, **extra):
        repo = repo or self.repo
        command, env = self.cmd(repo, *args, **extra)
        return subprocess.Popen(command, cwd=repo, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def run_overlapping(self, second_repo=None):
        """Start a run that holds the lock until released, then a second run that must wait for it."""
        release = self.root / "release"
        first = self.popen(STUB_HOOK=f"until [ -e '{release}' ]; do sleep 0.05; done")
        deadline = time.time() + 30
        while not (self.state() / "lock" / "pid").exists() and time.time() < deadline:
            time.sleep(0.02)
        command, env = self.cmd(second_repo or self.repo)
        second = subprocess.Popen(command, cwd=second_repo or self.repo, env=env,
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        # The second run reports that it is waiting; the watchdog stops a run that never does.
        watchdog = threading.Timer(30, second.kill)
        watchdog.start()
        first_line = second.stdout.readline()
        watchdog.cancel()
        release.write_text("go")
        rest, err = second.communicate()
        first.communicate()
        result = subprocess.CompletedProcess(command, second.returncode, first_line + rest, err)
        return first, result

    def call_lines(self):
        return self.calls.read_text().splitlines() if self.calls.exists() else []

    def ran_images(self):
        """(image ID, image reference) the stub container was started from, per run."""
        return [tuple(line.split()) for line in self.ran.read_text().splitlines()] if self.ran.exists() else []

    def docker_calls(self):
        return self.docker_log.read_text().splitlines() if self.docker_log.exists() else []

    def stamps(self, prefix=""):
        directory = self.repo / "out" / "ci_stamps"
        return sorted(p for p in directory.glob(f"{prefix}*") if p.is_file()) if directory.exists() else []

    def set_image(self, image_id):
        self.image_file.write_text(image_id + "\n")

    def state(self):
        return self.repo / ".git" / "ci_local"

    def wipes(self):
        return [line for line in self.docker_calls() if line.startswith("docker volume rm")]


@pytest.fixture
def env(tmp_path, template):
    return Env(tmp_path, template)


def set_value(env, value, message="value"):
    commit_file(env.repo, "value.txt", value + "\n", message)


def bump(env, message):
    """A new tree whose result does not change."""
    commit_file(env.repo, "note.txt", message + "\n", message)


# ---- Reuse, force, image


def test_first_run_is_full_and_second_is_reused(env):
    first = env.run()
    assert first.returncode == 0, first.stderr
    assert len(env.call_lines()) == 1
    second = env.run()
    assert second.returncode == 0
    assert len(env.call_lines()) == 1
    assert "REUSED" in second.stdout
    assert git(env.repo, "rev-parse", "HEAD") in second.stdout


def test_clean_run_uses_the_snapshot_and_its_own_project(env):
    env.run()
    (line,) = env.call_lines()
    assert str(env.state() / "src") in line
    assert "@ glim_localization_ci_" in line


def test_force_reruns_ignores_every_pass_and_wipes_the_volume(env):
    env.run()
    bump(env, "second")
    env.run()  # records the deps pass for this image
    result = env.run("--force")
    assert result.returncode == 0
    lines = env.call_lines()
    assert len(lines) == 3
    assert "--skip-deps" not in lines[2]
    assert len(env.wipes()) == 1  # only --force: the first run found no volume to wipe


def test_unknown_argument_is_rejected(env):
    assert env.run("--bogus").returncode == 2
    assert env.call_lines() == []


def test_rebuilt_image_with_same_tag_is_not_reused_and_wipes_the_volume(env):
    env.run()
    env.set_image("sha256:image-two")
    result = env.run()
    assert result.returncode == 0
    assert "REUSED" not in result.stdout
    assert len(env.call_lines()) == 2
    assert "image changed" in result.stdout


def test_failed_forced_run_removes_the_earlier_pass(env):
    env.run()
    assert len(env.stamps("pass_")) == 1
    failed = env.run("--force", STUB_RC="3")
    assert failed.returncode == 3
    assert env.stamps("pass_") == []
    assert "REUSED" not in env.run().stdout


def test_failed_run_records_nothing(env):
    assert env.run(STUB_RC="1").returncode == 1
    assert env.stamps() == []


def test_killed_run_records_nothing_and_its_lock_is_taken_over(env):
    result = env.run(STUB_KILL_PARENT="1")
    assert result.returncode != 0
    assert env.stamps() == []
    assert (env.state() / "lock").exists()  # a kill -9 leaves the lock behind
    again = env.run()
    assert again.returncode == 0, again.stderr
    assert "taking over the lock" in again.stdout
    assert len(env.stamps("pass_")) == 1
    assert not (env.state() / "lock").exists()


def test_killed_forced_run_does_not_leave_the_old_pass(env):
    env.run()
    assert env.run("--force", STUB_KILL_PARENT="1").returncode != 0
    assert env.stamps("pass_") == []


# ---- Regression cases: stale objects in the build volume, concurrent worktrees, a branch switched
# during the run, hidden skip-worktree edits; and the snapshot


def test_older_failing_tree_after_a_passing_one_fails(env):
    """Mtime false pass: a stale object in the volume must not hide a failing tree."""
    git(env.repo, "switch", "-q", "-c", "bad")
    set_value(env, "fail", "bad")
    past = time.time() - 3600
    os.utime(env.repo / "value.txt", (past, past))
    git(env.repo, "switch", "-q", "main")
    assert env.run().returncode == 0
    git(env.repo, "switch", "-q", "bad")
    result = env.run()
    assert result.returncode != 0
    assert len(env.stamps("pass_")) == 1  # only the passing tree
    git(env.repo, "switch", "-q", "main")
    assert "REUSED" in env.run().stdout
    # The reverse order and A, B, A, with --force to ignore the stamps.
    git(env.repo, "switch", "-q", "bad")
    assert env.run("--force").returncode != 0
    git(env.repo, "switch", "-q", "main")
    assert env.run("--force").returncode == 0


def test_two_worktrees_run_one_at_a_time_and_each_gets_its_own_result(env):
    worktree = env.root / "other"
    git(env.repo, "worktree", "add", "-q", "-b", "bad", str(worktree))
    (worktree / "value.txt").write_text("fail\n")
    git(worktree, "commit", "-q", "-am", "bad")
    first, second = env.run_overlapping(second_repo=worktree)
    assert first.returncode == 0
    assert second.returncode != 0
    assert "waiting for the run in" in second.stdout
    assert len(env.call_lines()) == 2


def test_waiting_run_reuses_the_pass_recorded_by_the_run_it_waited_for(env):
    first, second = env.run_overlapping()
    assert first.returncode == 0
    assert "waiting for the run in" in second.stdout
    assert "REUSED" in second.stdout
    assert len(env.call_lines()) == 1


def test_branch_switch_during_the_run_does_not_change_the_checked_tree(env):
    git(env.repo, "switch", "-q", "-c", "good")
    bump(env, "good")
    git(env.repo, "switch", "-q", "main")
    set_value(env, "fail", "bad")
    bad_tree = git(env.repo, "rev-parse", "HEAD^{tree}")
    hook = f"cd '{env.repo}' && git switch -q good && git switch -q main"
    result = env.run(STUB_HOOK=hook)
    assert result.returncode != 0  # the failing tree's real result
    assert env.stamps() == []
    # A passing start tree records a stamp for that tree even if HEAD moves during the run.
    git(env.repo, "switch", "-q", "good")
    good_tree = git(env.repo, "rev-parse", "HEAD^{tree}")
    hook = f"cd '{env.repo}' && git switch -q main && git switch -q good"
    assert env.run(STUB_HOOK=hook).returncode == 0
    assert [p.name.split("_")[1] for p in env.stamps("pass_")] == [good_tree]
    assert bad_tree != good_tree


@pytest.mark.parametrize("flag", ["skip-worktree", "assume-unchanged"])
def test_hidden_local_edits_count_as_dirty(env, flag):
    set_value(env, "fail", "bad")
    git(env.repo, "update-index", f"--{flag}", "value.txt")
    (env.repo / "value.txt").write_text("pass\n")
    assert git(env.repo, "status", "--porcelain") == ""
    result = env.run()
    assert "dirty" in result.stdout
    assert env.stamps() == []
    assert env.run().returncode == result.returncode  # no reuse either time
    assert len(env.call_lines()) == 2


def test_container_modifying_the_snapshot_is_not_recorded(env):
    result = env.run(STUB_HOOK='echo tampered >> "${ROOT}/code.cpp"')
    assert result.returncode != 0
    assert "snapshot changed" in result.stderr
    assert env.stamps() == []


def test_stale_snapshot_is_repaired_before_the_run(env):
    set_value(env, "fail", "bad")
    assert env.run().returncode != 0
    (env.state() / "src" / "value.txt").write_text("pass\n")  # tampered, looks passing
    result = env.run()
    assert result.returncode != 0
    assert (env.state() / "src" / "value.txt").read_text() == "fail\n"
    assert env.stamps() == []


def test_snapshot_leftover_files_are_removed(env):
    env.run()
    (env.state() / "src" / "leftover.txt").write_text("x\n")
    (env.state() / "src" / "out").mkdir(exist_ok=True)
    bump(env, "second")
    env.run()
    assert not (env.state() / "src" / "leftover.txt").exists()


def test_deleted_file_wipes_the_volume_and_added_file_does_not(env):
    env.run()
    wipes = len(env.wipes())
    commit_file(env.repo, "added.txt", "x\n", "add")
    env.run()
    assert len(env.wipes()) == wipes
    assert (env.state() / "src" / "added.txt").exists()
    git(env.repo, "rm", "-q", "code.cpp")
    git(env.repo, "commit", "-q", "-m", "delete")
    result = env.run()
    assert "files were deleted" in result.stdout
    assert len(env.wipes()) == wipes + 1
    assert not (env.state() / "src" / "code.cpp").exists()


def test_clock_skew_wipes_the_volume(env):
    env.run()
    bump(env, "second")
    result = env.run(STUB_CONTAINER_DATE=str(int(time.time()) + 60))
    assert "clock differs" in result.stdout


def test_interrupted_run_forces_a_wipe_next_time(env):
    env.run()
    bump(env, "second")
    env.run(STUB_KILL_PARENT="1")
    bump(env, "third")
    assert "no record of the last build" in env.run().stdout


# ---- Dockerfile


def test_image_without_the_baked_inputs_is_refused_before_the_build(env):
    shutil.rmtree(env.image_inputs)
    result = env.run()
    assert result.returncode == 1
    assert "docker compose build" in result.stderr
    assert env.call_lines() == []


def test_image_built_from_another_dockerfile_is_refused(env):
    commit_file(env.repo, "docker/Dockerfile", "FROM scratch\nRUN true\n", "dockerfile")
    result = env.run()
    assert result.returncode == 1
    assert env.call_lines() == []


@pytest.mark.parametrize(
    "change",
    ["glim patch", "glim_ros2 patch", "requirements", "entrypoint", "patch added", "patch removed"],
)
def test_changed_image_input_without_a_rebuild_is_refused_and_a_rebuild_fixes_it(env, change):
    patches = "third_party/patches"
    if change == "glim patch":
        commit_file(env.repo, f"{patches}/glim/fix.patch", "changed\n")
    elif change == "glim_ros2 patch":
        commit_file(env.repo, f"{patches}/glim_ros2/fix.patch", "changed\n")
    elif change == "requirements":
        commit_file(env.repo, "scripts/requirements.txt", "pytest==2\n")
    elif change == "entrypoint":
        commit_file(env.repo, "docker/entrypoint.sh", "#!/bin/bash\nexit 0\n")
    elif change == "patch added":
        commit_file(env.repo, f"{patches}/glim/new.patch", "new\n")
    else:
        git(env.repo, "rm", "-q", f"{patches}/glim_ros2/fix.patch")
        git(env.repo, "commit", "-q", "-m", "remove patch")
    result = env.run()
    assert result.returncode == 1
    assert "docker compose build" in result.stderr
    assert env.call_lines() == []
    assert env.stamps("pass_") == []
    env.rebuild_image_inputs()
    env.set_image("sha256:image-two")  # a rebuilt image has a new ID
    assert env.run().returncode == 0
    assert len(env.stamps("pass_")) == 1


def test_every_dockerfile_copy_source_is_a_listed_image_input():
    script = (SCRIPTS / "image_inputs_hash.sh").read_text()
    listed = re.search(r"image_inputs=\(\n(.*?)\n\)", script, re.S).group(1).split()
    assert listed == INPUTS
    dockerfile = (SCRIPTS.parent / "docker" / "Dockerfile").read_text()
    sources = []
    for line in dockerfile.replace("\\\n", " ").splitlines():
        parts = line.split()
        if parts and parts[0].upper() in ("COPY", "ADD") and not any(a.startswith("--from=") for a in parts):
            args = [a for a in parts[1:] if not a.startswith("--")]
            sources += [a.rstrip("/") for a in args[:-1]]
    assert sources, "the Dockerfile has no COPY lines"
    outside = [x for x in sources if x not in listed]
    assert not outside, f"COPY sources missing from image_inputs in image_inputs_hash.sh: {outside}"
    for path in listed:  # and each one is baked at the same path under inputs/
        assert f"/etc/glim_localization/inputs/{path}" in dockerfile, path


def test_image_inputs_hash_is_the_hash_of_the_manifest_and_tracks_only_image_inputs(env):
    script = env.repo / "scripts" / "image_inputs_hash.sh"

    def run(*args):
        return subprocess.run(["bash", str(script), *args], cwd=env.root, capture_output=True, text=True, check=True).stdout

    manifest = run("--manifest")
    assert [line.split("  ")[1] for line in manifest.splitlines()] == [
        "./docker/Dockerfile", "./docker/entrypoint.sh", "./scripts/requirements.txt",
        "./third_party/patches/glim/fix.patch", "./third_party/patches/glim_ros2/fix.patch",
    ]
    first = run().strip()
    assert re.fullmatch(r"[0-9a-f]{64}", first)
    assert first == run("HEAD").strip()
    base = git(env.repo, "rev-parse", "HEAD^{tree}")
    commit_file(env.repo, "code.cpp", "int main() { return 1; }\n", "source only")
    assert run().strip() == first
    commit_file(env.repo, "third_party/patches/glim/new.patch", "new\n", "patch added")
    assert run().strip() != first
    assert run(base).strip() == first
    assert run("--manifest", base) == manifest


# ---- Build definitions and root-owned files


def test_changed_cmake_default_wipes_the_volume_and_a_source_change_does_not(env):
    assert env.run().returncode == 0
    commit_file(env.repo, "code.cpp", "int main() { return 1; }\n", "source only")
    result = env.run()
    assert result.returncode == 0
    assert "build definition" not in result.stdout
    assert env.wipes() == []
    commit_file(env.repo, "CMakeLists.txt", "fail\n", "option default")
    result = env.run()
    assert "build definition" in result.stdout
    assert len(env.wipes()) == 1
    assert result.returncode != 0  # a fresh build sees the new default
    assert len(env.stamps("pass_")) == 2  # the first two trees only


def test_root_owned_files_in_the_snapshot_do_not_break_the_next_run(env):
    hook = (
        'mkdir -p "$ROOT/out/e2e" "$ROOT/.cache"; echo x > "$ROOT/out/e2e/f"; echo x > "$ROOT/.cache/f"; '
        'chmod 500 "$ROOT/out/e2e" "$ROOT/.cache"'
    )
    assert env.run(STUB_HOOK=hook).returncode == 0
    bump(env, "second")
    result = env.run()
    assert result.returncode == 0, result.stderr
    assert any("--entrypoint chown" in line for line in env.docker_calls())


def test_docker_runs_use_the_image_id_resolved_at_the_start_not_the_tag(env):
    """A tag moved to another image after the ID was resolved must not change what is read or cached."""
    other = env.root / "other_image_inputs"
    other.mkdir()  # the moved tag points at an image without the baked inputs
    result = env.run(STUB_TAG_INPUTS=str(other))
    assert result.returncode == 0, result.stderr
    assert len(env.stamps("pass_")) == 1
    runs = [line for line in env.docker_calls() if line.startswith("docker run")]
    assert runs and all("sha256:image-one" in line for line in runs)
    cached = (env.state() / "image_inputs").glob("*")
    assert all("image_one" in p.name for p in cached)


def test_container_runs_from_the_resolved_image_even_if_the_tag_moves_a_b_a(env):
    """The tag moves to another image before the container starts and back during the run."""
    move_away = f"echo sha256:image-two > '{env.image_file}'"
    move_back = f"echo sha256:image-one > '{env.image_file}'"
    result = env.run(STUB_PS_HOOK=move_away, STUB_HOOK=move_back)
    assert result.returncode == 0, result.stderr
    ((ran_id, ran_ref),) = env.ran_images()
    assert ran_id == "sha256:image-one"  # the image whose inputs were checked, not the moved tag
    assert ran_ref != "glim_localization:dev"
    assert ran_ref.startswith("glim_localization:ci-")


def test_private_run_tag_is_removed_after_a_run_and_after_a_failed_run(env):
    assert env.run().returncode == 0
    assert list(env.tags.iterdir()) == []
    bump(env, "second")
    assert env.run(STUB_RC="1").returncode == 1
    assert list(env.tags.iterdir()) == []
    assert any(line.startswith("docker rmi glim_localization:ci-") for line in env.docker_calls())
    assert not any("rmi glim_localization:dev" in line for line in env.docker_calls())


def test_private_run_tag_is_removed_only_after_the_post_run_chown(env):
    assert env.run().returncode == 0
    calls = env.docker_calls()
    last_chown = max(i for i, line in enumerate(calls) if "--entrypoint chown" in line)
    rmi = [i for i, line in enumerate(calls) if line.startswith("docker rmi glim_localization:ci-")]
    assert rmi and min(rmi) > last_chown


def test_private_run_tag_is_removed_when_the_run_is_terminated(env):
    result = env.run(STUB_TERM_PARENT="1")
    assert result.returncode == 143
    assert list(env.tags.iterdir()) == []
    assert env.stamps("pass_") == []


def test_container_clock_run_uses_the_image_id(env):
    assert env.run().returncode == 0
    bump(env, "second")  # a recorded build exists now, so the clock is read
    assert env.run().returncode == 0
    dates = [line for line in env.docker_calls() if "--entrypoint date" in line]
    assert dates and all("sha256:image-one" in line for line in dates)


def test_failed_chown_after_a_failed_run_keeps_the_exit_code_and_records_nothing(env):
    result = env.run(STUB_RC="3", STUB_CHOWN_FAIL="1")
    assert result.returncode == 3
    assert "chown" in result.stderr
    assert env.stamps("pass_") == []


def test_failed_chown_fails_the_run_and_records_nothing(env):
    result = env.run(STUB_CHOWN_FAIL="1")
    assert result.returncode != 0
    assert "chown" in result.stderr
    assert env.stamps("pass_") == []
    bump(env, "second")
    assert env.run().returncode == 0  # the failure left nothing behind


# ---- Dirty tree


@pytest.mark.parametrize("dirt", ["modify", "untracked", "staged"])
def test_dirty_tree_never_reuses_or_records(env, dirt):
    env.run()
    if dirt == "modify":
        (env.repo / "code.cpp").write_text("int changed;\n")
    elif dirt == "untracked":
        (env.repo / "extra.txt").write_text("x\n")
    else:
        (env.repo / "extra.txt").write_text("x\n")
        git(env.repo, "add", "extra.txt")
    before = env.stamps()
    result = env.run()
    assert result.returncode == 0
    assert "REUSED" not in result.stdout
    assert len(env.call_lines()) == 2
    assert str(env.repo) in env.call_lines()[1]  # the working tree, not the snapshot
    assert env.stamps() == before


def test_ignored_files_do_not_make_the_tree_dirty(env):
    (env.repo / "out").mkdir(exist_ok=True)
    (env.repo / "out" / "log.txt").write_text("x\n")
    env.run()
    assert "REUSED" in env.run().stdout


# ---- Stamps


@pytest.mark.parametrize("content", ["", "garbage", "v1 tree=wrong", None])
def test_corrupted_stamp_is_not_a_pass(env, content):
    env.run()
    (stamp,) = env.stamps("pass_")
    if content is None:
        stamp.write_bytes(stamp.read_bytes()[:-3])
    else:
        stamp.write_text(content)
    result = env.run()
    assert "REUSED" not in result.stdout
    assert len(env.call_lines()) == 2
    assert "REUSED" in env.run().stdout


def test_stamp_for_another_image_in_the_right_file_is_not_a_pass(env):
    env.run()
    (stamp,) = env.stamps("pass_")
    stamp.write_text(stamp.read_text().replace("image-one", "image-two"))
    assert "REUSED" not in env.run().stdout


# ---- Dependency check


def test_dependency_check_is_skipped_once_per_image(env):
    env.run()
    assert "--skip-deps" not in env.call_lines()[0]
    bump(env, "second")
    env.run()
    assert "--skip-deps" in env.call_lines()[1]


def test_dependency_check_runs_when_its_inputs_or_the_image_change(env):
    env.run()
    commit_file(env.repo, "scripts/check_deps.sh", "# deps v2\n")
    env.run()
    assert "--skip-deps" not in env.call_lines()[1]
    commit_file(env.repo, "scripts/requirements.in", "pytest==2\n")
    env.run()
    assert "--skip-deps" not in env.call_lines()[2]
    commit_file(env.repo, "scripts/check_deps/main.cpp", "int y;\n")
    env.run()
    assert "--skip-deps" not in env.call_lines()[3]
    env.set_image("sha256:image-two")
    bump(env, "second")
    env.run()
    assert "--skip-deps" not in env.call_lines()[4]


def test_failed_run_forgets_the_dependency_pass(env):
    env.run()
    bump(env, "second")
    env.run(STUB_RC="1")  # skipped deps and failed: the pass stays
    bump(env, "third")
    env.run("--force", STUB_RC="1")  # ran deps and failed: forgotten
    bump(env, "fourth")
    env.run()
    assert "--skip-deps" not in env.call_lines()[3]


def test_corrupted_dependency_stamp_runs_the_check(env):
    env.run()
    (stamp,) = env.stamps("deps_")
    stamp.write_text("")
    bump(env, "second")
    env.run()
    assert "--skip-deps" not in env.call_lines()[1]


# ---- Docs-only


def commit_on_branch(env, name, text):
    git(env.repo, "switch", "-q", "-c", "topic")
    commit_file(env.repo, name, text)


def test_docs_only_change_skips_and_is_not_recorded(env):
    commit_on_branch(env, "docs/notes.md", "text\n")
    result = env.run()
    assert result.returncode == 0
    assert "SKIPPED" in result.stdout and "docs-only" in result.stdout
    assert env.call_lines() == []
    assert env.stamps() == []
    git(env.repo, "update-ref", "refs/remotes/origin/main", "HEAD")  # the diff is empty now
    again = env.run()
    assert "SKIPPED" not in again.stdout and "REUSED" not in again.stdout
    assert len(env.call_lines()) == 1


def test_empty_diff_is_not_docs_only(env):
    assert "SKIPPED" not in env.run().stdout
    assert len(env.call_lines()) == 1


def test_code_change_with_docs_and_moving_code_into_docs_run_full_checks(env):
    commit_on_branch(env, "README.md", "text\n")
    commit_file(env.repo, "code.cpp", "int z;\n")
    assert "SKIPPED" not in env.run().stdout
    git(env.repo, "switch", "-q", "main")
    git(env.repo, "switch", "-q", "-c", "mv")
    (env.repo / "docs").mkdir()
    git(env.repo, "mv", "code.cpp", "docs/code.md")
    git(env.repo, "commit", "-q", "-m", "move")
    assert "SKIPPED" not in env.run().stdout
    assert len(env.call_lines()) == 2


def test_dirty_tree_or_missing_base_is_never_docs_only_skipped(env):
    commit_on_branch(env, "README.md", "text\n")
    (env.repo / "code.cpp").write_text("int dirty;\n")
    assert "SKIPPED" not in env.run().stdout
    git(env.repo, "checkout", "-q", "--", "code.cpp")
    assert "SKIPPED" not in env.run(CI_LOCAL_BASE_REF="refs/remotes/origin/missing").stdout
    assert len(env.call_lines()) == 2


def test_missing_image_is_an_error(env):
    result = env.run(CI_LOCAL_DOCKER="false")
    assert result.returncode != 0
    assert env.call_lines() == []


# ---- ci.sh itself: the dependency check is unconditional unless --skip-deps is given.
@pytest.fixture
def ci_dir(tmp_path):
    directory = tmp_path / "ci_scripts"
    directory.mkdir()
    shutil.copy(SCRIPTS / "ci.sh", directory / "ci.sh")
    log = tmp_path / "ci.log"
    for name in ("check_deps", "build_core", "build_colcon", "test_colcon", "test_interfaces", "test_python", "test_replay_e2e"):
        path = directory / f"{name}.sh"
        path.write_text(f'#!/bin/bash\necho {name} >> "{log}"\n')
        path.chmod(0o755)
    return directory, log


def run_ci(ci_dir, *args):
    directory, _ = ci_dir
    return subprocess.run(["bash", str(directory / "ci.sh"), *args], capture_output=True, text=True, check=False)


def test_ci_runs_the_dependency_check_by_default(ci_dir):
    assert run_ci(ci_dir).returncode == 0
    assert ci_dir[1].read_text().splitlines()[0] == "check_deps"


def test_ci_runs_test_colcon_after_build_colcon(ci_dir):
    assert run_ci(ci_dir).returncode == 0
    steps = ci_dir[1].read_text().splitlines()
    assert "test_colcon" in steps
    assert steps.index("build_colcon") < steps.index("test_colcon") < steps.index("test_interfaces")


def test_ci_skips_the_dependency_check_only_on_request(ci_dir):
    result = run_ci(ci_dir, "--skip-deps")
    assert result.returncode == 0
    assert "check_deps" not in ci_dir[1].read_text().splitlines()
    assert "SKIPPED" in result.stdout


def test_ci_rejects_unknown_arguments(ci_dir):
    assert run_ci(ci_dir, "--skip").returncode == 2
