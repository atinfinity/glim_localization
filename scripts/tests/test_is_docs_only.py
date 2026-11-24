"""Tests for the docs-only PR classifier used by CI."""
import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "is_docs_only.sh"


def is_docs_only(paths):
    text = "".join(f"{path}\0" for path in paths)
    result = subprocess.run(["bash", str(SCRIPT)], input=text, text=True, check=False)
    assert result.returncode in (0, 1)
    return result.returncode == 0


def git(repo, *args):
    subprocess.run(
        ["git", "-c", "user.name=test", "-c", "user.email=test@example.invalid", *args],
        cwd=repo,
        check=True,
        capture_output=True,
    )


def diff_is_docs_only(repo):
    diff = subprocess.run(
        ["git", "diff", "--name-only", "--no-renames", "-z", "HEAD^1", "HEAD"],
        cwd=repo,
        check=True,
        capture_output=True,
    )
    result = subprocess.run(["bash", str(SCRIPT)], input=diff.stdout, check=False)
    return result.returncode == 0


@pytest.mark.parametrize(
    "paths",
    [
        ["README.md"],
        ["docs/DESIGN.md", "docs/images/architecture.png"],
        ["glim_localization/notes.md", "LICENSE"],
        ["docs/name with spaces.md"],
    ],
)
def test_documentation_changes_are_docs_only(paths):
    assert is_docs_only(paths)


@pytest.mark.parametrize(
    "paths",
    [
        [],
        ["scripts/ci.sh"],
        ["README.md", "scripts/ci.sh"],
        ["docs/DESIGN.md", ".github/workflows/ci.yml"],
        ["docker/Dockerfile"],
        ["src/docs/file.cpp"],
        ["LICENSE.txt"],
        ["file.md.in"],
        ["src/a.md\ndocs/b.cpp"],
    ],
)
def test_other_changes_run_full_checks(paths):
    assert not is_docs_only(paths)


def test_moving_code_into_docs_runs_full_checks(tmp_path):
    git(tmp_path, "init", "-q")
    (tmp_path / "code.cpp").write_text("int main() { return 0; }\n" * 20)
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "-q", "-m", "base")
    (tmp_path / "docs").mkdir()
    git(tmp_path, "mv", "code.cpp", "docs/code.md")
    git(tmp_path, "commit", "-q", "-m", "move")
    assert not diff_is_docs_only(tmp_path)


def test_docs_commit_is_docs_only(tmp_path):
    git(tmp_path, "init", "-q")
    (tmp_path / "code.cpp").write_text("int main() { return 0; }\n")
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "-q", "-m", "base")
    (tmp_path / "README.md").write_text("text\n")
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "-q", "-m", "docs")
    assert diff_is_docs_only(tmp_path)
