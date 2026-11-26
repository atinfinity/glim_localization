"""Tests for the docs-only PR classifier used by CI."""
import subprocess
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / "scripts" / "is_docs_only.sh"
# Inputs of the docs site besides Markdown and docs/ (see .github/workflows/docs.yml).
SITE_INPUTS = [
    "zensical.toml",
    "requirements-docs.txt",
    "scripts/docs_site_index.py",
    ".github/workflows/docs.yml",
]


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


@pytest.mark.parametrize("path", SITE_INPUTS)
def test_each_docs_site_input_is_docs_only(path):
    assert is_docs_only([path])
    assert is_docs_only([path, "docs/progress/2026-11-27-entry.md"])


@pytest.mark.parametrize("path", SITE_INPUTS)
def test_docs_site_input_with_code_runs_full_checks(path):
    assert not is_docs_only([path, "glim_localization/src/odometry.cpp"])


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
        ["scripts/other.py"],
        ["scripts/tests/test_docs_site_index.py"],
        ["docs/zensical.toml", "sub/zensical.toml"],
        ["zensical.toml.bak"],
        ["requirements.txt"],
        ["scripts/requirements-docs.txt"],
        [".github/workflows/docs.yaml"],
        [".github/workflows/ci.yml"],
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


def test_moving_code_into_a_docs_site_input_runs_full_checks(tmp_path):
    git(tmp_path, "init", "-q")
    (tmp_path / "code.py").write_text("print('hello')\n" * 20)
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "-q", "-m", "base")
    (tmp_path / "scripts").mkdir()
    git(tmp_path, "mv", "code.py", "scripts/docs_site_index.py")
    git(tmp_path, "commit", "-q", "-m", "move")
    assert not diff_is_docs_only(tmp_path)


def test_docs_site_input_commit_is_docs_only(tmp_path):
    git(tmp_path, "init", "-q")
    (tmp_path / "code.cpp").write_text("int main() { return 0; }\n")
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "-q", "-m", "base")
    (tmp_path / "zensical.toml").write_text("[project]\n")
    (tmp_path / "docs").mkdir()
    (tmp_path / "docs" / "page.md").write_text("text\n")
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "-q", "-m", "site")
    assert diff_is_docs_only(tmp_path)


def load_workflow(name):
    workflow = yaml.safe_load((REPO / ".github" / "workflows" / name).read_text())
    return workflow.get("on", workflow.get(True))  # PyYAML reads the key on as True


def test_push_paths_ignore_matches_the_detector():
    triggers = load_workflow("ci.yml")
    assert triggers["push"]["paths-ignore"] == ["**.md", "docs/**", "LICENSE", *SITE_INPUTS]
    # The required checks must report on every PR, so the PR trigger has no paths filter.
    assert "paths" not in triggers["pull_request"]
    assert "paths-ignore" not in triggers["pull_request"]


def test_every_docs_site_trigger_path_is_docs_only():
    docs = load_workflow("docs.yml")
    paths = docs["push"]["paths"]
    assert docs["pull_request"]["paths"] == paths
    examples = [path.replace("**", "page.md") for path in paths]
    assert is_docs_only(examples)


def test_docs_commit_is_docs_only(tmp_path):
    git(tmp_path, "init", "-q")
    (tmp_path / "code.cpp").write_text("int main() { return 0; }\n")
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "-q", "-m", "base")
    (tmp_path / "README.md").write_text("text\n")
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "-q", "-m", "docs")
    assert diff_is_docs_only(tmp_path)
