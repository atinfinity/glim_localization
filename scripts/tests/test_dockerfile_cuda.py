"""Checks that docker/Dockerfile.cuda stays a copy of docker/Dockerfile plus the CUDA changes."""
import os
import re
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CPU = (ROOT / "docker" / "Dockerfile").read_text()
CUDA = (ROOT / "docker" / "Dockerfile.cuda").read_text()

# Pins that both images and scripts/check_deps.sh share.
SHARED_ARGS = ("GLIM_COMMIT", "GTSAM_VERSION", "GTSAM_POINTS_VERSION", "KOIDE3_PPA_KEY_FPR", "GLIM_ROS2_COMMIT")


def args(text):
    return dict(re.findall(r"^ARG (\w+)=(\S+)$", text, re.M))


def instructions(text):
    """Non-comment, non-blank lines, with surrounding whitespace removed."""
    return [line.strip() for line in text.splitlines() if line.strip() and not line.lstrip().startswith("#")]


def test_shared_pins_are_equal():
    cpu, cuda = args(CPU), args(CUDA)
    check_deps = (ROOT / "scripts" / "check_deps.sh").read_text()
    for name in SHARED_ARGS:
        assert name in cpu, name
        assert cuda.get(name) == cpu[name], name
        pinned = re.search(rf"^{name}=(\S+)$", check_deps, re.M)
        if pinned:
            assert pinned.group(1) == cpu[name], name


def test_cpu_lines_appear_in_order_in_the_cuda_dockerfile():
    # Every line of the CPU Dockerfile, except the ones the CUDA variant replaces and the image inputs
    # manifest (each image lists its own), must appear in the same order in the CUDA one. This is a
    # subset check by design: the CUDA-only lines in between are covered by the other tests.
    cpu = CPU.split("\n# Lets scripts/ci_local.sh check", 1)[0]
    # Each replaced line is skipped once, at its first occurrence.
    replaced = [
        "RUN apt-get update \\",  # Only the first RUN: it gains an architecture check before it.
        '"libgtsam-points-dev=${GTSAM_POINTS_VERSION}" \\',
        "-DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF \\",
        "-DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_CV_BRIDGE=OFF \\",
    ]
    cuda_lines = iter(instructions(CUDA))
    for line in instructions(cpu):
        if line in replaced:
            replaced.remove(line)
            continue
        assert any(line == other for other in cuda_lines), f"missing or out of order in Dockerfile.cuda: {line}"
    assert not replaced, f"replaced lines not found in docker/Dockerfile: {replaced}"


def test_cuda_dockerfile_builds_with_cuda_for_explicit_architectures():
    assert "BUILD_WITH_CUDA=OFF" not in CUDA
    assert CUDA.count("-DBUILD_WITH_CUDA=ON") == 2
    assert CUDA.count('"-DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCHITECTURES}"') == 2
    architectures = args(CUDA)["CUDA_ARCHITECTURES"]
    assert re.fullmatch(r"\d+(;\d+)*", architectures), architectures
    assert "native" not in "\n".join(instructions(CUDA)).replace("MARCH_NATIVE", "")
    assert '"libgtsam-points-dev=' not in CUDA
    assert args(CUDA)["GTSAM_POINTS_PACKAGE"] == "libgtsam-points-cuda13.1-dev"
    assert "GLIM_LOC_BUILD_WITH_CUDA=ON" in CUDA
    # Both signing keys are checked against a pinned fingerprint.
    assert 'if [ "${fprs}" != "${KOIDE3_PPA_KEY_FPR}" ]' in CUDA
    assert 'if [ "${fprs}" != "${CUDA_REPO_KEY_FPR}" ]' in CUDA


def test_cpu_dockerfile_stays_cpu_only():
    assert "-DBUILD_WITH_CUDA=ON" not in CPU
    assert "cuda" not in CPU.lower().replace("build_with_cuda", "")


CHECK_INPUTS = ROOT / "scripts" / "check_cuda_image_inputs.sh"


def cuda_image_inputs():
    body = re.search(r"^cuda_image_inputs=\((.*?)^\)", CHECK_INPUTS.read_text(), re.M | re.S).group(1)
    return body.split()


def test_cuda_image_inputs_match_the_dockerfile_copies():
    copies = re.findall(r"^COPY (\S+?)/? /etc/glim_localization/inputs/(\S+?)/?$", CUDA, re.M)
    assert copies, "Dockerfile.cuda has no inputs manifest"
    assert all(src == dst for src, dst in copies), copies
    assert sorted(src for src, _ in copies) == sorted(cuda_image_inputs())
    # The manifest is the last section, so every earlier COPY of a repository file is in it.
    assert "third_party/patches/glim_cuda" in cuda_image_inputs()


def run_check(inputs_dir, **env):
    return subprocess.run(
        [str(CHECK_INPUTS)], capture_output=True, text=True,
        env={**os.environ, "GLIM_LOC_IMAGE_INPUTS_DIR": str(inputs_dir), **env})


def test_check_cuda_image_inputs_detects_a_stale_image(tmp_path):
    image = tmp_path / "inputs"
    for path in cuda_image_inputs():
        source = ROOT / path
        if source.is_dir():
            shutil.copytree(source, image / path)
        else:
            (image / path).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(source, image / path)
    assert run_check(image).returncode == 0

    (image / "docker" / "Dockerfile.cuda").write_text("stale\n")
    stale = run_check(image)
    assert stale.returncode == 1
    assert "tree: " in stale.stderr and "./docker/Dockerfile.cuda" in stale.stderr
    assert "build dev-cuda" in stale.stderr
    assert run_check(image, GLIM_LOC_ALLOW_STALE_IMAGE="1").returncode == 0

    assert run_check(tmp_path / "missing").returncode == 1
