"""Checks that docker/Dockerfile.cuda stays a copy of docker/Dockerfile plus the CUDA changes."""
import re
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
    # manifest that only scripts/ci_local.sh uses, must appear in the same order in the CUDA one.
    cpu = CPU.split("\n# Lets scripts/ci_local.sh check", 1)[0]
    replaced = (
        "RUN apt-get update \\",  # The first RUN gains an architecture check before it.
        '"libgtsam-points-dev=${GTSAM_POINTS_VERSION}" \\',
        "-DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF \\",
        "-DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_CV_BRIDGE=OFF \\",
    )
    cuda_lines = iter(instructions(CUDA))
    for line in instructions(cpu):
        if line in replaced:
            continue
        assert any(line == other for other in cuda_lines), f"missing or out of order in Dockerfile.cuda: {line}"


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
