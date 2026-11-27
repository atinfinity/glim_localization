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
SHARED_ARGS = (
    "GLIM_COMMIT", "GTSAM_VERSION", "GTSAM_POINTS_VERSION", "KOIDE3_PPA_KEY_FPR", "GLIM_ROS2_COMMIT",
    "GTSAM_SHA256_AMD64", "GTSAM_SHA256_ARM64", "GTSAM_POINTS_SHA256_AMD64", "GTSAM_POINTS_SHA256_ARM64",
)


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


def test_both_images_pin_the_same_base_by_digest():
    bases = [re.findall(r"^FROM (\S+)", text, re.M) for text in (CPU, CUDA)]
    assert bases[0] == bases[1], bases
    assert len(bases[0]) == 1 and re.fullmatch(r"ros:jazzy-ros-base@sha256:[0-9a-f]{64}", bases[0][0]), bases


def test_sha256_pins_are_full_hashes():
    for name in SHARED_ARGS:
        if "SHA256" in name:
            assert re.fullmatch(r"[0-9a-f]{64}", args(CPU)[name]), name


def test_cpu_lines_appear_in_order_in_the_cuda_dockerfile():
    # Every line of the CPU Dockerfile, except the ones the CUDA variant replaces and the image inputs
    # manifest (each image lists its own), must appear in the same order in the CUDA one. This is a
    # subset check by design: the CUDA-only lines in between are covered by the other tests.
    cpu = CPU.split("\n# Lets scripts/ci_local.sh check", 1)[0]
    # Each replaced line is skipped once, at its first occurrence.
    replaced = [
        "RUN apt-get update \\",  # Only the first RUN: it gains an architecture check before it.
        # The CUDA build of gtsam_points is downloaded, checked against its own SHA256 and installed.
        '&& (cd "${debs}" && apt-get download "libgtsam-notbb-dev=${GTSAM_VERSION}" '
        '"libgtsam-points-dev=${GTSAM_POINTS_VERSION}") \\',
        '&& check_sha256 "libgtsam-points-dev_${GTSAM_POINTS_VERSION}" "GTSAM_POINTS_SHA256_${arch^^}" \\',
        '"${debs}/libgtsam-points-dev_${GTSAM_POINTS_VERSION}_${arch}.deb" \\',
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
    # Only the pinned keys are exported into the apt keyrings.
    assert 'gpg --batch --export "${KOIDE3_PPA_KEY_FPR}" > /etc/apt/keyrings/koide3_ppa.gpg' in CUDA
    assert 'gpg --batch --export "${CUDA_REPO_KEY_FPR}" > /etc/apt/keyrings/cuda_repo.gpg' in CUDA
    # The PPA debs are checked against their pinned SHA256, the CUDA build of gtsam_points against its own.
    assert re.fullmatch(r"[0-9a-f]{64}", args(CUDA)["GTSAM_POINTS_CUDA_SHA256_AMD64"])
    assert args(CUDA)["GTSAM_POINTS_CUDA_SHA256_AMD64"] != args(CPU)["GTSAM_POINTS_SHA256_AMD64"]
    assert ('&& check_sha256 "${GTSAM_POINTS_PACKAGE}_${GTSAM_POINTS_VERSION}" '
            '"GTSAM_POINTS_CUDA_SHA256_${arch^^}"') in CUDA
    assert '"${debs}/${GTSAM_POINTS_PACKAGE}_${GTSAM_POINTS_VERSION}_${arch}.deb"' in CUDA
    assert '"${GTSAM_POINTS_PACKAGE}=${GTSAM_POINTS_VERSION}" \\\n         "cuda-nvcc' not in CUDA


def test_cpu_dockerfile_stays_cpu_only():
    assert "-DBUILD_WITH_CUDA=ON" not in CPU
    # The SHA256 mismatch message names both Dockerfiles.
    assert "cuda" not in CPU.lower().replace("build_with_cuda", "").replace("docker/dockerfile.cuda", "")


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

    missing = run_check(tmp_path / "missing")
    assert missing.returncode == 1 and "is missing" in missing.stderr

    (tmp_path / "empty").mkdir()
    empty = run_check(tmp_path / "empty")
    assert empty.returncode == 1
    assert "is empty" in empty.stderr and "is missing" not in empty.stderr


def test_check_cuda_image_inputs_handles_names_with_spaces(tmp_path):
    # A file with a space in a baked-in directory: the tree and the image both list it whole.
    tree = tmp_path / "tree"
    image = tmp_path / "image"
    (tree / "scripts").mkdir(parents=True)
    shutil.copy(CHECK_INPUTS, tree / "scripts" / CHECK_INPUTS.name)
    for root in (tree, image):
        for path in cuda_image_inputs():
            source = ROOT / path
            if source.is_dir():
                shutil.copytree(source, root / path)
            else:
                (root / path).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy(source, root / path)
        (root / "third_party" / "patches" / "glim" / "a name with spaces.patch").write_text("same\n")
    check = [str(tree / "scripts" / CHECK_INPUTS.name)]
    env = {**os.environ, "GLIM_LOC_IMAGE_INPUTS_DIR": str(image)}
    ok = subprocess.run(check, capture_output=True, text=True, env=env)
    assert ok.returncode == 0, ok.stderr

    (image / "third_party" / "patches" / "glim" / "a name with spaces.patch").write_text("changed\n")
    stale = subprocess.run(check, capture_output=True, text=True, env=env)
    assert stale.returncode == 1
    assert "./third_party/patches/glim/a name with spaces.patch" in stale.stderr
    assert "No such file" not in stale.stderr
