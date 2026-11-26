"""Tests for the generator of the site home page (docs/index.md from README.md)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import docs_site_index  # noqa: E402

BLOB = "https://github.com/atinfinity/glim_localization/blob/main/"


def body(markdown):
    out = docs_site_index.convert(markdown)
    assert out.startswith(docs_site_index.HEADER)
    return out[len(docs_site_index.HEADER):]


def test_docs_links_become_relative_to_docs():
    assert body("[Build](docs/build.md)\n") == "[Build](build.md)\n"
    assert body("[Yaw](docs/ros2.md#initial-pose-yaw-search)\n") == "[Yaw](ros2.md#initial-pose-yaw-search)\n"
    assert body('<img src="docs/media/a.gif" width="720">\n') == '<img src="media/a.gif" width="720">\n'


def test_other_repository_paths_become_github_urls():
    assert body("[CFF](CITATION.cff)\n") == f"[CFF]({BLOB}CITATION.cff)\n"
    assert body("[C](./CONTRIBUTING.md)\n") == f"[C]({BLOB}CONTRIBUTING.md)\n"
    assert body("[W](.github/workflows/docs.yml)\n") == f"[W]({BLOB}.github/workflows/docs.yml)\n"


def test_urls_anchors_and_code_blocks_are_kept():
    text = (
        "[GLIM](https://github.com/koide3/glim) [L](#limitations) [M](mailto:a@example.invalid)\n"
        "```bash\n"
        "echo '[x](docs/build.md)'\n"
        "```\n"
    )
    assert body(text) == text


def test_tilde_fences_and_inline_code_are_kept():
    text = (
        "~~~\n"
        "[x](docs/build.md)\n"
        "```\n"  # A backtick line inside a ~~~ fence does not close it
        "[y](docs/build.md)\n"
        "~~~\n"
    )
    assert body(text) == text
    assert body("see `[e](docs/build.md)` and [f](docs/build.md)\n") == "see `[e](docs/build.md)` and [f](build.md)\n"
    assert body("``a ` [e](docs/x.md)`` [f](docs/x.md)\n") == "``a ` [e](docs/x.md)`` [f](x.md)\n"


def test_root_absolute_paths_are_repository_paths():
    assert body("[C](/CONTRIBUTING.md)\n") == f"[C]({BLOB}CONTRIBUTING.md)\n"
    assert body("[B](/docs/build.md)\n") == "[B](build.md)\n"


def test_repository_readme_converts():
    readme = os.path.join(os.path.dirname(__file__), "..", "..", "README.md")
    with open(readme, encoding="utf-8") as handle:
        out = body(handle.read())
    assert "](docs/" not in out and 'src="docs/' not in out
