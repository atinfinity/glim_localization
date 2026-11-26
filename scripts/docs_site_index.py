#!/usr/bin/env python3
"""Write the home page of the project site (docs/index.md) from README.md.

The site is built from docs/ (zensical.toml), but its home page is the README at the repository
root, whose links are relative to the root. This copies the README and rewrites its relative links
for docs/: "docs/<path>" becomes "<path>", and any other repository path becomes an absolute GitHub
URL, since the site has only the files under docs/. URLs and in-page anchors stay as they are, and
so does everything inside fenced code blocks.

docs/index.md is generated (ignored by git); edit README.md instead.
Usage: scripts/docs_site_index.py [--readme README.md] [--out docs/index.md]
"""

import argparse
import re
from pathlib import Path

REPO_URL = "https://github.com/atinfinity/glim_localization"
HEADER = "<!-- Generated from README.md by scripts/docs_site_index.py; edit README.md instead. -->\n\n"

# Inline Markdown link or image target "](target)", and the HTML attributes src="..." / href="...".
_MD_LINK = re.compile(r"(\]\()([^)\s]+)(\))")
_HTML_ATTR = re.compile(r"""(\b(?:src|href)=")([^"]+)(")""")
_SCHEME = re.compile(r"^[a-zA-Z][a-zA-Z0-9+.-]*:")


def rewrite_target(target: str) -> str:
    if target.startswith("#") or _SCHEME.match(target):
        return target  # in-page anchor, or a URL with a scheme (https:, mailto:, ...)
    target = target.removeprefix("./")
    if target.startswith("docs/"):
        return target.removeprefix("docs/")
    return f"{REPO_URL}/blob/main/{target}"


def convert(readme: str) -> str:
    def sub(match: re.Match) -> str:
        return match.group(1) + rewrite_target(match.group(2)) + match.group(3)

    lines = []
    in_fence = False
    for line in readme.splitlines(keepends=True):
        if line.lstrip().startswith("```"):
            in_fence = not in_fence
        elif not in_fence:
            line = _HTML_ATTR.sub(sub, _MD_LINK.sub(sub, line))
        lines.append(line)
    return HEADER + "".join(lines)


def main() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--readme", type=Path, default=repo_root / "README.md")
    parser.add_argument("--out", type=Path, default=repo_root / "docs" / "index.md")
    args = parser.parse_args()
    args.out.write_text(convert(args.readme.read_text(encoding="utf-8")), encoding="utf-8")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
