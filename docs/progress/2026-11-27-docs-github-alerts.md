# GitHub alerts on the project site

- **Change**: `zensical.toml` enables the `gh-admonitions` plugin. Zensical maps it to `pymdownx.quotes` with `callouts` on (pymdownx is already pinned in `requirements-docs.txt`), so the GitHub alerts in the docs (`> [!NOTE]`, `> [!WARNING]`, ...) render as admonitions on the site. Before, the site showed them as plain blockquotes with the literal `[!WARNING]` text.
- **Why**: the same Markdown reads correctly on GitHub and on the site. Rewriting the alerts as `!!! warning` would render on the site but show as raw text on GitHub.
- **Decisions**: the built-in shim and not the `markdown-callouts` package, which would add a dependency for the same result.
- **Verification** (host, Python 3.12 venv from `requirements-docs.txt`): `python scripts/docs_site_index.py && zensical build --clean --strict` reports no issues. The alerts of `docs/replay.md` become `<div class="admonition warning">` and `<div class="admonition note">`, and no literal `[!` is left on any page of `site/` outside code. The one IMPORTANT alert (`docs/build.md`) renders with the default admonition style, since the theme styles only note, tip and warning.
