# Progress entries

Every change that records verification results, measurements or design decisions adds **one new file** here. Do not append to one shared file: parallel pull requests that append to the same file conflict as soon as one of them merges.

The earlier history is in [../PROGRESS.md](../PROGRESS.md). That file is frozen: do not add new sections to it.

## File name

`YYYY-MM-DD-<slug>.md`, for example `2026-11-25-wheel-odometry.md`:
- the date is the day the entry is written;
- the slug is short and in kebab-case, usually the area and topic.

If two entries would get the same name, make the slug more specific.

## Content

Start with a level-1 heading that names the topic and links the issue or pull request, then use the same sections as before where they apply:

```markdown
# Wheel odometry factor ([#14](https://github.com/atinfinity/glim_localization/issues/14))

- **Change**: what was done and why.
- **Decisions**: design choices and the alternatives that were rejected.
- **Verification**: what was run, where (for example "Docker, arm64"), and the numbers.
- **Open**: what is left, with links to the issues.
```

Write every issue or pull request number as a link. To list the entries in order, sort them by file name, which starts with the date.
