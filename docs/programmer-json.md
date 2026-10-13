# Put a branch on the Rx programmer page

The page offers the newest commit of the branches `main`, `dev` and `dev-tyler` and every tag of each
firmware repository. No other branch and no older commit is on the page.

`programmer.json` names what the page offers from the branch, each item with a plain label:

```json
{
  "title": "Rover + monitor firmware",
  "description": "rover mode, monitor, telemetry",
  "esp32": { "env": "lolin_s2_mini", "label": "ESP32 rover firmware" },
  "stm32": [
    { "header": "stm32/app_v2.0.20.commit_header",
      "image": "stm32/app_v2.0.20.bin.encrypted",
      "label": "STM32 app" }
  ]
}
```

| Key | What |
|---|---|
| `title` | the name in the firmware menu, in plain words (the page adds the version) |
| `description` | one short line under the name: what the firmware does, no jargon |
| `esp32.env` | the PlatformIO environment to build |
| `esp32.label` | the name on the page button (the page adds the version) |
| `stm32[].header`, `stm32[].image` | the signed, encrypted STM32 update pair, paths from the repository root |
| `stm32[].label` | the name on the page button (the page adds the version from the header) |
| `"stm32": "latest"` | in place of the list: the page offers the newest encrypted pair in `stm32/` (newest by the version `vX.Y.Z` in the file name; the `.commit_header` and the `.bin.encrypted` must have the same name). An STM32 update then needs only the new files in `stm32/`, no edit of this file. No pair in `stm32/`: the branch is not offered |

Every key is optional. Without a title, the menu says "Release vX.Y.Z" for a tag, "Test release vX.Y.Z" for a
tag that ends in `-test.N`, and "Development build" for `main`. The menu lists every entry newest first and
marks the first one "latest", each with its branch or tag and commit in small print. Labels use letters, digits, spaces and `. , : ( ) + / _ -`, at most 60
characters. A file that does not follow this form takes the branch off the page.

The page builds the newest commit of `main`, `dev` and `dev-tyler` and of every tag at each run of its
workflow. An older commit or a deleted tag leaves the page at the next run. A ref with this file offers what
the file names; without it, the default PlatformIO environment and every STM32 update pair in the commit.
