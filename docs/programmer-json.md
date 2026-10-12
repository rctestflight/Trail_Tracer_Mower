# Put a branch on the Rx programmer page

The page offers the branch `main` and every tag of each firmware repository. Any other branch shows only
when it holds a file `programmer.json` at its root. No file: the branch is not on the page.

`programmer.json` names what the page offers from the branch, each item with a plain label:

```json
{
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
| `esp32.env` | the PlatformIO environment to build |
| `esp32.label` | the name on the page button (the page adds the version) |
| `stm32[].header`, `stm32[].image` | the signed, encrypted STM32 update pair, paths from the repository root |
| `stm32[].label` | the name on the page button (the page adds the version from the header) |

Both parts are optional. Labels use letters, digits, spaces and `. , : ( ) + / _ -`, at most 60
characters. A file that does not follow this form takes the branch off the page.

The page checks the public repositories every 30 minutes and builds each new commit of `main`, of every
tag and of every branch with this file. Remove the file, or delete the branch, and its builds leave the page
at the next check. The same rule applies to `main` and tags when they hold the file: it then names the
items; without it they offer the default PlatformIO environment and every STM32 update pair in the commit.
