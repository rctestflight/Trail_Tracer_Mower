# Rx programmer page

A web page that installs firmware on a Trail Tracer Rx board over USB, from Chrome or Edge on a computer.
This repo publishes it on its GitHub Pages: https://rctestflight.github.io/Trail_Tracer_Mower/

The workflow `.github/workflows/programmer-pages.yml` builds the firmware the page offers and publishes the
page. It runs on each push to `main`, on each tag, every 30 minutes (for opt-in branches) and by hand.
Which branches appear: `docs/programmer-json.md`.

Links: `?fw=<repo>/<branch or tag>` opens the page with that firmware chosen (it follows the branch or tag); `&commit=<sha>` pins one exact build. The page shows the link for the current choice under the menu. Files: `tools/programmer/` (page, build, check).

## One-time setup (repo owner)

| Step | Where |
|---|---|
| Pages source "GitHub Actions" (in place of "Deploy from a branch") | Settings, Pages |
| Secret `GATE_LISTS`: the word lists of the site check; without it the workflow publishes nothing | Settings, Secrets and variables, Actions |
| Run "Programmer page" once by hand | Actions |
