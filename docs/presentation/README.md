# LivingMap presentation

A ten-slide, five-minute English pitch built with Reveal.js. It uses the same
vector figures as the README and technical report, with a memory-survival reveal
and timed speaker notes.

## Open the deck

Open [index.html](index.html) in a desktop browser. The framework, styles and
figures are included locally; the deck needs no internet connection or build step.
GitHub displays the HTML source rather than playing the slides, so download the
repository first. For speaker view, serve the files locally from the repository root:

```bash
python3 -m http.server 8090
```

Open **http://localhost:8090/docs/presentation/**. This is separate from the
simulation dashboard on port 8081 and does not require ROS.

| Control | Action |
|---|---|
| Space / Right arrow | Reveal the next step or slide |
| Left arrow | Previous step or slide |
| Esc | Slide overview |
| F | Full screen |
| S | Speaker notes and timer when served locally |

Reduced-motion preferences disable the decorative packet animation. All figures
have alternative text. [slides.pdf](slides.pdf) is the static ten-page fallback;
it is separate from the required [six-page technical report](../report/report.pdf).

## Five-minute narrative

| Slide | Message | Target time |
|---|---|---:|
| 1 | The environment remembers | 20 s |
| 2 | Writer loss does not delete beacon knowledge | 30 s |
| 3 | The gateway is the communication boundary | 35 s |
| 4 | WHAT / WHERE / WHEN in 33 bytes | 30 s |
| 5 | Wall-aware RF and gateway GPS translation | 30 s |
| 6 | Inherited navigation and write-back | 35 s |
| 7 | Recorded evidence and its scope | 35 s |
| 8 | Failure responses and current limitations | 30 s |
| 9 | Physical prototype plan | 35 s |
| 10 | The next robot starts with knowledge | 20 s |
| **Total** | | **300 s** |

The explanatory beacon graphic on slide 2 is a concept illustration. The mission
map and results charts use the recorded sources identified on each slide.

## Print or export

Use [slides.pdf](slides.pdf), or open
**http://localhost:8090/docs/presentation/?print-pdf**, then print to PDF in Chromium
with landscape layout, backgrounds enabled and no browser headers or footers.
Fragments are combined on one page per slide. Reveal.js documents its
[PDF export workflow](https://revealjs.com/pdf-export/).

The presentation adds no dependency to the robot application. Framework version,
upstream source and license are documented in [vendor/README.md](vendor/README.md).
