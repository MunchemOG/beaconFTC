# 08 — Release

## Name
Pick something searchable and unclaimed before the first public commit. "ASTRO" collides with the
Astro web framework; "Luma" with PhotonVision's Luma camera. Candidates: `astro-ftc`,
`PhoneVIO-FTC`, `Lantern`. Check GitHub, PyPI and the FTC Discord pins.

## Licence
MIT for our code. Third-party licences preserved in `astro-native/third_party/`. If the estimator
is GPL-3, say so in the README and keep it buildable as a separable component.

## README structure
1. One paragraph: what it is, the accuracy table from `06` (measured), a 60–90 s video link
   (side-by-side with Pinpoint).
2. "Is this legal?" — the table from `07`, verbatim rule numbers.
3. Quick start, Tier 1: phone list, wiring, Gradle snippet, `astro.json`, `AstroTestTeleOp`.
4. Tier 2 build (links to `astro-native/README.md` and `calibration/README.md`).
5. Adapters: Road Runner / Pedro snippets.
6. Troubleshooting (phone-RC Wi-Fi Direct, USB sleep, ARCore install, thermal).
7. Credits: VINS-Fusion / OpenVINS / Basalt authors, ARCore, the original student's idea.

## Checklist
- [ ] All `06` tests logged with numbers in `TESTING.md`
- [ ] Side-by-side video with Pinpoint
- [ ] Inspection packet (`RULES.md`, `WIRING.md`) printable as PDF
- [ ] Example `astro.json` and per-phone calibration YAML
- [ ] CI: JVM unit tests + a native cross-compile job
- [ ] Mirror the repo (GitHub + Codeberg) so it does not vanish like the original
- [ ] Posts: FTC Discord #programming, r/FTC, Chief Delphi, with the measured table — no adjectives
