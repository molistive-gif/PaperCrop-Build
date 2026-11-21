# PaperCrop CLI - Windows x64

This repository builds the native C++17 / OpenCV CLI used by the Python/PySide6 frontend.

## Current A3 split behavior

For **landscape A3** scans, the C++ engine:
1. Splits the input image at the vertical midpoint into two A4 halves.
2. Resizes the **left** half without cropping and saves it to the requested `--output` path.
3. Finds separate bordered photos/documents on the **right** half and saves each cropped result as `_right_01`, `_right_02`, etc.
4. Keeps the old `crop`, `resize`, `combine` CLI functionality for non-A3 inputs.

The default `--a3-mode auto` detects typical landscape A3 scans at around 300 dpi (image width >= 4150 px and height >= 2900 px, ratio near 1.414). **A3 and A4 share an aspect ratio**; there is no reliable physical-size detection without DPI / scanner configuration. Use `--a3-mode force` for known lower-DPI landscape A3 scans, or `--a3-mode off` to bypass the split. Portrait A3 must be rotated landscape first. 

```bat
papercrop_cli.exe crop --input "a3_scan.jpg" --output "output_left.jpg" --max-width 595 --max-height 842 --a3-mode force
```

The right-hand document detector uses border-background contrast and edge contours. Truly invisible photo borders, overlapping items, and scanner artifacts can still need manual review; current tests use synthetic images.

## Building

The GitHub Actions workflow [Build PaperCrop CLI for Windows](../../actions/workflows/build-papercrop-windows.yml) runs on push to `main`, or with **Run workflow**. It uses Windows Server 2022, Visual Studio 2022, and official OpenCV 4.9 binaries, tests the existing commands plus A3/right-multi-crop, then uploads the artifact **PaperCrop-CLI-Windows-x64**. The EXE and DLLs belong under `native/bin/` in the frontend project.

Source extension: `extensions/papercrop_cli.cpp`. The workflow retains upstream `crop.hpp` and the original CLI code from `papercrop-native-source.zip`, compiling the extension as a wrapper.

To make the frontend list all multi-document outputs (not just the left half), update the `backend/processing/image_cropper.py` adapter to collect files matching `<output_stem>_right_*.jpg` after the CLI completes. The optional `PAPERCROP_A3_MODE` setting can be forwarded from `backend/processing/native_engine.py` to `--a3-mode`.
