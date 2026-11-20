# PaperCrop CLI - Windows x64 build

This repository builds a native C++17/OpenCV PaperCrop command-line engine for integration with the Python/PySide6 frontend.

GitHub Actions publishes the `PaperCrop-CLI-Windows-x64` artifact containing `native/bin/papercrop_cli.exe` and required OpenCV DLLs. The included source archive is extracted by the build workflow.

The `papercrop_cli.exe` is produced by GitHub Actions; this repository does not contain a precompiled executable.
