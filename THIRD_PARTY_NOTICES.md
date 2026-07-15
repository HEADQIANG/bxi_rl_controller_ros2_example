# Third-party notices for the SONIC integration

This branch adds a modified subset of NVIDIA GEAR-SONIC for ELF3 SONIC
teleoperation. The subset is located under
`src/bxi_example_py_elf3/bxi_example_py_elf3/sonic_pico/vendor/gear_sonic`.
It has been package-scoped, its headless robot model no longer loads CAD
geometry, and it includes local XRT shutdown and process-cleanup fixes.

GEAR-SONIC source code is Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES
and is provided under Apache License 2.0. See
`third_party/GEAR_SONIC_LICENSE.txt` and `third_party/APACHE-2.0.txt`.

The SONIC ONNX weights are governed by the NVIDIA Open Model License contained
in `third_party/GEAR_SONIC_LICENSE.txt`.

Licensed by NVIDIA Corporation under the NVIDIA Open Model License.

The checked-in ONNX differs from the original export only by removal of ONNX
node `doc_string` fields that contained export-machine stack traces. Its
inference contract is `(1, 1770) -> (1, 29)`, and the cleaned model produced
bit-identical CPU output in the publication check. Its SHA256 is:

`26dc3e96adfb894850b409e43f06178c79b74167c719f626eadbb9df3fcacd06`

`rotation_conversion.py` contains code derived from PyTorch3D, Copyright (c)
Meta Platforms, Inc. and affiliates, under the BSD 3-Clause License. See
`third_party/PYTORCH3D_LICENSE.txt`.

`kornia_transform.py` contains transformations derived from Kornia, which is
distributed under Apache License 2.0.

The XRoboToolkit Python binding and RoboticsService are external runtime
dependencies. No XRT SDK, shared object, DEB package, service executable, or
other vendor binary is distributed in this repository.
