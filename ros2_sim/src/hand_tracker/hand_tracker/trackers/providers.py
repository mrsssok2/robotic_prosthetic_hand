"""ONNX Runtime execution-provider choice: CUDA (Linux / NVIDIA), DirectML (Windows, any DX12 GPU), CPU fallback.

Pure functions, no onnxruntime import, so the logic is unit-tested anywhere.
"""
CUDA, DML, CPU = "CUDAExecutionProvider", "DmlExecutionProvider", "CPUExecutionProvider"
DEVICES = ("auto", "cuda", "dml", "cpu")


def select_providers(available, device="auto"):
    """Ordered provider list for ``device``; CPU is always last (fallback). ``available`` = ort.get_available_providers().

    auto = CUDA, then DirectML, then CPU (whatever the installed onnxruntime build offers). An explicit device that this
    build does not offer is skipped, not an error: the caller sees the CPU result via the session's active provider.
    """
    if device not in DEVICES:
        raise ValueError(f"device must be one of {DEVICES}, got {device!r}")
    wanted = {"auto": (CUDA, DML), "cuda": (CUDA,), "dml": (DML,), "cpu": ()}[device]
    return [p for p in wanted if p in available] + [CPU]


def provider_options(providers):
    """Per-provider options: DirectML wants device 0 explicitly."""
    return [{"device_id": 0} if p == DML else {} for p in providers]
