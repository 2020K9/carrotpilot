"""Device recurrence, adapted from zoompilot/jetlink d26001b TrtEngine.

Keep Carrot's all-state finite checks: scan IEEE exponent bits on the GPU and
copy one status word before the reply event. Never infer validity from just the
driving output. NVRTC/driver errors refuse activation instead of bypassing validation.
"""
import ctypes

import numpy as np

from jetlink.server.backends.trt import cudart


FINITE_SOURCE = r'''
extern "C" __global__ void check_finite(const void *data, unsigned long long n,
                                      unsigned int width, unsigned int *bad) {
  for (unsigned long long i = (unsigned long long)blockIdx.x * blockDim.x + threadIdx.x;
       i < n; i += (unsigned long long)gridDim.x * blockDim.x) {
    unsigned int bits = width == 2 ? ((const unsigned short *)data)[i] : ((const unsigned int *)data)[i];
    unsigned int mask = width == 2 ? 0x7c00u : 0x7f800000u;
    if ((bits & mask) == mask) atomicOr(bad, 1u);
  }
}
'''


def _checked(result):
  error, *values = result
  if int(error) != 0:
    raise RuntimeError(f'CUDA device-state operation failed: {error}')
  return values[0] if len(values) == 1 else tuple(values)


class FiniteKernel:
  def __init__(self):
    try:
      from cuda.bindings import driver, nvrtc
    except ImportError:
      from cuda import cuda as driver, nvrtc
    self.driver = driver
    device = cudart.check(cudart._rt.cudaGetDevice())
    _, major, minor = cudart.device_name(device)
    program = _checked(nvrtc.nvrtcCreateProgram(FINITE_SOURCE.encode(), b'carrot_state.cu', 0, [], []))
    try:
      _checked(nvrtc.nvrtcCompileProgram(program, 1, [f'--gpu-architecture=compute_{major}{minor}'.encode()]))
      size = _checked(nvrtc.nvrtcGetPTXSize(program))
      ptx = b'\0' * size
      _checked(nvrtc.nvrtcGetPTX(program, ptx))
    finally:
      _checked(nvrtc.nvrtcDestroyProgram(program))
    self.module = _checked(driver.cuModuleLoadData(ptx))
    try:
      self.function = _checked(driver.cuModuleGetFunction(self.module, b'check_finite'))
    except BaseException:
      self.close()
      raise

  def launch(self, binding, bad: int, stream: int):
    count = binding.nbytes // binding.dtype.itemsize
    args = (ctypes.c_uint64(binding.device_ptr), ctypes.c_uint64(count),
            ctypes.c_uint(binding.dtype.itemsize), ctypes.c_uint64(bad))
    pointers = (ctypes.c_void_p * len(args))(*(ctypes.addressof(a) for a in args))
    _checked(self.driver.cuLaunchKernel(self.function, min(4096, (count + 255) // 256), 1, 1,
                                      256, 1, 1, 0, stream, ctypes.addressof(pointers), 0))

  def close(self):
    if self.module is not None:
      _checked(self.driver.cuModuleUnload(self.module))
      self.module = None


class DeviceState:
  def __init__(self, inputs, outputs, pairs, stream, kernel_factory=FiniteKernel):
    if not pairs or len(set(pairs.values())) != len(pairs):
      raise ValueError('Device state requires distinct, nonempty state pairs')
    self.pairs = []
    for name, output in pairs.items():
      a, b = inputs[name], outputs[output]
      if (a.shape != b.shape or a.dtype != b.dtype or a.nbytes != b.nbytes or a.nbytes <= 0
          or a.dtype not in (np.dtype('float16'), np.dtype('float32'))):
        raise ValueError(f'Invalid device state contract: {name}')
      self.pairs.append((a, b))
    self.input_names = frozenset(pairs)
    self.output_names = frozenset(pairs.values())
    self.stream = stream
    self.kernel = self.bad_device = self.bad_host = self.reply = None
    try:
      self.kernel = kernel_factory()
      self.bad_device = cudart.malloc(4)
      self.bad_host = cudart.host_alloc(4)
      self.bad = (ctypes.c_uint32 * 1).from_address(int(self.bad_host))
      self.bad[0] = 0
      self.reply = cudart.event_create()
    except BaseException:
      self.close()
      raise
    self.needs_reset = True
    # K9 NOT_FINITE tracer (jetlink/server/nf_trace.py): device copies of this frame's input state, taken on the stream
    # right before the graph. Diagnostics only: never read back unless a frame failed; values never fed to a frame.
    self.trace = None

  def trace_enable(self):
    """Allocate the tracer's copy (snapshot) and the live-state save area used around a replay. Idempotent."""
    if self.trace is not None:
      return
    made = []
    try:
      for target, _ in self.pairs:
        made.append((int(cudart.malloc(target.nbytes)), int(cudart.malloc(target.nbytes))))
    except BaseException:
      for a, b in made:
        cudart.free(a)
        cudart.free(b)
      raise
    self.trace = made

  def trace_snapshot(self):
    if self.trace is not None:
      for (snap, _), (target, _) in zip(self.trace, self.pairs, strict=True):
        cudart.memcpy_d2d_async(snap, target.device_ptr, target.nbytes, self.stream)

  def _d2h(self, pointers_bindings):
    out = {}
    for ptr, b in pointers_bindings:
      host = np.empty(b.shape, b.dtype)
      cudart.memcpy_d2h_async(host.ctypes.data, ptr, b.nbytes, self.stream)
      out[b.name] = host
    cudart.stream_sync(self.stream)
    return out

  def trace_read_state(self):
    """Failure path, before the next frame: the failed frame's input state (from the snapshot)."""
    if self.trace is None:
      return {}
    cudart.stream_sync(self.stream)
    return self._d2h([(snap, target) for (snap, _), (target, _) in zip(self.trace, self.pairs, strict=True)])

  def trace_read_outputs(self):
    """The state outputs of the last run, by output name."""
    cudart.stream_sync(self.stream)
    return self._d2h([(source.device_ptr, source) for _, source in self.pairs])

  def trace_begin_replay(self, state):
    """Keep the live input state and flags aside, then load `state` (host arrays by input name) into the inputs."""
    cudart.stream_sync(self.stream)
    for (_, save), (target, _) in zip(self.trace, self.pairs, strict=True):
      cudart.memcpy_d2d_async(save, target.device_ptr, target.nbytes, self.stream)
    kept = (self.needs_reset, int(self.bad[0]))
    loaded = []
    for target, _ in self.pairs:
      value = state.get(target.name)
      if value is None or value.nbytes != target.nbytes:
        cudart.memset_async(target.device_ptr, target.nbytes, self.stream)
      else:
        value = np.ascontiguousarray(value, dtype=target.dtype)
        loaded.append(value)
        cudart.memcpy_h2d_async(target.device_ptr, value.ctypes.data, target.nbytes, self.stream)
    cudart.stream_sync(self.stream)
    self.needs_reset = False
    self.bad[0] = 0
    return kept

  def trace_end_replay(self, kept):
    """Put the live input state and flags back exactly as they were before trace_begin_replay."""
    cudart.stream_sync(self.stream)
    for (_, save), (target, _) in zip(self.trace, self.pairs, strict=True):
      cudart.memcpy_d2d_async(target.device_ptr, save, target.nbytes, self.stream)
    cudart.stream_sync(self.stream)
    self.needs_reset, self.bad[0] = kept[0], kept[1]

  def reset(self):
    # Host only; schedule zeroing before the next graph on the same stream.
    # A previous frame's trailing D2D can never overwrite the new zeros.
    self.needs_reset = True


  def before_run(self):
    if self.needs_reset:
      for target, _ in self.pairs:
        cudart.memset_async(target.device_ptr, target.nbytes, self.stream)
      self.needs_reset = False
    elif self.bad[0]:
      raise FloatingPointError('Non-finite GPU state; reset required')
    if self.trace is not None:
      self.trace_snapshot()

  def enqueue_check(self):
    cudart.memset_async(self.bad_device, 4, self.stream)
    for _, source in self.pairs:
      self.kernel.launch(source, self.bad_device, self.stream)
    cudart.memcpy_d2h_async(self.bad_host, self.bad_device, 4, self.stream)

  def enqueue_feedback(self):
    for target, source in self.pairs:
      cudart.memcpy_d2d_async(target.device_ptr, source.device_ptr, target.nbytes, self.stream)

  def validate(self):
    if self.bad[0]:
      self.reset()
      from jetlink.queues import finite_detail
      # Error only: the flag is aggregate. Read the same frame's output bindings
      # before another inference; trailing same-stream D2D does not modify them.
      for _, b in self.pairs:
        cudart.memcpy_d2h_async(b.host_ptr, b.device_ptr, b.nbytes, self.stream)
      cudart.stream_sync(self.stream)
      for _, b in self.pairs:
        if not np.isfinite(b.host).all():
          raise FloatingPointError('backend=gpu ' + finite_detail(b.name, b.host))
      raise FloatingPointError('backend=gpu reason=detector_disagreement')

  def close(self):
    # Caller synchronizes the owning stream first, including trailing D2D.
    if self.reply is not None:
      cudart.event_destroy(self.reply)
      self.reply = None
    if self.bad_device is not None:
      cudart.free(self.bad_device)
      self.bad_device = None
    if self.bad_host is not None:
      cudart.host_free(self.bad_host)
      self.bad_host = None
    for a, b in getattr(self, 'trace', None) or ():
      cudart.free(a)
      cudart.free(b)
    self.trace = None
    if self.kernel is not None:
      self.kernel.close()
      self.kernel = None
