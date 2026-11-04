"""Prepared stock model for the runtime student -> stock switch (runtime_guard).

Two things a stock model built at the moment of the switch lacks (worker measurement, switch2135):
  * build time: ModelState() for the stock PKL is built while no model output exists (worker CPU 0.41-0.44 s
    construction + 0.53 s first run, on top of the dropped frame). A StockStandby built at modeld start removes the
    construction from the switch, and warm() runs it once after the first published frame (then ModelState.reset), so
    the switch does not pay the slow first run either (real modeld process on worker: first stock run 0.68-0.74 s,
    later runs 0.16-0.24 s).
  * image history: the stock policy samples the current frame and the frame frame_skip (4) runs earlier from
    img_q / big_img_q. A fresh model has zeros there, so its first 4 outputs see a black past frame
    (worker ci_route: instantaneous desiredAcceleration 0.61-0.69 m/s^2 vs 0.24-0.45 for a stock model running
    from the start; raw outputs differ by up to 270). The student / direct model keeps the last warped frames
    (ModelState.keep_warp_history); they are copied into the stock image queues, so from the first stock output
    the past frame is the real camera image, as in a continuously running stock model.
Not carried over: the stock feature history (feat_q, produced only by running the stock policy) stays zero, and
modeld's prev_action is kept as before (runtime_guard docstring).
A stock -> stock rebuild (stock failure) does not seed: a stock model keeps no warp history, so it starts fresh as
before.
"""


class StockStandby:
  def __init__(self, build, log=None):
    self.build = build  # () -> fresh stock ModelState
    self.log = log
    self.spare = None
    self.warmed = False
    self.seeded = 0

  def prepare(self):
    if self.spare is None:
      self.spare = self.build()
    return self.spare

  @property
  def needs_warm(self) -> bool:
    return self.spare is not None and not self.warmed

  def warm(self, bufs, transforms, inputs):
    """Run the prepared model once on a real frame, then clear every queue/state it filled (ModelState.reset), so it
    starts the switch exactly like a fresh model but without the slow first run."""
    if not self.needs_warm:
      return False
    self.spare.run(bufs, transforms, {k: v.copy() for k, v in inputs.items()}, False)
    self.spare.reset()
    self.warmed = True
    return True

  @property
  def history_len(self) -> int:
    m = self.prepare()
    q = getattr(m, 'input_queues', {}).get('img_q')
    return int(q.shape[0]) if q is not None else 0

  def take(self, source=None):
    """Fresh stock model for a switch. Uses the prepared one if any (it never ran), else builds one now.
    If `source` kept warp history (student/direct), its frames seed the stock image queues."""
    prepared = self.spare is not None
    model, self.spare = (self.spare, None) if prepared else (self.build(), None)
    self.seeded = 0
    hist = getattr(source, 'warp_history', None) if source is not None else None
    if hist and hasattr(model, 'seed_image_history'):
      self.seeded = model.seed_image_history(list(hist))
    if self.log:
      self.log(f"stock model for switch: prepared={prepared} seeded_frames={self.seeded}")
    return model
