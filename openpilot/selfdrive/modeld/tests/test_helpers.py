from types import SimpleNamespace
import pytest

from openpilot.selfdrive.modeld.helpers import select_vision_streams


@pytest.mark.parametrize(
  ("available", "use_wide_camera", "main", "use_extra"),
  [
    ({"road", "wide"}, True, "road", True),
    ({"road", "wide"}, False, "road", False),
    ({"road"}, True, "road", False),
    ({"wide"}, True, "wide", False),
    ({"wide"}, False, None, False),
    (set(), True, None, False),
  ],
)
def test_select_vision_streams(available, use_wide_camera, main, use_extra):
  assert select_vision_streams(available, "road", "wide", use_wide_camera) == (main, use_extra)


def test_custom_usb_controller_waits_for_pcie_link(monkeypatch):
  from tinygrad.runtime.support.usb import CustomASM24Controller

  ltssm_values = iter((bytes([0x00]), bytes([0x00]), bytes([0x78])))
  power_calls = []
  monkeypatch.setattr(CustomASM24Controller, "read", lambda *_args: next(ltssm_values))
  monkeypatch.setattr(CustomASM24Controller, "set_pcie_power", lambda _self, enabled: power_calls.append(enabled))
  monkeypatch.setattr("tinygrad.runtime.support.usb.time.sleep", lambda _seconds: None)

  CustomASM24Controller(object())
  assert power_calls == [True]


def test_custom_usb_controller_preserves_ready_pcie_link(monkeypatch):
  from tinygrad.runtime.support.usb import CustomASM24Controller

  power_calls = []
  monkeypatch.setattr(CustomASM24Controller, "read", lambda *_args: bytes([0x78]))
  monkeypatch.setattr(CustomASM24Controller, "set_pcie_power", lambda _self, enabled: power_calls.append(enabled))

  CustomASM24Controller(object())
  assert power_calls == []


def test_custom_usb_controller_reports_failed_retrain(monkeypatch):
  from tinygrad.runtime.support.usb import CustomASM24Controller

  power_calls = []
  monotonic_values = iter((0.0, 3.0))
  monkeypatch.setattr(CustomASM24Controller, "read", lambda *_args: bytes([0x00]))
  monkeypatch.setattr(CustomASM24Controller, "set_pcie_power", lambda _self, enabled: power_calls.append(enabled))
  monkeypatch.setattr("tinygrad.runtime.support.usb.time.monotonic", lambda: next(monotonic_values))
  monkeypatch.setattr("tinygrad.runtime.support.usb.time.sleep", lambda _seconds: None)

  with pytest.raises(RuntimeError, match="PCIe link not up"):
    CustomASM24Controller(object())

  assert power_calls == [True]


def test_custom_usb_controller_retries_transient_xdata_read(monkeypatch):
  from tinygrad.runtime.support import usb

  controller = usb.CustomASM24Controller.__new__(usb.CustomASM24Controller)
  controller.usb = SimpleNamespace(handle=object())
  controller._f0_out_buf, controller._f0_out_mv = usb.alloc_cbuffer(0x1000)
  attempts = iter((-1, 8))
  sleeps = []

  def control_transfer(_handle, _request_type, _request, _value, _index, buffer, length, _timeout):
    ret = next(attempts)
    if ret == length:
      buffer[:length] = bytes(range(length))
    return ret

  monkeypatch.setattr(usb.libusb, "libusb_control_transfer", control_transfer)
  monkeypatch.setattr(usb.time, "sleep", lambda seconds: sleeps.append(seconds))

  assert controller.read(0xA8F0, 8) == bytes(range(8))
  assert sleeps == [controller.XDATA_READ_RETRY_INTERVAL_S]


def test_custom_usb_controller_limits_xdata_read_retries(monkeypatch):
  from tinygrad.runtime.support import usb

  controller = usb.CustomASM24Controller.__new__(usb.CustomASM24Controller)
  controller.usb = SimpleNamespace(handle=object())
  controller._f0_out_buf, controller._f0_out_mv = usb.alloc_cbuffer(0x1000)
  calls = []

  def control_transfer(*_args):
    calls.append(None)
    return -1

  monkeypatch.setattr(usb.libusb, "libusb_control_transfer", control_transfer)
  monkeypatch.setattr(usb.time, "sleep", lambda _seconds: None)

  with pytest.raises(AssertionError, match=r"read\(0xA8F0, 8\) failed: -1"):
    controller.read(0xA8F0, 8)

  assert len(calls) == controller.XDATA_READ_ATTEMPTS


def test_custom_usb_controller_does_not_retry_xdata_disconnect(monkeypatch):
  from tinygrad.runtime.support import usb

  controller = usb.CustomASM24Controller.__new__(usb.CustomASM24Controller)
  controller.usb = SimpleNamespace(handle=object())
  controller._f0_out_buf, controller._f0_out_mv = usb.alloc_cbuffer(0x1000)
  calls = []

  def control_transfer(*_args):
    calls.append(None)
    return usb.libusb.LIBUSB_ERROR_NO_DEVICE

  monkeypatch.setattr(usb.libusb, "libusb_control_transfer", control_transfer)

  with pytest.raises(AssertionError, match=r"read\(0xA8F0, 8\) failed: -4"):
    controller.read(0xA8F0, 8)

  assert len(calls) == 1


def _make_usb_for_bulk_test(usb):
  controller = usb.USB3.__new__(usb.USB3)
  controller.handle = object()
  controller._transferred = usb.ctypes.c_int(0)
  controller._bulk_out_buf, controller._bulk_out_mv = usb.alloc_cbuffer(32)
  return controller


def test_usb_bulk_out_retries_zero_length_io_error(monkeypatch):
  from tinygrad.runtime.support import usb

  controller = _make_usb_for_bulk_test(usb)
  attempts = iter((-1, 0))
  calls = []
  sleeps = []

  def bulk_transfer(_handle, _endpoint, _buffer, length, transferred, _timeout):
    calls.append(None)
    ret = next(attempts)
    transferred.value = length if ret == 0 else 0
    return ret

  monkeypatch.setattr(usb.libusb, "libusb_bulk_transfer", bulk_transfer)
  monkeypatch.setattr(usb.time, "sleep", lambda seconds: sleeps.append(seconds))

  controller._bulk_out(0x02, b"model data")

  assert len(calls) == 2
  assert sleeps == [controller.BULK_OUT_IO_RETRY_INTERVAL_S]


def test_usb_bulk_out_does_not_retry_partial_io_error(monkeypatch):
  from tinygrad.runtime.support import usb

  controller = _make_usb_for_bulk_test(usb)
  calls = []

  def bulk_transfer(_handle, _endpoint, _buffer, _length, transferred, _timeout):
    calls.append(None)
    transferred.value = 4
    return usb.libusb.LIBUSB_ERROR_IO

  monkeypatch.setattr(usb.libusb, "libusb_bulk_transfer", bulk_transfer)

  with pytest.raises(RuntimeError, match="bulk OUT 0x02 failed"):
    controller._bulk_out(0x02, b"model data")

  assert len(calls) == 1


def test_usb_bulk_out_limits_zero_length_io_retries(monkeypatch):
  from tinygrad.runtime.support import usb

  controller = _make_usb_for_bulk_test(usb)
  calls = []

  def bulk_transfer(_handle, _endpoint, _buffer, _length, transferred, _timeout):
    calls.append(None)
    transferred.value = 0
    return usb.libusb.LIBUSB_ERROR_IO

  monkeypatch.setattr(usb.libusb, "libusb_bulk_transfer", bulk_transfer)
  monkeypatch.setattr(usb.time, "sleep", lambda _seconds: None)

  with pytest.raises(RuntimeError, match="bulk OUT 0x02 failed"):
    controller._bulk_out(0x02, b"model data")

  assert len(calls) == controller.BULK_OUT_IO_ATTEMPTS


def test_usb_bulk_out_does_not_retry_disconnect(monkeypatch):
  from tinygrad.runtime.support import usb

  controller = _make_usb_for_bulk_test(usb)
  calls = []

  def bulk_transfer(_handle, _endpoint, _buffer, _length, transferred, _timeout):
    calls.append(None)
    transferred.value = 0
    return usb.libusb.LIBUSB_ERROR_NO_DEVICE

  monkeypatch.setattr(usb.libusb, "libusb_bulk_transfer", bulk_transfer)

  with pytest.raises(RuntimeError, match="bulk OUT 0x02 failed after 1 attempts"):
    controller._bulk_out(0x02, b"model data")

  assert len(calls) == 1


def test_usb_pci_device_releases_resources_after_init_failure(monkeypatch):
  from tinygrad.runtime.support import system

  events = []

  class FakeUSB:
    product = "custom eGPU"
    is_custom = True

    def __init__(self, *_args):
      events.append("usb-open")

    def close(self):
      events.append("usb-close")

  monkeypatch.setattr(system.System, "flock_acquire", lambda _name: 123)
  monkeypatch.setattr(system, "USB3", FakeUSB)

  def fail_controller(_usb):
    raise RuntimeError("PCIe link not up")

  monkeypatch.setattr(system, "CustomASM24Controller", fail_controller)
  monkeypatch.setattr(system.os, "close", lambda fd: events.append(("lock-close", fd)))

  with pytest.raises(RuntimeError, match="PCIe link not up"):
    system.USBPCIDevice("AM", object(), "usb:4-90")

  assert events == ["usb-open", "usb-close", ("lock-close", 123)]
