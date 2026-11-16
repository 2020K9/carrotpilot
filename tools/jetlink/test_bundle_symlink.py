import io
import tarfile
from pathlib import Path

from finalize_sd_image import extract_bundle


def _bundle(path, link, target):
  with tarfile.open(path, "w:gz") as out:
    data = b"{}"
    info = tarfile.TarInfo("openpilot/common/hardware/comma/agnos.json")
    info.size = len(data)
    out.addfile(info, io.BytesIO(data))
    sym = tarfile.TarInfo(link)
    sym.type = tarfile.SYMTYPE
    sym.linkname = target
    out.addfile(sym)


def test_relative_link_inside_bundle_extracts(tmp_path):
  bundle = tmp_path / "b.tar.gz"
  _bundle(bundle, "openpilot/system/hardware/comma/agnos.json", "../../../common/hardware/comma/agnos.json")
  dest = tmp_path / "a" / "b" / "c" / "release"
  dest.mkdir(parents=True)
  extract_bundle(bundle, dest)
  assert (dest / "openpilot/system/hardware/comma/agnos.json").read_text() == "{}"


def test_link_leaving_bundle_still_refused(tmp_path):
  bundle = tmp_path / "b.tar.gz"
  _bundle(bundle, "openpilot/x", "../../outside")
  dest = tmp_path / "r"
  dest.mkdir()
  try:
    extract_bundle(bundle, dest)
  except ValueError:
    return
  raise AssertionError("escape link accepted")
