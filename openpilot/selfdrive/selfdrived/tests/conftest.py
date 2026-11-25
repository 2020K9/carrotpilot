import pytest

from openpilot.selfdrive.selfdrived.events import EVENTS, Alert


@pytest.fixture(autouse=True)
def _keep_shared_events_table():
  """EVENTS and its Alert objects are one module-level table shared by every test in a pytest worker.
  test_state_machine writes EVENTS[0] and test_alertmanager sets .duration on randomly picked shared alerts;
  without putting them back, a later test in the same worker (test_steering_warning_delay,
  test_update_reboot_alert, ...) sees the altered table and fails only in the full parallel run."""
  table = {name: dict(alerts) for name, alerts in EVENTS.items()}
  fields = [(alert, dict(vars(alert))) for alerts in EVENTS.values() for alert in alerts.values() if isinstance(alert, Alert)]
  yield
  EVENTS.clear()
  EVENTS.update(table)
  for alert, saved in fields:
    vars(alert).clear()
    vars(alert).update(saved)
