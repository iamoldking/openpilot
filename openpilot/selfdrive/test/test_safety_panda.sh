#!/usr/bin/env bash
set -e

cd "$(dirname "${BASH_SOURCE[0]}")/../../.."

scons -C panda -j"$(nproc)"
python - <<'PY'
import time
from panda import Panda

p = Panda()
p.reset(enter_bootstub=True)
p.close()
time.sleep(1)
p = Panda()
assert p.bootstub
p.flash(reconnect=False)
p.close()
PY
sleep 2

export LIBSAFETY_BACKEND=openpilot.selfdrive.test.libsafety_panda
python -m unittest discover -v -s opendbc_repo/opendbc/safety/tests -p "${SAFETY_TEST_PATTERN:-test_*.py}"
