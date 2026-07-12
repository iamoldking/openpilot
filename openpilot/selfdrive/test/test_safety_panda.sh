#!/usr/bin/env bash
set -e

cd "$(dirname "${BASH_SOURCE[0]}")/../../.."

scons -C panda -j"$(nproc)"
python -c 'from panda import Panda; p = Panda(); p.flash(reconnect=False); p.close()'
sleep 2

export LIBSAFETY_BACKEND=openpilot.selfdrive.test.libsafety_panda
python -m unittest discover -s opendbc_repo/opendbc/safety/tests -p 'test_*.py'
