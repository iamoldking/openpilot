import struct
import time

from panda import DLC_TO_LEN, Panda


SAFETY_TEST_ADDR = 0x1FFFFF00

SET_HOOKS, RX_HOOK, TX_HOOK, FWD_HOOK, LOAD_PACKET, TICK, CONFIG_VALID, INIT, IGNITION_HOOK, GET, SET, RX_PACKET, TX_PACKET = range(1, 14)

VALUES = {
  "controls_allowed": 1, "longitudinal_allowed": 2, "alternative_experience": 3,
  "relay_malfunction": 4, "gas_pressed_prev": 5, "brake_pressed_prev": 6,
  "regen_braking_prev": 7, "steering_disengage_prev": 8, "acc_main_on": 9,
  "vehicle_speed_min": 10, "vehicle_speed_max": 11, "current_safety_mode": 12,
  "current_safety_param": 13, "torque_meas_min": 14, "torque_meas_max": 15,
  "torque_driver_min": 16, "torque_driver_max": 17, "desired_angle_last": 18,
  "angle_meas_min": 19, "angle_meas_max": 20, "desired_curvature_last": 21,
  "curvature_meas_min": 22, "curvature_meas_max": 23, "cruise_engaged_prev": 24,
  "vehicle_moving": 25, "honda_fwd_brake": 26, "honda_hw": 27, "ignition_can": 28,
  "timer": 29, "torque_meas": 30, "torque_driver": 31, "desired_torque_last": 32,
  "rt_torque_last": 33, "angle_meas": 34, "desired_curvature": 35,
  "curvature_meas": 36, "honda_alt_brake_msg": 37, "honda_bosch_long": 38,
}


class PandaSafety:
  def __init__(self):
    self.panda = Panda()
    self.panda.can_clear(0xFFFF)
    self.test_timer = 0
    self.panda_timer = None

  def _call(self, op, *args, payload=b""):
    dat = bytearray(64)
    dat[0] = op
    for i, arg in enumerate(args):
      struct.pack_into("<i", dat, 1 + i * 4, int(arg))
    dat[1 + 4 * len(args):1 + 4 * len(args) + len(payload)] = payload
    self.panda.can_send(SAFETY_TEST_ADDR, dat, 3, fd=True, timeout=1000)
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
      for addr, response, bus in self.panda.can_recv():
        if addr == SAFETY_TEST_ADDR and bus == 195:
          return struct.unpack_from("<i", response, 1)[0]
    raise TimeoutError("panda safety test RPC timed out")

  def _load(self, msg):
    packet = msg[0]
    dat = bytes(packet.data[0:DLC_TO_LEN[int(packet.data_len_code)]])
    header = bytes((int(packet.fd), int(packet.bus), int(packet.data_len_code))) + struct.pack("<i", int(packet.addr))
    self._call(LOAD_PACKET, payload=header + b"\0" + dat[:55])
    if len(dat) > 55:
      self._call(LOAD_PACKET, payload=header + bytes((55,)) + dat[55:])

  def _hook(self, op, packet_op, msg):
    packet = msg[0]
    dat = bytes(packet.data[0:DLC_TO_LEN[int(packet.data_len_code)]])
    if len(dat) <= 56:
      header = bytes((int(packet.fd), int(packet.bus), int(packet.data_len_code))) + struct.pack("<i", int(packet.addr))
      return bool(self._call(packet_op, payload=header + dat))
    self._load(msg)
    return bool(self._call(op))

  def set_safety_hooks(self, mode, param):
    return self._call(SET_HOOKS, mode, param)

  def safety_rx_hook(self, msg):
    return self._hook(RX_HOOK, RX_PACKET, msg)

  def safety_tx_hook(self, msg):
    return self._hook(TX_HOOK, TX_PACKET, msg)

  def safety_fwd_hook(self, bus, addr):
    return self._call(FWD_HOOK, bus, addr)

  def safety_tick_current_safety_config(self):
    self._call(TICK)

  def safety_config_valid(self):
    return bool(self._call(CONFIG_VALID))

  def init_tests(self):
    self._call(INIT)
    self.test_timer = 0
    self.panda_timer = self._call(GET, payload=bytes((VALUES["timer"],)))

  def set_timer(self, timer):
    delta = int(timer) - self.test_timer
    if delta < 0:
      raise ValueError("panda safety test timer cannot move backwards")
    if delta:
      time.sleep(delta / 1e6)

    now = self._call(GET, payload=bytes((VALUES["timer"],)))
    if self.panda_timer is not None:
      elapsed = (now - self.panda_timer) & 0xFFFFFFFF
      assert delta <= elapsed < delta + 100_000, (delta, elapsed)
    self.test_timer = int(timer)
    self.panda_timer = now

  def ignition_can_hook(self, msg):
    self._load(msg)
    self._call(IGNITION_HOOK)

  def __getattr__(self, name):
    if name.startswith("get_"):
      value = VALUES[name[4:]]
      def get_value():
        ret = self._call(GET, payload=bytes((value,)))
        return ret / 1000.0 if name in ("get_vehicle_speed_min", "get_vehicle_speed_max") else ret
      return get_value
    if name.startswith("set_"):
      value = VALUES[name[4:]]
      return lambda a, b=0: self._call(SET, payload=bytes((value,)) + struct.pack("<ii", int(a), int(b)))
    raise AttributeError(name)


libsafety = PandaSafety()
