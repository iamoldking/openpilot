import atexit
import struct
import threading
import time

from panda import DLC_TO_LEN, Panda


SAFETY_TEST_ADDR = 0x1FFFFF00

SET_HOOKS, RX_HOOK, TX_HOOK, FWD_HOOK, LOAD_PACKET, TICK, CONFIG_VALID, INIT, IGNITION_HOOK, GET, SET, RX_PACKET, TX_PACKET, \
  FWD_HOOK_STATE, TX_PACKET_STATE = range(1, 16)
RX_PACKET_STATE = 16

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
  "timer_elapsed": 39,
}


class PandaSafety:
  def __init__(self):
    self.panda = Panda()
    self.panda.can_clear(0xFFFF)
    for bus in range(3):
      self.panda.set_can_enable(bus, False)
    self.panda_lock = threading.Lock()
    self.stop_event = threading.Event()
    self.heartbeat_thread = threading.Thread(target=self._heartbeat, daemon=True)
    self.heartbeat_thread.start()
    atexit.register(self._close)
    self.test_timer = 0
    self.panda_timer = None
    self.timer_advanced = False
    self.timer_elapsed = 0
    self.relay_malfunction = False
    self.controls_allowed = False

  def _close(self):
    self.stop_event.set()
    self.heartbeat_thread.join()
    for bus in range(3):
      self.panda.set_can_enable(bus, True)

  def _heartbeat(self):
    while not self.stop_event.is_set():
      with self.panda_lock:
        self.panda.send_heartbeat(engaged=True)
      self.stop_event.wait(0.5)

  def _call(self, op, *args, payload=b""):
    with self.panda_lock:
      return self._call_locked(op, *args, payload=payload)

  def _call_locked(self, op, *args, payload=b""):
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
          self.last_relay_malfunction = bool(response[5])
          mode = struct.unpack_from("<H", response, 6)[0]
          if hasattr(self, "safety_mode") and op != SET_HOOKS:
            assert mode == self.safety_mode, (mode, self.safety_mode)
          self.last_controls_allowed = bool(response[8])
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
    self.relay_malfunction = False
    self.controls_allowed = False
    ret = self._call(SET_HOOKS, mode, param)
    self.safety_mode = int(mode)
    self.relay_malfunction = self.last_relay_malfunction
    self.controls_allowed = self.last_controls_allowed
    return ret

  def safety_rx_hook(self, msg):
    packet = msg[0]
    dat = bytes(packet.data[0:DLC_TO_LEN[int(packet.data_len_code)]])
    if len(dat) <= 54:
      header = bytes((int(packet.fd), int(packet.bus), int(packet.data_len_code))) + struct.pack("<i", int(packet.addr))
      state = bytes((self.controls_allowed, self.relay_malfunction))
      ret = bool(self._call(RX_PACKET_STATE, payload=header + dat + bytes(54 - len(dat)) + state))
    else:
      self._set_value("controls_allowed", self.controls_allowed)
      self._set_value("relay_malfunction", self.relay_malfunction)
      ret = self._hook(RX_HOOK, RX_PACKET, msg)
    self.controls_allowed = self.last_controls_allowed
    self.relay_malfunction = self.last_relay_malfunction
    return ret

  def safety_tx_hook(self, msg):
    packet = msg[0]
    dat = bytes(packet.data[0:DLC_TO_LEN[int(packet.data_len_code)]])
    if len(dat) <= 54:
      if self.timer_advanced:
        self._set_value("timer_elapsed", self.timer_elapsed)
      fd_flags = int(packet.fd) | (4 if self.timer_advanced else 2)
      header = bytes((fd_flags, int(packet.bus), int(packet.data_len_code))) + struct.pack("<i", int(packet.addr))
      state = bytes((self.controls_allowed, self.relay_malfunction))
      ret = bool(self._call(TX_PACKET_STATE, payload=header + dat + bytes(54 - len(dat)) + state))
      self.timer_advanced = False
      self.timer_elapsed = 0
      self.controls_allowed = self.last_controls_allowed
      self.relay_malfunction = self.last_relay_malfunction
      return ret
    self._set_value("controls_allowed", self.controls_allowed)
    self._set_value("relay_malfunction", self.relay_malfunction)
    ret = self._hook(TX_HOOK, TX_PACKET, msg)
    self.controls_allowed = self.last_controls_allowed
    self.relay_malfunction = self.last_relay_malfunction
    return ret

  def safety_fwd_hook(self, bus, addr):
    return self._call(FWD_HOOK_STATE, bus, addr, payload=bytes((self.relay_malfunction,)))

  def _set_value(self, name, a, b=0):
    return self._call(SET, payload=bytes((VALUES[name],)) + struct.pack("<ii", int(a), int(b)))

  def set_relay_malfunction(self, malfunction):
    self.relay_malfunction = bool(malfunction)
    self._set_value("relay_malfunction", malfunction)

  def get_relay_malfunction(self):
    return self.relay_malfunction

  def set_controls_allowed(self, allowed):
    self.controls_allowed = bool(allowed)
    self._set_value("controls_allowed", allowed)

  def get_controls_allowed(self):
    return self.controls_allowed

  def safety_tick_current_safety_config(self):
    self._call(TICK)
    self.controls_allowed = self.last_controls_allowed

  def safety_config_valid(self):
    return bool(self._call(CONFIG_VALID))

  def init_tests(self):
    self._call(INIT)
    self.test_timer = 0
    self.panda_timer = self._call(GET, payload=bytes((VALUES["timer"],)))
    self.timer_advanced = False
    self.timer_elapsed = 0

  def set_timer(self, timer):
    delta = int(timer) - self.test_timer
    if delta < 0:
      raise ValueError("panda safety test timer cannot move backwards")
    before = self._call(GET, payload=bytes((VALUES["timer"],)))
    if delta:
      time.sleep(delta / 1e6)

    now = self._call(GET, payload=bytes((VALUES["timer"],)))
    elapsed = (now - before) & 0xFFFFFFFF
    assert delta <= elapsed < delta + 100_000, (delta, elapsed)
    self.test_timer = int(timer)
    self.panda_timer = now
    self.timer_advanced = delta != 0
    self.timer_elapsed = delta

  def ignition_can_hook(self, msg):
    self._load(msg)
    self._call(IGNITION_HOOK)

  def __getattr__(self, name):
    if name.startswith("get_"):
      value = VALUES[name[4:]]
      def get_value():
        self._set_value("controls_allowed", self.controls_allowed)
        ret = self._call(GET, payload=bytes((value,)))
        return ret / 1000.0 if name in ("get_vehicle_speed_min", "get_vehicle_speed_max") else ret
      return get_value
    if name.startswith("set_"):
      value = "desired_curvature" if name == "set_desired_curvature_last" else name[4:]
      return lambda a, b=0: self._set_value(value, a, b)
    raise AttributeError(name)


libsafety = PandaSafety()
