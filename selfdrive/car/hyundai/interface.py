#!/usr/bin/env python3
from cereal import car
from panda import Panda
from common.conversions import Conversions as CV
from selfdrive.car.hyundai.values import HyundaiFlags, CAR, DBC, CANFD_CAR, CAMERA_SCC_CAR, EV_CAR, HYBRID_CAR, LEGACY_SAFETY_MODE_CAR, Buttons, CarControllerParams
from selfdrive.car.hyundai.radar_interface import RADAR_START_ADDR
from selfdrive.car import STD_CARGO_KG, create_button_event, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.disable_ecu import disable_ecu
from common.params import Params

Ecu = car.CarParams.Ecu
ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
ENABLE_BUTTONS = (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL)
BUTTONS_DICT = {Buttons.RES_ACCEL: ButtonType.accelCruise, Buttons.SET_DECEL: ButtonType.decelCruise,
                Buttons.GAP_DIST: ButtonType.gapAdjustCruise, Buttons.CANCEL: ButtonType.cancel}


class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[], experimental_long=False):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "hyundai"
    ret.radarOffCan = RADAR_START_ADDR not in fingerprint[1] or DBC[ret.carFingerprint]["radar"] is None

    # These cars have been put into dashcam only due to both a lack of users and test coverage.
    # These cars likely still work fine. Once a user confirms each car works and a test route is
    # added to selfdrive/car/tests/routes.py, we can remove it from this list.
    ret.dashcamOnly = candidate in {CAR.KIA_OPTIMA_H, }

    if candidate in CANFD_CAR:
      # detect HDA2 with ADAS Driving ECU
      if Ecu.adas in [fw.ecu for fw in car_fw]:
        ret.flags |= HyundaiFlags.CANFD_HDA2.value
      else:
        # non-HDA2
        if 0x1cf not in fingerprint[4]:
          ret.flags |= HyundaiFlags.CANFD_ALT_BUTTONS.value

    ret.steerActuatorDelay = 0.1  # Default delay
    ret.steerLimitTimer = 0.4
    tire_stiffness_factor = 1.
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate in (CAR.SANTA_FE, CAR.SANTA_FE_2022, CAR.SANTA_FE_HEV_2022, CAR.SANTA_FE_PHEV_2022):
      ret.mass = 3982. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.766
      # Values from optimizer
      ret.steerRatio = 16.55  # 13.8 is spec end-to-end
      tire_stiffness_factor = 0.82
    elif candidate in (CAR.SONATA, CAR.SONATA_HYBRID):
      ret.mass = 1513. + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      tire_stiffness_factor = 0.65
    elif candidate == CAR.SONATA_LF:
      ret.mass = 4497. * CV.LB_TO_KG
      ret.wheelbase = 2.804
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
    elif candidate == CAR.PALISADE:
      ret.mass = 1999. + STD_CARGO_KG
      ret.wheelbase = 2.90
      ret.steerRatio = 15.6 * 1.15
      tire_stiffness_factor = 0.63
    elif candidate == CAR.ELANTRA:
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 15.4            # 14 is Stock | Settled Params Learner values are steerRatio: 15.401566348670535
      tire_stiffness_factor = 0.385    # stiffnessFactor settled on 1.0081302973865127
      ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.ELANTRA_2021:
      ret.mass = (2800. * CV.LB_TO_KG) + STD_CARGO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 12.9
      tire_stiffness_factor = 0.65
    elif candidate == CAR.ELANTRA_HEV_2021:
      ret.mass = (3017. * CV.LB_TO_KG) + STD_CARGO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 12.9
      tire_stiffness_factor = 0.65
    elif candidate == CAR.HYUNDAI_GENESIS:
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
      ret.minSteerSpeed = 60 * CV.KPH_TO_MS
    elif candidate in (CAR.KONA, CAR.KONA_EV, CAR.KONA_HEV, CAR.KONA_EV_2022):
      ret.mass = {CAR.KONA_EV: 1685., CAR.KONA_HEV: 1425., CAR.KONA_EV_2022: 1743.}.get(candidate, 1275.) + STD_CARGO_KG
      ret.wheelbase = 2.6
      ret.steerRatio = 13.42  # Spec
      tire_stiffness_factor = 0.385
    elif candidate in (CAR.IONIQ, CAR.IONIQ_EV_LTD, CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV, CAR.IONIQ_HEV_2022):
      ret.mass = 1490. + STD_CARGO_KG  # weight per hyundai site https://www.hyundaiusa.com/ioniq-electric/specifications.aspx
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73  # Spec
      tire_stiffness_factor = 0.385
      if candidate not in (CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV, CAR.IONIQ_HEV_2022):
        ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.IONIQ_PHEV_2019:
      ret.mass = 1550. + STD_CARGO_KG  # weight per hyundai site https://www.hyundaiusa.com/us/en/vehicles/2019-ioniq-plug-in-hybrid/compare-specs
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73
      ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.VELOSTER:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75 * 1.15
      tire_stiffness_factor = 0.5
    elif candidate == CAR.TUCSON:
      ret.mass = 3520. * CV.LB_TO_KG
      ret.wheelbase = 2.67
      ret.steerRatio = 14.00 * 1.15
      tire_stiffness_factor = 0.385
    elif candidate == CAR.TUCSON_HYBRID_4TH_GEN:
      ret.mass = 1680. + STD_CARGO_KG  # average of all 3 trims
      ret.wheelbase = 2.756
      ret.steerRatio = 16.
      tire_stiffness_factor = 0.385

    # Kia
    elif candidate == CAR.KIA_SORENTO:
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.1   # 10% higher at the center seems reasonable
    elif candidate in (CAR.KIA_NIRO_EV, CAR.KIA_NIRO_PHEV, CAR.KIA_NIRO_HEV_2021):
      ret.mass = 1737. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.9 if CAR.KIA_NIRO_HEV_2021 else 13.73  # Spec
      tire_stiffness_factor = 0.385
      if candidate == CAR.KIA_NIRO_PHEV:
        ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.KIA_SELTOS:
      ret.mass = 1337. + STD_CARGO_KG
      ret.wheelbase = 2.63
      ret.steerRatio = 14.56
      tire_stiffness_factor = 1
    elif candidate in (CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL, CAR.KIA_OPTIMA_H):
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.5
      if candidate == CAR.KIA_OPTIMA_G4:
        ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.KIA_STINGER:
      ret.mass = 1825. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.15   # 15% higher at the center seems reasonable
    elif candidate == CAR.KIA_FORTE:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.5
    elif candidate == CAR.KIA_CEED:
      ret.mass = 1450. + STD_CARGO_KG
      ret.wheelbase = 2.65
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.5
    elif candidate == CAR.KIA_K5_2021:
      ret.mass = 3228. * CV.LB_TO_KG
      ret.wheelbase = 2.85
      ret.steerRatio = 13.27  # 2021 Kia K5 Steering Ratio (all trims)
      tire_stiffness_factor = 0.5
    elif candidate == CAR.KIA_EV6:
      ret.mass = 2055 + STD_CARGO_KG
      ret.wheelbase = 2.9
      ret.steerRatio = 16.
      tire_stiffness_factor = 0.65
    elif candidate == CAR.IONIQ_5:
      ret.mass = 2012 + STD_CARGO_KG
      ret.wheelbase = 3.0
      ret.steerRatio = 16.
      tire_stiffness_factor = 0.65
    elif candidate == CAR.KIA_SPORTAGE_HYBRID_5TH_GEN:
      ret.mass = 1767. + STD_CARGO_KG  # SX Prestige trim support only
      ret.wheelbase = 2.756
      ret.steerRatio = 13.6

    # Genesis
    elif candidate == CAR.GENESIS_G70:
      ret.steerActuatorDelay = 0.1
      ret.mass = 1640.0 + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.steerRatio = 13.56
    elif candidate == CAR.GENESIS_G70_2020:
      ret.mass = 3673.0 * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.83
      ret.steerRatio = 12.9
    elif candidate == CAR.GENESIS_G80:
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
    elif candidate == CAR.GENESIS_G90:
      ret.mass = 2200
      ret.wheelbase = 3.15
      ret.steerRatio = 12.069

    # *** longitudinal control ***
    if candidate in CANFD_CAR:
      ret.longitudinalTuning.kpV = [0.1]
      ret.longitudinalTuning.kiV = [0.0]
      ret.experimentalLongitudinalAvailable = bool(ret.flags & HyundaiFlags.CANFD_HDA2)
    else:
      ret.longitudinalTuning.kpV = [0.5]
      ret.longitudinalTuning.kiV = [0.0]
      ret.experimentalLongitudinalAvailable = candidate not in (LEGACY_SAFETY_MODE_CAR | CAMERA_SCC_CAR)
    ret.openpilotLongitudinalControl = experimental_long and ret.experimentalLongitudinalAvailable
    ret.pcmCruise = not ret.openpilotLongitudinalControl

    ret.stoppingControl = True
    ret.startingState = True
    ret.vEgoStarting = 0.1
    ret.startAccel = 2.0
    ret.longitudinalActuatorDelayLowerBound = 0.5
    ret.longitudinalActuatorDelayUpperBound = 0.5

    # *** feature detection ***
    if candidate in CANFD_CAR:
      bus = 5 if ret.flags & HyundaiFlags.CANFD_HDA2 else 4
      ret.enableBsm = 0x1e5 in fingerprint[bus]
    else:
      ret.enableBsm = 0x58b in fingerprint[0]

    # *** panda safety config ***
    CarInterfaceBase.configure_dp_tune(candidate, ret.lateralTuning)

    # panda safety config
    if candidate in CANFD_CAR:
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.noOutput),
                           get_safety_config(car.CarParams.SafetyModel.hyundaiCanfd)]

      if ret.flags & HyundaiFlags.CANFD_HDA2:
        ret.safetyConfigs[1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_HDA2
      if ret.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
        ret.safetyConfigs[1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_ALT_BUTTONS
    else:
      if candidate in LEGACY_SAFETY_MODE_CAR:
        # these cars require a special panda safety mode due to missing counters and checksums in the messages
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy)]
      else:
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundai, 0)]

      if candidate in CAMERA_SCC_CAR:
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_CAMERA_SCC

    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_LONG
    if candidate in HYBRID_CAR:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_HYBRID_GAS
    elif candidate in EV_CAR:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_EV_GAS

    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)
    params = Params()
    if int(params.get("dp_atl").decode('utf-8')) == 1:
      ret.openpilotLongitudinalControl = False
    params.put("dp_lateral_steer_rate_cost", "0.5")
    return ret

  @staticmethod
  def init(CP, logcan, sendcan):
    if CP.openpilotLongitudinalControl:
      addr, bus = 0x7d0, 0
      if CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, 5
      disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)
    ret.cruiseState.enabled, ret.cruiseState.available = self.dp_atl_mode(ret)

    if self.CS.CP.openpilotLongitudinalControl and self.CS.cruise_buttons[-1] != self.CS.prev_cruise_buttons:
      buttonEvents = [create_button_event(self.CS.cruise_buttons[-1], self.CS.prev_cruise_buttons, BUTTONS_DICT)]
      # Handle CF_Clu_CruiseSwState changing buttons mid-press
      if self.CS.cruise_buttons[-1] != 0 and self.CS.prev_cruise_buttons != 0:
        buttonEvents.append(create_button_event(0, self.CS.prev_cruise_buttons, BUTTONS_DICT))

      ret.buttonEvents = buttonEvents

    # On some newer model years, the CANCEL button acts as a pause/resume button based on the PCM state
    # To avoid re-engaging when openpilot cancels, check user engagement intention via buttons
    # Main button also can trigger an engagement on these cars
    allow_enable = any(btn in ENABLE_BUTTONS for btn in self.CS.cruise_buttons) or any(self.CS.main_buttons)
    events = self.create_common_events(ret, pcm_enable=self.CS.CP.pcmCruise, allow_enable=allow_enable)
    events = self.dp_atl_warning(ret, events)

    if self.CS.brake_error:
      events.add(EventName.brakeUnavailable)

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    return ret

  def apply(self, c):
    return self.CC.update(c, self.CS)