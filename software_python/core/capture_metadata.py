"""Strict main-profile metadata checks; no hardware access or data repair."""
import hashlib
import json
import math


MAHONY_FIELDS = (
    'kp_static ki_static kp_dynamic ki_dynamic stationary_enter_dps stationary_enter_g '
    'stationary_exit_dps stationary_exit_g accel_reject_g output_alpha output_tau_s '
    'musical_yaw_tau_s bias_xy_tau_s bias_z_tau_s bias_z_engage_s gyro_filter_tau_s '
    'accel_filter_tau_s variance_tau_s gyro_variance_dps2 accel_variance_g2 jerk_reject_gps rate_reject_dps'
).split()
NAV_FIELDS = (
    'accel_noise_g_sqrt_hz gyro_noise_dps_sqrt_hz accel_bias_walk_g_sqrt_s gyro_bias_walk_dps_sqrt_s '
    'zupt_noise_mps zupt_min_still_s zupt_bypass_s zupt_bypass_starved_s zupt_gate_d2 zupt_starvation_s '
    'attitude_noise_rad attitude_yaw_variance attitude_gate_d2 attitude_bypass_samples accel_bias_limit_g '
    'gyro_bias_limit_dps accel_bias_init_s stationary_enter_dps stationary_exit_dps gyro_variance_dps2 '
    'linear_accel_variance_g2 detector_tau_s linear_accel_enter_g linear_accel_exit_g gravity_trust_floor_g gravity_trust_ceil_g'
).split()


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def verify_configuration_hash(raw):
    """Hash the exact embedded object bytes, avoiding JSON float reformatting."""
    decoder = json.JSONDecoder()
    pos = 0
    while raw[pos].isspace(): pos += 1
    if raw[pos] != '{': raise ValueError('metadata must be an object')
    pos += 1
    configuration = None
    digest = None
    while True:
        while raw[pos].isspace(): pos += 1
        if raw[pos] == '}': break
        key, pos = decoder.raw_decode(raw, pos)
        while raw[pos].isspace(): pos += 1
        if raw[pos] != ':': raise ValueError('metadata key lacks colon')
        pos += 1
        while raw[pos].isspace(): pos += 1
        start = pos
        value, pos = decoder.raw_decode(raw, pos)
        if key == 'configuration': configuration = raw[start:pos]
        if key == 'config_sha256': digest = value
        while raw[pos].isspace(): pos += 1
        if raw[pos] == '}': break
        if raw[pos] != ',': raise ValueError('metadata field lacks comma')
        pos += 1
    if configuration is None or hashlib.sha256(configuration.encode('utf-8')).hexdigest() != digest:
        raise ValueError('configuration SHA-256 mismatch')


def metadata_detail_errors(meta):
    errors = []
    if meta.get('metadata_schema') != 1:
        return ['missing detailed metadata schema; main capture blocked']
    caps = meta.get('capabilities')
    if not isinstance(caps, dict) or caps.get('estimator_snapshots') is not True:
        errors.append('missing periodic/arming estimator snapshots; main capture blocked')
    config = meta.get('configuration')
    if not isinstance(config, dict):
        return ['missing configuration object; main capture blocked']
    for key in ('accel_bias_g', 'active_gyro_bias_dps', 'finger_calibration'):
        if config.get(key) != meta.get(key): errors.append(f'configuration/root {key} mismatch')
    sensor = config.get('sensor', {})
    if not isinstance(sensor, dict): return errors + ['invalid configuration.sensor']
    if sensor.get('complete') is not True: errors.append('sensor interpretation incomplete')
    if sensor.get('read_ok') is not True: errors.append('sensor register read failed')
    if sensor.get('driver_scale_matches_registers') is not True: errors.append('sensor/driver scale mismatch')
    registers = sensor.get('registers', {})
    if not isinstance(registers, dict): return errors + ['invalid sensor registers']
    names = ('who_am_i', 'smplrt_div', 'config', 'gyro_config', 'accel_config', 'accel_config2',
             'user_ctrl', 'pwr_mgmt_1', 'pwr_mgmt_2', 'lp_mode_cfg', 'fifo_en')
    if any(type(registers.get(k)) is not int or not 0 <= registers[k] <= 255 for k in names):
        return errors + ['missing/invalid sensor register bytes']
    if registers['who_am_i'] != 0x19: errors.append('unsupported sensor identity')
    ar = 2 << ((registers['accel_config'] >> 3) & 3)
    gr = 250 << ((registers['gyro_config'] >> 3) & 3)
    expected = dict(accel_range_g=ar, gyro_range_dps=gr,
                    accel_register_g_per_count=ar / 32768., gyro_register_dps_per_count=gr / 32768.,
                    driver_accel_g_per_count=ar / 32768., driver_gyro_dps_per_count=gr / 32768.)
    for key, value in expected.items():
        actual = sensor.get(key)
        # Decimal wire floats round-trip float32; compare as float32, not float64 literals.
        if not finite(actual) or not math.isclose(actual, value, rel_tol=1e-8, abs_tol=0):
            errors.append(f'invalid sensor {key}')
    gd = registers['config'] & 7
    known = (registers['gyro_config'] & 3) == 0 and 0 < gd < 7 and not (registers['accel_config2'] & 8)
    known &= not (registers['pwr_mgmt_1'] & 0x70 or registers['pwr_mgmt_2'] & 0x3f
                  or registers['user_ctrl'] & 0x70 or registers['fifo_en'] or registers['lp_mode_cfg']
                  or registers['gyro_config'] & 0xe0 or registers['accel_config'] & 0xe0)
    if not known: errors.append('unsupported sensor operating/filter mode')
    if known:
        rate = 1000. / (1 + registers['smplrt_div'])
        bw = (218.1, 218.1, 99., 44.8, 21.2, 10.2, 5.1, 420.)[registers['accel_config2'] & 7]
        for key, value in dict(gyro_output_hz=rate, accel_output_hz=rate,
                               gyro_dlpf_3db_hz=(250, 176, 92, 41, 20, 10, 5, 3281)[gd],
                               accel_dlpf_3db_hz=bw).items():
            if not finite(sensor.get(key)) or not math.isclose(sensor[key], value, rel_tol=1e-7):
                errors.append(f'invalid sensor {key}')
    offsets = sensor.get('driver_offsets_counts_q16')
    if not isinstance(offsets, list) or len(offsets) != 6 or any(type(x) is not int or not -(2**31) <= x < 2**31 for x in offsets):
        errors.append('missing/invalid driver offsets')
    sha = sensor.get('driver_source_sha256', '')
    if not isinstance(sha, str) or len(sha) != 64 or any(x not in '0123456789abcdef' for x in sha):
        errors.append('missing driver source hash')
    if sensor.get('physical_scale_verification') != 'passed':
        errors.append('physical scale/axis verification pending; main capture blocked')
    estimators = config.get('estimators', {})
    if not isinstance(estimators, dict): return errors + ['invalid estimator configuration']
    if estimators.get('source_sha256') != meta.get('firmware_source_sha256'):
        errors.append('estimator/source hash mismatch')
    for name, fields in (('mahony', MAHONY_FIELDS), ('nav', NAV_FIELDS)):
        params = estimators.get(name, {})
        if not isinstance(params, dict): errors.append(f'missing {name} parameters'); continue
        for key in fields:
            if not finite(params.get(key)) or params[key] < 0: errors.append(f'missing/invalid {name}.{key}')
    nav = estimators.get('nav', {})
    if isinstance(nav, dict):
        for key in ('tangent_gravity', 'block_gravity_bias'):
            if type(nav.get(key)) is not bool: errors.append(f'missing/invalid nav.{key}')
    acquisition = config.get('acquisition', {})
    if not isinstance(acquisition, dict) or any(not finite(acquisition.get(k)) or acquisition[k] <= 0
                                               for k in ('scheduled_interval_us', 'max_nav_gap_s')):
        errors.append('missing/invalid acquisition timing')
    return errors
