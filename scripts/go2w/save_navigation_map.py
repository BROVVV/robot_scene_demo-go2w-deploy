#!/usr/bin/python3
"""Save occupancy + posegraph + immutable provenance. Saving grants no motion authority."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml

#: 物理验证证据的自报 schema。授权由**证据推导**（见 authorization()），不是一条
#: 可以单独写下去的布尔量。
EVIDENCE_SCHEMA = 'go2w_physical_validation_v2'
MAP_FILES = ('map.yaml', 'map.pgm', 'map.posegraph', 'map.data')
EVIDENCE_FIELDS = ('schema_version', 'kind', 'map_bundle_sha256', 'recorded_at', 'measurements', 'passed')
# Nav2's trinary PGM writer uses shade 205 for unknown cells.  Humble's loader
# converts that shade to an occupancy probability of 50 / 255 ~= 0.196078.
# Its default free threshold (0.25) therefore reloads every unknown cell as
# free space.  Keep the threshold strictly below the encoded unknown value so
# a save -> load round trip preserves the frontier/unknown boundary.
TRINARY_UNKNOWN_OCCUPANCY = (255.0 - 205.0) / 255.0
SAVE_FREE_THRESHOLD = 0.19
SAVE_OCCUPIED_THRESHOLD = 0.65


def map_saver_command(output):
    # Foxy declares an integer duration in milliseconds; Humble uses seconds.
    timeout_value = '10000' if os.environ.get('ROS_DISTRO') == 'foxy' else '10.0'
    return ['ros2','run','nav2_map_server','map_saver_cli','-f',str(output),
            '--fmt','pgm','--free',str(SAVE_FREE_THRESHOLD),
            '--occ',str(SAVE_OCCUPIED_THRESHOLD), '--ros-args',
            '-p','save_map_timeout:='+timeout_value,
            '-p','free_thresh_default:='+str(SAVE_FREE_THRESHOLD),
            '-p','occupied_thresh_default:='+str(SAVE_OCCUPIED_THRESHOLD),
            '-p','map_subscribe_transient_local:=true']


def validate_saved_thresholds(path):
    value=yaml.safe_load(Path(path).read_text())
    if (not isinstance(value,dict) or value.get('mode')!='trinary'
            or not math.isclose(float(value.get('free_thresh',-1)),SAVE_FREE_THRESHOLD,abs_tol=1e-9)
            or not math.isclose(float(value.get('occupied_thresh',-1)),SAVE_OCCUPIED_THRESHOLD,abs_tol=1e-9)):
        raise ValueError('saved map thresholds do not preserve unknown/occupied cells')


def normalize_saved_image_path(path):
    """Make a new bundle relocatable before its provenance hashes are taken."""
    path = Path(path).resolve()
    value = yaml.safe_load(path.read_text())
    image = value.get('image') if isinstance(value, dict) else None
    if not isinstance(image, str) or not image:
        raise ValueError('saved map has no image reference')
    try:
        target = (path.parent/image).resolve(strict=True)
    except OSError as error:
        raise ValueError('saved map image does not exist') from error
    if target != path.parent/'map.pgm' or not target.is_file():
        raise ValueError('saved map image is outside its bundle')
    if image != 'map.pgm':
        value['image'] = 'map.pgm'
        temporary = path.with_suffix('.yaml.tmp')
        temporary.write_text(yaml.safe_dump(value, sort_keys=False))
        temporary.replace(path)


def validate_posegraph_response(response, directory):
    if response is None:
        raise RuntimeError('posegraph serialization returned no response')
    if hasattr(response,'result'):
        if response.result != 0:
            raise RuntimeError('posegraph serialization failed')
    elif not (os.environ.get('ROS_DISTRO')=='foxy'
              and response.get_fields_and_field_types()=={}):
        raise RuntimeError('unsupported posegraph serialization response')
    # Foxy's response is empty. The caller uses a new directory and verifies
    # that this request produced both nonempty files, never just an RPC ack.
    if any(not (directory/name).is_file() or (directory/name).stat().st_size==0
           for name in ('map.posegraph','map.data')):
        raise RuntimeError('posegraph serialization did not produce both artifacts')


def run_owned_command(command, *, log, timeout=20):
    """A timed-out ros2 wrapper must not leave its native child running."""
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    try:
        returncode = process.wait(timeout=timeout)
    except BaseException:
        def signal_group(value):
            try: os.killpg(process.pid, value)
            except ProcessLookupError: pass
        signal_group(signal.SIGTERM)
        try: process.wait(timeout=2)
        except subprocess.TimeoutExpired: pass
        # The wrapper may exit first, while its child still owns this group.
        signal_group(signal.SIGKILL)
        process.wait()
        raise
    if returncode:
        raise subprocess.CalledProcessError(returncode, command)


def stale_epoch_archive_blockers(report, *, sensor_profile_sha256):
    """Checks that are sufficient to preserve a stale map, never to use it.

    A robot reboot invalidates the localization epoch before Nav2 may start.
    Requiring an active planner in order to archive that map creates a cycle:
    the planner entry refuses the stale epoch, while the archive command used
    to refuse a missing planner.  This narrower contract accepts only fresh
    mapping/geometry evidence with the physical output disabled.  The saved
    bundle remains explicitly unauthorized.
    """
    evidence = report.get('evidence') or {}
    mapping = report.get('mapping_frontend') or {}
    geometry = report.get('sensor_geometry') or {}
    floor = report.get('floor_visibility') or {}
    physical = report.get('physical_execution') or {}
    blockers = []
    if report.get('audit_mode') != 'mapping':
        blockers.append('audit_mode')
    if not (evidence.get('map') or {}).get('valid'):
        blockers.append('map')
    if mapping.get('configuration_valid') is not True:
        blockers.append('mapping_frontend')
    if geometry.get('valid') is not True or geometry.get('profile_sha256') != sensor_profile_sha256:
        blockers.append('sensor_geometry')
    if floor and floor.get('valid') is not True:
        blockers.append('floor_visibility')
    if physical.get('verdict') != 'disabled':
        blockers.append('physical_execution')
    return blockers


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def bundle_digest(manifest):
    hashes = manifest.get('sha256')
    if not isinstance(hashes, dict) or any(
            not isinstance(hashes.get(name), str) or len(hashes[name]) != 64
            or any(c not in '0123456789abcdef' for c in hashes[name]) for name in MAP_FILES):
        return None
    return hashlib.sha256(json.dumps({name: hashes[name] for name in MAP_FILES},
                                    sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def numeric_measurements(value):
    if isinstance(value, dict):
        return bool(value) and all(isinstance(k, str) and numeric_measurements(v) for k, v in value.items())
    if isinstance(value, list):
        return bool(value) and all(numeric_measurements(v) for v in value)
    return type(value) in (int, float) and math.isfinite(value)


def lookup_pose(buffer, parent='map', child='base_link'):
    """``parent->child`` 在 TF 里的位姿；查不到返回 None（不抛）。

    ``rclpy`` 在函数里 import：``--verify`` 这条路不该因为没装 ROS 就跑不起来
    （测试直接 import 本模块）。
    """
    import rclpy
    try:
        transform = buffer.lookup_transform(parent, child, rclpy.time.Time())
    except Exception:
        return None
    translation, rotation = transform.transform.translation, transform.transform.rotation
    return {'x': translation.x, 'y': translation.y,
            'yaw': math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                              1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z)),
            'frame': parent}


def bound_evidence(manifest):
    """挑出**真的绑在这张图上**的物理验证证据。

    计划书 §P4-6：原来 ``require_authorized`` 只查
    ``navigation_authorized is True`` 且 ``physical_validation_evidence`` 非空，
    于是手写 ``{"navigation_authorized": true, "physical_validation_evidence": ["x"]}``
    就能过 —— 空壳证据。这里把"授权"从"一条可以随便写的布尔量"改成"由证据推导"：

    * 证据必须自报 schema 且字段齐；
    * ``map_bundle_sha256`` binds YAML, occupancy image, posegraph and data;
      changing the image alone invalidates the old validation;
    * a supported physical run must explicitly report ``passed: true``;
    * measurements must be nonempty finite numbers, not strings or booleans.

    仍然要诚实：这些字段**挡不住蓄意伪造**（可以在同一个文件里把 sha256 抄对、
    把 measurements 编出来）。它拦的是"顺手写一句 true 就放行"，并且全仓没有
    任何代码路径会写 ``navigation_authorized: true``。真正的门是"这条证据必须来自
    一次真实运行"，那件事靠流程不靠代码，见 P4 文档。
    """
    entries = manifest.get('physical_validation_evidence')
    if not isinstance(entries, list):
        return []
    map_sha = bundle_digest(manifest)
    if map_sha is None:
        return []
    good = []
    for entry in entries:
        if not isinstance(entry, dict) or any(field not in entry for field in EVIDENCE_FIELDS):
            continue
        if entry['schema_version'] != EVIDENCE_SCHEMA or entry['map_bundle_sha256'] != map_sha:
            continue
        geometry=manifest.get('navigation_geometry')
        if geometry and entry.get('sensor_profile_sha256')!=geometry.get('sensor_profile_sha256'):
            continue
        if type(entry['recorded_at']) not in (int, float) or not math.isfinite(entry['recorded_at']) or entry['recorded_at'] <= 0:
            continue
        if entry['kind'] not in ('real_robot_short_distance', 'real_robot_large_field') or entry['passed'] is not True:
            continue
        measurements = entry['measurements']
        if not isinstance(measurements, (dict, list)) or not numeric_measurements(measurements):
            continue
        good.append(entry)
    return good


def authorization(manifest):
    """授权是**推导**出来的：有绑在这张图上的有效证据才有授权。

    ``declared_mismatch`` 单独报出来：有人手写了 ``navigation_authorized: true``
    而证据通不过 —— 那不是"授权了"，那是这份 provenance 自相矛盾，按拒处理。
    """
    evidence = bound_evidence(manifest)
    declared = manifest.get('navigation_authorized')
    return {'navigation_authorized': bool(evidence),
            'evidence': evidence,
            'declared_true_without_evidence': declared is True and not evidence}


def verify(map_file, require_authorized=False, sensor_profile_sha256=None):
    target = Path(map_file).resolve(strict=True)
    directory = target.parent
    manifest = json.loads((directory/'provenance.json').read_text())
    if manifest.get('schema_version') != 'go2w_navigation_map_v1':
        raise ValueError('unsupported map provenance')
    expected = set(MAP_FILES)
    if not expected.issubset(manifest.get('sha256', {})):
        raise ValueError('missing occupancy or posegraph artifacts')
    if target.name != 'map.yaml':
        raise ValueError('map YAML must match the provenance bundle')
    geometry=manifest.get('navigation_geometry')
    if geometry:
        profile=directory/'sensor_profile.yaml'
        if (not profile.is_file() or digest(profile)!=geometry.get('sensor_profile_sha256')):
            raise ValueError('saved map sensor geometry missing or changed')
    if sensor_profile_sha256 is not None and (not geometry
            or geometry.get('sensor_profile_sha256')!=sensor_profile_sha256):
        raise ValueError('saved map geometry differs from the active sensor profile')
    for name in expected:
        path = (directory/name).resolve(strict=True)
        if path.parent != directory or digest(path) != manifest['sha256'][name]:
            raise ValueError('saved map hash mismatch: '+name)
    contents = yaml.safe_load(target.read_text())
    if (directory/contents['image']).resolve() != directory/'map.pgm' or float(contents['resolution']) <= 0:
        raise ValueError('invalid saved occupancy metadata')
    if str(contents.get('mode', '')).lower() == 'trinary':
        free_thresh = float(contents.get('free_thresh', float('nan')))
        occupied_thresh = float(contents.get('occupied_thresh', float('nan')))
        if (not math.isfinite(free_thresh) or not math.isfinite(occupied_thresh)
                or not 0.0 <= free_thresh < TRINARY_UNKNOWN_OCCUPANCY < occupied_thresh <= 1.0):
            raise ValueError('unsafe trinary thresholds would reload standard unknown cells as free')
    if require_authorized:
        grants = authorization(manifest)
        if grants['declared_true_without_evidence']:
            raise ValueError('provenance declares navigation_authorized without valid evidence')
        if not grants['navigation_authorized']:
            raise ValueError('map has no completed physical validation; navigation remains disabled')
    return manifest


def main():
    p = argparse.ArgumentParser()
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument('--output-dir');g.add_argument('--verify')
    g.add_argument('--initial-pose', metavar='MAP_YAML',
                   help='打印历史存档位姿；不能替代当前 AMCL 初始位姿估计')
    p.add_argument('--require-authorized', action='store_true')
    p.add_argument('--stale-epoch-archive', action='store_true',
                   help='archive a fresh mapping after a robot reboot without requiring Nav2; never authorizes it')
    p.add_argument('--mapping-session-archive', action='store_true',
                   help='save the active mapping-only session without starting Nav2; never authorizes it')
    p.add_argument('--sensor-profile', help='require this current geometry when verifying a saved map')
    args = p.parse_args()
    if args.stale_epoch_archive and not args.output_dir:
        p.error('--stale-epoch-archive requires --output-dir')
    if args.mapping_session_archive and not args.output_dir:
        p.error('--mapping-session-archive requires --output-dir')
    if args.stale_epoch_archive and args.mapping_session_archive:
        p.error('--stale-epoch-archive and --mapping-session-archive are mutually exclusive')
    if args.initial_pose:
        manifest = verify(args.initial_pose)
        pose = manifest.get('initial_pose')
        # AMCL 在 set_initial_pose=false 且没人发 /initialpose 时**一个位姿都不发、
        # 连 map->odom_fused 都不发**（源码依据见 P4 文档）。所以初始位姿是定位模式
        # 的硬前提：没有就拒绝启动，绝不用 (0,0,0) 顶上 —— 那正是"拿旧 pose 假定位"
        # 里最坏的一种，因为它连"旧"都不是，是编的。
        if (not isinstance(pose, dict)
                or pose.get('frame') != 'map'
                or any(type(pose.get(k)) not in (int, float) or not math.isfinite(pose[k]) for k in ('x', 'y', 'yaw'))):
            print('saved map has no recorded initial pose; localization mode cannot start',
                  file=sys.stderr)
            return 2
        for name in ('x', 'y', 'yaw'):
            print('initial_pose_%s=%r' % (name, float(pose[name])))
        return 0
    if args.verify:
        profile_hash=digest(Path(args.sensor_profile)) if args.sensor_profile else None
        result = verify(args.verify, args.require_authorized, profile_hash)
        print(json.dumps({'verified': True,
                          'navigation_authorized': authorization(result)['navigation_authorized']}))
        return 0
    directory = Path(args.output_dir).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    audit = directory/'runtime_audit.json'
    mapping_archive = args.stale_epoch_archive or args.mapping_session_archive
    audit_result = subprocess.run(
        ['/usr/bin/python3', str(Path(__file__).with_name('audit_navigation_runtime.py')),
         '--duration','5','--json',str(audit)], check=not mapping_archive, timeout=20)
    report = json.loads(audit.read_text())
    profile=Path(__file__).resolve().parents[2]/'configs/go2w/navigation_sensor_profile.yaml'
    profile_hash=digest(profile) if profile.exists() else ''
    archive_blockers = stale_epoch_archive_blockers(
        report, sensor_profile_sha256=profile_hash) if mapping_archive else []
    if mapping_archive and (audit_result.returncode not in (0, 2) or archive_blockers):
        raise ValueError('mapping archive checks failed: ' + ','.join(archive_blockers))
    validation_status = (
        'stale_epoch_archive' if args.stale_epoch_archive
        else ('mapping_session_archive' if args.mapping_session_archive else 'stationary_only')
    )
    manifest = {'schema_version':'go2w_navigation_map_v1', 'created_at':time.time(),
                'frame':'map', 'source':'slam_toolbox_2d', 'scan':'/go2w/lidar/mapping_scan',
                'odom':'/go2w/odom/fused', 'navigation_authorized':False,
                'physical_validation_evidence':[],
                'validation_status':validation_status,
                'stale_epoch_archive':bool(args.stale_epoch_archive),
                'mapping_session_archive':bool(args.mapping_session_archive),
                'archive_readiness':not archive_blockers if mapping_archive else None,
                # 这份内嵌审计**只**覆盖它自己跑的那个 mode。写进 provenance 是为了
                # 让"拿 mapping 的审计去声称 localization 验收过"在文件层面就不成立。
                'audit_mode':report.get('audit_mode'), 'audit_healthy':report.get('healthy_for_plan'),
                'map_statistics':report['evidence']['map'], 'sha256':{},
                # 存档这一刻的位姿，将来是 AMCL 的初始位姿来源；拿不到就留 None。
                'initial_pose':None, 'odom_at_save':None}
    if profile.exists():
        profile_bytes=profile.read_bytes();profile_hash=hashlib.sha256(profile_bytes).hexdigest()
        runtime_geometry=report.get('sensor_geometry',{})
        if runtime_geometry.get('valid') is not True or runtime_geometry.get('profile_sha256')!=profile_hash:
            raise ValueError('cannot save a map with mismatched runtime sensor geometry')
        (directory/'sensor_profile.yaml').write_bytes(profile_bytes)
        manifest['navigation_geometry']=dict(sensor_profile_sha256=profile_hash,
            execution_scope=runtime_geometry['execution_scope'],manufacturer_calibration=False)
        manifest['occupancy_source']='/map (SLAM plus finite observed floor visibility)'
        manifest['posegraph_occupancy_source']='/go2w/slam/map_raw (SLAM only)'
    path = directory/'provenance.json'
    path.write_text(json.dumps(manifest,indent=2)+'\n')
    with (directory/'save.log').open('w') as log:
        run_owned_command(map_saver_command(directory/'map'), log=log)
        normalize_saved_image_path(directory/'map.yaml')
        validate_saved_thresholds(directory/'map.yaml')
        # A ROS service response must be checked; exit zero alone proves only transport success.
        import rclpy
        from slam_toolbox.srv import SerializePoseGraph
        rclpy.init();node=rclpy.create_node('go2w_map_bundle_saver')
        client=node.create_client(SerializePoseGraph,'/slam_toolbox/serialize_map')
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('posegraph serialization unavailable')
        if any((directory/name).exists() for name in ('map.posegraph','map.data')):
            raise RuntimeError('posegraph output already exists before serialization')
        request=SerializePoseGraph.Request();request.filename=str(directory/'map')
        future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        response=future.result() if future.done() else None
        validate_posegraph_response(response,directory)
        from tf2_ros import Buffer, TransformListener
        buffer=Buffer();TransformListener(buffer,node)
        deadline=time.time()+10
        while time.time()<deadline and rclpy.ok():
            # 必须一边 spin 一边查：TF 是回调灌进 buffer 的，光 sleep 永远查不到，
            # 而异常会被 lookup_pose 吞成"没位姿"。查不到 odom 就再等——它和 map
            # 是两条边，不一定同一拍到齐。
            rclpy.spin_once(node,timeout_sec=0.2)
            manifest['initial_pose']=manifest['initial_pose'] or lookup_pose(buffer,'map')
            manifest['odom_at_save']=lookup_pose(buffer,'odom_fused')
            if manifest['initial_pose'] and manifest['odom_at_save']:break
        node.destroy_node();rclpy.shutdown()
    for name in ('map.yaml','map.pgm','map.posegraph','map.data'):
        manifest['sha256'][name]=digest(directory/name)
    path.write_text(json.dumps(manifest,indent=2)+'\n')
    verify(directory/'map.yaml')
    print(json.dumps({'saved':str(directory), 'navigation_authorized':False,
                      'initial_pose':manifest['initial_pose'],
                      'audit_mode':manifest['audit_mode']}))
    return 0

if __name__=='__main__':raise SystemExit(main())
