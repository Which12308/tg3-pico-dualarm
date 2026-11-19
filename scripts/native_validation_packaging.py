"""Validate the actual colcon reader and built wheel metadata, not a mock."""
import argparse
import configparser
from email.parser import BytesParser
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wheel',type=Path,required=True)
    parser.add_argument('--check-colcon',action='store_true')
    args=parser.parse_args()
    result=dict(passed=False,actual_colcon_reader_executed=False,robot_commands_sent=False,
                physical_robot_tested=False,headset_tested=False)
    source=Path('/home/tg3/project.zip')
    if source.exists():result['source_bundle_sha256']=hashlib.file_digest(source.open('rb'),'sha256').hexdigest()
    result['wheel_sha256']=hashlib.file_digest(args.wheel.open('rb'),'sha256').hexdigest()
    with zipfile.ZipFile(args.wheel) as wheel:
        metadata=BytesParser().parsebytes(wheel.read(next(n for n in wheel.namelist() if n.endswith('.dist-info/METADATA'))))
        assert metadata['Requires-Python']=='>=3.10',metadata['Requires-Python']
        entries=configparser.ConfigParser()
        entries.read_string(wheel.read(next(n for n in wheel.namelist() if n.endswith('.dist-info/entry_points.txt'))).decode())
        assert entries['console_scripts']['bridge_node']=='tg3_pico_dualarm.ros_bridge:main'
        assert entries['console_scripts']['tg3-vr']=='tg3_pico_dualarm.cli:main'
        assert 'tg3_pico_dualarm/assets/tiangong3.urdf' in wheel.namelist()
        result.update(wheel_requires_python=metadata['Requires-Python'],version=metadata['Version'],wheel_entry_points=True)
    if args.check_colcon:
        from colcon_python_setup_py.package_identification.python_setup_py import _get_setup_information
        data=_get_setup_information(ROOT/'setup.py')
        assert data['python_requires']=='>=3.10',data['python_requires']
        assert data['metadata']['python_requires']=='>=3.10'
        result['actual_colcon_reader_executed']=True
    result['passed']=True
    (ROOT/'artifacts').mkdir(exist_ok=True)
    (ROOT/'artifacts/native_packaging_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
