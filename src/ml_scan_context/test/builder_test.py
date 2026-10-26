"""离线工具边界测试：坏输入不产出库，已有文件不覆盖。"""
import pathlib
import subprocess
import sys
import tempfile

binary = sys.argv[1]
with tempfile.TemporaryDirectory(prefix='polar_builder_test_') as temp:
    root = pathlib.Path(temp)
    scans = root / 'Scans'
    scans.mkdir()
    output = root / 'db.mlpc'

    def run(ok):
        result = subprocess.run([binary, str(scans), str(output), 'test_frame'], capture_output=True, text=True)
        assert (result.returncode == 0) == ok, result.stdout + result.stderr

    run(False)
    assert not output.exists()
    pcd = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
           'WIDTH 6\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 6\nDATA ascii\n'
           + ''.join(f'{1+i*.6} 0 0\n' for i in range(6)))
    (scans / '000007.pcd').write_text(pcd)
    (scans / '7.pcd').write_text(pcd)
    run(False)
    assert not output.exists()
    (scans / '7.pcd').unlink()
    (scans / '000007.pcd').write_text(pcd.replace('FIELDS x y z', 'FIELDS x y w'))
    run(False)
    assert not output.exists()
    (scans / '000007.pcd').write_text(pcd)
    run(True)
    before = output.read_bytes()
    assert before.startswith(b'MLPCDB')
    run(False)
    assert output.read_bytes() == before
print('builder boundary tests passed')
