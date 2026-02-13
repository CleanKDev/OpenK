import numpy as np

from utils.scurve_profile import run_scurve_self_test


def test_scurve_profile_limits(tmp_path):
    out_path = tmp_path / "scurve_test.npz"
    result = run_scurve_self_test(
        distance=40.0,
        T=2.0,
        dt=0.01,
        v_lim=50.0,
        a_lim=150.0,
        j_lim=400.0,
        save_path=out_path,
    )
    assert out_path.is_file()
    assert result["max_velocity"] <= 50.0 + 1e-6


def test_scurve_profile_npz_contents(tmp_path):
    out_path = tmp_path / "scurve_test_arrays.npz"
    run_scurve_self_test(save_path=out_path)
    with np.load(out_path) as data:
        assert "t" in data and "q" in data and "dq" in data
