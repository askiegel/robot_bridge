import hashlib

import pytest

from localization_control import LocalizationControl
from navigation_control import NavigationControl
from planning_control import PlanningControl


CONTROLS = (
    LocalizationControl,
    PlanningControl,
    NavigationControl,
)


def _sha256(path):
    return hashlib.sha256(
        path.read_bytes()
    ).hexdigest()


def _make_raster_map(directory):
    yaml_path = (
        directory
        / "mayday_supervised_route_03.yaml"
    )

    pgm_path = (
        directory
        / "mayday_supervised_route_03.pgm"
    )

    yaml_path.write_text(
        "image: mayday_supervised_route_03.pgm\n"
        "resolution: 0.05\n"
        "origin: [-2.6, -2.55, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n",
        encoding="utf-8",
    )

    pgm_path.write_bytes(
        b"P5\n"
        b"1 1\n"
        b"255\n"
        b"\x00"
    )

    manifest = directory / "SHA256SUMS"

    manifest.write_text(
        f"{_sha256(yaml_path)}  "
        f"{yaml_path.name}\n"
        f"{_sha256(pgm_path)}  "
        f"{pgm_path.name}\n",
        encoding="utf-8",
    )


@pytest.mark.parametrize(
    "control_class",
    CONTROLS,
)
def test_runtime_accepts_checksum_verified_raster_map(
    tmp_path,
    control_class,
):
    _make_raster_map(tmp_path)

    control = control_class(
        map_directory=tmp_path,
    )

    control._verify_artifacts()


@pytest.mark.parametrize(
    "control_class",
    CONTROLS,
)
def test_runtime_rejects_tampered_raster_map(
    tmp_path,
    control_class,
):
    _make_raster_map(tmp_path)

    (
        tmp_path
        / "mayday_supervised_route_03.pgm"
    ).write_bytes(b"tampered")

    control = control_class(
        map_directory=tmp_path,
    )

    with pytest.raises(Exception):
        control._verify_artifacts()
