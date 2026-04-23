"""Convert a URDF file to MuJoCo MJCF XML using MuJoCo's own compiler.

The conversion pipeline:

1. Parse the URDF and inject a ``<mujoco>`` configuration element (with
   ``<compiler>`` sub-element) into ``<robot>``. MuJoCo reads this block when
   loading a URDF to control meshdir, inertia handling, etc.
2. Write the annotated URDF to a sibling temp file so that relative mesh
   paths (``meshes/visual/foo.STL``) still resolve correctly.
3. Load the annotated URDF as an ``MjSpec`` and compile it.
4. Serialize the compiled spec back out with ``spec.to_xml()`` -- this is the
   canonical MJCF representation MuJoCo would produce itself.
"""

from __future__ import annotations

import argparse
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Optional
from xml.etree import ElementTree as ET

try:
    import mujoco
except ImportError as exc:
    raise SystemExit(
        "The `mujoco` package is required. Install it (e.g. `pixi install`)."
    ) from exc


DEFAULT_COMPILER_ATTRS: dict[str, str] = {
    "balanceinertia": "true",
    "discardvisual": "false",
    "fusestatic": "true",
    "strippath": "false",
}

MIN_MASS = 1e-6
MIN_INERTIA = 1e-9


def _inertia_is_valid(elem: ET.Element) -> bool:
    """Check <inertial> has positive mass and positive-definite inertia."""
    mass_el = elem.find("mass")
    inertia_el = elem.find("inertia")
    if mass_el is None or inertia_el is None:
        return False
    try:
        mass = float(mass_el.attrib.get("value", "0"))
        ixx = float(inertia_el.attrib.get("ixx", "0"))
        iyy = float(inertia_el.attrib.get("iyy", "0"))
        izz = float(inertia_el.attrib.get("izz", "0"))
        ixy = float(inertia_el.attrib.get("ixy", "0"))
        ixz = float(inertia_el.attrib.get("ixz", "0"))
        iyz = float(inertia_el.attrib.get("iyz", "0"))
    except ValueError:
        return False

    if mass <= MIN_MASS:
        return False
    if min(ixx, iyy, izz) <= MIN_INERTIA:
        return False
    if ixx + iyy < izz or iyy + izz < ixx or ixx + izz < iyy:
        return False
    if abs(ixy) > (ixx * iyy) ** 0.5 or abs(ixz) > (ixx * izz) ** 0.5 or abs(iyz) > (iyy * izz) ** 0.5:
        return False
    return True


def _set_nominal_inertial(inertial: ET.Element) -> None:
    """Overwrite a degenerate <inertial> with a tiny but valid one.

    Values are small enough to be effectively ignored after ``fusestatic``
    merges the link into its parent, but large enough to pass MuJoCo's
    positive-eigenvalue check.
    """
    for child in list(inertial):
        inertial.remove(child)
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": f"{MIN_MASS}"})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": f"{MIN_INERTIA}",
            "iyy": f"{MIN_INERTIA}",
            "izz": f"{MIN_INERTIA}",
            "ixy": "0",
            "ixz": "0",
            "iyz": "0",
        },
    )


def _sanitize_inertials(root: ET.Element, verbose: bool = False) -> int:
    """Replace degenerate <inertial> blocks with a nominal valid one.

    Returns the number of links that were fixed.
    """
    fixed = 0
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        if not _inertia_is_valid(inertial):
            _set_nominal_inertial(inertial)
            fixed += 1
            if verbose:
                print(f"[urdf2mjcf] sanitized inertial for link '{link.get('name')}'")
    return fixed


def _annotate_urdf(
    urdf_path: Path,
    meshdir: Optional[str],
    extra_compiler: Optional[dict[str, str]] = None,
    verbose: bool = False,
) -> str:
    """Return the URDF XML with a canonical ``<mujoco>`` block inserted
    and degenerate inertials sanitized for MuJoCo.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    if root.tag != "robot":
        raise ValueError(
            f"Expected root element <robot>, got <{root.tag}> in {urdf_path}"
        )

    n_fixed = _sanitize_inertials(root, verbose=verbose)
    if verbose and n_fixed:
        print(f"[urdf2mjcf] sanitized {n_fixed} degenerate inertial block(s)")

    for old in list(root.findall("mujoco")):
        root.remove(old)

    mj = ET.Element("mujoco")
    compiler_attrs = dict(DEFAULT_COMPILER_ATTRS)
    if meshdir is not None:
        compiler_attrs["meshdir"] = meshdir
    if extra_compiler:
        compiler_attrs.update(extra_compiler)
    ET.SubElement(mj, "compiler", compiler_attrs)

    root.insert(0, mj)
    return ET.tostring(root, encoding="unicode")


def convert(
    urdf_path: Path,
    mjcf_path: Path,
    meshdir: Optional[str] = None,
    copy_meshes: bool = False,
    extra_compiler: Optional[dict[str, str]] = None,
    verbose: bool = False,
) -> None:
    """Convert ``urdf_path`` to an MJCF file at ``mjcf_path``.

    Args:
        urdf_path: Input URDF file.
        mjcf_path: Output MJCF (.xml) file path.
        meshdir: Value for the MJCF ``<compiler meshdir=...>`` attribute.
            If None, MuJoCo uses the URDF file's directory as the base.
        copy_meshes: If True and the output dir differs from the URDF dir,
            copy the mesh tree next to the MJCF so it works standalone.
        extra_compiler: Extra ``<compiler>`` attributes to merge in.
        verbose: Print progress.
    """
    urdf_path = urdf_path.resolve()
    mjcf_path = mjcf_path.resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(urdf_path)

    annotated = _annotate_urdf(
        urdf_path,
        meshdir=meshdir,
        extra_compiler=extra_compiler,
        verbose=verbose,
    )

    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".urdf",
        dir=urdf_path.parent,
        delete=False,
        encoding="utf-8",
    ) as tmp:
        tmp.write(annotated)
        tmp_path = Path(tmp.name)

    if verbose:
        print(f"[urdf2mjcf] annotated URDF -> {tmp_path}")

    try:
        spec = mujoco.MjSpec.from_file(str(tmp_path))
        model = spec.compile()
    finally:
        tmp_path.unlink(missing_ok=True)

    if verbose:
        print(
            f"[urdf2mjcf] compiled: nbody={model.nbody} njnt={model.njnt} "
            f"ngeom={model.ngeom} nmesh={model.nmesh}"
        )

    mjcf_path.parent.mkdir(parents=True, exist_ok=True)
    xml_str = spec.to_xml()
    mjcf_path.write_text(xml_str, encoding="utf-8")

    if copy_meshes and urdf_path.parent != mjcf_path.parent:
        src_mesh_dir = urdf_path.parent / "meshes"
        if src_mesh_dir.is_dir():
            dst_mesh_dir = mjcf_path.parent / "meshes"
            if verbose:
                print(f"[urdf2mjcf] copying meshes {src_mesh_dir} -> {dst_mesh_dir}")
            shutil.copytree(src_mesh_dir, dst_mesh_dir, dirs_exist_ok=True)

    if verbose:
        print(f"[urdf2mjcf] wrote {mjcf_path} ({len(xml_str):,} bytes)")


def _parse_kv(pairs: list[str]) -> dict[str, str]:
    out: dict[str, str] = {}
    for p in pairs:
        if "=" not in p:
            raise argparse.ArgumentTypeError(f"Expected key=value, got: {p!r}")
        k, v = p.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Convert a URDF file to MuJoCo MJCF XML.",
    )
    ap.add_argument("urdf", type=Path, help="Input URDF file")
    ap.add_argument(
        "mjcf",
        type=Path,
        nargs="?",
        default=None,
        help="Output MJCF file (default: <urdf_stem>.xml next to the URDF)",
    )
    ap.add_argument(
        "--meshdir",
        type=str,
        default=None,
        help="Value for <compiler meshdir=...>. Defaults to URDF directory.",
    )
    ap.add_argument(
        "--copy-meshes",
        action="store_true",
        help="Also copy the meshes/ tree next to the output MJCF.",
    )
    ap.add_argument(
        "--compiler-option",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="Extra <compiler> attribute, e.g. --compiler-option angle=radian. "
             "Can be specified multiple times.",
    )
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    mjcf_path = args.mjcf or args.urdf.with_suffix(".xml")
    extra_compiler = _parse_kv(args.compiler_option)

    convert(
        urdf_path=args.urdf,
        mjcf_path=mjcf_path,
        meshdir=args.meshdir,
        copy_meshes=args.copy_meshes,
        extra_compiler=extra_compiler,
        verbose=args.verbose,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
