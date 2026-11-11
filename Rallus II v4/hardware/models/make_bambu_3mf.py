#!/usr/bin/env python3
"""Convert plain 3MF exports into Bambu Studio project 3MFs.

Structure mirrors a real Bambu Studio 02.07 saved project (verified against
a known-good reference file):

  3D/3dmodel.model           geometry, patched with the BambuStudio
                             namespace + metadata block (Application,
                             BambuStudio:3mfVersion, Title, Description...)
  Metadata/project_settings.config   flat JSON of slicer settings
  Metadata/model_settings.config     XML: per-object entries (name, extruder,
                             face_count, part w/ matrix + mesh_stat) and the
                             plate block with model_instance list
  Metadata/slice_info.config         minimal X-BBL client header

Usage:
    python3 make_bambu_3mf.py            # all *_bambu.3mf files
"""

import json
import re
import zipfile
from datetime import date
from pathlib import Path

HERE = Path(__file__).parent
APP = "BambuStudio-02.07.01.62"
TODAY = date.today().isoformat()

# ---------------------------------------------------------------- profiles
BLADE_PETG = {
    "layer_height": "0.2",
    "initial_layer_print_height": "0.2",
    "wall_loops": "4",
    "top_shell_layers": "5",
    "bottom_shell_layers": "4",
    "sparse_infill_density": "30%",
    "sparse_infill_pattern": "grid",
    "outer_wall_speed": "60",
    "inner_wall_speed": "120",
    "sparse_infill_speed": "150",
    "travel_speed": "250",
    "enable_support": "false",
    "nozzle_diameter": ["0.4"],
    "filament_type": ["PETG"],
    "filament_diameter": ["1.75"],
    "filament_colour": ["#00A651"],
    "filament_flow_ratio": ["0.98"],
    "filament_max_volumetric_speed": ["8"],
    "nozzle_temperature": ["250"],
    "nozzle_temperature_initial_layer": ["250"],
    "bed_temperature": ["70"],
    "print_settings_id": "rallus-v3-blade",
    "filament_settings_id": "rallus-v3-petg",
}

RING_PACF = dict(BLADE_PETG, **{
    "sparse_infill_density": "35%",
    "outer_wall_speed": "40",
    "inner_wall_speed": "80",
    "sparse_infill_speed": "100",
    "travel_speed": "200",
    "filament_type": ["PA-CF"],
    "filament_colour": ["#3B3B3B"],
    "filament_max_volumetric_speed": ["6"],
    "nozzle_temperature": ["280"],
    "nozzle_temperature_initial_layer": ["280"],
    "print_settings_id": "rallus-v3-ring",
    "filament_settings_id": "rallus-v3-pacf",
})

NOSE_PETG = dict(BLADE_PETG, **{
    "sparse_infill_density": "10%",
    "outer_wall_speed": "50",
    "inner_wall_speed": "100",
    "sparse_infill_speed": "120",
    "filament_colour": ["#FFFFFF"],
    "print_settings_id": "rallus-v4-nose",
})

BLADE_NOTES = (
    "Rallus avionics blade. PETG (CF-nylon also fine). Print flat as "
    "oriented, no supports: bridges under 10 mm. 4 walls / 30% grid per "
    "embedded profile. VERIFY dimensions against your parts first."
)
RING_NOTES = (
    "Rallus retention ring. PA-CF / CF-nylon strongly recommended; "
    "hardened nozzle REQUIRED, dry the spool, glue stick on smooth PEI. "
    "Flat as oriented, no supports. For PETG use 250/70 instead."
)
NOSE_NOTES = (
    "Rallus nose piece (split cone; X1C beds are 256 mm, the full cone is "
    "299 mm). PETG or ASA, pointy end down, no supports. Glue base + tip "
    "at the sleeve joint (epoxy, 0.25 mm glue gap) BEFORE ballasting."
)
PLATE_NOTES = (
    "Rallus full part set: nose cone, avionics blade, retention ring, "
    "base plug. Settings embedded are the PETG set; print the retention "
    "ring separately from retention_ring_bambu.3mf if you want PA-CF. "
    "Nose tip-down, blade flat, ring flat."
)


# ---------------------------------------------------------------- helpers
def parse_objects(xml):
    """[(id, name, face_count)] for every mesh object in a 3dmodel XML."""
    out = []
    for m in re.finditer(r'<object\s+id="(\d+)"[^>]*>(.*?)</object>', xml, re.S):
        oid, body = m.group(1), m.group(2)
        if "<mesh>" not in body:
            continue
        nm = re.search(r'partnumber="([^"]*)"', m.group(0))
        faces = body.count("<triangle")
        out.append((int(oid), nm.group(1) if nm else f"object_{oid}", faces))
    return out


def patch_model_xml(xml, title, desc):
    md = (
        f' <metadata name="Application">{APP}</metadata>\n'
        ' <metadata name="BambuStudio:3mfVersion">1</metadata>\n'
        ' <metadata name="Copyright"></metadata>\n'
        f' <metadata name="CreationDate">{TODAY}</metadata>\n'
        f' <metadata name="Description">{desc}</metadata>\n'
        ' <metadata name="Designer">Delphian Ballistics Group</metadata>\n'
        ' <metadata name="License"></metadata>\n'
        f' <metadata name="ModificationDate">{TODAY}</metadata>\n'
        ' <metadata name="Origin"></metadata>\n'
        f' <metadata name="Title">{title}</metadata>\n'
    )
    if "BambuStudio:3mfVersion" in xml:
        return xml
    m = re.search(r"<model[^>]*>", xml)
    assert m, "no <model> element"
    tag = m.group(0)
    if "xmlns:BambuStudio" not in tag:
        xml = xml.replace(
            tag, tag[:-1] +
            ' xmlns:BambuStudio="http://schemas.bambulab.com/package/2021">', 1)
    m = re.search(r"<model[^>]*>", xml)
    return xml[:m.end()] + "\n" + md + xml[m.end():]


def model_settings(objects, plate_name):
    objs = []
    for oid, name, faces in objects:
        objs.append(
            f'  <object id="{oid}">\n'
            f'    <metadata key="name" value="{name}"/>\n'
            '    <metadata key="extruder" value="1"/>\n'
            f'    <metadata face_count="{faces}"/>\n'
            '    <part id="1" subtype="normal_part">\n'
            f'      <metadata key="name" value="{name}"/>\n'
            '      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>\n'
            f'      <metadata key="source_file" value="{name}.stl"/>\n'
            '      <metadata key="source_object_id" value="0"/>\n'
            '      <metadata key="source_volume_id" value="0"/>\n'
            '      <metadata key="source_offset_x" value="0.000000"/>\n'
            '      <metadata key="source_offset_y" value="0.000000"/>\n'
            '      <metadata key="source_offset_z" value="0.000000"/>\n'
            f'      <mesh_stat face_count="{faces}" edges_fixed="0" '
            'degenerate_facets="0" facets_removed="0" facets_reversed="0" '
            'backwards_edges="0"/>\n'
            '    </part>\n'
            '  </object>\n')
    inst = "".join(
        f'    <model_instance>\n'
        f'      <metadata key="object_id" value="{oid}"/>\n'
        f'      <metadata key="instance_id" value="0"/>\n'
        f'      <metadata key="identify_id" value="{oid * 1000}"/>\n'
        f'    </model_instance>\n'
        for oid, _, _ in objects)
    return ('<?xml version="1.0" encoding="UTF-8"?>\n<config>\n' + "".join(objs) +
            '  <plate>\n'
            '    <metadata key="plater_id" value="1"/>\n'
            f'    <metadata key="plater_name" value="{plate_name}"/>\n'
            '    <metadata key="locked" value="false"/>\n'
            '    <metadata key="filament_map_mode" value="Auto For Flush"/>\n'
            + inst + '  </plate>\n</config>\n')


SLICE_INFO = (
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<config>\n'
    '  <header>\n'
    '    <header_item key="X-BBL-Client-Type" value="slicer"/>\n'
    f'    <header_item key="X-BBL-Client-Version" value="{APP.split("-")[1]}"/>\n'
    '  </header>\n'
    '</config>\n'
)

CONTENT_TYPES = (
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">\n'
    ' <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>\n'
    ' <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>\n'
    ' <Default Extension="png" ContentType="image/png"/>\n'
    ' <Default Extension="gcode" ContentType="text/x.gcode"/>\n'
    '</Types>'
)


def convert(src: Path, dst: Path, plate_name: str, settings: dict, desc: str):
    with zipfile.ZipFile(src) as zin:
        members = {i.filename: zin.read(i.filename) for i in zin.infolist()}

    xml = members["3D/3dmodel.model"].decode()
    objects = parse_objects(xml)
    assert objects, "no mesh objects found"
    members["3D/3dmodel.model"] = patch_model_xml(xml, plate_name, desc).encode()
    members["Metadata/project_settings.config"] = \
        json.dumps(settings, indent=4, sort_keys=True).encode()
    members["Metadata/model_settings.config"] = \
        model_settings(objects, plate_name).encode()
    members["Metadata/slice_info.config"] = SLICE_INFO.encode()
    members["[Content_Types].xml"] = CONTENT_TYPES.encode()

    with zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
        for fn, data in members.items():
            zout.writestr(fn, data)
    print(f"wrote {dst.name}  objects: {[o[1] for o in objects]}")


def main():
    convert(HERE / "avionics_blade.3mf", HERE / "avionics_blade_bambu.3mf",
            "avionics_blade", BLADE_PETG, BLADE_NOTES)
    convert(HERE / "retention_ring.3mf", HERE / "retention_ring_bambu.3mf",
            "retention_ring", RING_PACF, RING_NOTES)
    convert(HERE / "nose_base.3mf", HERE / "nose_base_bambu.3mf",
            "nose_base", NOSE_PETG, NOSE_NOTES)
    convert(HERE / "nose_tip.3mf", HERE / "nose_tip_bambu.3mf",
            "nose_tip", NOSE_PETG, NOSE_NOTES)
    plate = HERE / "all_parts.3mf"
    if plate.exists():
        convert(plate, HERE / "rallus_plate_bambu.3mf",
                "rallus_v4_all_parts", BLADE_PETG, PLATE_NOTES)


if __name__ == "__main__":
    main()
