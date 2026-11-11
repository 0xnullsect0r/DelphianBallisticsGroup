#!/usr/bin/env python3
"""Convert plain 3MF exports into Bambu Studio project 3MFs.

Bambu Studio (and Orca) store slicer settings inside the 3MF zip:
  3D/3dmodel.model                  geometry (patched: Application/Title/
                                    Description metadata so Bambu treats the
                                    file as its own project and keeps the
                                    settings instead of discarding them)
  Metadata/project_settings.config  flat JSON object of resolved settings
  Metadata/model_settings.config    XML with object/plate naming

Usage:
    python3 make_bambu_3mf.py            # builds both project files
"""

import json
import re
import shutil
import zipfile
from pathlib import Path

HERE = Path(__file__).parent
APP = "BambuStudio-01.10.00.89"      # identity; makes settings load

# ---------------------------------------------------------------- profiles
BLADE_PETG = {
    # process
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
    # filament (arrays are per-slot)
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

RING_PACF = {
    "layer_height": "0.2",
    "initial_layer_print_height": "0.2",
    "wall_loops": "4",
    "top_shell_layers": "5",
    "bottom_shell_layers": "4",
    "sparse_infill_density": "35%",
    "sparse_infill_pattern": "grid",
    "outer_wall_speed": "40",
    "inner_wall_speed": "80",
    "sparse_infill_speed": "100",
    "travel_speed": "200",
    "enable_support": "false",
    "nozzle_diameter": ["0.4"],
    "filament_type": ["PA-CF"],
    "filament_diameter": ["1.75"],
    "filament_colour": ["#3B3B3B"],
    "filament_flow_ratio": ["0.98"],
    "filament_max_volumetric_speed": ["6"],
    "nozzle_temperature": ["280"],
    "nozzle_temperature_initial_layer": ["280"],
    "bed_temperature": ["70"],
    "print_settings_id": "rallus-v3-ring",
    "filament_settings_id": "rallus-v3-pacf",
}

NOSE_PETG = {
    "layer_height": "0.2",
    "initial_layer_print_height": "0.2",
    "wall_loops": "4",
    "top_shell_layers": "5",
    "bottom_shell_layers": "4",
    "sparse_infill_density": "10%",
    "sparse_infill_pattern": "grid",
    "outer_wall_speed": "50",
    "inner_wall_speed": "100",
    "sparse_infill_speed": "120",
    "travel_speed": "250",
    "enable_support": "false",
    "nozzle_diameter": ["0.4"],
    "filament_type": ["PETG"],
    "filament_diameter": ["1.75"],
    "filament_colour": ["#FFFFFF"],
    "filament_flow_ratio": ["0.98"],
    "filament_max_volumetric_speed": ["8"],
    "nozzle_temperature": ["250"],
    "nozzle_temperature_initial_layer": ["250"],
    "bed_temperature": ["70"],
    "print_settings_id": "rallus-v4-nose",
    "filament_settings_id": "rallus-v3-petg",
}

NOSE_NOTES = (
    "Rallus II v4 printed nose cone. PETG or ASA. Print TIP DOWN as "
    "oriented, no supports (5 deg cone overhang). AFTER printing: fill the "
    "cavity with lead shot + epoxy through the base to ~285 g (assembled "
    "nose target ~360 g to preserve CG/margins - verify on a scale), then "
    "glue the base plug into the spigot bore. Shock cord through the base "
    "bore."
)

BLADE_NOTES = (
    "Rallus II v3 avionics blade. PETG (CF-nylon also fine). "
    "Print flat as oriented, no supports: all bridges are under 10 mm. "
    "4 walls / 30% grid infill per embedded profile. "
    "VERIFY dimensions in avionics_slide_mount.py against your parts "
    "before committing plastic."
)

RING_NOTES = (
    "Rallus II v3 retention ring. PA-CF / CF-nylon strongly recommended "
    "(clamp screws bite into this part). Hardened steel nozzle REQUIRED "
    "for CF filament. Dry the spool. Glue stick on smooth PEI. "
    "Flat as oriented, no supports. If printing in PETG instead, "
    "switch to the blade profile temperatures (250/70)."
)


def patch_model_xml(xml: str, title: str, desc: str) -> str:
    md = (
        f'  <metadata name="Application">{APP}</metadata>\n'
        f'  <metadata name="Title">{title}</metadata>\n'
        f'  <metadata name="Designer">Delphian Ballistics Group</metadata>\n'
        f'  <metadata name="Description">{desc}</metadata>\n'
    )
    # Core 3MF: model-level metadata goes right after the <model> open tag.
    m = re.search(r"<model[^>]*>", xml)
    assert m, "no <model> element"
    if "Application" not in xml:
        xml = xml[:m.end()] + "\n" + md + xml[m.end():]
    return xml


def model_settings(object_name: str) -> str:
    return (
        "<config>\n"
        "  <plate>\n"
        f'    <metadata key="name" value="{object_name}"/>\n'
        '    <metadata key="extruder" value="1"/>\n'
        "  </plate>\n"
        "  <object>\n"
        '    <metadata key="id" value="1"/>\n'
        f'    <metadata key="name" value="{object_name}"/>\n'
        '    <metadata key="extruder" value="1"/>\n'
        "  </object>\n"
        "</config>\n"
    )


def convert(src: Path, dst: Path, name: str, settings: dict, desc: str):
    with zipfile.ZipFile(src) as zin:
        members = {i.filename: zin.read(i.filename) for i in zin.infolist()}

    xml = members["3D/3dmodel.model"].decode()
    members["3D/3dmodel.model"] = patch_model_xml(xml, name, desc).encode()
    members["Metadata/project_settings.config"] = \
        json.dumps(settings, indent=1, sort_keys=True).encode()
    members["Metadata/model_settings.config"] = \
        model_settings(name).encode()

    # Make sure .config parts have a content type in the OPC manifest.
    ct = members["[Content_Types].xml"].decode()
    if 'Extension="config"' not in ct:
        ct = ct.replace(
            "</Types>",
            '<Default Extension="config" ContentType="text/plain"/></Types>')
        members["[Content_Types].xml"] = ct.encode()

    with zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
        for fn, data in members.items():
            zout.writestr(fn, data)
    print(f"wrote {dst.name}")


def main():
    convert(HERE / "avionics_blade.3mf", HERE / "avionics_blade_bambu.3mf",
            "avionics_blade", BLADE_PETG, BLADE_NOTES)
    convert(HERE / "retention_ring.3mf", HERE / "retention_ring_bambu.3mf",
            "retention_ring", RING_PACF, RING_NOTES)
    convert(HERE / "nosecone.3mf", HERE / "nosecone_bambu.3mf",
            "nosecone", NOSE_PETG, NOSE_NOTES)


if __name__ == "__main__":
    main()
