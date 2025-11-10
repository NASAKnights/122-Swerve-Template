#!/usr/bin/env python3
import json
import argparse
from pathlib import Path
from typing import Dict, List, Any

# Import your classes. Adjust the module names as needed.
from waypoint import Waypoint
from path_file_indexer import PathFileIndexer
from calibrated_point_indexer import CalibratedPointIndexer


def update_paths_from_calibrated(poi_file: str, path_dir: str) -> None:
    """
    For every .path file in path_dir, update any waypoint with a linkedName that exists
    in the calibrated POI file. The waypoint's anchor (x,y) will be replaced with the POI's values.
    """
    # Build POI index (mapping POI name -> Waypoint)
    poi_indexer = CalibratedPointIndexer(poi_file)
    poi_index: Dict[str, Waypoint] = poi_indexer.build_index()

    path_dir_path = Path(path_dir)
    for path_file in path_dir_path.rglob("**/*.path"):
        try:
            with path_file.open("r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            print(f"Error reading {path_file}: {e}")
            continue

        updated = False
        waypoints = data.get("waypoints", [])
        for waypoint in waypoints:
            linked_name = waypoint.get("linkedName")
            if linked_name and linked_name in poi_index:
                poi_wp = poi_index[linked_name]
                # Update the anchor dictionary with the POI's x,y values.
                if "anchor" in waypoint:
                    waypoint["anchor"]["x"] = poi_wp.x
                    waypoint["anchor"]["y"] = poi_wp.y
                else:
                    waypoint["anchor"] = {"x": poi_wp.x, "y": poi_wp.y}
                updated = True
                print(
                    f"Updated {path_file} waypoint '{linked_name}' to x={poi_wp.x}, y={poi_wp.y}"
                )
        if updated:
            try:
                # with path_file.open("w", encoding="utf-8") as f:
                # json.dump(data, f, indent=2)
                print(f"Written updates to file: {path_file}")
            except Exception as e:
                print(f"Error writing {path_file}: {e}")


def update_calibrated_from_paths(poi_file: str, path_dir: str) -> None:
    """
    Loads the .path files in path_dir, and for each POI entry in the calibrated file,
    updates its value array with the first matching waypoint from the .path index.
    """
    # Build path file index (mapping linkedName -> list of Waypoint objects)
    path_indexer = PathFileIndexer(path_dir)
    path_index: Dict[str, List[Waypoint]] = path_indexer.build_index()

    poi_path = Path(poi_file)
    try:
        with poi_path.open("r", encoding="utf-8") as f:
            poi_data = json.load(f)
    except Exception as e:
        print(f"Error reading {poi_file}: {e}")
        return

    prefix = "/SmartDashboard/POI/"
    updated = False
    # poi_data is expected to be a list of entries.
    for entry in poi_data:
        full_name = entry.get("name", "")
        if full_name.startswith(prefix):
            key = full_name[len(prefix) :].strip()
            if key in path_index and path_index[key]:
                # Use the first waypoint from the .path file index.
                wp = path_index[key][0]
                # Update the value array using the Waypoint's data.
                entry["value"] = wp.to_value_array()
                updated = True
                print(
                    f"Updated POI entry '{full_name}' with value array {wp.to_value_array()}"
                )
    if updated:
        try:
            # with poi_path.open("w", encoding="utf-8") as f:
            # json.dump(poi_data, f, indent=2)
            print(f"Calibrated POI file updated: {poi_file}")
        except Exception as e:
            print(f"Error writing {poi_file}: {e}")
    else:
        print("No updates made to the calibrated POI file.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build both indexes and copy either from the calibrated file into .path files "
        "or from .path files' linked waypoints into the calibrated file."
    )
    parser.add_argument(
        "poi_file", help="Path to the calibrated persistent POI JSON file"
    )
    parser.add_argument("path_dir", help="Directory containing .path files")
    parser.add_argument(
        "--mode",
        choices=["cal_to_paths", "paths_to_cal"],
        required=True,
        help="Direction of update: 'cal_to_paths' to copy from calibrated file into .path files, "
        "or 'paths_to_cal' to copy from .path files into the calibrated file.",
    )
    args = parser.parse_args()

    if args.mode == "cal_to_paths":
        update_paths_from_calibrated(args.poi_file, args.path_dir)
    elif args.mode == "paths_to_cal":
        update_calibrated_from_paths(args.poi_file, args.path_dir)


if __name__ == "__main__":
    main()
