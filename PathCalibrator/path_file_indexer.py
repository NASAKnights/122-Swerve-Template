#!/usr/bin/env python3
import json
from pathlib import Path
from typing import Dict, List, Any
import argparse
from waypoint import Waypoint


class PathFileIndexer:
    def __init__(self, directory: str):
        """
        Initialize the indexer with the directory to scan.

        :param directory: The directory to search for .path files.
        """
        self.directory = Path(directory)

    def build_index(self) -> Dict[str, List[Waypoint]]:
        """
        Scans the directory recursively for .path files and builds a dictionary
        mapping each linkedName to a list of Waypoint objects created from the "anchor" data.

        :return: A dictionary where keys are linkedName strings and values are lists
                 of Waypoint objects.
        """
        index: Dict[str, List[Waypoint]] = {}

        # Loop over all .path files recursively
        for file_path in Path.cwd().rglob("**/*.path"):
            try:
                with file_path.open("r", encoding="utf-8") as f:
                    data = json.load(f)
            except Exception as e:
                print(f"Error reading {file_path}: {e}")
                continue

            waypoints = data.get("waypoints", [])
            for waypoint in waypoints:
                linked_name = waypoint.get("linkedName")
                if linked_name:
                    anchor = waypoint.get("anchor", {})
                    # Create a Waypoint using the anchor information and the linked name.
                    wp = Waypoint.from_anchor(anchor, name=linked_name)
                    index.setdefault(linked_name, []).append(wp)
        return index


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Copy persistent NetworkTables POI values into matching linkedName poses in a PathPlanner JSON file."
    )
    parser.add_argument("path_dir", help="Path to the PathPlanner JSON file")
    args = parser.parse_args()

    # Example usage: change "path_files" to your directory containing .path files.
    indexer = PathFileIndexer(args.path_dir)
    linked_points = indexer.build_index()

    # Print the resulting dictionary in a pretty format.
    import pprint

    pprint.pprint(linked_points)
