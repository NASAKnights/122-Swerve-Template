#!/usr/bin/env python3
import json
import argparse
from pathlib import Path
from typing import Dict, Any
from waypoint import Waypoint


class CalibratedPointIndexer:
    def __init__(self, file_path: str):
        """
        Initialize the indexer with the JSON file containing persistent POI values.

        :param file_path: The file path to the persistent POI JSON file.
        """
        self.file_path = Path(file_path)

    def build_index(self) -> Dict[str, Waypoint]:
        """
        Reads the JSON file and builds a dictionary mapping each POI name (the substring after
        '/SmartDashboard/POI/') to its corresponding Waypoint object.

        :return: A dictionary where keys are POI names and values are Waypoint objects.
        """
        try:
            with self.file_path.open("r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            print(f"Error reading {self.file_path}: {e}")
            return {}

        index: Dict[str, Waypoint] = {}
        prefix = "/SmartDashboard/POI/"
        for entry in data:
            full_name = entry.get("name", "")
            if full_name.startswith(prefix):
                # Extract the POI name after the prefix.
                key = full_name[len(prefix) :].strip()
                value_array = entry.get("value", [])
                try:
                    wp = Waypoint.from_value_array(value_array, name=key)
                    index[key] = wp
                except Exception as e:
                    print(f"Error processing entry '{full_name}': {e}")
        return index


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Build an index of persistent POI values from a JSON file."
    )
    parser.add_argument("file", help="Path to the persistent POI JSON file")
    args = parser.parse_args()

    indexer = CalibratedPointIndexer(args.file)
    poi_index = indexer.build_index()

    # Pretty-print the resulting index.
    import pprint

    pprint.pprint(poi_index)
