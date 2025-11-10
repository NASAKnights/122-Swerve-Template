#!/usr/bin/env python3
import json
from dataclasses import dataclass, asdict
from typing import List, Dict, Any


@dataclass
class Waypoint:
    name: str = ""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    @classmethod
    def from_value_array(cls, arr: List[float], name: str = "") -> "Waypoint":
        """
        Create a Waypoint from an array of values.
        Expected format: [x, y, z, qx, qy, qz, qw]
        (Note: This ignores the time element if present.)
        """
        if len(arr) < 7:
            raise ValueError("Expected a value array with at least 7 elements.")
        return cls(
            name=name,
            x=float(arr[0]),
            y=float(arr[1]),
            z=float(arr[2]),
            qx=float(arr[3]),
            qy=float(arr[4]),
            qz=float(arr[5]),
            qw=float(arr[6]),
        )

    def to_value_array(self) -> List[float]:
        """
        Convert the Waypoint into a value array.
        Format: [x, y, z, qx, qy, qz, qw]
        """
        return [self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw]

    @classmethod
    def from_anchor(cls, anchor: Dict[str, Any], name: str = "") -> "Waypoint":
        """
        Create a Waypoint from an anchor dictionary (as in .path files).
        Expected keys: "x" and "y". Other fields will be defaulted (z=0, identity quaternion).
        """
        x = float(anchor.get("x", 0.0))
        y = float(anchor.get("y", 0.0))
        return cls(name=name, x=x, y=y)

    def to_anchor(self) -> Dict[str, float]:
        """
        Convert the Waypoint to an anchor dictionary (only x and y).
        """
        return {"x": self.x, "y": self.y}

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert the Waypoint to a dictionary including all fields.
        """
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Waypoint":
        """
        Create a Waypoint from a dictionary.
        """
        return cls(**data)


# Example usage:
if __name__ == "__main__":
    # Example: Converting from a persistent POI value array, with a name.
    poi_value = [9.962750400000001, 4.057394024242425, 0.0, 0.0, 0.0, 0.0, 1.0]
    waypoint_from_poi = Waypoint.from_value_array(poi_value, name="TestPOI")
    print("Waypoint from value array:")
    print(waypoint_from_poi)
    print("As value array:", waypoint_from_poi.to_value_array())
    print("As dictionary:", waypoint_from_poi.to_dict())

    # Example: Converting from an anchor dictionary (as in a .path file), with a name.
    anchor = {"x": 5.8, "y": 4.0}
    waypoint_from_anchor = Waypoint.from_anchor(anchor, name="A2")
    print("\nWaypoint from anchor:")
    print(waypoint_from_anchor)
    print("As anchor dictionary:", waypoint_from_anchor.to_anchor())

    # Serialize the waypoint to JSON and then parse it back.
    waypoint_json = json.dumps(waypoint_from_poi.to_dict(), indent=2)
    print("\nWaypoint JSON representation:")
    print(waypoint_json)
    waypoint_dict = json.loads(waypoint_json)
    waypoint_from_json = Waypoint.from_dict(waypoint_dict)
    print("\nWaypoint reconstructed from JSON:")
    print(waypoint_from_json)
