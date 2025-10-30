from __future__ import annotations

from typing import Dict, List, Set, Optional

from ..exceptions import ValidationError


class Modes:
    """Utility to parse architecture mode axes and test item enablement.

    Schema (no backward compatibility assumed):
      - Architecture `modes` is a list of axis blocks.
        Each block is a dict with a single key (axis name) and a list of entries.
        Each entry is a dict with keys: `mode` (str), optional `description`.

        Example:
            modes:
              - operation:
                - mode: Runtime
                - mode: Simulation
              - lidar_model:
                - mode: Centerpoint
                - mode: Transfusion

      - Items (components/connections/nodes) may include `modes` as either:
          a) dict: { axis: [mode_name, ...], ... }  (AND across axes)
          b) list of dicts as above, one dict per axis (equivalent to single dict)

    Selection:
      - `selection` is a dict axis -> selected mode name.
      - Omitting an axis in selection means wildcard (applies to all values of that axis).

    Matching rule:
      - An item matches if for every axis specified in the item's `modes`,
        the selected value for that axis (if provided) is among the allowed names.
      - Axes not listed in the item's `modes` are wildcards.
    """

    def __init__(self, available_axes: Dict[str, Set[str]]):
        self.available_axes: Dict[str, Set[str]] = available_axes

    @staticmethod
    def from_architecture_modes(modes_section: Optional[List[dict]]) -> "Modes":
        axes: Dict[str, Set[str]] = {}
        if not modes_section:
            # Provide a default axis/mode to cover empty case
            axes["default"] = {"default"}
            return Modes(axes)
        if not isinstance(modes_section, list):
            raise ValidationError("'modes' must be a list of axis blocks")
        for axis_block in modes_section:
            if not isinstance(axis_block, dict) or len(axis_block.keys()) != 1:
                raise ValidationError(f"Invalid modes axis block: {axis_block}")
            axis_name = next(iter(axis_block.keys()))
            entries = axis_block[axis_name]
            if not isinstance(entries, list):
                raise ValidationError(f"Axis '{axis_name}' entries must be a list")
            names: Set[str] = set()
            for entry in entries:
                if not isinstance(entry, dict) or 'mode' not in entry:
                    raise ValidationError(f"Axis '{axis_name}' entry invalid: {entry}")
                mode_name = entry['mode']
                if not isinstance(mode_name, str):
                    raise ValidationError(f"Axis '{axis_name}' mode must be string: {entry}")
                names.add(mode_name)
            axes[axis_name] = names
        return Modes(axes)

    def normalize_item_modes(self, item_modes: Optional[dict | List[dict]]) -> Dict[str, Set[str]]:
        """Normalize an item's `modes` field to dict axis -> set(names).

        Accepts:
          - None: wildcard (empty dict)
          - dict: { axis: [names] }
          - list of one-key dicts: [ {axis: [names]}, ... ]
        """
        if item_modes is None:
            return {}

        if isinstance(item_modes, dict):
            selectors = [item_modes]
        elif isinstance(item_modes, list):
            # flatten list of dicts into one dict (AND across axes)
            selectors = []
            combined: Dict[str, List[str]] = {}
            for d in item_modes:
                if not isinstance(d, dict) or len(d.keys()) != 1:
                    raise ValidationError(f"Invalid modes selector: {d}")
                axis = next(iter(d.keys()))
                names = d[axis]
                if isinstance(names, str):
                    names = [names]
                if not isinstance(names, list):
                    raise ValidationError(f"Invalid names list for axis '{axis}': {names}")
                combined.setdefault(axis, []).extend(names)
            selectors = [combined]
        else:
            raise ValidationError(f"Invalid 'modes' type: {type(item_modes)}")

        normalized: Dict[str, Set[str]] = {}
        for axis, names in selectors[0].items():
            if isinstance(names, str):
                names = [names]
            if not isinstance(names, list):
                raise ValidationError(f"Invalid names for axis '{axis}': {names}")
            normalized[axis] = set(names)
        return normalized

    @staticmethod
    def matches(selector: Dict[str, Set[str]], selection: Dict[str, str]) -> bool:
        """Return True if `selector` matches the given selection per rules.

        - For each axis in selector, selection MUST provide a value and it must
          be in the allowed set. If selection omits the axis, it does not match.
        - Axes not in selector are wildcards.
        """
        for axis, allowed in selector.items():
            sel_value = selection.get(axis)
            if sel_value is None:
                return False
            if sel_value not in allowed:
                return False
        return True

    def filter_items(self, items: List[dict], selection: Dict[str, str], name_key: Optional[str]) -> List[dict]:
        """Filter a list of config items (components/nodes/connections) by modes.

        `name_key` is used only to build duplicate detection keys for components/nodes.
        Connections use a synthesized key from 'from' and 'to'.
        """
        # First pass: collect matched items and group by key to detect duplicates with context
        grouped: Dict[str, List[dict]] = {}
        for item in items:
            selector = self.normalize_item_modes(item.get('modes'))
            if self.matches(selector, selection):
                if name_key is None:
                    key = f"{item.get('from')}->{item.get('to')}"
                else:
                    key = str(item.get(name_key))
                grouped.setdefault(key, []).append(item)

        if name_key in ('component', 'node'):
            # Detect duplicates and provide actionable diagnostics
            duplicates: List[str] = []
            for key, items_for_key in grouped.items():
                if len(items_for_key) > 1:
                    # Summarize mode constraints for each conflicting item
                    constraint_summaries: List[str] = []
                    relevant_axes: Set[str] = set()
                    for it in items_for_key:
                        sel = self.normalize_item_modes(it.get('modes'))
                        if sel:
                            axes_list = [f"{ax}={sorted(list(vals))}" for ax, vals in sel.items()]
                            constraint_summaries.append("{" + ", ".join(axes_list) + "}")
                            relevant_axes.update(sel.keys())
                        else:
                            constraint_summaries.append("{}")
                    duplicates.append(
                        f"'{key}' conflicts across variants with constraints: "
                        + "; ".join(constraint_summaries)
                        + (f". Specify selection for axes: {sorted(list(relevant_axes))}" if relevant_axes else "")
                    )
            if duplicates:
                raise ValidationError(
                    "Duplicate enablement under current mode selection: " + " | ".join(duplicates)
                )

        # Flatten grouped items in original insertion order per key
        enabled: List[dict] = []
        for items_for_key in grouped.values():
            enabled.extend(items_for_key)
        return enabled


