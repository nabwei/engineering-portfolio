"""Content-hash audit: identical image bytes must never cross a split boundary.

Grouping by physical-item folder name is necessary but not sufficient for an
item-disjoint evaluation. If two folders contain byte-identical images, a split
that separates the folders still places the same pixels on both sides of the
boundary. This module makes that condition observable and enforces one of two
explicit policies before any split is drawn:

* ``reject`` (default): raise if any identical content occurs in more than one
  item folder. Nothing is trained or evaluated.
* ``merge``: items connected by identical content are fused into one split group
  (label ``itemA+itemB``) so that the content stays on one side of every split and
  fold boundary. The merged group is recorded in every manifest.

There is no dataset-specific special case: the policy is a command-line choice and
the audit result is written beside every run. Only the standard library is used so
the command can be executed in isolated mode (``python -I``).
"""

import argparse
import hashlib
import json
from collections import defaultdict
from pathlib import Path

POLICIES = ("reject", "merge")


def image_entries(dataset, pattern="kos*/*.jpg"):
    entries = []
    for path in sorted(Path(dataset).glob(pattern)):
        entries.append({"item": path.parent.name, "image": path.name,
                        "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    if not entries:
        raise ValueError(f"no images matching {pattern} under {dataset}")
    return entries


def _name(entry):
    return f'{entry["item"]}/{entry.get("image") or Path(entry["path"]).name}'


def identical_content_groups(entries):
    """Groups of images (size >= 2) sharing one SHA-256, sorted by digest."""
    by_hash = defaultdict(list)
    for entry in entries:
        by_hash[entry["sha256"]].append(entry)
    return [{"sha256": digest, "images": [_name(member) for member in members],
             "items": sorted({member["item"] for member in members})}
            for digest, members in sorted(by_hash.items()) if len(members) > 1]


def cross_item_groups(groups):
    return [group for group in groups if len(group["items"]) > 1]


def merged_item_map(entries):
    """Map every item to its split group: items linked by identical content are fused."""
    parent = {}

    def find(item):
        parent.setdefault(item, item)
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = parent[item]
        return item

    for entry in entries:
        find(entry["item"])
    for group in cross_item_groups(identical_content_groups(entries)):
        roots = sorted(find(item) for item in group["items"])
        for other in roots[1:]:
            parent[other] = roots[0]
    members = defaultdict(set)
    for item in list(parent):
        members[find(item)].add(item)
    return {item: "+".join(sorted(members[find(item)])) for item in parent}


def content_audit(entries, policy):
    if policy not in POLICIES:
        raise ValueError(f"duplicate policy must be one of {POLICIES}")
    groups = identical_content_groups(entries)
    crossing = cross_item_groups(groups)
    mapping = merged_item_map(entries)
    return {"policy": policy,
            "image_count": len(entries),
            "item_folder_count": len({entry["item"] for entry in entries}),
            "distinct_content_count": len({entry["sha256"] for entry in entries}),
            "identical_content_groups": groups,
            "cross_item_groups": crossing,
            "cross_item_image_count": sum(len(group["images"]) for group in crossing),
            "merged_split_groups": sorted({label for label in mapping.values() if "+" in label}),
            "split_group_count": len(set(mapping.values())),
            "boundary_safe": not crossing or policy == "merge",
            "definition": "Images are identical when their file SHA-256 values are equal; "
                          "cross-item groups are identical images found in more than one item folder."}


def assign_split_groups(entries, policy):
    """Set ``group`` on every entry according to the policy; raise under ``reject``."""
    audit = content_audit(entries, policy)
    if audit["cross_item_groups"] and policy == "reject":
        shown = ", ".join(" == ".join(group["images"]) for group in audit["cross_item_groups"][:3])
        more = len(audit["cross_item_groups"]) - 3
        raise ValueError(
            f"identical image content crosses item folders ({audit['cross_item_image_count']} images in "
            f"{len(audit['cross_item_groups'])} groups): {shown}{f' and {more} more' if more > 0 else ''}; "
            "rerun with --duplicate-policy merge to keep identical content inside one split group")
    mapping = merged_item_map(entries) if policy == "merge" else {}
    for entry in entries:
        entry["group"] = mapping.get(entry["item"], entry["item"])
    return entries


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--policy", choices=POLICIES, default="reject")
    parser.add_argument("--output", type=Path, help="write the audit JSON to this new file")
    arguments = parser.parse_args()
    audit = content_audit(image_entries(arguments.dataset), arguments.policy)
    text = json.dumps(audit, indent=2)
    if arguments.output is not None:
        with arguments.output.open("x", encoding="utf-8") as stream:
            stream.write(text)
    print(text)
    raise SystemExit(0 if audit["boundary_safe"] else 1)


if __name__ == "__main__":
    main()
