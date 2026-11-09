"""Seeded RRT* in two-joint configuration space; radians and Euclidean joint metric."""

from dataclasses import dataclass
import numpy as np


@dataclass
class Node:
    configuration: np.ndarray
    parent: int
    cost: float


def edge_free(first, second, collision_free):
    intervals = max(1, int(np.ceil(np.linalg.norm(second - first) / 0.025)))
    return all(
        collision_free(first + (second - first) * fraction)
        for fraction in np.linspace(0, 1, intervals + 1)
    )


def update_descendants(nodes, ancestor, difference):
    for index, node in enumerate(nodes):
        if node.parent == ancestor:
            node.cost += difference
            update_descendants(nodes, index, difference)


def rewire(nodes, new_index, neighbors, collision_free):
    newest = nodes[new_index]
    for index in neighbors:
        node = nodes[index]
        candidate = newest.cost + np.linalg.norm(
            node.configuration - newest.configuration
        )
        if candidate + 1e-12 >= node.cost or not edge_free(
            newest.configuration, node.configuration, collision_free
        ):
            continue
        difference = candidate - node.cost
        node.parent, node.cost = new_index, candidate
        update_descendants(nodes, index, difference)


def plan(start, goal, collision_free, iterations=450):
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    if not collision_free(start) or not collision_free(goal):
        raise ValueError("start or goal collides or violates joint limits")
    random = np.random.default_rng(27)
    nodes = [Node(start, -1, 0.0)]
    for iteration in range(iterations):
        sample = goal if iteration % 8 == 0 else random.uniform(-2.8, 2.8, 2)
        distances = [np.linalg.norm(node.configuration - sample) for node in nodes]
        nearest = int(np.argmin(distances))
        delta = sample - nodes[nearest].configuration
        if np.linalg.norm(delta) < 1e-10:
            continue
        candidate = nodes[nearest].configuration + delta * min(
            1, 0.25 / np.linalg.norm(delta)
        )
        if not edge_free(nodes[nearest].configuration, candidate, collision_free):
            continue
        neighbors = [
            index
            for index, node in enumerate(nodes)
            if np.linalg.norm(node.configuration - candidate) < 0.65
        ]
        feasible = [
            index
            for index in neighbors
            if edge_free(nodes[index].configuration, candidate, collision_free)
        ]
        parent = min(
            feasible,
            key=lambda index: nodes[index].cost
            + np.linalg.norm(nodes[index].configuration - candidate),
        )
        nodes.append(
            Node(
                candidate,
                parent,
                nodes[parent].cost
                + np.linalg.norm(nodes[parent].configuration - candidate),
            )
        )
        rewire(nodes, len(nodes) - 1, neighbors, collision_free)
    connections = [
        index
        for index, node in enumerate(nodes)
        if edge_free(node.configuration, goal, collision_free)
    ]
    if not connections:
        raise RuntimeError("RRT* exhausted iteration budget without a path")
    nearest = min(
        connections,
        key=lambda index: nodes[index].cost
        + np.linalg.norm(nodes[index].configuration - goal),
    )
    path = [goal]
    while nearest != -1:
        path.append(nodes[nearest].configuration.copy())
        nearest = nodes[nearest].parent
    return list(reversed(path))
