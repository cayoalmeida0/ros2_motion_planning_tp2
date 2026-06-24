#!/usr/bin/env python3

"""
Planejador RRT para o TP2.

O RRT sorteia pontos livres no mapa, expande uma árvore a partir do start
e tenta conectar essa árvore ao goal.

A verificação de colisão é feita usando o grid expandido gerado pelo grid_map.py.
"""

import argparse
import math
import os
import random
from dataclasses import dataclass
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.colors import ListedColormap

try:
    from ld90_gz.grid_map import (
        MapBounds,
        grid_to_world,
        is_cell_free,
        load_grid_package,
        world_to_grid,
    )
except ImportError:
    from grid_map import (
        MapBounds,
        grid_to_world,
        is_cell_free,
        load_grid_package,
        world_to_grid,
    )


Point2D = Tuple[float, float]
GridCell = Tuple[int, int]


@dataclass
class Node:
    x: float
    y: float
    parent: Optional[int]


def distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_is_free(
    point: Point2D,
    grid: np.ndarray,
    bounds: MapBounds,
    resolution: float,
) -> bool:
    row, col = world_to_grid(point[0], point[1], bounds, resolution)
    return is_cell_free(grid, row, col)


def segment_is_free(
    p1: Point2D,
    p2: Point2D,
    grid: np.ndarray,
    bounds: MapBounds,
    resolution: float,
    step: float,
) -> bool:
    """
    Testa se o segmento p1 -> p2 está livre.

    O segmento é dividido em vários pontos intermediários.
    Cada ponto é convertido para célula do grid e testado contra obstáculos.
    """
    d = distance(p1, p2)

    if d == 0.0:
        return point_is_free(p1, grid, bounds, resolution)

    n = max(2, int(math.ceil(d / step)))

    for i in range(n + 1):
        t = i / n
        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])

        if not point_is_free((x, y), grid, bounds, resolution):
            return False

    return True


def nearest_free_point(
    grid: np.ndarray,
    point: Point2D,
    bounds: MapBounds,
    resolution: float,
    max_radius: int = 60,
) -> Optional[Point2D]:
    """
    Ajusta start ou goal para uma célula livre próxima, caso necessário.
    """
    row, col = world_to_grid(point[0], point[1], bounds, resolution)

    if is_cell_free(grid, row, col):
        return point

    for radius in range(1, max_radius + 1):
        candidates = []

        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                rr = row + dr
                cc = col + dc

                if is_cell_free(grid, rr, cc):
                    dist = math.hypot(dr, dc)
                    candidates.append((dist, (rr, cc)))

        if candidates:
            candidates.sort(key=lambda item: item[0])
            best_row, best_col = candidates[0][1]
            return grid_to_world(best_row, best_col, bounds, resolution)

    return None


def sample_point(
    grid: np.ndarray,
    bounds: MapBounds,
    resolution: float,
    goal: Point2D,
    goal_bias: float,
) -> Point2D:
    """
    Sorteia um ponto livre.

    Em algumas iterações, o próprio goal é usado como amostra.
    Isso ajuda a árvore a crescer em direção ao objetivo.
    """
    if random.random() < goal_bias:
        return goal

    while True:
        x = random.uniform(bounds.xmin, bounds.xmax)
        y = random.uniform(bounds.ymin, bounds.ymax)

        if point_is_free((x, y), grid, bounds, resolution):
            return x, y


def nearest_node(nodes: List[Node], point: Point2D) -> int:
    """
    Encontra o nó da árvore mais próximo do ponto sorteado.
    """
    best_index = 0
    best_distance = float("inf")

    for i, node in enumerate(nodes):
        d = distance((node.x, node.y), point)

        if d < best_distance:
            best_distance = d
            best_index = i

    return best_index


def steer(from_point: Point2D, to_point: Point2D, step_size: float) -> Point2D:
    """
    Avança de um ponto em direção ao outro, respeitando o passo máximo.
    """
    d = distance(from_point, to_point)

    if d <= step_size:
        return to_point

    theta = math.atan2(to_point[1] - from_point[1], to_point[0] - from_point[0])

    x = from_point[0] + step_size * math.cos(theta)
    y = from_point[1] + step_size * math.sin(theta)

    return x, y


def reconstruct_path(nodes: List[Node], goal_index: int) -> List[Point2D]:
    """
    Reconstrói o caminho seguindo os pais dos nós.
    """
    path = []
    current = goal_index

    while current is not None:
        node = nodes[current]
        path.append((node.x, node.y))
        current = node.parent

    path.reverse()
    return path


def rrt(
    grid: np.ndarray,
    bounds: MapBounds,
    resolution: float,
    start: Point2D,
    goal: Point2D,
    max_iterations: int,
    step_size: float,
    goal_tolerance: float,
    goal_bias: float,
    collision_step: float,
) -> Tuple[Optional[List[Point2D]], List[Node]]:
    """
    Implementação principal do RRT.
    """
    nodes = [Node(start[0], start[1], parent=None)]

    for iteration in range(max_iterations):
        random_point = sample_point(grid, bounds, resolution, goal, goal_bias)

        nearest_index = nearest_node(nodes, random_point)
        nearest = nodes[nearest_index]
        nearest_point = (nearest.x, nearest.y)

        new_point = steer(nearest_point, random_point, step_size)

        if not point_is_free(new_point, grid, bounds, resolution):
            continue

        if not segment_is_free(
            nearest_point,
            new_point,
            grid,
            bounds,
            resolution,
            collision_step,
        ):
            continue

        nodes.append(Node(new_point[0], new_point[1], parent=nearest_index))
        new_index = len(nodes) - 1

        if distance(new_point, goal) <= goal_tolerance:
            if segment_is_free(
                new_point,
                goal,
                grid,
                bounds,
                resolution,
                collision_step,
            ):
                nodes.append(Node(goal[0], goal[1], parent=new_index))
                goal_index = len(nodes) - 1

                print(f"RRT encontrou caminho na iteração {iteration + 1}.")
                return reconstruct_path(nodes, goal_index), nodes

    return None, nodes


def smooth_path(
    path: List[Point2D],
    grid: np.ndarray,
    bounds: MapBounds,
    resolution: float,
    collision_step: float,
    iterations: int,
) -> List[Point2D]:
    """
    Suavização simples por atalhos.

    Dois pontos do caminho são sorteados. Se a reta entre eles estiver livre,
    os pontos intermediários são removidos.
    """
    if len(path) <= 2:
        return path

    new_path = path.copy()

    for _ in range(iterations):
        if len(new_path) <= 2:
            break

        i = random.randint(0, len(new_path) - 2)
        j = random.randint(i + 1, len(new_path) - 1)

        if j <= i + 1:
            continue

        if segment_is_free(
            new_path[i],
            new_path[j],
            grid,
            bounds,
            resolution,
            collision_step,
        ):
            new_path = new_path[: i + 1] + new_path[j:]

    return new_path


def path_to_cells(
    path_world: List[Point2D],
    bounds: MapBounds,
    resolution: float,
) -> List[GridCell]:
    return [
        world_to_grid(x, y, bounds, resolution)
        for x, y in path_world
    ]


def save_yaml(
    output_path: str,
    map_name: str,
    path_world: List[Point2D],
    path_cells: List[GridCell],
    start: Point2D,
    goal: Point2D,
    resolution: float,
    nodes_count: int,
    args,
):
    data = {
        "map_name": map_name,
        "algorithm": "RRT",
        "resolution": float(resolution),
        "start_world": [float(start[0]), float(start[1])],
        "goal_world": [float(goal[0]), float(goal[1])],
        "path_length_points": len(path_world),
        "tree_nodes_count": nodes_count,
        "parameters": {
            "max_iterations": int(args.max_iterations),
            "step_size": float(args.step_size),
            "goal_tolerance": float(args.goal_tolerance),
            "goal_bias": float(args.goal_bias),
            "collision_check_step": float(args.collision_check_step),
            "smooth_iterations": int(args.smooth_iterations),
            "seed": int(args.seed),
        },
        "path_cells": [
            {"row": int(row), "col": int(col)}
            for row, col in path_cells
        ],
        "path_world": [
            {"x": float(x), "y": float(y)}
            for x, y in path_world
        ],
    }

    with open(output_path, "w", encoding="utf-8") as file:
        yaml.safe_dump(data, file, sort_keys=False, allow_unicode=True)


def render_result(
    grid: np.ndarray,
    bounds: MapBounds,
    resolution: float,
    nodes: List[Node],
    path: List[Point2D],
    start: Point2D,
    goal: Point2D,
    output_path: str,
    draw_tree: bool,
    title: str,
):
    cmap = ListedColormap(["white", "black", "lightgray"])

    fig, ax = plt.subplots(figsize=(10, 10))

    ax.imshow(
        grid,
        origin="lower",
        extent=[bounds.xmin, bounds.xmax, bounds.ymin, bounds.ymax],
        cmap=cmap,
        vmin=0,
        vmax=2,
        interpolation="none",
    )

    x_lines = np.arange(bounds.xmin, bounds.xmax + resolution, resolution)
    y_lines = np.arange(bounds.ymin, bounds.ymax + resolution, resolution)

    for x in x_lines:
        ax.axvline(x, color="gray", linewidth=0.15, alpha=0.25)

    for y in y_lines:
        ax.axhline(y, color="gray", linewidth=0.15, alpha=0.25)

    if draw_tree:
        for node in nodes:
            if node.parent is None:
                continue

            parent = nodes[node.parent]

            ax.plot(
                [parent.x, node.x],
                [parent.y, node.y],
                color="deepskyblue",
                linewidth=0.8,
                alpha=0.45,
            )

    path_x = [p[0] for p in path]
    path_y = [p[1] for p in path]

    ax.plot(path_x, path_y, color="orange", linewidth=2.5, label="Caminho RRT")
    ax.scatter(path_x, path_y, color="orange", s=12)

    ax.plot(start[0], start[1], "go", markersize=8, label="Start")
    ax.plot(goal[0], goal[1], "rx", markersize=9, label="Goal")

    ax.set_xticks(np.arange(math.ceil(bounds.xmin), math.floor(bounds.xmax) + 1, 1.0))
    ax.set_yticks(np.arange(math.ceil(bounds.ymin), math.floor(bounds.ymax) + 1, 1.0))
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right")

    plt.tight_layout()
    fig.savefig(output_path, dpi=300)
    plt.close(fig)


def get_start_goal(args, metadata):
    if args.start_x is not None and args.start_y is not None:
        start = (args.start_x, args.start_y)
    else:
        start = tuple(metadata["start"])

    if args.goal_x is not None and args.goal_y is not None:
        goal = (args.goal_x, args.goal_y)
    else:
        goal = tuple(metadata["goal"])

    return start, goal


def main():
    parser = argparse.ArgumentParser(description="RRT para o TP2.")

    parser.add_argument("--map-name", required=True)
    parser.add_argument("--results-dir", default="results")

    parser.add_argument("--start-x", type=float, default=None)
    parser.add_argument("--start-y", type=float, default=None)
    parser.add_argument("--goal-x", type=float, default=None)
    parser.add_argument("--goal-y", type=float, default=None)

    parser.add_argument("--snap-to-free", action="store_true")

    parser.add_argument("--max-iterations", type=int, default=12000)
    parser.add_argument("--step-size", type=float, default=0.60)
    parser.add_argument("--goal-tolerance", type=float, default=0.60)
    parser.add_argument("--goal-bias", type=float, default=0.10)
    parser.add_argument("--collision-check-step", type=float, default=0.05)
    parser.add_argument("--smooth-iterations", type=int, default=150)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--no-tree", action="store_true")

    args = parser.parse_args()

    random.seed(args.seed)

    grid, metadata, bounds = load_grid_package(
        map_name=args.map_name,
        results_dir=args.results_dir,
    )

    resolution = float(metadata["resolution"])
    start, goal = get_start_goal(args, metadata)

    print("")
    print("Executando RRT")
    print(f"Mapa: {args.map_name}")
    print(f"Start: {start}")
    print(f"Goal: {goal}")
    print(f"Step size: {args.step_size}")
    print(f"Goal bias: {args.goal_bias}")
    print(f"Semente: {args.seed}")

    if not point_is_free(start, grid, bounds, resolution):
        if not args.snap_to_free:
            raise RuntimeError("Start em região ocupada. Use --snap-to-free.")

        start = nearest_free_point(grid, start, bounds, resolution)

        if start is None:
            raise RuntimeError("Não foi possível ajustar o start.")

        print(f"Start ajustado para {start}")

    if not point_is_free(goal, grid, bounds, resolution):
        if not args.snap_to_free:
            raise RuntimeError("Goal em região ocupada. Use --snap-to-free.")

        goal = nearest_free_point(grid, goal, bounds, resolution)

        if goal is None:
            raise RuntimeError("Não foi possível ajustar o goal.")

        print(f"Goal ajustado para {goal}")

    path, nodes = rrt(
        grid=grid,
        bounds=bounds,
        resolution=resolution,
        start=start,
        goal=goal,
        max_iterations=args.max_iterations,
        step_size=args.step_size,
        goal_tolerance=args.goal_tolerance,
        goal_bias=args.goal_bias,
        collision_step=args.collision_check_step,
    )

    if path is None:
        raise RuntimeError("RRT não encontrou caminho.")

    print(f"Caminho original: {len(path)} pontos")
    print(f"Nós gerados: {len(nodes)}")

    path = smooth_path(
        path=path,
        grid=grid,
        bounds=bounds,
        resolution=resolution,
        collision_step=args.collision_check_step,
        iterations=args.smooth_iterations,
    )

    print(f"Caminho suavizado: {len(path)} pontos")

    path_cells = path_to_cells(path, bounds, resolution)

    output_dir = os.path.join(args.results_dir, args.map_name)
    os.makedirs(output_dir, exist_ok=True)

    world_path = os.path.join(output_dir, f"{args.map_name}_rrt_path_world.npy")
    cells_path = os.path.join(output_dir, f"{args.map_name}_rrt_path_cells.npy")
    yaml_path = os.path.join(output_dir, f"{args.map_name}_rrt_path.yaml")
    image_path = os.path.join(output_dir, f"{args.map_name}_rrt_path.png")

    np.save(world_path, np.array(path, dtype=np.float64))
    np.save(cells_path, np.array(path_cells, dtype=np.int32))

    save_yaml(
        output_path=yaml_path,
        map_name=args.map_name,
        path_world=path,
        path_cells=path_cells,
        start=start,
        goal=goal,
        resolution=resolution,
        nodes_count=len(nodes),
        args=args,
    )

    render_result(
        grid=grid,
        bounds=bounds,
        resolution=resolution,
        nodes=nodes,
        path=path,
        start=start,
        goal=goal,
        output_path=image_path,
        draw_tree=not args.no_tree,
        title=f"{args.map_name}: caminho planejado por RRT",
    )

    print("")
    print("Arquivos gerados:")
    print(f"- {world_path}")
    print(f"- {cells_path}")
    print(f"- {yaml_path}")
    print(f"- {image_path}")
    print("")


if __name__ == "__main__":
    main()