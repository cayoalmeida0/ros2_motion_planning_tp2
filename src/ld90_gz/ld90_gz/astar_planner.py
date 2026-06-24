#!/usr/bin/env python3

"""
Planejador A* para o TP2.

O algoritmo usa o grid expandido gerado pelo grid_map.py.
Células livres possuem valor 0.
Obstáculos originais e expandidos não são atravessados.
"""

import argparse
import heapq
import math
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

try:
    from ld90_gz.grid_map import (
        MapBounds,
        grid_to_world,
        is_cell_free,
        load_grid_package,
        render_map,
        world_to_grid,
    )
except ImportError:
    from grid_map import (
        MapBounds,
        grid_to_world,
        is_cell_free,
        load_grid_package,
        render_map,
        world_to_grid,
    )


GridCell = Tuple[int, int]


def heuristic(a: GridCell, b: GridCell) -> float:
    """Distância euclidiana usada como heurística do A*."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def get_neighbors(grid: np.ndarray, cell: GridCell, diagonal: bool = True):
    """Retorna as células vizinhas livres."""
    row, col = cell

    motions = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
    ]

    if diagonal:
        d = math.sqrt(2.0)
        motions += [
            (-1, -1, d),
            (-1, 1, d),
            (1, -1, d),
            (1, 1, d),
        ]

    neighbors = []

    for dr, dc, cost in motions:
        nr = row + dr
        nc = col + dc

        if is_cell_free(grid, nr, nc):
            neighbors.append(((nr, nc), cost))

    return neighbors


def reconstruct_path(
    came_from: Dict[GridCell, GridCell],
    start: GridCell,
    goal: GridCell,
) -> List[GridCell]:
    """Reconstrói o caminho do goal até o start."""
    path = [goal]
    current = goal

    while current != start:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def astar(
    grid: np.ndarray,
    start: GridCell,
    goal: GridCell,
    diagonal: bool = True,
) -> Optional[List[GridCell]]:
    """Implementação simples do A* em grid."""
    open_list = []
    heapq.heappush(open_list, (heuristic(start, goal), start))

    came_from: Dict[GridCell, GridCell] = {}
    cost_so_far: Dict[GridCell, float] = {start: 0.0}
    visited = set()

    while open_list:
        _, current = heapq.heappop(open_list)

        if current in visited:
            continue

        if current == goal:
            return reconstruct_path(came_from, start, goal)

        visited.add(current)

        for neighbor, move_cost in get_neighbors(grid, current, diagonal):
            new_cost = cost_so_far[current] + move_cost

            if new_cost < cost_so_far.get(neighbor, float("inf")):
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal)
                came_from[neighbor] = current
                heapq.heappush(open_list, (priority, neighbor))

    return None


def nearest_free_cell(
    grid: np.ndarray,
    cell: GridCell,
    max_radius: int = 40,
) -> Optional[GridCell]:
    """
    Procura uma célula livre próxima.

    Isso é usado apenas se o start ou goal caírem em uma região ocupada
    por causa da inflação dos obstáculos.
    """
    row, col = cell

    if is_cell_free(grid, row, col):
        return cell

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
            return candidates[0][1]

    return None


def path_cells_to_world(
    path_cells: List[GridCell],
    bounds: MapBounds,
    resolution: float,
) -> List[Tuple[float, float]]:
    """Converte o caminho do grid para coordenadas do mundo."""
    return [
        grid_to_world(row, col, bounds, resolution)
        for row, col in path_cells
    ]


def save_yaml(
    output_path: str,
    map_name: str,
    path_cells: List[GridCell],
    path_world: List[Tuple[float, float]],
    start_world: Tuple[float, float],
    goal_world: Tuple[float, float],
    resolution: float,
):
    """Salva um resumo do resultado em YAML."""
    data = {
        "map_name": map_name,
        "algorithm": "A*",
        "resolution": float(resolution),
        "start_world": [float(start_world[0]), float(start_world[1])],
        "goal_world": [float(goal_world[0]), float(goal_world[1])],
        "path_length_cells": len(path_cells),
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


def get_start_goal(args, metadata):
    """Lê start e goal dos argumentos ou do metadata."""
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
    parser = argparse.ArgumentParser(description="A* para o TP2.")

    parser.add_argument("--map-name", required=True)
    parser.add_argument("--results-dir", default="results")

    parser.add_argument("--start-x", type=float, default=None)
    parser.add_argument("--start-y", type=float, default=None)
    parser.add_argument("--goal-x", type=float, default=None)
    parser.add_argument("--goal-y", type=float, default=None)

    parser.add_argument("--no-diagonal", action="store_true")
    parser.add_argument("--snap-to-free", action="store_true")

    args = parser.parse_args()

    grid, metadata, bounds = load_grid_package(
        map_name=args.map_name,
        results_dir=args.results_dir,
    )

    resolution = float(metadata["resolution"])
    start_world, goal_world = get_start_goal(args, metadata)

    start_cell = world_to_grid(
        start_world[0],
        start_world[1],
        bounds,
        resolution,
    )

    goal_cell = world_to_grid(
        goal_world[0],
        goal_world[1],
        bounds,
        resolution,
    )

    print("")
    print("Executando A*")
    print(f"Mapa: {args.map_name}")
    print(f"Start: {start_world} -> {start_cell}")
    print(f"Goal:  {goal_world} -> {goal_cell}")

    if not is_cell_free(grid, start_cell[0], start_cell[1]):
        if not args.snap_to_free:
            raise RuntimeError("Start em célula ocupada. Use --snap-to-free.")

        start_cell = nearest_free_cell(grid, start_cell)

        if start_cell is None:
            raise RuntimeError("Não foi encontrada célula livre próxima ao start.")

        start_world = grid_to_world(start_cell[0], start_cell[1], bounds, resolution)
        print(f"Start ajustado para {start_cell}")

    if not is_cell_free(grid, goal_cell[0], goal_cell[1]):
        if not args.snap_to_free:
            raise RuntimeError("Goal em célula ocupada. Use --snap-to-free.")

        goal_cell = nearest_free_cell(grid, goal_cell)

        if goal_cell is None:
            raise RuntimeError("Não foi encontrada célula livre próxima ao goal.")

        goal_world = grid_to_world(goal_cell[0], goal_cell[1], bounds, resolution)
        print(f"Goal ajustado para {goal_cell}")

    path_cells = astar(
        grid=grid,
        start=start_cell,
        goal=goal_cell,
        diagonal=not args.no_diagonal,
    )

    if path_cells is None:
        raise RuntimeError("A* não encontrou caminho.")

    path_world = path_cells_to_world(path_cells, bounds, resolution)

    output_dir = os.path.join(args.results_dir, args.map_name)
    os.makedirs(output_dir, exist_ok=True)

    cells_path = os.path.join(output_dir, f"{args.map_name}_astar_path_cells.npy")
    world_path = os.path.join(output_dir, f"{args.map_name}_astar_path_world.npy")
    yaml_path = os.path.join(output_dir, f"{args.map_name}_astar_path.yaml")
    image_path = os.path.join(output_dir, f"{args.map_name}_astar_path.png")

    np.save(cells_path, np.array(path_cells, dtype=np.int32))
    np.save(world_path, np.array(path_world, dtype=np.float64))

    save_yaml(
        output_path=yaml_path,
        map_name=args.map_name,
        path_cells=path_cells,
        path_world=path_world,
        start_world=start_world,
        goal_world=goal_world,
        resolution=resolution,
    )

    render_map(
        grid=grid,
        bounds=bounds,
        resolution=resolution,
        output_path=image_path,
        title=f"{args.map_name}: caminho planejado por A*",
        start=start_world,
        goal=goal_world,
        path_world=path_world,
    )

    print(f"Caminho encontrado com {len(path_cells)} células.")
    print("Arquivos gerados:")
    print(f"- {cells_path}")
    print(f"- {world_path}")
    print(f"- {yaml_path}")
    print(f"- {image_path}")
    print("")


if __name__ == "__main__":
    main()