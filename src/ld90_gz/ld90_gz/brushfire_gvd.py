#!/usr/bin/env python3

"""
Brushfire + GVD para o TP2.

O código usa o grid gerado pelo grid_map.py.
A ideia é:

1. Propagar distâncias a partir dos obstáculos pelo Brushfire;
2. Encontrar células aproximadamente equidistantes de dois obstáculos;
3. Usar essas células como uma aproximação discreta do GVD;
4. Planejar o caminho:
   start -> GVD -> goal.
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
        bounds_from_metadata,
        grid_to_world,
        is_cell_free,
        load_metadata,
        render_map,
        world_to_grid,
    )
except ImportError:
    from grid_map import (
        MapBounds,
        bounds_from_metadata,
        grid_to_world,
        is_cell_free,
        load_metadata,
        render_map,
        world_to_grid,
    )


GridCell = Tuple[int, int]


def is_inside(grid: np.ndarray, row: int, col: int) -> bool:
    return 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]


def cell_distance(a: GridCell, b: GridCell) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def neighbors8(cell: GridCell) -> List[Tuple[GridCell, float]]:
    row, col = cell
    d = math.sqrt(2.0)

    return [
        ((row - 1, col), 1.0),
        ((row + 1, col), 1.0),
        ((row, col - 1), 1.0),
        ((row, col + 1), 1.0),
        ((row - 1, col - 1), d),
        ((row - 1, col + 1), d),
        ((row + 1, col - 1), d),
        ((row + 1, col + 1), d),
    ]


def reconstruct_path(
    came_from: Dict[GridCell, GridCell],
    start: GridCell,
    goal: GridCell,
) -> List[GridCell]:
    path = [goal]
    current = goal

    while current != start:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def cells_to_world(
    path_cells: List[GridCell],
    bounds: MapBounds,
    resolution: float,
) -> List[Tuple[float, float]]:
    return [
        grid_to_world(row, col, bounds, resolution)
        for row, col in path_cells
    ]


def nearest_free_cell(
    grid: np.ndarray,
    cell: GridCell,
    max_radius: int = 60,
) -> Optional[GridCell]:
    """
    Procura uma célula livre próxima ao start ou goal.
    """
    if is_cell_free(grid, cell[0], cell[1]):
        return cell

    row, col = cell

    for radius in range(1, max_radius + 1):
        candidates = []

        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                rr = row + dr
                cc = col + dc

                if is_cell_free(grid, rr, cc):
                    candidates.append((math.hypot(dr, dc), (rr, cc)))

        if candidates:
            candidates.sort(key=lambda item: item[0])
            return candidates[0][1]

    return None


def brushfire(
    original_grid: np.ndarray,
    obstacle_labels: np.ndarray,
):
    """
    Brushfire com duas distâncias por célula.

    Para cada célula, são guardados:
    - o obstáculo mais próximo;
    - o segundo obstáculo mais próximo.

    O GVD aparece nas regiões onde essas duas distâncias são parecidas.
    """
    rows, cols = original_grid.shape

    first_dist = np.full((rows, cols), np.inf)
    second_dist = np.full((rows, cols), np.inf)

    first_label = np.zeros((rows, cols), dtype=np.int32)
    second_label = np.zeros((rows, cols), dtype=np.int32)

    heap = []

    obstacle_cells = np.argwhere(original_grid == 1)

    for row, col in obstacle_cells:
        label = int(obstacle_labels[row, col])

        if label <= 0:
            continue

        first_dist[row, col] = 0.0
        first_label[row, col] = label

        heapq.heappush(heap, (0.0, int(row), int(col), label))

    while heap:
        current_dist, row, col, label = heapq.heappop(heap)

        is_first = (
            first_label[row, col] == label
            and current_dist <= first_dist[row, col] + 1e-9
        )

        is_second = (
            second_label[row, col] == label
            and current_dist <= second_dist[row, col] + 1e-9
        )

        if not is_first and not is_second:
            continue

        for neighbor, move_cost in neighbors8((row, col)):
            nr, nc = neighbor

            if not is_inside(original_grid, nr, nc):
                continue

            new_dist = current_dist + move_cost
            updated = False

            if first_label[nr, nc] == label:
                if new_dist < first_dist[nr, nc]:
                    first_dist[nr, nc] = new_dist
                    updated = True

            elif new_dist < first_dist[nr, nc]:
                second_dist[nr, nc] = first_dist[nr, nc]
                second_label[nr, nc] = first_label[nr, nc]

                first_dist[nr, nc] = new_dist
                first_label[nr, nc] = label
                updated = True

            elif second_label[nr, nc] == label:
                if new_dist < second_dist[nr, nc]:
                    second_dist[nr, nc] = new_dist
                    updated = True

            elif label != first_label[nr, nc] and new_dist < second_dist[nr, nc]:
                second_dist[nr, nc] = new_dist
                second_label[nr, nc] = label
                updated = True

            if updated:
                heapq.heappush(heap, (new_dist, nr, nc, label))

    return first_dist, first_label, second_dist, second_label


def extract_gvd(
    inflated_grid: np.ndarray,
    first_dist: np.ndarray,
    second_dist: np.ndarray,
    first_label: np.ndarray,
    second_label: np.ndarray,
    min_distance: float,
    tolerance: float,
) -> np.ndarray:
    """
    Extrai as células do GVD.

    Uma célula pertence ao GVD quando:
    - está livre no grid expandido;
    - possui dois obstáculos próximos diferentes;
    - as distâncias para esses obstáculos são próximas.
    """
    rows, cols = inflated_grid.shape
    gvd = np.zeros((rows, cols), dtype=bool)

    for row in range(rows):
        for col in range(cols):
            if inflated_grid[row, col] != 0:
                continue

            if first_label[row, col] <= 0 or second_label[row, col] <= 0:
                continue

            if first_label[row, col] == second_label[row, col]:
                continue

            if first_dist[row, col] < min_distance:
                continue

            if not np.isfinite(second_dist[row, col]):
                continue

            diff = abs(second_dist[row, col] - first_dist[row, col])

            if diff <= tolerance:
                gvd[row, col] = True

    return gvd


def mask_to_cells(mask: np.ndarray) -> List[GridCell]:
    cells = np.argwhere(mask)
    return [(int(row), int(col)) for row, col in cells]


def astar_on_mask(
    mask: np.ndarray,
    start: GridCell,
    goal: GridCell,
) -> Optional[List[GridCell]]:
    """
    A* simples sobre uma máscara booleana.

    True = célula liberada para navegação.
    False = célula bloqueada.
    """
    if not is_inside(mask, start[0], start[1]):
        return None

    if not is_inside(mask, goal[0], goal[1]):
        return None

    if not mask[start[0], start[1]] or not mask[goal[0], goal[1]]:
        return None

    open_list = []
    heapq.heappush(open_list, (cell_distance(start, goal), start))

    came_from: Dict[GridCell, GridCell] = {}
    cost: Dict[GridCell, float] = {start: 0.0}
    visited = set()

    while open_list:
        _, current = heapq.heappop(open_list)

        if current in visited:
            continue

        if current == goal:
            return reconstruct_path(came_from, start, goal)

        visited.add(current)

        for neighbor, move_cost in neighbors8(current):
            nr, nc = neighbor

            if not is_inside(mask, nr, nc):
                continue

            if not mask[nr, nc]:
                continue

            new_cost = cost[current] + move_cost

            if new_cost < cost.get(neighbor, float("inf")):
                cost[neighbor] = new_cost
                priority = new_cost + cell_distance(neighbor, goal)
                came_from[neighbor] = current
                heapq.heappush(open_list, (priority, neighbor))

    return None


def path_to_mask(
    free_mask: np.ndarray,
    start: GridCell,
    target_mask: np.ndarray,
) -> Optional[List[GridCell]]:
    """
    Encontra o menor caminho do start até qualquer célula da target_mask.

    É usado para conectar:
    - start ao GVD;
    - goal ao GVD.
    """
    if not is_inside(free_mask, start[0], start[1]):
        return None

    if not free_mask[start[0], start[1]]:
        return None

    open_list = []
    heapq.heappush(open_list, (0.0, start))

    came_from: Dict[GridCell, GridCell] = {}
    cost: Dict[GridCell, float] = {start: 0.0}
    visited = set()

    while open_list:
        current_cost, current = heapq.heappop(open_list)

        if current in visited:
            continue

        if target_mask[current[0], current[1]]:
            return reconstruct_path(came_from, start, current)

        visited.add(current)

        for neighbor, move_cost in neighbors8(current):
            nr, nc = neighbor

            if not is_inside(free_mask, nr, nc):
                continue

            if not free_mask[nr, nc]:
                continue

            new_cost = current_cost + move_cost

            if new_cost < cost.get(neighbor, float("inf")):
                cost[neighbor] = new_cost
                came_from[neighbor] = current
                heapq.heappush(open_list, (new_cost, neighbor))

    return None


def plan_with_gvd(
    inflated_grid: np.ndarray,
    gvd_mask: np.ndarray,
    start: GridCell,
    goal: GridCell,
) -> Tuple[List[GridCell], np.ndarray]:
    """
    Planeja usando o GVD.

    O caminho final tem três partes:
    1. start até o GVD;
    2. deslocamento sobre o GVD;
    3. GVD até o goal.
    """
    free_mask = inflated_grid == 0
    gvd_navigation = gvd_mask.copy()

    if np.count_nonzero(gvd_navigation) == 0:
        raise RuntimeError("Nenhuma célula de GVD foi encontrada.")

    start_to_gvd = path_to_mask(free_mask, start, gvd_navigation)

    if start_to_gvd is None:
        raise RuntimeError("Não foi possível conectar o start ao GVD.")

    goal_to_gvd = path_to_mask(free_mask, goal, gvd_navigation)

    if goal_to_gvd is None:
        raise RuntimeError("Não foi possível conectar o goal ao GVD.")

    start_gvd = start_to_gvd[-1]
    goal_gvd = goal_to_gvd[-1]

    gvd_path = astar_on_mask(gvd_navigation, start_gvd, goal_gvd)

    if gvd_path is None:
        raise RuntimeError(
            "Não foi possível planejar sobre o GVD. "
            "Tente aumentar a tolerância de equidistância."
        )

    gvd_to_goal = list(reversed(goal_to_gvd))

    full_path = []
    full_path.extend(start_to_gvd)
    full_path.extend(gvd_path[1:])
    full_path.extend(gvd_to_goal[1:])

    return full_path, gvd_navigation


def load_files(map_name: str, results_dir: str):
    """
    Carrega os arquivos gerados pelo grid_map.py.
    """
    map_dir = os.path.join(results_dir, map_name)

    original_path = os.path.join(map_dir, f"{map_name}_grid_original.npy")
    inflated_path = os.path.join(map_dir, f"{map_name}_grid_inflated.npy")
    labels_path = os.path.join(map_dir, f"{map_name}_obstacle_site_labels.npy")
    metadata_path = os.path.join(map_dir, f"{map_name}_metadata.yaml")

    original_grid = np.load(original_path)
    inflated_grid = np.load(inflated_path)
    obstacle_labels = np.load(labels_path)

    metadata = load_metadata(metadata_path)
    bounds = bounds_from_metadata(metadata)

    return original_grid, inflated_grid, obstacle_labels, metadata, bounds


def save_yaml(
    output_path: str,
    map_name: str,
    path_cells: List[GridCell],
    path_world: List[Tuple[float, float]],
    start_world: Tuple[float, float],
    goal_world: Tuple[float, float],
    resolution: float,
    gvd_cells_count: int,
    obstacle_count: int,
    args,
):
    data = {
        "map_name": map_name,
        "algorithm": "Brushfire + GVD",
        "resolution": float(resolution),
        "start_world": [float(start_world[0]), float(start_world[1])],
        "goal_world": [float(goal_world[0]), float(goal_world[1])],
        "path_length_cells": len(path_cells),
        "gvd_cells_count": int(gvd_cells_count),
        "obstacle_components_count": int(obstacle_count),
        "parameters": {
            "min_distance_cells": float(args.min_distance_cells),
            "equidistance_tolerance_cells": float(args.equidistance_tolerance_cells),
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
    parser = argparse.ArgumentParser(description="Brushfire + GVD para o TP2.")

    parser.add_argument("--map-name", required=True)
    parser.add_argument("--results-dir", default="results")

    parser.add_argument("--start-x", type=float, default=None)
    parser.add_argument("--start-y", type=float, default=None)
    parser.add_argument("--goal-x", type=float, default=None)
    parser.add_argument("--goal-y", type=float, default=None)

    parser.add_argument("--snap-to-free", action="store_true")
    parser.add_argument("--min-distance-cells", type=float, default=1.0)
    parser.add_argument("--equidistance-tolerance-cells", type=float, default=2.0)

    # Mantido apenas para compatibilidade com os launches.
    # Nesta versão, o caminho usa diretamente o GVD extraído.
    parser.add_argument("--gvd-dilation", type=int, default=0)

    args = parser.parse_args()

    original_grid, inflated_grid, obstacle_labels, metadata, bounds = load_files(
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
    print("Executando Brushfire + GVD")
    print(f"Mapa: {args.map_name}")
    print(f"Start: {start_world} -> {start_cell}")
    print(f"Goal:  {goal_world} -> {goal_cell}")

    if not is_cell_free(inflated_grid, start_cell[0], start_cell[1]):
        if not args.snap_to_free:
            raise RuntimeError("Start em célula ocupada. Use --snap-to-free.")

        start_cell = nearest_free_cell(inflated_grid, start_cell)

        if start_cell is None:
            raise RuntimeError("Não foi encontrada célula livre próxima ao start.")

        start_world = grid_to_world(start_cell[0], start_cell[1], bounds, resolution)
        print(f"Start ajustado para {start_cell}")

    if not is_cell_free(inflated_grid, goal_cell[0], goal_cell[1]):
        if not args.snap_to_free:
            raise RuntimeError("Goal em célula ocupada. Use --snap-to-free.")

        goal_cell = nearest_free_cell(inflated_grid, goal_cell)

        if goal_cell is None:
            raise RuntimeError("Não foi encontrada célula livre próxima ao goal.")

        goal_world = grid_to_world(goal_cell[0], goal_cell[1], bounds, resolution)
        print(f"Goal ajustado para {goal_cell}")

    obstacle_count = int(np.max(obstacle_labels))

    if obstacle_count < 2:
        raise RuntimeError("O GVD precisa de pelo menos dois obstáculos.")

    print(f"Obstáculos rotulados: {obstacle_count}")

    first_dist, first_label, second_dist, second_label = brushfire(
        original_grid=original_grid,
        obstacle_labels=obstacle_labels,
    )

    gvd_mask = extract_gvd(
        inflated_grid=inflated_grid,
        first_dist=first_dist,
        second_dist=second_dist,
        first_label=first_label,
        second_label=second_label,
        min_distance=args.min_distance_cells,
        tolerance=args.equidistance_tolerance_cells,
    )

    gvd_cells = mask_to_cells(gvd_mask)

    if len(gvd_cells) == 0:
        raise RuntimeError(
            "Nenhuma célula de GVD foi encontrada. "
            "Tente aumentar --equidistance-tolerance-cells."
        )

    print(f"Células do GVD: {len(gvd_cells)}")

    path_cells, gvd_navigation = plan_with_gvd(
        inflated_grid=inflated_grid,
        gvd_mask=gvd_mask,
        start=start_cell,
        goal=goal_cell,
    )

    path_world = cells_to_world(path_cells, bounds, resolution)

    output_dir = os.path.join(args.results_dir, args.map_name)
    os.makedirs(output_dir, exist_ok=True)

    gvd_mask_path = os.path.join(output_dir, f"{args.map_name}_gvd_mask.npy")
    path_cells_path = os.path.join(output_dir, f"{args.map_name}_gvd_path_cells.npy")
    path_world_path = os.path.join(output_dir, f"{args.map_name}_gvd_path_world.npy")
    yaml_path = os.path.join(output_dir, f"{args.map_name}_gvd_path.yaml")
    gvd_image_path = os.path.join(output_dir, f"{args.map_name}_gvd_map.png")
    path_image_path = os.path.join(output_dir, f"{args.map_name}_gvd_path.png")

    np.save(gvd_mask_path, gvd_mask)
    np.save(path_cells_path, np.array(path_cells, dtype=np.int32))
    np.save(path_world_path, np.array(path_world, dtype=np.float64))

    save_yaml(
        output_path=yaml_path,
        map_name=args.map_name,
        path_cells=path_cells,
        path_world=path_world,
        start_world=start_world,
        goal_world=goal_world,
        resolution=resolution,
        gvd_cells_count=len(gvd_cells),
        obstacle_count=obstacle_count,
        args=args,
    )

    render_map(
        grid=inflated_grid,
        bounds=bounds,
        resolution=resolution,
        output_path=gvd_image_path,
        title=f"{args.map_name}: GVD obtido por Brushfire",
        start=start_world,
        goal=goal_world,
        gvd_cells=gvd_cells,
    )

    render_map(
        grid=inflated_grid,
        bounds=bounds,
        resolution=resolution,
        output_path=path_image_path,
        title=f"{args.map_name}: caminho planejado usando GVD",
        start=start_world,
        goal=goal_world,
        path_world=path_world,
        gvd_cells=gvd_cells,
    )

    print(f"Caminho GVD encontrado com {len(path_cells)} células.")
    print("Arquivos gerados:")
    print(f"- {gvd_mask_path}")
    print(f"- {path_cells_path}")
    print(f"- {path_world_path}")
    print(f"- {yaml_path}")
    print(f"- {gvd_image_path}")
    print(f"- {path_image_path}")
    print("")


if __name__ == "__main__":
    main()