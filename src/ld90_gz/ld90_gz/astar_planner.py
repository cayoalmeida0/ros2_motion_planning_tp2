#!/usr/bin/env python3

"""
astar_planner.py

Implementação do algoritmo A* para o Trabalho Prático 2.

Entrada:
- Grid expandido gerado pelo grid_map.py;
- Metadata do mapa;
- Posição inicial e posição objetivo.

Saída:
- Caminho planejado em coordenadas de grid;
- Caminho planejado em coordenadas do mundo;
- Imagem PNG do mapa com o caminho planejado.

Convenção do grid:
0 = célula livre
1 = obstáculo original
2 = obstáculo expandido

O A* considera transitáveis apenas as células com valor 0.
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
        load_grid_package,
        render_map,
        world_to_grid,
    )
except ImportError:
    from grid_map import (
        MapBounds,
        bounds_from_metadata,
        grid_to_world,
        is_cell_free,
        load_grid_package,
        render_map,
        world_to_grid,
    )


GridCell = Tuple[int, int]


# =============================================================================
# Funções auxiliares
# =============================================================================

def euclidean_heuristic(a: GridCell, b: GridCell) -> float:
    """
    Heurística euclidiana no espaço discreto.
    """
    dr = a[0] - b[0]
    dc = a[1] - b[1]

    return math.sqrt(dr * dr + dc * dc)


def get_neighbors(
    grid: np.ndarray,
    cell: GridCell,
    allow_diagonal: bool = True,
) -> List[Tuple[GridCell, float]]:
    """
    Retorna vizinhos livres da célula atual.

    Se allow_diagonal=True, usa vizinhança-8.
    Caso contrário, usa vizinhança-4.
    """
    row, col = cell

    if allow_diagonal:
        motions = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ]
    else:
        motions = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
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
    """
    Reconstrói o caminho do goal até o start.
    """
    current = goal
    path = [current]

    while current != start:
        current = came_from[current]
        path.append(current)

    path.reverse()

    return path


def find_nearest_free_cell(
    grid: np.ndarray,
    cell: GridCell,
    max_radius: int = 30,
) -> Optional[GridCell]:
    """
    Procura a célula livre mais próxima de uma célula ocupada.

    Isso é útil quando o start ou o goal caem por pouco dentro de uma
    região expandida por causa da discretização ou da margem de segurança.
    """
    row, col = cell

    if is_cell_free(grid, row, col):
        return cell

    best_cell = None
    best_distance = float("inf")

    for radius in range(1, max_radius + 1):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                rr = row + dr
                cc = col + dc

                if not is_cell_free(grid, rr, cc):
                    continue

                distance = math.sqrt(dr * dr + dc * dc)

                if distance < best_distance:
                    best_distance = distance
                    best_cell = (rr, cc)

        if best_cell is not None:
            return best_cell

    return None


def cells_to_world_path(
    path_cells: List[GridCell],
    bounds: MapBounds,
    resolution: float,
) -> List[Tuple[float, float]]:
    """
    Converte caminho em células para caminho em coordenadas do mundo.
    """
    return [grid_to_world(row, col, bounds, resolution) for row, col in path_cells]


# =============================================================================
# A*
# =============================================================================

def astar_search(
    grid: np.ndarray,
    start: GridCell,
    goal: GridCell,
    allow_diagonal: bool = True,
) -> Optional[List[GridCell]]:
    """
    Executa o algoritmo A* sobre o grid.

    Retorna:
    - Lista de células do caminho, se houver solução;
    - None, se não houver caminho.
    """
    open_heap = []
    heapq.heappush(open_heap, (0.0, start))

    came_from: Dict[GridCell, GridCell] = {}

    g_score: Dict[GridCell, float] = {start: 0.0}
    f_score: Dict[GridCell, float] = {
        start: euclidean_heuristic(start, goal)
    }

    closed_set = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current in closed_set:
            continue

        if current == goal:
            return reconstruct_path(came_from, start, goal)

        closed_set.add(current)

        for neighbor, move_cost in get_neighbors(grid, current, allow_diagonal):
            if neighbor in closed_set:
                continue

            tentative_g = g_score[current] + move_cost

            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g

                f = tentative_g + euclidean_heuristic(neighbor, goal)
                f_score[neighbor] = f

                heapq.heappush(open_heap, (f, neighbor))

    return None


# =============================================================================
# Salvamento
# =============================================================================

def save_path_yaml(
    output_path: str,
    map_name: str,
    algorithm: str,
    path_cells: List[GridCell],
    path_world: List[Tuple[float, float]],
    start_world: Tuple[float, float],
    goal_world: Tuple[float, float],
    resolution: float,
):
    data = {
        "map_name": map_name,
        "algorithm": algorithm,
        "resolution": resolution,
        "start_world": list(start_world),
        "goal_world": list(goal_world),
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


# =============================================================================
# Função principal
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Planejador A* para mapas discretizados do TP2."
    )

    parser.add_argument(
        "--map-name",
        required=True,
        help="Nome do mapa. Exemplo: tp2_map1 ou tp2_map2.",
    )

    parser.add_argument(
        "--results-dir",
        default="results",
        help="Diretório onde estão os grids gerados pelo grid_map.py.",
    )

    parser.add_argument(
        "--start-x",
        type=float,
        default=None,
        help="Coordenada x inicial. Se omitida, usa o valor do metadata.",
    )

    parser.add_argument(
        "--start-y",
        type=float,
        default=None,
        help="Coordenada y inicial. Se omitida, usa o valor do metadata.",
    )

    parser.add_argument(
        "--goal-x",
        type=float,
        default=None,
        help="Coordenada x objetivo. Se omitida, usa o valor do metadata.",
    )

    parser.add_argument(
        "--goal-y",
        type=float,
        default=None,
        help="Coordenada y objetivo. Se omitida, usa o valor do metadata.",
    )

    parser.add_argument(
        "--no-diagonal",
        action="store_true",
        help="Usa vizinhança-4 em vez de vizinhança-8.",
    )

    parser.add_argument(
        "--snap-to-free",
        action="store_true",
        help="Move start/goal para a célula livre mais próxima, se necessário.",
    )

    args = parser.parse_args()

    grid, metadata, bounds = load_grid_package(
        map_name=args.map_name,
        results_dir=args.results_dir,
    )

    resolution = float(metadata["resolution"])

    # -------------------------------------------------------------------------
    # Start e goal
    # -------------------------------------------------------------------------
    metadata_start = metadata.get("start", None)
    metadata_goal = metadata.get("goal", None)

    if args.start_x is not None and args.start_y is not None:
        start_world = (args.start_x, args.start_y)
    elif metadata_start is not None:
        start_world = (float(metadata_start[0]), float(metadata_start[1]))
    else:
        raise ValueError("Start não informado e inexistente no metadata.")

    if args.goal_x is not None and args.goal_y is not None:
        goal_world = (args.goal_x, args.goal_y)
    elif metadata_goal is not None:
        goal_world = (float(metadata_goal[0]), float(metadata_goal[1]))
    else:
        raise ValueError("Goal não informado e inexistente no metadata.")

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
    print(f"Start mundo: {start_world} -> célula {start_cell}")
    print(f"Goal mundo: {goal_world} -> célula {goal_cell}")
    print(f"Resolução: {resolution:.3f} m/célula")
    print(f"Vizinhança: {'4' if args.no_diagonal else '8'}")

    # -------------------------------------------------------------------------
    # Verificação de células livres
    # -------------------------------------------------------------------------
    if not is_cell_free(grid, start_cell[0], start_cell[1]):
        print("[AVISO] Start está em célula ocupada ou expandida.")

        if args.snap_to_free:
            new_start = find_nearest_free_cell(grid, start_cell)

            if new_start is None:
                raise RuntimeError("Não foi possível encontrar célula livre próxima ao start.")

            print(f"        Start ajustado para célula livre: {new_start}")
            start_cell = new_start
            start_world = grid_to_world(start_cell[0], start_cell[1], bounds, resolution)
        else:
            raise RuntimeError(
                "Start inválido. Use outro ponto ou execute com --snap-to-free."
            )

    if not is_cell_free(grid, goal_cell[0], goal_cell[1]):
        print("[AVISO] Goal está em célula ocupada ou expandida.")

        if args.snap_to_free:
            new_goal = find_nearest_free_cell(grid, goal_cell)

            if new_goal is None:
                raise RuntimeError("Não foi possível encontrar célula livre próxima ao goal.")

            print(f"        Goal ajustado para célula livre: {new_goal}")
            goal_cell = new_goal
            goal_world = grid_to_world(goal_cell[0], goal_cell[1], bounds, resolution)
        else:
            raise RuntimeError(
                "Goal inválido. Use outro ponto ou execute com --snap-to-free."
            )

    # -------------------------------------------------------------------------
    # Planejamento
    # -------------------------------------------------------------------------
    path_cells = astar_search(
        grid=grid,
        start=start_cell,
        goal=goal_cell,
        allow_diagonal=not args.no_diagonal,
    )

    if path_cells is None:
        raise RuntimeError("A* não encontrou caminho entre start e goal.")

    path_world = cells_to_world_path(path_cells, bounds, resolution)

    print(f"Caminho encontrado com {len(path_cells)} células.")

    # -------------------------------------------------------------------------
    # Salvamento
    # -------------------------------------------------------------------------
    map_output_dir = os.path.join(args.results_dir, args.map_name)
    os.makedirs(map_output_dir, exist_ok=True)

    path_cells_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_astar_path_cells.npy",
    )

    path_world_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_astar_path_world.npy",
    )

    path_yaml_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_astar_path.yaml",
    )

    image_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_astar_path.png",
    )

    np.save(path_cells_path, np.array(path_cells, dtype=np.int32))
    np.save(path_world_path, np.array(path_world, dtype=np.float64))

    save_path_yaml(
        output_path=path_yaml_path,
        map_name=args.map_name,
        algorithm="A*",
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

    print("")
    print("Arquivos gerados:")
    print(f"- {path_cells_path}")
    print(f"- {path_world_path}")
    print(f"- {path_yaml_path}")
    print(f"- {image_path}")
    print("")


if __name__ == "__main__":
    main()
