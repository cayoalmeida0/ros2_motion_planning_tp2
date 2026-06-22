#!/usr/bin/env python3

"""
brushfire_gvd.py

Questão 2 do Trabalho Prático 2:
- Implementação do algoritmo Brushfire;
- Construção de uma aproximação discreta do Diagrama de Voronoi Generalizado (GVD);
- Planejamento de caminho utilizando o GVD;
- Geração de imagens do GVD e do caminho planejado;
- Salvamento do caminho em coordenadas do grid e do mundo.

Entrada esperada:
- results/<map_name>/<map_name>_grid_original.npy
- results/<map_name>/<map_name>_grid_inflated.npy
- results/<map_name>/<map_name>_metadata.yaml

Saídas principais:
- <map_name>_brushfire_first_distance.npy
- <map_name>_brushfire_second_distance.npy
- <map_name>_brushfire_first_label.npy
- <map_name>_brushfire_second_label.npy
- <map_name>_gvd_mask.npy
- <map_name>_gvd_navigation_mask.npy
- <map_name>_gvd_map.png
- <map_name>_gvd_path.png
- <map_name>_gvd_path_cells.npy
- <map_name>_gvd_path_world.npy
- <map_name>_gvd_path.yaml

Convenção do grid:
0 = célula livre
1 = obstáculo original
2 = obstáculo expandido

Observação:
O GVD é extraído apenas em células livres do grid expandido, pois esse é
o espaço seguro considerado pelo planejador.
"""

import argparse
import heapq
import math
import os
from collections import deque
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


# =============================================================================
# Funções auxiliares de grid
# =============================================================================

def is_inside(grid: np.ndarray, row: int, col: int) -> bool:
    return 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]


def euclidean_cells(a: GridCell, b: GridCell) -> float:
    dr = a[0] - b[0]
    dc = a[1] - b[1]
    return math.sqrt(dr * dr + dc * dc)


def get_8_neighbors(cell: GridCell) -> List[Tuple[GridCell, float]]:
    row, col = cell

    return [
        ((row - 1, col), 1.0),
        ((row + 1, col), 1.0),
        ((row, col - 1), 1.0),
        ((row, col + 1), 1.0),
        ((row - 1, col - 1), math.sqrt(2.0)),
        ((row - 1, col + 1), math.sqrt(2.0)),
        ((row + 1, col - 1), math.sqrt(2.0)),
        ((row + 1, col + 1), math.sqrt(2.0)),
    ]


def reconstruct_path(
    came_from: Dict[GridCell, GridCell],
    start: GridCell,
    goal: GridCell,
) -> List[GridCell]:
    current = goal
    path = [current]

    while current != start:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def cells_to_world_path(
    path_cells: List[GridCell],
    bounds: MapBounds,
    resolution: float,
) -> List[Tuple[float, float]]:
    return [grid_to_world(row, col, bounds, resolution) for row, col in path_cells]


def find_nearest_free_cell(
    grid: np.ndarray,
    cell: GridCell,
    max_radius: int = 60,
) -> Optional[GridCell]:
    """
    Procura a célula livre mais próxima de uma célula ocupada/expandida.

    Usado quando start ou goal caem dentro de obstáculo expandido.
    """
    if is_cell_free(grid, cell[0], cell[1]):
        return cell

    row, col = cell

    best_cell = None
    best_distance = float("inf")

    for radius in range(1, max_radius + 1):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                rr = row + dr
                cc = col + dc

                if not is_inside(grid, rr, cc):
                    continue

                if not is_cell_free(grid, rr, cc):
                    continue

                distance = math.sqrt(dr * dr + dc * dc)

                if distance < best_distance:
                    best_distance = distance
                    best_cell = (rr, cc)

        if best_cell is not None:
            return best_cell

    return None


# =============================================================================
# Rotulagem dos obstáculos
# =============================================================================

def label_obstacle_components(original_grid: np.ndarray) -> Tuple[np.ndarray, int]:
    """
    Rotula componentes conexas dos obstáculos originais.

    Cada componente recebe um rótulo:
    0 = sem rótulo
    1, 2, 3, ... = componente de obstáculo

    A rotulagem é feita com vizinhança-8.
    """
    labels = np.zeros_like(original_grid, dtype=np.int32)
    current_label = 0

    rows, cols = original_grid.shape

    for row in range(rows):
        for col in range(cols):
            if original_grid[row, col] != 1:
                continue

            if labels[row, col] != 0:
                continue

            current_label += 1

            queue = deque()
            queue.append((row, col))
            labels[row, col] = current_label

            while queue:
                current = queue.popleft()

                for neighbor, _ in get_8_neighbors(current):
                    nr, nc = neighbor

                    if not is_inside(original_grid, nr, nc):
                        continue

                    if original_grid[nr, nc] != 1:
                        continue

                    if labels[nr, nc] != 0:
                        continue

                    labels[nr, nc] = current_label
                    queue.append((nr, nc))

    return labels, current_label


# =============================================================================
# Brushfire com primeiro e segundo obstáculos mais próximos
# =============================================================================

def compute_brushfire(
    original_grid: np.ndarray,
    obstacle_labels: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Calcula o Brushfire mantendo duas frentes relevantes por célula:

    - primeiro obstáculo mais próximo;
    - segundo obstáculo mais próximo.

    Isso permite identificar o GVD por equidistância aproximada:
    uma célula tende a pertencer ao GVD quando as distâncias ao primeiro
    e ao segundo obstáculos são próximas.

    Retorna:
    - first_distance;
    - first_label;
    - second_distance;
    - second_label.
    """
    rows, cols = original_grid.shape

    first_distance = np.full((rows, cols), np.inf, dtype=np.float64)
    second_distance = np.full((rows, cols), np.inf, dtype=np.float64)

    first_label = np.zeros((rows, cols), dtype=np.int32)
    second_label = np.zeros((rows, cols), dtype=np.int32)

    heap = []

    obstacle_cells = np.argwhere(original_grid == 1)

    for row, col in obstacle_cells:
        label = int(obstacle_labels[row, col])

        if label <= 0:
            continue

        first_distance[row, col] = 0.0
        first_label[row, col] = label

        heapq.heappush(heap, (0.0, int(row), int(col), label))

    while heap:
        current_distance, row, col, label = heapq.heappop(heap)

        valid_first = (
            first_label[row, col] == label
            and current_distance <= first_distance[row, col] + 1e-9
        )

        valid_second = (
            second_label[row, col] == label
            and current_distance <= second_distance[row, col] + 1e-9
        )

        if not valid_first and not valid_second:
            continue

        for neighbor, move_cost in get_8_neighbors((row, col)):
            nr, nc = neighbor

            if not is_inside(original_grid, nr, nc):
                continue

            new_distance = current_distance + move_cost
            updated = False

            # Mesmo rótulo do primeiro obstáculo mais próximo
            if first_label[nr, nc] == label:
                if new_distance < first_distance[nr, nc]:
                    first_distance[nr, nc] = new_distance
                    updated = True

            # Novo rótulo é melhor que o primeiro atual
            elif new_distance < first_distance[nr, nc]:
                second_distance[nr, nc] = first_distance[nr, nc]
                second_label[nr, nc] = first_label[nr, nc]

                first_distance[nr, nc] = new_distance
                first_label[nr, nc] = label

                updated = True

            # Mesmo rótulo do segundo obstáculo mais próximo
            elif second_label[nr, nc] == label:
                if new_distance < second_distance[nr, nc]:
                    second_distance[nr, nc] = new_distance
                    updated = True

            # Novo rótulo candidato a segundo obstáculo mais próximo
            elif label != first_label[nr, nc] and new_distance < second_distance[nr, nc]:
                second_distance[nr, nc] = new_distance
                second_label[nr, nc] = label

                updated = True

            if updated:
                heapq.heappush(heap, (new_distance, nr, nc, label))

    return first_distance, first_label, second_distance, second_label


# =============================================================================
# Extração do GVD
# =============================================================================

def extract_gvd_mask(
    inflated_grid: np.ndarray,
    first_label: np.ndarray,
    second_label: np.ndarray,
    first_distance: np.ndarray,
    second_distance: np.ndarray,
    min_distance_cells: float = 1.0,
    equidistance_tolerance_cells: float = 2.0,
) -> np.ndarray:
    """
    Extrai uma aproximação discreta do GVD.

    Critérios:
    - a célula deve estar livre no grid expandido;
    - deve possuir primeiro e segundo obstáculos mais próximos distintos;
    - deve estar suficientemente afastada dos obstáculos;
    - as distâncias ao primeiro e segundo obstáculos devem ser próximas.

    O parâmetro equidistance_tolerance_cells controla a espessura/sensibilidade
    da extração do GVD.
    """
    rows, cols = inflated_grid.shape
    gvd_mask = np.zeros((rows, cols), dtype=bool)

    for row in range(rows):
        for col in range(cols):
            if inflated_grid[row, col] != 0:
                continue

            if first_label[row, col] <= 0:
                continue

            if second_label[row, col] <= 0:
                continue

            if first_label[row, col] == second_label[row, col]:
                continue

            if first_distance[row, col] < min_distance_cells:
                continue

            if not np.isfinite(second_distance[row, col]):
                continue

            distance_difference = abs(
                second_distance[row, col] - first_distance[row, col]
            )

            if distance_difference <= equidistance_tolerance_cells:
                gvd_mask[row, col] = True

    return gvd_mask


def dilate_mask(
    mask: np.ndarray,
    free_grid: np.ndarray,
    radius_cells: int,
) -> np.ndarray:
    """
    Dilata a máscara do GVD para melhorar conectividade no grid discreto.

    A dilatação é restrita às células livres do grid expandido.
    """
    if radius_cells <= 0:
        return mask.copy()

    rows, cols = mask.shape
    dilated = mask.copy()

    gvd_cells = np.argwhere(mask)

    for row, col in gvd_cells:
        row = int(row)
        col = int(col)

        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                rr = row + dr
                cc = col + dc

                if not is_inside(mask, rr, cc):
                    continue

                if free_grid[rr, cc] != 0:
                    continue

                distance = math.sqrt(dr * dr + dc * dc)

                if distance <= radius_cells:
                    dilated[rr, cc] = True

    return dilated


def gvd_cells_from_mask(gvd_mask: np.ndarray) -> List[GridCell]:
    cells = np.argwhere(gvd_mask)
    return [(int(row), int(col)) for row, col in cells]


# =============================================================================
# A* genérico em máscara livre
# =============================================================================

def astar_on_mask(
    free_mask: np.ndarray,
    start: GridCell,
    goal: GridCell,
) -> Optional[List[GridCell]]:
    """
    Executa A* sobre uma máscara booleana.

    True = célula navegável
    False = célula bloqueada
    """
    if not is_inside(free_mask, start[0], start[1]):
        return None

    if not is_inside(free_mask, goal[0], goal[1]):
        return None

    if not free_mask[start[0], start[1]]:
        return None

    if not free_mask[goal[0], goal[1]]:
        return None

    open_heap = []
    heapq.heappush(open_heap, (0.0, start))

    came_from: Dict[GridCell, GridCell] = {}
    g_score: Dict[GridCell, float] = {start: 0.0}
    closed_set = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current in closed_set:
            continue

        if current == goal:
            return reconstruct_path(came_from, start, goal)

        closed_set.add(current)

        for neighbor, move_cost in get_8_neighbors(current):
            nr, nc = neighbor

            if not is_inside(free_mask, nr, nc):
                continue

            if not free_mask[nr, nc]:
                continue

            if neighbor in closed_set:
                continue

            tentative_g = g_score[current] + move_cost

            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g

                f = tentative_g + euclidean_cells(neighbor, goal)
                heapq.heappush(open_heap, (f, neighbor))

    return None


def astar_to_any_target(
    free_mask: np.ndarray,
    start: GridCell,
    target_mask: np.ndarray,
) -> Optional[List[GridCell]]:
    """
    Busca o menor caminho entre start e qualquer célula da target_mask.

    Usa custo acumulado como prioridade, equivalente a Dijkstra.
    """
    if not is_inside(free_mask, start[0], start[1]):
        return None

    if not free_mask[start[0], start[1]]:
        return None

    open_heap = []
    heapq.heappush(open_heap, (0.0, start))

    came_from: Dict[GridCell, GridCell] = {}
    g_score: Dict[GridCell, float] = {start: 0.0}
    closed_set = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current in closed_set:
            continue

        if target_mask[current[0], current[1]]:
            return reconstruct_path(came_from, start, current)

        closed_set.add(current)

        for neighbor, move_cost in get_8_neighbors(current):
            nr, nc = neighbor

            if not is_inside(free_mask, nr, nc):
                continue

            if not free_mask[nr, nc]:
                continue

            if neighbor in closed_set:
                continue

            tentative_g = g_score[current] + move_cost

            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g

                heapq.heappush(open_heap, (tentative_g, neighbor))

    return None


# =============================================================================
# Planejamento usando GVD
# =============================================================================

def plan_using_gvd(
    inflated_grid: np.ndarray,
    gvd_mask: np.ndarray,
    start_cell: GridCell,
    goal_cell: GridCell,
    gvd_dilation_cells: int = 0,
) -> Tuple[List[GridCell], np.ndarray]:
    """
    Planeja um caminho usando diretamente as células do GVD.

    Estratégia:
    1. Conectar start até a célula mais próxima do GVD;
    2. Planejar sobre o próprio GVD;
    3. Conectar o GVD até o goal.

    Nesta versão, o caminho sobre o GVD é calculado exatamente sobre
    as células azuis exibidas na figura, sem usar uma máscara dilatada
    diferente da visualização.
    """
    free_mask = inflated_grid == 0

    if not is_inside(inflated_grid, start_cell[0], start_cell[1]):
        raise RuntimeError("Start fora dos limites do grid.")

    if not is_inside(inflated_grid, goal_cell[0], goal_cell[1]):
        raise RuntimeError("Goal fora dos limites do grid.")

    if not free_mask[start_cell[0], start_cell[1]]:
        raise RuntimeError("Start não está em célula livre do grid expandido.")

    if not free_mask[goal_cell[0], goal_cell[1]]:
        raise RuntimeError("Goal não está em célula livre do grid expandido.")

    if np.count_nonzero(gvd_mask) == 0:
        raise RuntimeError("Nenhuma célula de GVD foi encontrada.")

    # A máscara de navegação agora é o próprio GVD.
    # Assim, o caminho calculado sobre o GVD coincide com os pontos azuis.
    gvd_navigation_mask = gvd_mask.copy()

    # -------------------------------------------------------------------------
    # 1. Conexão start -> GVD
    # -------------------------------------------------------------------------
    start_to_gvd = astar_to_any_target(
        free_mask=free_mask,
        start=start_cell,
        target_mask=gvd_navigation_mask,
    )

    if start_to_gvd is None:
        raise RuntimeError("Não foi possível conectar o start ao GVD.")

    start_gvd_cell = start_to_gvd[-1]

    # -------------------------------------------------------------------------
    # 2. Conexão goal -> GVD
    # -------------------------------------------------------------------------
    goal_to_gvd = astar_to_any_target(
        free_mask=free_mask,
        start=goal_cell,
        target_mask=gvd_navigation_mask,
    )

    if goal_to_gvd is None:
        raise RuntimeError("Não foi possível conectar o goal ao GVD.")

    goal_gvd_cell = goal_to_gvd[-1]

    # -------------------------------------------------------------------------
    # 3. Caminho sobre o próprio GVD
    # -------------------------------------------------------------------------
    gvd_path = astar_on_mask(
        free_mask=gvd_navigation_mask,
        start=start_gvd_cell,
        goal=goal_gvd_cell,
    )

    if gvd_path is None:
        raise RuntimeError(
            "Não foi possível encontrar caminho diretamente sobre o GVD. "
            "Tente aumentar --equidistance-tolerance-cells para 2.0 ou 3.0. "
            "Evite usar dilatação se quiser que o caminho siga exatamente os pontos azuis."
        )

    # goal_to_gvd está no sentido goal -> GVD.
    # Invertemos para obter GVD -> goal.
    gvd_to_goal = list(reversed(goal_to_gvd))

    # -------------------------------------------------------------------------
    # 4. Caminho final
    # -------------------------------------------------------------------------
    full_path = []
    full_path.extend(start_to_gvd)
    full_path.extend(gvd_path[1:])
    full_path.extend(gvd_to_goal[1:])

    return full_path, gvd_navigation_mask


# =============================================================================
# Salvamento e carregamento
# =============================================================================

def load_map_files(map_name: str, results_dir: str):
    """
    Carrega os arquivos necessários para o Brushfire/GVD.

    Arquivos esperados:
    - grid original;
    - grid expandido;
    - grid de rótulos geométricos dos obstáculos;
    - metadata do mapa.

    O grid de rótulos geométricos é gerado pelo grid_map.py e permite
    que cada obstáculo do SDF seja tratado como uma fonte distinta no
    Brushfire.
    """
    map_dir = os.path.join(results_dir, map_name)

    original_grid_path = os.path.join(
        map_dir,
        f"{map_name}_grid_original.npy",
    )

    inflated_grid_path = os.path.join(
        map_dir,
        f"{map_name}_grid_inflated.npy",
    )

    site_labels_path = os.path.join(
        map_dir,
        f"{map_name}_obstacle_site_labels.npy",
    )

    metadata_path = os.path.join(
        map_dir,
        f"{map_name}_metadata.yaml",
    )

    if not os.path.exists(original_grid_path):
        raise FileNotFoundError(
            f"Grid original não encontrado: {original_grid_path}"
        )

    if not os.path.exists(inflated_grid_path):
        raise FileNotFoundError(
            f"Grid expandido não encontrado: {inflated_grid_path}"
        )

    if not os.path.exists(site_labels_path):
        raise FileNotFoundError(
            f"Grid de rótulos geométricos não encontrado: {site_labels_path}\n"
            "Execute novamente o grid_map.py atualizado antes de rodar o GVD."
        )

    if not os.path.exists(metadata_path):
        raise FileNotFoundError(
            f"Metadata não encontrado: {metadata_path}"
        )

    original_grid = np.load(original_grid_path)
    inflated_grid = np.load(inflated_grid_path)
    obstacle_site_labels = np.load(site_labels_path)

    metadata = load_metadata(metadata_path)
    bounds = bounds_from_metadata(metadata)

    return original_grid, inflated_grid, obstacle_site_labels, metadata, bounds


def save_path_yaml(
    output_path: str,
    map_name: str,
    algorithm: str,
    path_cells: List[GridCell],
    path_world: List[Tuple[float, float]],
    start_world: Tuple[float, float],
    goal_world: Tuple[float, float],
    resolution: float,
    gvd_cells_count: int,
    gvd_navigation_cells_count: int,
    obstacle_components_count: int,
    min_distance_cells: float,
    equidistance_tolerance_cells: float,
    gvd_dilation_cells: int,
):
    data = {
        "map_name": map_name,
        "algorithm": algorithm,
        "resolution": float(resolution),
        "start_world": [float(start_world[0]), float(start_world[1])],
        "goal_world": [float(goal_world[0]), float(goal_world[1])],
        "path_length_cells": int(len(path_cells)),
        "gvd_cells_count": int(gvd_cells_count),
        "gvd_navigation_cells_count": int(gvd_navigation_cells_count),
        "obstacle_components_count": int(obstacle_components_count),
        "parameters": {
            "min_distance_cells": float(min_distance_cells),
            "equidistance_tolerance_cells": float(equidistance_tolerance_cells),
            "gvd_dilation_cells": int(gvd_dilation_cells),
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


# =============================================================================
# Função principal
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Brushfire + GVD para mapas discretizados do TP2."
    )

    parser.add_argument(
        "--map-name",
        required=True,
        help="Nome do mapa. Exemplo: tp2_map1 ou tp2_map2.",
    )

    parser.add_argument(
        "--results-dir",
        default="results",
        help="Diretório dos resultados gerados pelo grid_map.py.",
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
        "--snap-to-free",
        action="store_true",
        help="Move start/goal para a célula livre mais próxima, se necessário.",
    )

    parser.add_argument(
        "--min-distance-cells",
        type=float,
        default=1.0,
        help="Distância mínima aos obstáculos, em células, para aceitar uma célula como GVD.",
    )

    parser.add_argument(
        "--equidistance-tolerance-cells",
        type=float,
        default=2.0,
        help="Tolerância, em células, para considerar dois obstáculos aproximadamente equidistantes.",
    )

    parser.add_argument(
        "--gvd-dilation",
        type=int,
        default=1,
        help="Dilatação da máscara do GVD para melhorar conectividade.",
    )

    args = parser.parse_args()

    original_grid, inflated_grid, obstacle_site_labels, metadata, bounds = load_map_files(
        map_name=args.map_name,
        results_dir=args.results_dir,
    )

    resolution = float(metadata["resolution"])

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
    print("Executando Brushfire + GVD")
    print(f"Mapa: {args.map_name}")
    print(f"Resolução: {resolution:.3f} m/célula")
    print(f"Start mundo: {start_world} -> célula {start_cell}")
    print(f"Goal mundo: {goal_world} -> célula {goal_cell}")
    print(f"Min. distância GVD: {args.min_distance_cells:.2f} células")
    print(f"Tolerância de equidistância: {args.equidistance_tolerance_cells:.2f} células")
    print(f"Dilatação do GVD: {args.gvd_dilation} células")

    if not is_inside(inflated_grid, start_cell[0], start_cell[1]):
        raise RuntimeError("Start está fora dos limites do grid.")

    if not is_inside(inflated_grid, goal_cell[0], goal_cell[1]):
        raise RuntimeError("Goal está fora dos limites do grid.")

    if not is_cell_free(inflated_grid, start_cell[0], start_cell[1]):
        print("[AVISO] Start está em célula ocupada ou expandida.")

        if args.snap_to_free:
            new_start = find_nearest_free_cell(inflated_grid, start_cell)

            if new_start is None:
                raise RuntimeError("Não foi possível ajustar o start para célula livre.")

            start_cell = new_start
            start_world = grid_to_world(start_cell[0], start_cell[1], bounds, resolution)

            print(f"        Start ajustado para célula {start_cell}, mundo {start_world}")
        else:
            raise RuntimeError("Start inválido. Use --snap-to-free ou altere o ponto inicial.")

    if not is_cell_free(inflated_grid, goal_cell[0], goal_cell[1]):
        print("[AVISO] Goal está em célula ocupada ou expandida.")

        if args.snap_to_free:
            new_goal = find_nearest_free_cell(inflated_grid, goal_cell)

            if new_goal is None:
                raise RuntimeError("Não foi possível ajustar o goal para célula livre.")

            goal_cell = new_goal
            goal_world = grid_to_world(goal_cell[0], goal_cell[1], bounds, resolution)

            print(f"        Goal ajustado para célula {goal_cell}, mundo {goal_world}")
        else:
            raise RuntimeError("Goal inválido. Use --snap-to-free ou altere o ponto objetivo.")

    
    # -------------------------------------------------------------------------
    # Rótulos geométricos dos obstáculos
    # -------------------------------------------------------------------------
    obstacle_labels = obstacle_site_labels
    obstacle_count = int(np.max(obstacle_labels))

    print(f"Obstáculos geométricos rotulados: {obstacle_count}")

    if obstacle_count < 2:
        raise RuntimeError(
            "O GVD precisa de pelo menos dois obstáculos geométricos rotulados. "
            "Verifique o grid de rótulos gerado pelo grid_map.py."
        )

    # -------------------------------------------------------------------------
    # Brushfire
    # -------------------------------------------------------------------------
    first_distance, first_label, second_distance, second_label = compute_brushfire(
        original_grid=original_grid,
        obstacle_labels=obstacle_labels,
    )

    # -------------------------------------------------------------------------
    # Extração do GVD
    # -------------------------------------------------------------------------
    gvd_mask = extract_gvd_mask(
        inflated_grid=inflated_grid,
        first_label=first_label,
        second_label=second_label,
        first_distance=first_distance,
        second_distance=second_distance,
        min_distance_cells=args.min_distance_cells,
        equidistance_tolerance_cells=args.equidistance_tolerance_cells,
    )

    gvd_cells = gvd_cells_from_mask(gvd_mask)

    print(f"Células do GVD extraídas: {len(gvd_cells)}")

    if len(gvd_cells) == 0:
        raise RuntimeError(
            "Nenhuma célula de GVD foi extraída. "
            "Tente aumentar --equidistance-tolerance-cells para 3.0 ou 4.0, "
            "ou reduzir a inflação do grid."
        )

    # -------------------------------------------------------------------------
    # Planejamento usando o GVD
    # -------------------------------------------------------------------------
    path_cells, gvd_navigation_mask = plan_using_gvd(
        inflated_grid=inflated_grid,
        gvd_mask=gvd_mask,
        start_cell=start_cell,
        goal_cell=goal_cell,
        gvd_dilation_cells=args.gvd_dilation,
    )

    path_world = cells_to_world_path(path_cells, bounds, resolution)

    print(f"Caminho GVD encontrado com {len(path_cells)} células.")
    print(f"Células na máscara navegável do GVD: {np.count_nonzero(gvd_navigation_mask)}")

    # -------------------------------------------------------------------------
    # Salvamento
    # -------------------------------------------------------------------------
    map_output_dir = os.path.join(args.results_dir, args.map_name)
    os.makedirs(map_output_dir, exist_ok=True)

    first_distance_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_brushfire_first_distance.npy",
    )

    second_distance_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_brushfire_second_distance.npy",
    )

    first_label_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_brushfire_first_label.npy",
    )

    second_label_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_brushfire_second_label.npy",
    )

    obstacle_labels_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_obstacle_labels.npy",
    )

    gvd_mask_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_gvd_mask.npy",
    )

    gvd_navigation_mask_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_gvd_navigation_mask.npy",
    )

    path_cells_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_gvd_path_cells.npy",
    )

    path_world_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_gvd_path_world.npy",
    )

    path_yaml_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_gvd_path.yaml",
    )

    gvd_image_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_gvd_map.png",
    )

    path_image_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_gvd_path.png",
    )

    np.save(first_distance_path, first_distance)
    np.save(second_distance_path, second_distance)
    np.save(first_label_path, first_label)
    np.save(second_label_path, second_label)
    np.save(obstacle_labels_path, obstacle_labels)
    np.save(gvd_mask_path, gvd_mask)
    np.save(gvd_navigation_mask_path, gvd_navigation_mask)
    np.save(path_cells_path, np.array(path_cells, dtype=np.int32))
    np.save(path_world_path, np.array(path_world, dtype=np.float64))

    save_path_yaml(
        output_path=path_yaml_path,
        map_name=args.map_name,
        algorithm="Brushfire + GVD",
        path_cells=path_cells,
        path_world=path_world,
        start_world=start_world,
        goal_world=goal_world,
        resolution=resolution,
        gvd_cells_count=len(gvd_cells),
        gvd_navigation_cells_count=int(np.count_nonzero(gvd_navigation_mask)),
        obstacle_components_count=obstacle_count,
        min_distance_cells=args.min_distance_cells,
        equidistance_tolerance_cells=args.equidistance_tolerance_cells,
        gvd_dilation_cells=args.gvd_dilation,
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

    print("")
    print("Arquivos gerados:")
    print(f"- {first_distance_path}")
    print(f"- {second_distance_path}")
    print(f"- {first_label_path}")
    print(f"- {second_label_path}")
    print(f"- {obstacle_labels_path}")
    print(f"- {gvd_mask_path}")
    print(f"- {gvd_navigation_mask_path}")
    print(f"- {path_cells_path}")
    print(f"- {path_world_path}")
    print(f"- {path_yaml_path}")
    print(f"- {gvd_image_path}")
    print(f"- {path_image_path}")
    print("")


if __name__ == "__main__":
    main()