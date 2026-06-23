#!/usr/bin/env python3

"""
rrt_planner.py

Questão 3 do Trabalho Prático 2:
- Implementação do algoritmo RRT;
- Planejamento baseado em amostragem;
- Verificação de colisão usando o grid expandido;
- Geração de imagem com árvore RRT e caminho planejado;
- Salvamento do caminho para navegação no Gazebo.

Entrada:
- results/<map_name>/<map_name>_grid_inflated.npy
- results/<map_name>/<map_name>_metadata.yaml

Saídas:
- <map_name>_rrt_path_world.npy
- <map_name>_rrt_path_cells.npy
- <map_name>_rrt_path.yaml
- <map_name>_rrt_path.png

Convenção do grid:
0 = célula livre
1 = obstáculo original
2 = obstáculo expandido
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
class RRTNode:
    x: float
    y: float
    parent: Optional[int]


# =============================================================================
# Funções geométricas
# =============================================================================

def euclidean_distance(p1: Point2D, p2: Point2D) -> float:
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.sqrt(dx * dx + dy * dy)


def steer(from_point: Point2D, to_point: Point2D, step_size: float) -> Point2D:
    """
    Avança de from_point na direção de to_point com passo máximo step_size.
    """
    distance = euclidean_distance(from_point, to_point)

    if distance <= step_size:
        return to_point

    theta = math.atan2(to_point[1] - from_point[1], to_point[0] - from_point[0])

    new_x = from_point[0] + step_size * math.cos(theta)
    new_y = from_point[1] + step_size * math.sin(theta)

    return new_x, new_y


def nearest_node_index(nodes: List[RRTNode], point: Point2D) -> int:
    """
    Retorna o índice do nó da árvore mais próximo do ponto amostrado.
    """
    best_index = 0
    best_distance = float("inf")

    for i, node in enumerate(nodes):
        distance = euclidean_distance((node.x, node.y), point)

        if distance < best_distance:
            best_distance = distance
            best_index = i

    return best_index


# =============================================================================
# Verificação de colisão
# =============================================================================

def point_is_free(
    point: Point2D,
    grid: np.ndarray,
    bounds: MapBounds,
    resolution: float,
) -> bool:
    """
    Verifica se um ponto contínuo do mundo está em uma célula livre.
    """
    row, col = world_to_grid(point[0], point[1], bounds, resolution)

    return is_cell_free(grid, row, col)


def segment_is_free(
    p1: Point2D,
    p2: Point2D,
    grid: np.ndarray,
    bounds: MapBounds,
    resolution: float,
    collision_check_step: float,
) -> bool:
    """
    Verifica se o segmento p1 -> p2 está livre.

    O segmento é discretizado em pequenos passos contínuos, e cada ponto
    intermediário é convertido para célula do grid expandido.
    """
    distance = euclidean_distance(p1, p2)

    if distance == 0.0:
        return point_is_free(p1, grid, bounds, resolution)

    n_steps = max(2, int(math.ceil(distance / collision_check_step)))

    for i in range(n_steps + 1):
        t = i / n_steps

        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])

        if not point_is_free((x, y), grid, bounds, resolution):
            return False

    return True


def find_nearest_free_point(
    grid: np.ndarray,
    point: Point2D,
    bounds: MapBounds,
    resolution: float,
    max_radius_cells: int = 60,
) -> Optional[Point2D]:
    """
    Caso start ou goal estejam em região ocupada/expandida, procura
    a célula livre mais próxima e retorna seu centro em coordenadas do mundo.
    """
    row, col = world_to_grid(point[0], point[1], bounds, resolution)

    if is_cell_free(grid, row, col):
        return point

    best_cell = None
    best_distance = float("inf")

    for radius in range(1, max_radius_cells + 1):
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
            return grid_to_world(best_cell[0], best_cell[1], bounds, resolution)

    return None


# =============================================================================
# RRT
# =============================================================================

def sample_free_point(
    grid: np.ndarray,
    bounds: MapBounds,
    resolution: float,
    goal: Point2D,
    goal_bias: float,
) -> Point2D:
    """
    Sorteia um ponto livre no espaço de trabalho.

    Com probabilidade goal_bias, retorna o próprio goal.
    Isso acelera a convergência do RRT para o objetivo.
    """
    if random.random() < goal_bias:
        return goal

    while True:
        x = random.uniform(bounds.xmin, bounds.xmax)
        y = random.uniform(bounds.ymin, bounds.ymax)

        if point_is_free((x, y), grid, bounds, resolution):
            return x, y


def reconstruct_rrt_path(nodes: List[RRTNode], goal_index: int) -> List[Point2D]:
    """
    Reconstrói o caminho do goal até o start usando os pais dos nós.
    """
    path = []

    current_index = goal_index

    while current_index is not None:
        node = nodes[current_index]
        path.append((node.x, node.y))
        current_index = node.parent

    path.reverse()
    return path


def rrt_search(
    grid: np.ndarray,
    bounds: MapBounds,
    resolution: float,
    start: Point2D,
    goal: Point2D,
    max_iterations: int,
    step_size: float,
    goal_tolerance: float,
    goal_bias: float,
    collision_check_step: float,
) -> Tuple[Optional[List[Point2D]], List[RRTNode]]:
    """
    Executa o RRT.

    Retorna:
    - caminho em coordenadas do mundo, se encontrado;
    - lista completa de nós da árvore.
    """
    nodes: List[RRTNode] = [RRTNode(start[0], start[1], parent=None)]

    for iteration in range(max_iterations):
        random_point = sample_free_point(
            grid=grid,
            bounds=bounds,
            resolution=resolution,
            goal=goal,
            goal_bias=goal_bias,
        )

        nearest_index = nearest_node_index(nodes, random_point)
        nearest_node = nodes[nearest_index]
        nearest_point = (nearest_node.x, nearest_node.y)

        new_point = steer(nearest_point, random_point, step_size)

        if not point_is_free(new_point, grid, bounds, resolution):
            continue

        if not segment_is_free(
            p1=nearest_point,
            p2=new_point,
            grid=grid,
            bounds=bounds,
            resolution=resolution,
            collision_check_step=collision_check_step,
        ):
            continue

        nodes.append(
            RRTNode(
                x=new_point[0],
                y=new_point[1],
                parent=nearest_index,
            )
        )

        new_index = len(nodes) - 1

        # Tenta conectar o novo nó ao objetivo
        distance_to_goal = euclidean_distance(new_point, goal)

        if distance_to_goal <= goal_tolerance:
            if segment_is_free(
                p1=new_point,
                p2=goal,
                grid=grid,
                bounds=bounds,
                resolution=resolution,
                collision_check_step=collision_check_step,
            ):
                nodes.append(
                    RRTNode(
                        x=goal[0],
                        y=goal[1],
                        parent=new_index,
                    )
                )

                goal_index = len(nodes) - 1
                path = reconstruct_rrt_path(nodes, goal_index)

                print(f"RRT encontrou solução na iteração {iteration + 1}.")
                return path, nodes

    return None, nodes


# =============================================================================
# Suavização opcional
# =============================================================================

def shortcut_smooth_path(
    path: List[Point2D],
    grid: np.ndarray,
    bounds: MapBounds,
    resolution: float,
    collision_check_step: float,
    smooth_iterations: int,
) -> List[Point2D]:
    """
    Suaviza o caminho com atalhos aleatórios.

    Essa etapa não faz parte essencial do RRT, mas melhora o seguimento
    pelo robô, removendo zigue-zagues desnecessários.
    """
    if len(path) <= 2 or smooth_iterations <= 0:
        return path

    smoothed = path.copy()

    for _ in range(smooth_iterations):
        if len(smoothed) <= 2:
            break

        i = random.randint(0, len(smoothed) - 2)
        j = random.randint(i + 1, len(smoothed) - 1)

        if j <= i + 1:
            continue

        if segment_is_free(
            p1=smoothed[i],
            p2=smoothed[j],
            grid=grid,
            bounds=bounds,
            resolution=resolution,
            collision_check_step=collision_check_step,
        ):
            smoothed = smoothed[: i + 1] + smoothed[j:]

    return smoothed


# =============================================================================
# Conversão e salvamento
# =============================================================================

def world_path_to_cells(
    path_world: List[Point2D],
    bounds: MapBounds,
    resolution: float,
) -> List[GridCell]:
    return [
        world_to_grid(x, y, bounds, resolution)
        for x, y in path_world
    ]


def save_path_yaml(
    output_path: str,
    map_name: str,
    algorithm: str,
    path_world: List[Point2D],
    path_cells: List[GridCell],
    start: Point2D,
    goal: Point2D,
    resolution: float,
    max_iterations: int,
    step_size: float,
    goal_tolerance: float,
    goal_bias: float,
    collision_check_step: float,
    smooth_iterations: int,
    tree_nodes_count: int,
):
    data = {
        "map_name": map_name,
        "algorithm": algorithm,
        "resolution": float(resolution),
        "start_world": [float(start[0]), float(start[1])],
        "goal_world": [float(goal[0]), float(goal[1])],
        "path_length_points": int(len(path_world)),
        "tree_nodes_count": int(tree_nodes_count),
        "parameters": {
            "max_iterations": int(max_iterations),
            "step_size": float(step_size),
            "goal_tolerance": float(goal_tolerance),
            "goal_bias": float(goal_bias),
            "collision_check_step": float(collision_check_step),
            "smooth_iterations": int(smooth_iterations),
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
# Renderização
# =============================================================================

def render_rrt_result(
    grid: np.ndarray,
    bounds: MapBounds,
    resolution: float,
    nodes: List[RRTNode],
    path_world: List[Point2D],
    start: Point2D,
    goal: Point2D,
    output_path: str,
    title: str,
    draw_tree: bool = True,
):
    """
    Renderiza o grid expandido, a árvore RRT e o caminho planejado.
    """
    cmap = ListedColormap(
        [
            "white",      # livre
            "black",      # obstáculo original
            "lightgray",  # obstáculo expandido
        ]
    )

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

    if len(path_world) > 0:
        path_x = [p[0] for p in path_world]
        path_y = [p[1] for p in path_world]

        ax.plot(
            path_x,
            path_y,
            color="orange",
            linewidth=2.5,
            label="Caminho RRT",
        )

        ax.scatter(
            path_x,
            path_y,
            color="orange",
            s=12,
        )

    ax.plot(
        start[0],
        start[1],
        marker="o",
        markersize=8,
        color="green",
        label="Start",
    )

    ax.plot(
        goal[0],
        goal[1],
        marker="x",
        markersize=9,
        color="red",
        label="Goal",
    )

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


# =============================================================================
# Função principal
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Planejador RRT para a Questão 3 do TP2."
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
        help="Coordenada x inicial. Se omitida, usa metadata.",
    )

    parser.add_argument(
        "--start-y",
        type=float,
        default=None,
        help="Coordenada y inicial. Se omitida, usa metadata.",
    )

    parser.add_argument(
        "--goal-x",
        type=float,
        default=None,
        help="Coordenada x objetivo. Se omitida, usa metadata.",
    )

    parser.add_argument(
        "--goal-y",
        type=float,
        default=None,
        help="Coordenada y objetivo. Se omitida, usa metadata.",
    )

    parser.add_argument(
        "--snap-to-free",
        action="store_true",
        help="Move start/goal para célula livre mais próxima, se necessário.",
    )

    parser.add_argument(
        "--max-iterations",
        type=int,
        default=12000,
        help="Número máximo de iterações do RRT.",
    )

    parser.add_argument(
        "--step-size",
        type=float,
        default=0.60,
        help="Passo máximo de expansão da árvore em metros.",
    )

    parser.add_argument(
        "--goal-tolerance",
        type=float,
        default=0.60,
        help="Distância para tentar conectar ao objetivo.",
    )

    parser.add_argument(
        "--goal-bias",
        type=float,
        default=0.10,
        help="Probabilidade de amostrar diretamente o goal.",
    )

    parser.add_argument(
        "--collision-check-step",
        type=float,
        default=0.05,
        help="Passo de verificação de colisão ao longo dos segmentos.",
    )

    parser.add_argument(
        "--smooth-iterations",
        type=int,
        default=150,
        help="Número de tentativas de suavização por atalhos.",
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Semente aleatória para reprodutibilidade.",
    )

    parser.add_argument(
        "--no-tree",
        action="store_true",
        help="Não desenha a árvore RRT na imagem final.",
    )

    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)

    grid, metadata, bounds = load_grid_package(
        map_name=args.map_name,
        results_dir=args.results_dir,
    )

    resolution = float(metadata["resolution"])

    metadata_start = metadata.get("start", None)
    metadata_goal = metadata.get("goal", None)

    if args.start_x is not None and args.start_y is not None:
        start = (args.start_x, args.start_y)
    elif metadata_start is not None:
        start = (float(metadata_start[0]), float(metadata_start[1]))
    else:
        raise ValueError("Start não informado e inexistente no metadata.")

    if args.goal_x is not None and args.goal_y is not None:
        goal = (args.goal_x, args.goal_y)
    elif metadata_goal is not None:
        goal = (float(metadata_goal[0]), float(metadata_goal[1]))
    else:
        raise ValueError("Goal não informado e inexistente no metadata.")

    print("")
    print("Executando RRT")
    print(f"Mapa: {args.map_name}")
    print(f"Resolução: {resolution:.3f} m/célula")
    print(f"Start: {start}")
    print(f"Goal: {goal}")
    print(f"Iterações máximas: {args.max_iterations}")
    print(f"Step size: {args.step_size:.3f} m")
    print(f"Goal bias: {args.goal_bias:.3f}")
    print(f"Semente aleatória: {args.seed}")

    if not point_is_free(start, grid, bounds, resolution):
        print("[AVISO] Start está em célula ocupada ou expandida.")

        if args.snap_to_free:
            new_start = find_nearest_free_point(grid, start, bounds, resolution)

            if new_start is None:
                raise RuntimeError("Não foi possível ajustar o start.")

            print(f"        Start ajustado para {new_start}")
            start = new_start
        else:
            raise RuntimeError("Start inválido. Use --snap-to-free.")

    if not point_is_free(goal, grid, bounds, resolution):
        print("[AVISO] Goal está em célula ocupada ou expandida.")

        if args.snap_to_free:
            new_goal = find_nearest_free_point(grid, goal, bounds, resolution)

            if new_goal is None:
                raise RuntimeError("Não foi possível ajustar o goal.")

            print(f"        Goal ajustado para {new_goal}")
            goal = new_goal
        else:
            raise RuntimeError("Goal inválido. Use --snap-to-free.")

    path_world, nodes = rrt_search(
        grid=grid,
        bounds=bounds,
        resolution=resolution,
        start=start,
        goal=goal,
        max_iterations=args.max_iterations,
        step_size=args.step_size,
        goal_tolerance=args.goal_tolerance,
        goal_bias=args.goal_bias,
        collision_check_step=args.collision_check_step,
    )

    if path_world is None:
        raise RuntimeError(
            "RRT não encontrou caminho. Tente aumentar --max-iterations, "
            "--goal-bias ou ajustar --step-size."
        )

    print(f"Caminho original encontrado com {len(path_world)} pontos.")
    print(f"Nós gerados na árvore: {len(nodes)}")

    smoothed_path = shortcut_smooth_path(
        path=path_world,
        grid=grid,
        bounds=bounds,
        resolution=resolution,
        collision_check_step=args.collision_check_step,
        smooth_iterations=args.smooth_iterations,
    )

    print(f"Caminho após suavização: {len(smoothed_path)} pontos.")

    path_cells = world_path_to_cells(smoothed_path, bounds, resolution)

    map_output_dir = os.path.join(args.results_dir, args.map_name)
    os.makedirs(map_output_dir, exist_ok=True)

    path_world_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_rrt_path_world.npy",
    )

    path_cells_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_rrt_path_cells.npy",
    )

    path_yaml_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_rrt_path.yaml",
    )

    image_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_rrt_path.png",
    )

    np.save(path_world_path, np.array(smoothed_path, dtype=np.float64))
    np.save(path_cells_path, np.array(path_cells, dtype=np.int32))

    save_path_yaml(
        output_path=path_yaml_path,
        map_name=args.map_name,
        algorithm="RRT",
        path_world=smoothed_path,
        path_cells=path_cells,
        start=start,
        goal=goal,
        resolution=resolution,
        max_iterations=args.max_iterations,
        step_size=args.step_size,
        goal_tolerance=args.goal_tolerance,
        goal_bias=args.goal_bias,
        collision_check_step=args.collision_check_step,
        smooth_iterations=args.smooth_iterations,
        tree_nodes_count=len(nodes),
    )

    render_rrt_result(
        grid=grid,
        bounds=bounds,
        resolution=resolution,
        nodes=nodes,
        path_world=smoothed_path,
        start=start,
        goal=goal,
        output_path=image_path,
        title=f"{args.map_name}: caminho planejado por RRT",
        draw_tree=not args.no_tree,
    )

    print("")
    print("Arquivos gerados:")
    print(f"- {path_world_path}")
    print(f"- {path_cells_path}")
    print(f"- {path_yaml_path}")
    print(f"- {image_path}")
    print("")


if __name__ == "__main__":
    main()
