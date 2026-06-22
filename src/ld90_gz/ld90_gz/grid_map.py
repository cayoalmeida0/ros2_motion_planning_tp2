#!/usr/bin/env python3

"""
grid_map.py

Gera um mapa discretizado 2D a partir de um arquivo SDF do Gazebo.

Funcionalidades:
- Lê mapas .sdf;
- Extrai obstáculos retangulares do tipo <box>;
- Extrai obstáculos circulares do tipo <cylinder>;
- Ignora o ground_plane;
- Gera occupancy grid;
- Expande obstáculos considerando raio de segurança;
- Salva arquivos .npy e .yaml;
- Gera imagem da vista superior com:
    - células livres;
    - obstáculos originais;
    - obstáculos expandidos;
    - grade discreta;
    - start e goal.

Convenção do grid:
0 = célula livre
1 = obstáculo original
2 = obstáculo expandido
"""

import argparse
import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.colors import ListedColormap


# =============================================================================
# Estruturas de dados
# =============================================================================

@dataclass
class Pose2D:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass
class MapBounds:
    xmin: float
    xmax: float
    ymin: float
    ymax: float


@dataclass
class BoxObstacle:
    name: str
    collision_name: str
    x: float
    y: float
    yaw: float
    size_x: float
    size_y: float
    size_z: float


@dataclass
class CylinderObstacle:
    name: str
    collision_name: str
    x: float
    y: float
    yaw: float
    radius: float
    length: float


# =============================================================================
# Funções de pose e leitura do SDF
# =============================================================================

def parse_pose(element: Optional[ET.Element]) -> Pose2D:
    """
    Lê uma pose no padrão SDF:
    x y z roll pitch yaw

    Caso a pose não exista, retorna pose nula.
    """
    if element is None or element.text is None:
        return Pose2D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    values = [float(v) for v in element.text.split()]

    while len(values) < 6:
        values.append(0.0)

    return Pose2D(
        x=values[0],
        y=values[1],
        z=values[2],
        roll=values[3],
        pitch=values[4],
        yaw=values[5],
    )


def compose_pose(parent: Pose2D, child: Pose2D) -> Pose2D:
    """
    Compõe duas poses usando uma aproximação planar.

    Para este trabalho, os obstáculos relevantes estão apoiados no plano XY.
    Assim, a composição considera corretamente translação e yaw em 2D.

    A pose final é:
    T_final = T_parent * T_child
    """
    c = math.cos(parent.yaw)
    s = math.sin(parent.yaw)

    x = parent.x + c * child.x - s * child.y
    y = parent.y + s * child.x + c * child.y
    z = parent.z + child.z

    roll = parent.roll + child.roll
    pitch = parent.pitch + child.pitch
    yaw = parent.yaw + child.yaw

    return Pose2D(x, y, z, roll, pitch, yaw)


def extract_ground_bounds(root: ET.Element) -> MapBounds:
    """
    Define os limites do mapa com base no ground_plane.

    Para os mapas do TP2, o ground_plane é um box de 20 x 20 m
    centrado na origem. Caso ele não seja encontrado, usa fallback
    em [-10, 10] para x e y.
    """
    for model in root.findall(".//model"):
        model_name = model.attrib.get("name", "")

        if model_name != "ground_plane":
            continue

        model_pose = parse_pose(model.find("pose"))

        box_size = model.find(".//collision/geometry/box/size")
        if box_size is None or box_size.text is None:
            continue

        sx, sy, _ = [float(v) for v in box_size.text.split()]

        return MapBounds(
            xmin=model_pose.x - sx / 2.0,
            xmax=model_pose.x + sx / 2.0,
            ymin=model_pose.y - sy / 2.0,
            ymax=model_pose.y + sy / 2.0,
        )

    return MapBounds(xmin=-10.0, xmax=10.0, ymin=-10.0, ymax=10.0)


def extract_obstacles_from_sdf(
    sdf_path: str,
) -> Tuple[MapBounds, List[BoxObstacle], List[CylinderObstacle]]:
    """
    Extrai obstáculos do arquivo SDF.

    São considerados:
    - box: paredes e obstáculos retangulares;
    - cylinder: obstáculos circulares/orgânicos.

    O modelo ground_plane é ignorado.
    """
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    bounds = extract_ground_bounds(root)

    box_obstacles: List[BoxObstacle] = []
    cylinder_obstacles: List[CylinderObstacle] = []

    for model in root.findall(".//model"):
        model_name = model.attrib.get("name", "")

        if model_name == "ground_plane":
            continue

        model_pose = parse_pose(model.find("pose"))

        for link in model.findall("link"):
            link_pose = parse_pose(link.find("pose"))
            model_link_pose = compose_pose(model_pose, link_pose)

            for collision in link.findall("collision"):
                collision_name = collision.attrib.get("name", "")
                collision_pose = parse_pose(collision.find("pose"))
                total_pose = compose_pose(model_link_pose, collision_pose)

                # -------------------------------------------------------------
                # Obstáculos do tipo box
                # -------------------------------------------------------------
                box_size = collision.find("geometry/box/size")

                if box_size is not None and box_size.text is not None:
                    sx, sy, sz = [float(v) for v in box_size.text.split()]

                    # Ignora geometrias praticamente planas.
                    # O ground_plane já foi ignorado pelo nome do modelo,
                    # mas este filtro evita ruídos futuros.
                    if sz >= 0.05:
                        box_obstacles.append(
                            BoxObstacle(
                                name=model_name,
                                collision_name=collision_name,
                                x=total_pose.x,
                                y=total_pose.y,
                                yaw=total_pose.yaw,
                                size_x=sx,
                                size_y=sy,
                                size_z=sz,
                            )
                        )

                    continue

                # -------------------------------------------------------------
                # Obstáculos do tipo cylinder
                # -------------------------------------------------------------
                cylinder_radius = collision.find("geometry/cylinder/radius")
                cylinder_length = collision.find("geometry/cylinder/length")

                if (
                    cylinder_radius is not None
                    and cylinder_radius.text is not None
                    and cylinder_length is not None
                    and cylinder_length.text is not None
                ):
                    radius = float(cylinder_radius.text)
                    length = float(cylinder_length.text)

                    if length >= 0.05:
                        cylinder_obstacles.append(
                            CylinderObstacle(
                                name=model_name,
                                collision_name=collision_name,
                                x=total_pose.x,
                                y=total_pose.y,
                                yaw=total_pose.yaw,
                                radius=radius,
                                length=length,
                            )
                        )

    return bounds, box_obstacles, cylinder_obstacles


# =============================================================================
# Conversões entre mundo e grid
# =============================================================================

def create_empty_grid(
    bounds: MapBounds,
    resolution: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Cria uma matriz de ocupação vazia.

    Retorna:
    - grid;
    - centros das células no eixo x;
    - centros das células no eixo y.
    """
    width = bounds.xmax - bounds.xmin
    height = bounds.ymax - bounds.ymin

    cols = int(math.ceil(width / resolution))
    rows = int(math.ceil(height / resolution))

    grid = np.zeros((rows, cols), dtype=np.uint8)

    x_centers = bounds.xmin + (np.arange(cols) + 0.5) * resolution
    y_centers = bounds.ymin + (np.arange(rows) + 0.5) * resolution

    return grid, x_centers, y_centers


def world_to_grid(
    x: float,
    y: float,
    bounds: MapBounds,
    resolution: float,
) -> Tuple[int, int]:
    """
    Converte coordenada do mundo para índice do grid.

    Retorno:
    row, col
    """
    col = int((x - bounds.xmin) / resolution)
    row = int((y - bounds.ymin) / resolution)

    return row, col


def grid_to_world(
    row: int,
    col: int,
    bounds: MapBounds,
    resolution: float,
) -> Tuple[float, float]:
    """
    Converte índice do grid para coordenada do mundo,
    retornando o centro da célula.
    """
    x = bounds.xmin + (col + 0.5) * resolution
    y = bounds.ymin + (row + 0.5) * resolution

    return x, y


def is_cell_inside(grid: np.ndarray, row: int, col: int) -> bool:
    return 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]


def is_cell_free(grid: np.ndarray, row: int, col: int) -> bool:
    if not is_cell_inside(grid, row, col):
        return False

    return grid[row, col] == 0


# =============================================================================
# Rasterização e inflação dos obstáculos
# =============================================================================

def rasterize_obstacles(
    grid: np.ndarray,
    x_centers: np.ndarray,
    y_centers: np.ndarray,
    box_obstacles: List[BoxObstacle],
    cylinder_obstacles: List[CylinderObstacle],
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Rasteriza os obstáculos do SDF no grid.

    Além do grid de ocupação original, esta função gera também um grid
    de rótulos geométricos, no qual cada obstáculo individual do SDF
    recebe um identificador próprio.

    Retorna:
    - occupied:
        0 = célula livre
        1 = obstáculo original

    - site_labels:
        0 = célula sem obstáculo
        1, 2, 3, ... = rótulo de cada obstáculo geométrico do SDF

    Observação:
    A rotulagem por obstáculo geométrico é importante para o Brushfire/GVD,
    pois permite que paredes externas e internas diferentes sejam tratadas
    como fontes distintas de propagação.
    """
    occupied = grid.copy()
    site_labels = np.zeros_like(grid, dtype=np.int32)

    xx, yy = np.meshgrid(x_centers, y_centers)

    site_id = 0

    # -------------------------------------------------------------------------
    # Obstáculos retangulares do tipo box
    # -------------------------------------------------------------------------
    for obs in box_obstacles:
        site_id += 1

        cos_yaw = math.cos(-obs.yaw)
        sin_yaw = math.sin(-obs.yaw)

        dx = xx - obs.x
        dy = yy - obs.y

        local_x = cos_yaw * dx - sin_yaw * dy
        local_y = sin_yaw * dx + cos_yaw * dy

        half_x = obs.size_x / 2.0
        half_y = obs.size_y / 2.0

        mask = (np.abs(local_x) <= half_x) & (np.abs(local_y) <= half_y)

        occupied[mask] = 1

        # Mantém o primeiro rótulo em caso de sobreposição.
        unlabeled_mask = mask & (site_labels == 0)
        site_labels[unlabeled_mask] = site_id

    # -------------------------------------------------------------------------
    # Obstáculos circulares do tipo cylinder
    # -------------------------------------------------------------------------
    for obs in cylinder_obstacles:
        site_id += 1

        dx = xx - obs.x
        dy = yy - obs.y

        mask = (dx * dx + dy * dy) <= obs.radius * obs.radius

        occupied[mask] = 1

        # Mantém o primeiro rótulo em caso de sobreposição.
        unlabeled_mask = mask & (site_labels == 0)
        site_labels[unlabeled_mask] = site_id

    return occupied, site_labels


def inflate_obstacles(
    original_grid: np.ndarray,
    resolution: float,
    inflation_radius: float,
) -> np.ndarray:
    """
    Expande os obstáculos originais usando uma vizinhança circular.

    Valor:
    1 = obstáculo original
    2 = obstáculo expandido

    A expansão representa uma aproximação do espaço de configuração:
    o robô passa a ser tratado como ponto e os obstáculos são aumentados.
    """
    inflated_grid = original_grid.copy()

    inflation_cells = int(math.ceil(inflation_radius / resolution))
    obstacle_cells = np.argwhere(original_grid == 1)

    for row, col in obstacle_cells:
        for dr in range(-inflation_cells, inflation_cells + 1):
            for dc in range(-inflation_cells, inflation_cells + 1):
                rr = row + dr
                cc = col + dc

                if not is_cell_inside(original_grid, rr, cc):
                    continue

                distance = math.sqrt((dr * resolution) ** 2 + (dc * resolution) ** 2)

                if distance <= inflation_radius and inflated_grid[rr, cc] == 0:
                    inflated_grid[rr, cc] = 2

    return inflated_grid


# =============================================================================
# Renderização
# =============================================================================

def render_map(
    grid: np.ndarray,
    bounds: MapBounds,
    resolution: float,
    output_path: str,
    title: str = "Mapa discretizado com obstáculos expandidos",
    start: Optional[Tuple[float, float]] = None,
    goal: Optional[Tuple[float, float]] = None,
    path_world: Optional[List[Tuple[float, float]]] = None,
    gvd_cells: Optional[List[Tuple[int, int]]] = None,
):
    """
    Renderiza o mapa em vista superior.

    Esta função será reaproveitada depois pelos planejadores:
    - A*: path_world;
    - GVD: gvd_cells e path_world;
    - RRT: path_world.
    """
    cmap = ListedColormap(
        [
            "white",      # 0 livre
            "black",      # 1 obstáculo original
            "lightgray",  # 2 obstáculo expandido
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

    # Grade de células discretizadas
    x_lines = np.arange(bounds.xmin, bounds.xmax + resolution, resolution)
    y_lines = np.arange(bounds.ymin, bounds.ymax + resolution, resolution)

    for x in x_lines:
        ax.axvline(x, color="gray", linewidth=0.15, alpha=0.35)

    for y in y_lines:
        ax.axhline(y, color="gray", linewidth=0.15, alpha=0.35)

    # Marcações inteiras nos eixos
    ax.set_xticks(np.arange(math.ceil(bounds.xmin), math.floor(bounds.xmax) + 1, 1.0))
    ax.set_yticks(np.arange(math.ceil(bounds.ymin), math.floor(bounds.ymax) + 1, 1.0))

    # GVD, usado depois na questão 2
    if gvd_cells is not None and len(gvd_cells) > 0:
        gvd_x = []
        gvd_y = []

        for row, col in gvd_cells:
            x, y = grid_to_world(row, col, bounds, resolution)
            gvd_x.append(x)
            gvd_y.append(y)

        ax.scatter(gvd_x, gvd_y, s=3, color="blue", label="GVD")

    # Caminho planejado, usado depois nas questões
    if path_world is not None and len(path_world) > 0:
        path_x = [p[0] for p in path_world]
        path_y = [p[1] for p in path_world]

        ax.plot(path_x, path_y, linewidth=2.0, color="orange", label="Caminho planejado")
        ax.scatter(path_x, path_y, s=8, color="orange")

    # Start e goal
    if start is not None:
        ax.plot(
            start[0],
            start[1],
            marker="o",
            markersize=8,
            color="green",
            label="Start",
        )

    if goal is not None:
        ax.plot(
            goal[0],
            goal[1],
            marker="x",
            markersize=9,
            color="red",
            label="Goal",
        )

    if start is not None or goal is not None or path_world is not None or gvd_cells is not None:
        ax.legend(loc="upper right")

    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    plt.tight_layout()
    fig.savefig(output_path, dpi=300)
    plt.close(fig)


# =============================================================================
# Salvamento e carregamento
# =============================================================================

def bounds_to_dict(bounds: MapBounds) -> dict:
    return {
        "xmin": bounds.xmin,
        "xmax": bounds.xmax,
        "ymin": bounds.ymin,
        "ymax": bounds.ymax,
    }


def box_to_dict(obs: BoxObstacle) -> dict:
    return {
        "type": "box",
        "name": obs.name,
        "collision_name": obs.collision_name,
        "x": obs.x,
        "y": obs.y,
        "yaw": obs.yaw,
        "size_x": obs.size_x,
        "size_y": obs.size_y,
        "size_z": obs.size_z,
    }


def cylinder_to_dict(obs: CylinderObstacle) -> dict:
    return {
        "type": "cylinder",
        "name": obs.name,
        "collision_name": obs.collision_name,
        "x": obs.x,
        "y": obs.y,
        "yaw": obs.yaw,
        "radius": obs.radius,
        "length": obs.length,
    }


def save_metadata(
    output_path: str,
    map_name: str,
    sdf_path: str,
    bounds: MapBounds,
    resolution: float,
    inflation_radius: float,
    start: Optional[Tuple[float, float]],
    goal: Optional[Tuple[float, float]],
    box_obstacles: List[BoxObstacle],
    cylinder_obstacles: List[CylinderObstacle],
):
    metadata = {
        "map_name": map_name,
        "source_sdf": sdf_path,
        "bounds": bounds_to_dict(bounds),
        "resolution": resolution,
        "inflation_radius": inflation_radius,
        "start": list(start) if start is not None else None,
        "goal": list(goal) if goal is not None else None,
        "obstacles": {
            "boxes": [box_to_dict(obs) for obs in box_obstacles],
            "cylinders": [cylinder_to_dict(obs) for obs in cylinder_obstacles],
        },
    }

    with open(output_path, "w", encoding="utf-8") as file:
        yaml.safe_dump(metadata, file, sort_keys=False, allow_unicode=True)


def load_metadata(metadata_path: str) -> dict:
    with open(metadata_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def bounds_from_metadata(metadata: dict) -> MapBounds:
    b = metadata["bounds"]

    return MapBounds(
        xmin=float(b["xmin"]),
        xmax=float(b["xmax"]),
        ymin=float(b["ymin"]),
        ymax=float(b["ymax"]),
    )


def load_grid_package(
    map_name: str,
    results_dir: str = "results",
) -> Tuple[np.ndarray, dict, MapBounds]:
    """
    Função auxiliar para os planejadores futuros.

    Carrega:
    - grid expandido;
    - metadata;
    - bounds.
    """
    map_dir = os.path.join(results_dir, map_name)

    grid_path = os.path.join(map_dir, f"{map_name}_grid_inflated.npy")
    metadata_path = os.path.join(map_dir, f"{map_name}_metadata.yaml")

    grid = np.load(grid_path)
    metadata = load_metadata(metadata_path)
    bounds = bounds_from_metadata(metadata)

    return grid, metadata, bounds


# =============================================================================
# Verificações
# =============================================================================

def check_start_goal(
    grid: np.ndarray,
    bounds: MapBounds,
    resolution: float,
    start: Tuple[float, float],
    goal: Tuple[float, float],
):
    start_row, start_col = world_to_grid(start[0], start[1], bounds, resolution)
    goal_row, goal_col = world_to_grid(goal[0], goal[1], bounds, resolution)

    if not is_cell_free(grid, start_row, start_col):
        print("[AVISO] A posição inicial está em célula ocupada ou expandida.")
        print(f"        start = {start} -> row={start_row}, col={start_col}")

    if not is_cell_free(grid, goal_row, goal_col):
        print("[AVISO] A posição objetivo está em célula ocupada ou expandida.")
        print(f"        goal = {goal} -> row={goal_row}, col={goal_col}")


# =============================================================================
# Função principal
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Discretiza um mapa SDF do TP2 em occupancy grid 2D."
    )

    parser.add_argument(
        "--sdf",
        required=True,
        help="Caminho para o arquivo SDF do mapa.",
    )

    parser.add_argument(
        "--map-name",
        required=True,
        help="Nome do mapa. Exemplo: tp2_map1 ou tp2_map2.",
    )

    parser.add_argument(
        "--resolution",
        type=float,
        default=0.10,
        help="Resolução do grid em metros por célula.",
    )

    parser.add_argument(
        "--inflation-radius",
        type=float,
        default=0.60,
        help="Raio de expansão dos obstáculos em metros.",
    )

    parser.add_argument(
        "--start-x",
        type=float,
        default=-8.5,
        help="Coordenada x inicial do robô.",
    )

    parser.add_argument(
        "--start-y",
        type=float,
        default=-8.5,
        help="Coordenada y inicial do robô.",
    )

    parser.add_argument(
        "--goal-x",
        type=float,
        default=8.5,
        help="Coordenada x do objetivo.",
    )

    parser.add_argument(
        "--goal-y",
        type=float,
        default=8.5,
        help="Coordenada y do objetivo.",
    )

    parser.add_argument(
        "--output-dir",
        default="results",
        help="Diretório de saída dos resultados.",
    )

    args = parser.parse_args()

    start = (args.start_x, args.start_y)
    goal = (args.goal_x, args.goal_y)

    map_output_dir = os.path.join(args.output_dir, args.map_name)
    os.makedirs(map_output_dir, exist_ok=True)

    bounds, box_obstacles, cylinder_obstacles = extract_obstacles_from_sdf(args.sdf)

    base_grid, x_centers, y_centers = create_empty_grid(bounds, args.resolution)


    original_grid, obstacle_site_labels = rasterize_obstacles(
        grid=base_grid,
        x_centers=x_centers,
        y_centers=y_centers,
        box_obstacles=box_obstacles,
        cylinder_obstacles=cylinder_obstacles,
    )

    inflated_grid = inflate_obstacles(
        original_grid=original_grid,
        resolution=args.resolution,
        inflation_radius=args.inflation_radius,
    )

    check_start_goal(
        grid=inflated_grid,
        bounds=bounds,
        resolution=args.resolution,
        start=start,
        goal=goal,
    )

    original_grid_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_grid_original.npy",
    )

    inflated_grid_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_grid_inflated.npy",
    )

    metadata_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_metadata.yaml",
    )

    image_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_discretized.png",
    )

    obstacle_site_labels_path = os.path.join(
        map_output_dir,
        f"{args.map_name}_obstacle_site_labels.npy",
    )

    np.save(original_grid_path, original_grid)
    np.save(inflated_grid_path, inflated_grid)
    np.save(obstacle_site_labels_path, obstacle_site_labels)

    save_metadata(
        output_path=metadata_path,
        map_name=args.map_name,
        sdf_path=args.sdf,
        bounds=bounds,
        resolution=args.resolution,
        inflation_radius=args.inflation_radius,
        start=start,
        goal=goal,
        box_obstacles=box_obstacles,
        cylinder_obstacles=cylinder_obstacles,
    )

    render_map(
        grid=inflated_grid,
        bounds=bounds,
        resolution=args.resolution,
        output_path=image_path,
        title=f"{args.map_name}: mapa discretizado com obstáculos expandidos",
        start=start,
        goal=goal,
    )

    print("")
    print("Mapa discretizado com sucesso.")
    print(f"Mapa: {args.map_name}")
    print(f"Arquivo SDF: {args.sdf}")
    print(f"Resolução: {args.resolution:.3f} m/célula")
    print(f"Raio de expansão: {args.inflation_radius:.3f} m")
    print(f"Dimensão do grid: {inflated_grid.shape[1]} colunas x {inflated_grid.shape[0]} linhas")
    print(f"Obstáculos retangulares extraídos: {len(box_obstacles)}")
    print(f"Obstáculos cilíndricos extraídos: {len(cylinder_obstacles)}")
    print("")
    print("Arquivos gerados:")
    print(f"- {original_grid_path}")
    print(f"- {inflated_grid_path}")
    print(f"- {metadata_path}")
    print(f"- {image_path}")
    print("")


if __name__ == "__main__":
    main()