<<'EOF'
# ROS 2 Motion Planning TP2

Repositório desenvolvido para o Trabalho Prático 2 da disciplina de Planejamento de Movimento de Robôs.

O projeto implementa e testa três estratégias de planejamento de movimento para o robô móvel LD90 em ambiente simulado no Gazebo:

- Questão 1: planejamento em grid com A*;
- Questão 2: planejamento baseado em Brushfire e Diagrama de Voronoi Generalizado (GVD);
- Questão 3: planejamento baseado em amostragem com RRT.

Os mapas são definidos em arquivos SDF e executados no Gazebo Sim. A navegação do robô é realizada por um seguidor de caminho que lê os pontos planejados e envia comandos de velocidade ao robô.

---

## Ambiente utilizado

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3
- NumPy
- Matplotlib
- PyYAML

---

## Estrutura principal

```text
src/
├── amr_description/
│   └── meshes/
│       └── LD90.obj
│
└── ld90_gz/
    ├── config/
    │   └── bridge.yaml
    │
    ├── gui/
    │   └── tp2_top_view.config
    │
    ├── launch/
    │   ├── tp2_q1_astar_map1.launch.py
    │   ├── tp2_q1_astar_map2.launch.py
    │   ├── tp2_q2_gvd_map1.launch.py
    │   ├── tp2_q2_gvd_map2.launch.py
    │   ├── tp2_q3_rrt_map1.launch.py
    │   └── tp2_q3_rrt_map2.launch.py
    │
    ├── ld90_gz/
    │   ├── grid_map.py
    │   ├── astar_planner.py
    │   ├── brushfire_gvd.py
    │   ├── rrt_planner.py
    │   └── path_follower.py
    │
    ├── models/
    │   └── ld90_gz.sdf
    │
    └── worlds/
        ├── tp2_map1.sdf
        └── tp2_map2.sdf
