#!/usr/bin/env python3
"""
plot_global_path_smoother.py
==============================
Visualización 3D comparativa: waypoints iniciales vs path suavizado
producida por global_path_smoother_node.

Modos de uso
------------
(A) Auto — lanza el nodo, captura su salida y plotea:
        python3 scripts/plot_global_path_smoother.py

(B) Desde fichero — si ya tienes la salida guardada:
        roslaunch heuristic_planners global_path_smoother.launch > /tmp/opt.txt 2>&1
        python3 scripts/plot_global_path_smoother.py --output /tmp/opt.txt

Opciones adicionales
--------------------
--yaml   <ruta>   YAML de waypoints (por defecto: config/global_path_smoother_waypoints.yaml)
--save   <ruta>   Guardar figura en fichero PNG en lugar de mostrarla en ventana
--no-lines        No dibujar líneas entre waypoints iniciales (solo puntos)
"""

import argparse
import os
import re
import subprocess
import sys

import yaml

try:
    import matplotlib
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D          # noqa: F401 – registro 3D
except ImportError:
    sys.exit("[plot] ERROR: matplotlib no instalado.  pip install matplotlib")


# ─────────────────────────────────────────────────────────────────────────────
# Utilidades
# ─────────────────────────────────────────────────────────────────────────────
_ANSI = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')

def _strip_ansi(text: str) -> str:
    return _ANSI.sub('', text)


# ─────────────────────────────────────────────────────────────────────────────
# 1. Lectura de waypoints iniciales desde YAML
# ─────────────────────────────────────────────────────────────────────────────
def load_yaml_waypoints(yaml_path: str):
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    wps = data['waypoints']
    return (
        [float(w['x']) for w in wps],
        [float(w['y']) for w in wps],
        [float(w['z']) for w in wps],
    )


# ─────────────────────────────────────────────────────────────────────────────
# 2. Parseo de la sección RESULTADO 2 de la salida del nodo
# ─────────────────────────────────────────────────────────────────────────────
_WP_RE = re.compile(
    r'\[([+-]?\d+(?:\.\d+)?),\s*([+-]?\d+(?:\.\d+)?),\s*([+-]?\d+(?:\.\d+)?)\]'
)

def parse_optimized_waypoints(raw: str):
    text = _strip_ansi(raw)

    # Localizar sección RESULTADO 2
    idx = text.find('RESULTADO 2')
    if idx < 0:
        sys.exit("[plot] ERROR: 'RESULTADO 2' no encontrado en la salida del nodo.\n"
                 "       Asegúrate de que el nodo compiló con los cambios más recientes.")

    tail = text[idx:]
    matches = _WP_RE.findall(tail)
    if not matches:
        sys.exit("[plot] ERROR: No se encontraron waypoints en formato [x, y, z] "
                 "tras la cabecera RESULTADO 2.")

    xs = [float(m[0]) for m in matches]
    ys = [float(m[1]) for m in matches]
    zs = [float(m[2]) for m in matches]
    return xs, ys, zs


# ─────────────────────────────────────────────────────────────────────────────
# 3. Ejecución de roslaunch con captura de stdout+stderr
# ─────────────────────────────────────────────────────────────────────────────
def run_roslaunch() -> str:
    print("[plot] Ejecutando: roslaunch heuristic_planners global_path_smoother.launch")
    try:
        proc = subprocess.run(
            ['roslaunch', 'heuristic_planners', 'global_path_smoother.launch'],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,   # mezclar stderr en stdout para no perder nada
            timeout=90,
            text=True,
        )
        return proc.stdout
    except subprocess.TimeoutExpired:
        sys.exit("[plot] ERROR: roslaunch superó el timeout de 90 s.")
    except FileNotFoundError:
        sys.exit("[plot] ERROR: 'roslaunch' no encontrado.\n"
                 "       Ejecuta primero: source devel/setup.bash")


# ─────────────────────────────────────────────────────────────────────────────
# 4. Plot 3D
# ─────────────────────────────────────────────────────────────────────────────
def plot_3d(init, opt, save_path=None, draw_init_lines=True):
    ix, iy, iz = init
    ox, oy, oz = opt

    fig = plt.figure(figsize=(14, 9))
    ax  = fig.add_subplot(111, projection='3d')

    # ── Waypoints iniciales ─────────────────────────────────────────────────
    ax.scatter(ix, iy, iz,
               c='tomato', s=30, depthshade=True, zorder=4,
               label=f'WPs iniciales  ({len(ix)} puntos)')
    if draw_init_lines:
        ax.plot(ix, iy, iz,
                color='tomato', lw=1.0, alpha=0.45, linestyle='--')

    # ── Trayectoria optimizada ──────────────────────────────────────────────
    ax.plot(ox, oy, oz,
            color='royalblue', lw=2.0,
            label=f'Path suavizado ({len(ox)} puntos)')

    # Start y Goal
    ax.scatter([ix[0]],  [iy[0]],  [iz[0]],
               c='limegreen', s=100, marker='^', zorder=6,
               label=f'Start  [{int(ix[0])}, {int(iy[0])}, {int(iz[0])}]')
    ax.scatter([ix[-1]], [iy[-1]], [iz[-1]],
               c='darkorange', s=100, marker='s', zorder=6,
               label=f'Goal   [{int(ix[-1])}, {int(iy[-1])}, {int(iz[-1])}]')

    # ── Decoración ─────────────────────────────────────────────────────────
    ax.set_xlabel('X  (casillas)', labelpad=8)
    ax.set_ylabel('Y  (casillas)', labelpad=8)
    ax.set_zlabel('Z  (casillas)', labelpad=8)
    ax.set_title('Global Path Smoother\nWPs iniciales  vs  path suavizado',
                 pad=12)
    ax.legend(loc='upper left', fontsize=9)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[plot] Figura guardada en: {save_path}")
    else:
        plt.show()


# ─────────────────────────────────────────────────────────────────────────────
# main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description='Compara waypoints iniciales y path suavizado en 3D.')
    ap.add_argument('--yaml', default=None,
                    help='Ruta al YAML de waypoints '
                         '(por defecto: config/global_path_smoother_waypoints.yaml)')
    ap.add_argument('--output', default=None,
                    help='Fichero con la salida capturada del nodo '
                         '(si se omite, ejecuta roslaunch automáticamente)')
    ap.add_argument('--save', default=None, metavar='IMAGEN.png',
                    help='Guardar la figura en este fichero en lugar de mostrarla')
    ap.add_argument('--no-lines', action='store_true',
                    help='No dibujar líneas entre waypoints iniciales')
    args = ap.parse_args()

    # ── Localizar YAML ───────────────────────────────────────────────────────
    if args.yaml:
        yaml_path = os.path.abspath(args.yaml)
    else:
        here = os.path.dirname(os.path.abspath(__file__))
        yaml_path = os.path.normpath(
            os.path.join(here, '..', 'config',
                         'global_path_smoother_waypoints.yaml'))

    if not os.path.isfile(yaml_path):
        sys.exit(f"[plot] ERROR: YAML no encontrado: {yaml_path}")

    # ── Cargar WPs iniciales ─────────────────────────────────────────────────
    init = load_yaml_waypoints(yaml_path)
    print(f"[plot] WPs iniciales cargados: {len(init[0])}")

    # ── Obtener salida del optimizador ───────────────────────────────────────
    if args.output:
        with open(args.output) as f:
            raw = f.read()
    else:
        raw = run_roslaunch()

    # ── Parsear WPs suavizados ──────────────────────────────────────────────
    opt = parse_optimized_waypoints(raw)
    print(f"[plot] WPs suavizados parseados: {len(opt[0])}")

    # ── Plot ─────────────────────────────────────────────────────────────────
    plot_3d(init, opt,
            save_path=args.save,
            draw_init_lines=not args.no_lines)


if __name__ == '__main__':
    main()
