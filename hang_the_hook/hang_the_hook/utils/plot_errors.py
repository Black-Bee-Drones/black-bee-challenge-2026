#!/usr/bin/env python3
import sys
import os
import pandas as pd
import matplotlib.pyplot as plt


def plot_log(csv_path: str = None):
    """
    Reads the error log CSV file containing position error (cx), angle error, and timestamp,
    and plots them on a 2-panel chart.
    """
    if csv_path is None:
        # Default path: hang_the_hook/utils/errors/error_log.csv
        base_dir = os.path.abspath(os.path.dirname(__file__))
        csv_path = os.path.join(base_dir, "errors", "error_log.csv")

    if not os.path.exists(csv_path):
        print(f"[ERROR] Arquivo de log não encontrado em: {csv_path}")
        return

    print(f"Lendo dados de log de: {csv_path}")

    try:
        # Load CSV (handles optional header or no header)
        df = pd.read_csv(csv_path, skipinitialspace=True)
        if list(df.columns) != ["cx_error", "angle_error", "time"]:
            # If no header was present, reload specifying header names
            df = pd.read_csv(csv_path, names=["cx_error", "angle_error", "time"], skipinitialspace=True)
    except Exception as e:
        print(f"[ERROR] Falha ao ler o arquivo CSV: {e}")
        return

    if df.empty:
        print("[WARN] O arquivo CSV está vazio. Nenhum dado para plotar.")
        return

    # Create figure with 2 subplots (stacked vertically)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Plot 1: Position Error (CX)
    ax1.plot(df["time"], df["cx_error"], label="Erro de Posição (CX)", color="blue", linewidth=1.5, marker="o", markersize=2)
    ax1.set_ylabel("Erro CX (pixels)")
    ax1.set_title("Desempenho dos Erros do Controlador PID - FollowLine")
    ax1.grid(True, linestyle="--", alpha=0.7)
    ax1.legend(loc="upper right")

    # Plot 2: Angle Error
    ax2.plot(df["time"], df["angle_error"], label="Erro de Ângulo (Graus)", color="red", linewidth=1.5, marker="s", markersize=2)
    ax2.set_xlabel("Tempo (MM:SS)")
    ax2.set_ylabel("Erro de Ângulo")
    ax2.grid(True, linestyle="--", alpha=0.7)
    ax2.legend(loc="upper right")

    # Rotate x-axis timestamp labels for better legibility
    plt.xticks(rotation=45)
    plt.tight_layout()

    # Save plot output
    output_img = os.path.join(os.path.dirname(csv_path), "error_plot.png")
    plt.savefig(output_img, dpi=300)
    print(f"[INFO] Gráfico salvo com sucesso em: {output_img}")

    # Display window
    plt.show()


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else None
    plot_log(path)


if __name__ == "__main__":
    main()
