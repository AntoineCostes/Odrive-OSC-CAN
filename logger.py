import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
import os
from datetime import datetime
from PIL import Image

class Logger:

    def __init__(self):

        # Données enregistrées
        #
        # plot_id
        #     curve_id
        #         style_id
        #             name
        #                 times
        #                 values
        #
        self.data = defaultdict(
            lambda: defaultdict(
                lambda: defaultdict(
                    lambda: {
                        "names": [],
                        "times": [],
                        "values": [],
                    }
                )
            )
        )

        # Configuration visuelle
        self.curve_colors = {}
        self.styles = {}

        # Titres des graphiques
        self.plot_titles = {}

        # Labels des axes
        self.plot_ylabels = {}

        self.set_curve_color(
            "motor_A",
            "tab:blue",
        )

        self.set_curve_color(
            "motor_B",
            "tab:orange",
        )

        self.set_curve_color(
            "motor_C",
            "tab:green",
        )

        self.set_style(
            "actual",
            "-",
        )

        self.set_style(
            "target",
            "--",
        )
        self.set_plot_config(
            "position",
            title="Positions",
            ylabel="Position",
        )

        self.set_plot_config(
            "velocity",
            title="Vitesses",
            ylabel="Vitesse",
        )

    # ==========================================================
    # Configuration
    # ==========================================================

    def set_curve_color(
        self,
        curve_id,
        color,
    ):

        self.curve_colors[
            curve_id
        ] = color

    def set_style(
        self,
        style_id,
        linestyle,
    ):

        self.styles[
            style_id
        ] = linestyle

    def set_plot_config(
        self,
        plot_id,
        title=None,
        ylabel=None,
    ):

        self.plot_titles[
            plot_id
        ] = title

        self.plot_ylabels[
            plot_id
        ] = ylabel

    # ==========================================================
    # Enregistrement
    # ==========================================================

    def record(
        self,
        value,
        timestamp,
        plot_id,
        curve_id,
        style_id,
        label,
    ):

        key = (curve_id, style_id)

        if plot_id not in self.data:
            self.data[plot_id] = {}

        if key not in self.data[plot_id]:
            self.data[plot_id][key] = {
                "times": [],
                "values": [],
                "label": label,
            }

        self.data[plot_id][key]["times"].append(
            timestamp
        )

        self.data[plot_id][key]["values"].append(
            value
        )


    def plot(self):

        for plot_id, curves in self.data.items():

            fig, ax = plt.subplots(
                figsize=(12, 6)
            )

            all_values = []

            for key, data in curves.items():

                curve_id, style_id = key

                times = np.array(
                    data["times"]
                )

                values = np.array(
                    data["values"]
                )

                if len(values) == 0:
                    continue

                all_values.extend(
                    values
                )

                color = self.curve_colors.get(
                    curve_id
                )

                linestyle = self.styles.get(
                    style_id,
                    "-"
                )

                ax.plot(
                    times,
                    values,
                    color=color,
                    linestyle=linestyle,
                    label=data["label"],
                )

            if all_values:

                y_min = min(all_values)
                y_max = max(all_values)

                margin = 0.05 * (
                    y_max - y_min
                )

                if margin == 0:
                    margin = 1.0

                ax.set_ylim(
                    y_min - margin,
                    y_max + margin,
                )

            ax.set_title(
                self.plot_titles.get(
                    plot_id,
                    plot_id,
                )
            )

            ax.set_xlabel(
                "Temps (s)"
            )

            ax.set_ylabel(
                self.plot_ylabels.get(
                    plot_id,
                    "",
                )
            )

            ax.grid(True)
            ax.legend()

            fig.tight_layout()

        plt.show(block=False)
        plt.pause( 0.1)
        
    def save_plots(self, directory="plots"):


        os.makedirs(
            directory,
            exist_ok=True,
        )

        for plot_id, curves in self.data.items():

            fig, ax = plt.subplots(
                figsize=(12, 6)
            )

            all_values = []

            for key, data in curves.items():

                curve_id, style_id = key

                times = np.asarray(
                    data["times"],
                    dtype=float,
                )

                values = np.asarray(
                    data["values"],
                    dtype=float,
                )

                if len(times) == 0:
                    continue

                color = self.curve_colors.get(
                    curve_id,
                    None,
                )

                linestyle = self.styles.get(
                    style_id,
                    "-",
                )

                ax.plot(
                    times,
                    values,
                    color=color,
                    linestyle=linestyle,
                    label=data["label"],
                )

                all_values.extend(
                    values
                )

            if all_values:

                y_min = min(all_values)
                y_max = max(all_values)

                margin = (
                    0.05
                    * (y_max - y_min)
                )

                if margin == 0:
                    margin = 1.0

                ax.set_ylim(
                    y_min - margin,
                    y_max + margin,
                )

            ax.set_title(
                self.plot_titles.get(
                    plot_id,
                    plot_id,
                )
            )

            ax.set_xlabel(
                "Temps (s)"
            )

            ax.set_ylabel(
                self.plot_ylabels.get(
                    plot_id,
                    "",
                )
            )

            ax.grid(True)
            ax.legend()

            fig.tight_layout()

            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = os.path.join(
                directory,
                f"{plot_id}_"+f"{timestamp}.png",
            )

            fig.savefig(
                filename,
                dpi=150,
                bbox_inches="tight",
            )

            plt.close(fig)

            print(
                f"Plot saved: {filename}"
            )
            img = Image.open(filename)
            img.show()
