import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict


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
        name,
        value,
        timestamp,
        plot_id,
        curve_id,
        style_id,
    ):

        data = self.data[
            plot_id
        ][
            curve_id
        ][
            style_id
        ]

        data["names"].append(
            name
        )

        data["times"].append(
            timestamp
        )

        data["values"].append(
            value
        )

    # ==========================================================
    # Plot
    # ==========================================================

    def plot(
        self,
    ):

        for plot_id, curves in (
            self.data.items()
        ):

            # --------------------------------------------------
            # Figure
            # --------------------------------------------------

            fig, ax = plt.subplots(
                figsize=(12, 6)
            )

            # Pour calculer l'échelle Y commune
            all_values = []

            # --------------------------------------------------
            # Courbes
            # --------------------------------------------------

            for curve_id, styles in (
                curves.items()
            ):

                for style_id, data in (
                    styles.items()
                ):

                    times = np.array(
                        data["times"]
                    )

                    values = np.array(
                        data["values"]
                    )

                    if len(values) == 0:
                        continue

                    # Sauvegarde pour l'échelle
                    all_values.extend(
                        values
                    )

                    # Couleur
                    color = (
                        self.curve_colors.get(
                            curve_id,
                            None,
                        )
                    )

                    # Style
                    linestyle = (
                        self.styles.get(
                            style_id,
                            "-",
                        )
                    )

                    # Label
                    label = (
                        f"{curve_id} "
                        f"({style_id})"
                    )

                    ax.plot(
                        times,
                        values,
                        color=color,
                        linestyle=linestyle,
                        label=label,
                    )

            # --------------------------------------------------
            # Échelle Y commune
            # --------------------------------------------------

            if len(all_values) > 0:

                y_min = min(
                    all_values
                )

                y_max = max(
                    all_values
                )

                # Marge
                margin = (
                    0.05
                    * (y_max - y_min)
                )

                # Cas valeur constante
                if margin == 0:

                    margin = 1.0

                ax.set_ylim(
                    y_min - margin,
                    y_max + margin,
                )

            # --------------------------------------------------
            # Configuration graphique
            # --------------------------------------------------

            title = (
                self.plot_titles.get(
                    plot_id,
                    plot_id,
                )
            )

            ylabel = (
                self.plot_ylabels.get(
                    plot_id,
                    "",
                )
            )

            ax.set_title(
                title
            )

            ax.set_xlabel(
                "Temps (s)"
            )

            ax.set_ylabel(
                ylabel
            )

            ax.grid(
                True
            )

            ax.legend()

            plt.tight_layout()

            plt.show()