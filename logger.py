import csv
import datetime
import matplotlib.pyplot as plt


class DataLogger:
    def __init__(self):
        self.data = {} 

    def append(self, label, timestamp, value):
        if label not in self.data:
            self.data[label] = []
        self.data[label].append((timestamp, value))

    def save_and_plot(self):
        self.save()
        self.plot()

    def save(self):
        labels = list(self.data.keys())

        if not labels:
            print("Aucune donnée à sauvegarder.")
            return

        max_length = max( len(self.data[label]) for label in labels )
        
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filepath = f"log/logs_{timestamp}.csv"

        with open( filepath, "w", newline="", encoding="utf-8") as file:

            writer = csv.writer(file)
            header = []
            for label in labels:
                header.append(f"{label}_t")
                header.append(label)
            writer.writerow(header)

            for i in range(max_length):
                row = []
                for label in labels:
                    if i < len(self.data[label]):
                        row.append(self.data[label][0])
                        row.append(self.data[label][1])
                    else:
                        row.append("")
                        row.append("")

                writer.writerow(row)

        print(
            f"Données sauvegardées dans : "
            f"{filepath}"
        )


    def plot(self):

        if not self.data:
            print("Aucune donnée à afficher.")
            return

        labels = list(self.data.keys())

        fig, ax = plt.subplots()

        # first axis
        axes = [ax]

        colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

        for i, label in enumerate(labels):

            timestamps = [p[0] for p in self.data[label]]
            values = [p[1] for p in self.data[label]]

            if i == 0:
                current_ax = ax
            else:
                current_ax = ax.twinx()
                # Décalage des axes Y supplémentaires
                current_ax.spines["right"].set_position( ("outward", 60 * (i - 1)) )
                axes.append(current_ax)

            color = colors[i % len(colors)]

            current_ax.plot(timestamps, values, label=label, color=color)

            current_ax.set_ylabel( label, color=color)

            current_ax.tick_params( axis="y", colors=color)

            current_ax.relim()
            current_ax.autoscale_view()

        ax.set_xlabel("Time (s)")

        ax.set_title("Data acquisition")

        ax.grid(True)

        lines = []
        labels_legend = []

        for current_ax in axes:
            line_list, label_list = ( current_ax.get_legend_handles_labels() )
            lines.extend(line_list)
            labels_legend.extend(label_list)

        ax.legend(lines, labels_legend, loc="upper left")

        fig.tight_layout()

        plt.show()
