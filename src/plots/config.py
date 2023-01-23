"""
Write your plot configuration script here
Below you can see just an example about how to write a config file.
You can import constants, lists and dictionaries in plot_data.py
"""

# *** EXAMPLE ***
import numpy as np
import matplotlib.pyplot as plt

LABEL_SIZE = 22
LEGEND_SIZE = 20
TITLE_SIZE = 26
TICKS_SIZE = 20
OTHER_SIZES = 20

METRICS_OF_INTEREST = [
    "number_of_packets_to_depot",
    "packet_mean_delivery_time",
    "mean_number_of_relays"]

SCALE_LIM_DICT = {
    "number_of_packets_to_depot": {
        "scale": "linear",
        "ylim": (0, 1000)
    },
    "packet_mean_delivery_time": {
        "scale": "linear",
        "ylim": (0, 5)
    },
    "mean_number_of_relays": {
        "scale": "linear",
        "ylim": (0, 10)
    }
}

PLOT_DICT = {
    "RND": {
        "hatch": "",
        "markers": "X",
        "linestyle": "-",
        "color": plt.cm.tab10(0),
        "label": "RND",
        "x_ticks_positions": np.array(np.linspace(0, 8, 5))
    },
    "GEO": {
        "hatch": "",
        "markers": "p",
        "linestyle": "-",
        "color": plt.cm.tab10(1),
        "label": "GEO",
        "x_ticks_positions": np.array(np.linspace(0, 8, 5))

    },
    "QL_new": {
        "hatch": "",
        "markers": "s",
        "linestyle": "-",
        "color": plt.cm.tab10(5),
        "label": "QL_new",
        "x_ticks_positions": np.array(np.linspace(0, 8, 5))

    },
    "QL_old": {
        "hatch": "",
        "markers": "s",
        "linestyle": "-",
        "color": plt.cm.tab10(8),
        "label": "QL_old",
        "x_ticks_positions": np.array(np.linspace(0, 8, 5))

    }
}

# *** EXAMPLE ***


# TODO: Implement your code HERE