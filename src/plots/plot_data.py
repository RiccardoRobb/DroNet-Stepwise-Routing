
import matplotlib.pyplot as plt
import numpy as np
from src.experiments.json_and_plot import ALL_SIZE
from src.plots.config import PLOT_DICT, LABEL_SIZE, LEGEND_SIZE

from src.plots.data.data_elaboration import compute_data_avg_std

import os

def plot( lines: list, type: str ):
    """
    This method has the ONLY responsibility to plot data
    @param y_data_std:
    @param y_data:
    @param algorithm:
    @param type:
    @return:
    """

    fig, ax1 = plt.subplots(nrows=1, ncols=1, figsize=(8.5, 6.5))

    for line in lines:

        print(f"Algorithm: {line[2]}")

        print(f"y_data: {line[0]}\ny_data_std: {line[1]}")

        ax1.errorbar(x=[i for i in range(5,25,5)],
                    y=line[0],
                    yerr=line[1],
                    label=PLOT_DICT[line[2]]["label"],
                    marker=PLOT_DICT[line[2]]["markers"],
                    linestyle=PLOT_DICT[line[2]]["linestyle"],
                    color=PLOT_DICT[line[2]]["color"],
                    markersize=8)

    ax1.set_ylabel(ylabel="Mean number of relays", fontsize=LABEL_SIZE)
    ax1.set_xlabel(xlabel="UAVs", fontsize=LABEL_SIZE)
    ax1.tick_params(axis='both', which='major', labelsize=ALL_SIZE)

    plt.legend(ncol=1,
               handletextpad=0.1,
               columnspacing=0.7,
               prop={'size': LEGEND_SIZE})

    
    plt.grid(linewidth=0.3)
    plt.tight_layout()
    plt.savefig("../../src/plots/figures/" + type + ".svg")
    plt.savefig("../../src/plots/figures/" + type + ".png", dpi=400)
    plt.clf()

if __name__ == "__main__":
    # mean                  
    # ratio                 
    # mean_number_of_relays 
    # all_discovery_packets_generated_in_simulation
    # all_neighbor_table_packets_in_simulation

    # RND mean_number_of_relays [1.0990712755187326, 1.5137287726553947, 1.9478723168186922, 2.2585042870798113]
    # GEO mean_number_of_relays [1.0783672031823257, 1.3119848736276725, 1.6457435038766979, 1.7616448457066107]
    # QL_old mean_number_of_rel [1.0940555517735129, 1.4469598774044345, 1.8700420265386426, 2.089685821427048]
    # QL_new mean_number_of_rel [1.097544348420853, 1.5043177592420696, 1.9347018495945822, 2.236364970552375]
    #
    # RND mean_time [117.5975216368416, 114.89414021353042, 112.59323338278014, 106.90253652040786]
    # GEO mean_time [126.17777831494095, 112.91273752590263, 106.4448881366316, 96.39836119466283]
    # QL_old mean_t [116.69001127073064, 114.87224578402206, 116.7053076760415, 115.55377993054009]
    # QL_new mean_t [116.04053754623799, 115.90259152256729, 111.74903137127552, 107.72488646893571]
    #
    # RND ratio [0.5747292418772562, 0.6051744885679903, 0.5762936221419976, 0.5524669073405536]
    # GEO ratio [0.7530685920577618, 0.8274368231046935, 0.9030084235860405, 0.9312876052948253]
    # QL_old ra [0.5959085439229844, 0.6812274368231047, 0.7547533092659445, 0.7829121540312877]
    # QL_new ra [0.5558363417569194, 0.6037304452466908, 0.5687123947051745, 0.553309265944645]
    #
    # all_discovery_packets_generated_in_simulation [277.0, 277.0, 277.0, 277.0]
    # all_neighbor_table_packets_in_simulation [20.0, 90.0, 292.0, 513.0]

    lines = list()

    os.chdir("data/evaluation_tests")

    metric = "mean_number_of_relays"

    routing = {
        "RND" : [1.0990712755187326, 1.5137287726553947, 1.9478723168186922, 2.2585042870798113],
        "GEO" : [1.0783672031823257, 1.3119848736276725, 1.6457435038766979, 1.7616448457066107],
        "QL_old" : [1.0940555517735129, 1.4469598774044345, 1.8700420265386426, 2.089685821427048],
        "QL_new" : [1.097544348420853, 1.5043177592420696, 1.9347018495945822, 2.236364970552375]
    }

    #for rt in "GEO", "RND", "QL":
    for rt in routing:
        #y_data = compute_data_avg_std( path="data/evaluation_tests", algo="RoutingAlgorithm."+rt, metric=metric )

        y_data = routing[ rt ]
        y_data_std = np.std( y_data )

        lines.append( (y_data, y_data_std, rt) )


    type = "AllFor_mean"

    plot(lines=lines, type=type+metric)