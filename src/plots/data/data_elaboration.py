"""
You can write here the data elaboration function/s

You should read all the JSON files containing simulations results and compute
average and std of all the metrics of interest.

You can find the JSON file from the simulations into the data.evaluation_tests folder.
Each JSON file follows the naming convention: simulation-current date-simulation id__seed_drones number_routing algorithm

In this way you can parse the name and properly aggregate the data.

To aggregate data you can use also external libraries such as Pandas!

IMPORTANT: Both averages and stds must be computed over different seeds for the same metric!
"""

import matplotlib.pyplot as plt
import json
import os

tests = dict()

def compute_data_avg_std(path: str, algo: str, metric:str):
    """
    Computes averages and stds from JSON files
    @param path: results folder path
    @return: one or more data structure containing data
    """
    
    os.chdir( path )
    print(os.getcwd())

    for file in os.listdir():
        
        if file.endswith(".json"):
            file_path = f"{file}"

            read_text_file(file_path)

    outs = dict()

    for i in range(5,25,5):
        print(i)
        outs[i] = dict()

    for l in tests:
        n_drones = tests[l]["mission_setup"]["n_drones"]
        algo_ = tests[l]["mission_setup"]["routing_algorithm"]  

        if n_drones not in outs:
            outs[ n_drones ] = dict()

        if algo_ not in outs[ n_drones ]:
            if algo_ == "RoutingAlgorithm.QL_new":
                outs[ n_drones ][ algo_ ] = { "mean":0, "ratio":0, "mean_number_of_relays":0, "all_discovery_packets_generated_in_simulation":0, "all_neighbor_table_packets_in_simulation":0, "tests":0 }
            else:
                outs[ n_drones ][ algo_ ] = { "mean":0, "ratio":0, "mean_number_of_relays":0, "tests":0 }

        outs[ n_drones ][ algo_ ]["mean"] += tests[l]["packet_mean_delivery_time"]
        outs[ n_drones ][ algo_ ]["ratio"] += tests[l]["packet_delivery_ratio"]
        outs[ n_drones ][ algo_ ]["mean_number_of_relays"] += tests[l]["mean_number_of_relays"]
        outs[ n_drones ][ algo_ ]["tests"] += 1

        if algo_ == "RoutingAlgorithm.QL_new":
            outs[ n_drones ][ algo_ ]["all_discovery_packets_generated_in_simulation"] += tests[l]["all_discovery_packets_generated_in_simulation"]
            outs[ n_drones ][ algo_ ]["all_neighbor_table_packets_in_simulation"] += tests[l]["all_neighbor_table_packets_in_simulation"]

    return [ outs[ i ][algo][metric] / outs[ i ][ algo ]["tests"] for i in range(5,25,5) ]

def read_text_file(file_path):
        with open(file_path, 'r') as f:
            text = f.read()
            tests[file_path] = json.loads(text)
            used_algo = tests[file_path]["mission_setup"]["routing_algorithm"]

            if "QL" in used_algo:
                if "QL_old" in file_path:
                    algo_ = used_algo + "_old"
                else:
                    algo_ = used_algo + "_new"

                tests[file_path]["mission_setup"]["routing_algorithm"] = algo_

if __name__ == "__main__":
    """
    You can run this file to test your script
    """

    path = "data/evaluation_tests"

    algo = "RoutingAlgorithm." + "QL_new" 

    # mean                  
    # ratio                 
    # mean_number_of_relays 
    # all_discovery_packets_generated_in_simulation
    # all_neighbor_table_packets_in_simulation

    metric = "all_neighbor_table_packets_in_simulation"

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
    # all_neighbor_table_packets_in_simulation

    print(compute_data_avg_std(path=path, algo=algo, metric=metric))
