from src.routing_algorithms.BASE_routing import BASE_routing
from src.routing_algorithms.random_routing import RandomRouting as RND
from src.utilities import utilities as util
import math, random

LEARNING_RATE = 0.77
DISCOUNT_FACTOR = 0.85

# is a coefficient weight value used to calculate the link stability
WEIGHT_VALUE = 1
# is the tuning parameter used by the WMEWMA algorithm
LINK_QUALITY_ALPHA = 0.7
# is the weighting factor that balances the bias between the link stability and the number of hops
WEIGHTING_FACTOR = 0.5

PROBABILITY_OF_EXPLORATION = 0.4

# the maximum value of the reward
MAX_VALUE = 1
# the minimum value of the reward
MIN_VALUE = -1

class QLearningRouting(BASE_routing):

    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, entity=drone, simulator=simulator)
        self.taken_actions = {}  # id event : (old_state, old_action)

        # state: the current drone
        # actions: all drones in the simulation
        self.qtable = [0 for _ in range(self.simulator.n_drones)]

        # this structure is used to store the delivery ratio of each drone,
        # the ratio will be used for calculate the link quality
        self.delivery_ratio = {
            drone: {
                "all_packets": 0,
                "packets_to_depot": 0
            } for drone in range(self.simulator.n_drones)
        }

        # link_quality associates a list of values with a specific drone,
        # this values 
        self.link_quality = dict()

        random.seed(self.simulator.seed)
        self.random_routing = RND(self.entity, self.simulator)

    def feedback(self, drone, id_event, delay, outcome):
        """
        Feedback returned when the packet arrives at the depot or
        Expire. This function have to be implemented in RL-based protocols ONLY
        @param drone: The drone that holds the packet
        @param id_event: The Event id
        @param delay: packet delay
        @param outcome: -1 or 1 (read below)
        @return:
        """

        if id_event in self.taken_actions:
            state = self.taken_actions[id_event]["state"]
            action = self.taken_actions[id_event]["action"]
            num_of_neighbors = self.taken_actions[id_event]["len_opt_neighbors"]
            action_qtable = self.taken_actions[id_event]["current_state"]
            relay_speed = self.taken_actions[id_event]["relay_speed"]
            relay_coords = self.taken_actions[id_event]["relay_coords"]

            #print(f"[QL] Sono il drone {self.entity} - {state} ho inviato a {action} - outcome: {outcome} - drone: {drone}\nneighbor table: {self.entity.neighbor_table}\nqtable: {self.qtable}")

            self.delivery_ratio[action.identifier]["all_packets"] += 1    
            if outcome == 1:
                self.delivery_ratio[action.identifier]["packets_to_depot"] += 1

            #if action == drone:
            
            link_stability = None
            if action in state.neighbor_table:
                delivery_ratio = self.delivery_ratio[action.identifier]["packets_to_depot"] / self.delivery_ratio[action.identifier]["all_packets"]

                #print(f"[INFO] Il mio delivery ratio verso il drone {action} -> {delivery_ratio} - il dr varrà {(1 - LINK_QUALITY_ALPHA) * delivery_ratio}")

                src_drone_speed = state.speed
                dst_drone_speed = relay_speed
                speed_move_away = min(src_drone_speed, dst_drone_speed) / max(src_drone_speed, dst_drone_speed)

                #print(f"[INFO] La mia speed_move_away verso il drone {action} -> {speed_move_away} - nel ls varrà {(1 - WEIGHT_VALUE) * math.exp(1 / speed_move_away)}")

                if action.identifier not in self.link_quality:
                    self.link_quality[action.identifier] = [(1 - LINK_QUALITY_ALPHA) * delivery_ratio]
                else:
                    self.link_quality[action.identifier].append((self.link_quality[action.identifier][-1] * LINK_QUALITY_ALPHA) + ((1 - LINK_QUALITY_ALPHA) * delivery_ratio))

                #print(f"[INFO] Link quality settato nei confronti del drone {action} -> {self.link_quality[action.identifier][-1]} - il lq precedente varrà {self.link_quality[action.identifier][-1] * LINK_QUALITY_ALPHA}")

                truncated_link_quality = self.link_quality[action.identifier].copy()[:-1]
                if len(truncated_link_quality) > num_of_neighbors:
                    truncated_link_quality = truncated_link_quality[-num_of_neighbors:]

                #print(f"[INFO] Sum di truncated link quality varrà {sum(truncated_link_quality)} - IL valore varrà {WEIGHT_VALUE * ((sum(truncated_link_quality)) / (num_of_neighbors))}")

                link_stability = (1 - WEIGHT_VALUE) * math.exp(1 / speed_move_away) + \
                    WEIGHT_VALUE * ((sum(truncated_link_quality)) / (num_of_neighbors))

            reward = 0
            if outcome == 1 and action == drone:
                reward = MAX_VALUE
                #print("[INFO] Reward 1 caso")
            else:
                if util.euclidean_distance(self.simulator.depot.coords, state.coords) < util.euclidean_distance(self.simulator.depot.coords, relay_coords):
                    reward = MIN_VALUE
                    #print("[INFO] Reward 2 caso")
                elif link_stability is not None:
                    hops_count = [state.neighbor_table[drone]["hop_count_from_CC"] for drone in state.neighbor_table]

                    if hops_count == []:
                        reward = (1 - WEIGHTING_FACTOR) * link_stability
                        #print("[INFO] Reward 3 caso")
                    else:
                        reward = WEIGHTING_FACTOR * math.exp(1 / min(hops_count)) + (1 - WEIGHTING_FACTOR) * link_stability
                        #print(f"[INFO] Reward 4 caso - {WEIGHTING_FACTOR * math.exp(1 / min(hops_count))} + {(1 - WEIGHTING_FACTOR) * link_stability}")
                else:
                    reward = MAX_VALUE

                    #print(f"[INFO] Reward 5 caso - reward: {reward}")

            #print(f"[INFO] Reward: {reward}")

            #self.qtable[action.identifier] = (1 - LEARNING_RATE) * self.qtable[action.identifier] + \
            #    LEARNING_RATE * (reward + (DISCOUNT_FACTOR * max(action_qtable)))

            self.qtable[action.identifier] = (1 - LEARNING_RATE) * self.qtable[action.identifier] + \
                LEARNING_RATE * (reward + (DISCOUNT_FACTOR * max(action_qtable)))

            #print(f"[OUTCOME] QTable: {self.qtable}")

            del self.taken_actions[id_event]

            #input()

    def relay_selection(self, opt_neighbors: list, packet):
        """
        This function returns the best relay to send packets.
        @param packet:
        @param opt_neighbors: a list of tuple (hello_packet, source_drone)
        @return: The best drone to use as relay
        """
        relay = self.entity

        # info structure used to store many information about the
        # current drone and the neighbor drones
        #  
        info = {
            drone: {
                "coords": pck.cur_pos,
                "speed": pck.speed,
                "relay_qtable": pck.optional_data["relay_qtable"]
            } for pck, drone in opt_neighbors
        }

        info[self.entity] = {
            "coords": self.entity.coords,
            "speed": self.entity.speed,
            "relay_qtable": self.qtable
        }

        probability = random.random()
        # do exploration
        if probability < PROBABILITY_OF_EXPLORATION:
            # removes from opt_neighbors all nodes for which knowledge already exists 
            temp_opt_neighbors = [pck for pck in opt_neighbors if self.qtable[pck[1].identifier] == 0]
            if len(temp_opt_neighbors) != 0:
                opt_neighbors = temp_opt_neighbors

            # do the random routing
            relay = self.random_routing.relay_selection(opt_neighbors, packet)
        else:
            # values structure used for associate the value 
            values = { self.qtable[self.entity.identifier]: self.entity }
            for _, drone in opt_neighbors:
                values[self.qtable[drone.identifier]] = drone

            # choose the drone with the maximum value
            relay = values[max(values)]

        state, action = self.entity, relay

        """
        if packet.event_ref.identifier in self.taken_actions:
            path = self.taken_actions[packet.event_ref.identifier]["path"]
        else:
            path = [state]
        path.append(relay)
        """

        self.taken_actions[packet.event_ref.identifier] = {
            "state": state,
            "action": action,
            "len_opt_neighbors": len(opt_neighbors),
            "current_state": info[relay]["relay_qtable"],
            "relay_speed": info[relay]["speed"],
            "relay_coords": info[relay]["coords"],
        }

        return relay  