from src.entities.uav_entities import DataPacket, ACKPacket, HelloPacket, Packet, DiscoveryPacket, DPACKPacket, NeighborTable
from src.utilities import utilities as util
from src.utilities import config

from scipy.stats import norm
import abc

from copy import copy

class BASE_routing(metaclass=abc.ABCMeta):

    def __init__(self, entity, simulator):
        """ The drone that is doing routing and simulator object. """

        self.entity = entity
        self.current_n_transmission = 0
        self.hello_messages = {}  # { drone_id : most recent hello packet}
        self.network_disp = simulator.network_dispatcher
        self.simulator = simulator

        if self.simulator.communication_error_type == config.ChannelError.GAUSSIAN:
            self.buckets_probability = self.__init_guassian()
        self.no_transmission = False
        

    @abc.abstractmethod
    def relay_selection(self, geo_neighbors, packet):
        pass

    def routing_close(self):
        self.no_transmission = False

    def drone_reception(self, src_entity, packet: Packet, current_ts):

        """ handle reception an ACKs for a packets """
        if isinstance(packet, HelloPacket):
            src_id = packet.src_drone.identifier
            self.hello_messages[src_id] = packet  # add packet to our dictionary

        elif isinstance(packet, DataPacket):
            self.no_transmission = True
            self.entity.accept_packets([packet])
            # build ack for the reception
            ack_packet = ACKPacket(self.entity, src_entity, self.simulator, packet, current_ts)
            self.unicast_message(ack_packet, self.entity, src_entity, current_ts)

        elif isinstance(packet, ACKPacket):
            self.entity.remove_packets([packet.acked_packet])
            # packet.acked_packet.optional_data
            # print(self.is_packet_received_drone_reward, "ACK", self.entity.identifier)
            if self.entity.buffer_length() == 0:
                self.current_n_transmission = 0
                self.entity.move_routing = False

        elif isinstance(packet, DiscoveryPacket):
            if self.entity != self.simulator.depot:
                # if the discovery packet is the first one received
                if packet.event_ref.identifier not in self.entity.discovery_packets:
                    # save the event identifier to avoid the future reception of the same discovery packet
                    self.entity.discovery_packets.add(packet.event_ref.identifier)
                    # save the sender id of the discovery packet so that the neighbor table can be sent to the sender
                    self.entity.discovery_packets_to_parent_id[packet.event_ref.identifier] = packet.src_drone
                    self.temporary_neighbor_table = dict()

                    # copy the packet instance so that i can change the hop count
                    # (using during dpack packet creation) without affecting all packets
                    # with the same id stored by all different drones
                    packet = copy(packet)
                    packet.add_hop(self.entity)

                    # is the sender is not the depot i send them the dpack packet
                    if src_entity != self.simulator.depot:
                        # once a discovery packet is received, i instantiate a dpack packet to notify the sender
                        # of the discovery packet of the following information: speed, location, and hop_count.
                        # The receiver will use this information to construct the neighbor table.
                        dpack_packet = DPACKPacket(self.entity, src_entity, packet, current_ts, self.simulator, info={
                            "speed": self.entity.speed, 
                            "location": self.entity.coords, 
                            "hop_count_from_CC": packet.get_TTL()}, event_ref=packet.event_ref)
                                                
                        self.unicast_message(dpack_packet, self.entity, src_entity, current_ts)

                    # change the src_drone of the discovery packet
                    packet.src_drone = self.entity
                    self.broadcast_message(packet, self.entity, self.simulator.drones, current_ts)

        elif isinstance(packet, DPACKPacket):        
            # if the dpack packet is not about an expired event
            if current_ts < packet.event_ref.deadline:
                # update the temporary neighbor table
                self.entity.temporary_neighbor_table[packet.src_drone] = packet.info
                # instantiate a NeighborTable containing the information collected 
                # through the dpack packets and send it to the sender of the discovery packets
                pck = NeighborTable(self.entity, self.entity.temporary_neighbor_table, current_ts, self.simulator, packet.event_ref)
                self.unicast_message(pck, self.entity, self.entity.discovery_packets_to_parent_id[packet.event_ref.identifier], current_ts)

        elif isinstance(packet, NeighborTable):
            # when the drone receives the NeighborTable it uses the 
            # information in it to update the internal neighbor_table
            for drone in packet.info:
                self.entity.neighbor_table[drone] = packet.info[drone]

            # if i haven't sent my neighbor table back yet. 
            if packet.resend:
                pck = NeighborTable(self.entity, self.entity.neighbor_table, current_ts, self.simulator, packet.event_ref)
                pck.resend = False
                self.unicast_message(pck, self.entity, packet.sender, current_ts)

    def drone_identification(self, drones, cur_step):
        """ handle drone hello messages to identify neighbors """
        # if self.entity in drones: drones.remove(self.entity)  # do not send hello to yourself
        if cur_step % config.HELLO_DELAY != 0:  # still not time to communicate
            return

        my_hello = HelloPacket(self.entity, cur_step, self.simulator, self.entity.coords,
                        self.entity.speed, self.entity.next_target())
        
        # to update the QTable we need to have the qtable of the drone chosen as relay, 
        # then we added it as optional data to the hello packet
        if self.simulator.routing_algorithm == config.RoutingAlgorithm.QL:
            my_hello.append_optional_data(data={
                "relay_qtable": self.qtable
            })
        
        self.broadcast_message(my_hello, self.entity, drones, cur_step)

    def routing(self, depot, drones, cur_step):
        # set up this routing pass
        self.drone_identification(drones, cur_step)

        self.send_packets(cur_step)

        # close this routing pass
        self.routing_close()

    # an ad-hoc routing used by the depot
    def ad_hoc_routing(self, depot, drones, cur_step):
        buffer_length = depot.buffer_length()
        if buffer_length != 0:
            buffer = depot.all_packets()
            # iterate over all packets in reverse order
            for pck_index in reversed(range(buffer_length)):
                # if the packet has not yet been shipped
                if buffer[pck_index].identifier not in depot.discovery_packets:
                    # structure used for save the discovery packets sent
                    depot.discovery_packets.add(buffer[pck_index].identifier)

                    if isinstance(buffer[pck_index], DiscoveryPacket):
                        pck: DiscoveryPacket = buffer[pck_index]
                        self.broadcast_message(pck, self.entity, drones, cur_step)
                    
                    elif isinstance(buffer[pck_index], NeighborTable):
                        pck: NeighborTable = buffer[pck_index]

                        # updates the depot's neighbor table and, once updated, sends it
                        # to the sender of the packet.
                        for drone in pck.info:
                            self.entity.neighbor_table[drone] = pck.info[drone]

                        pck = NeighborTable(self.entity, self.entity.neighbor_table, cur_step, self.simulator, pck.event_ref)
                        self.unicast_message(pck, self.entity, pck.sender, cur_step)

    def send_packets(self, cur_step):
        """ procedure 3 -> choice next hop and try to send it the data packet """

        # FLOW 0
        if self.no_transmission or self.entity.buffer_length() == 0:
            return

        # FLOW 1
        if util.euclidean_distance(self.simulator.depot.coords, self.entity.coords) <= self.simulator.depot_com_range:
            # add error in case
            self.transfer_to_depot(self.entity.depot, cur_step)

            self.entity.move_routing = False
            self.current_n_transmission = 0
            return



        if cur_step % self.simulator.drone_retransmission_delta == 0:

            opt_neighbors = []
            for hpk_id in self.hello_messages:
                hpk: HelloPacket = self.hello_messages[hpk_id]

                # check if packet is too old
                if hpk.time_step_creation < cur_step - config.OLD_HELLO_PACKET:
                    continue

                opt_neighbors.append((hpk, hpk.src_drone))

            if len(opt_neighbors) == 0:
                return

            # send packets
            for pkd in self.entity.all_packets():

                self.simulator.metrics.mean_numbers_of_possible_relays.append(len(opt_neighbors))

                best_neighbor = self.relay_selection(opt_neighbors, pkd)  # compute score

                if best_neighbor is not None:

                    self.unicast_message(pkd, self.entity, best_neighbor, cur_step)

                self.current_n_transmission += 1

    def geo_neighborhood(self, drones, no_error=False):
        """
        @param drones:
        @param no_error:
        @return: A list all the Drones that are in self.entity neighbourhood (no matter the distance to depot),
            in all direction in its transmission range, paired with their distance from self.entity
        """

        closest_drones = []  # list of this drone's neighbours and their distance from self.entity: (drone, distance)

        for other_drone in drones:

            if self.entity.identifier != other_drone.identifier:  # not the same drone
                drones_distance = util.euclidean_distance(self.entity.coords,
                                                          other_drone.coords)  # distance between two drones

                if drones_distance <= min(self.entity.communication_range,
                                          other_drone.communication_range):  # one feels the other & vv

                    # CHANNEL UNPREDICTABILITY
                    if self.channel_success(drones_distance, no_error=no_error):
                        closest_drones.append((other_drone, drones_distance))

        return closest_drones

    def channel_success(self, drones_distance, no_error=False):
        """
        Precondition: two drones are close enough to communicate. Return true if the communication
        goes through, false otherwise.
        """

        assert (drones_distance <= self.entity.communication_range)

        if no_error:
            return True

        if self.simulator.communication_error_type == config.ChannelError.NO_ERROR:
            return True

        elif self.simulator.communication_error_type == config.ChannelError.UNIFORM:
            return self.simulator.rnd_routing.rand() <= self.simulator.drone_communication_success

        elif self.simulator.communication_error_type == config.ChannelError.GAUSSIAN:
            return self.simulator.rnd_routing.rand() <= self.gaussian_success_handler(drones_distance)

    def broadcast_message(self, packet, src_drone, dst_drones, curr_step):
        """ send a message to my neigh drones"""
        for d_drone in dst_drones:
            self.unicast_message(packet, src_drone, d_drone, curr_step)

    def unicast_message(self, packet, src_drone, dst_drone, curr_step):
        """ send a message to my neigh drones"""
        # Broadcast using Network dispatcher

        if isinstance(packet, DiscoveryPacket) and config.MORE_OPTIONS:
            self.simulator.network_dispatcher.send_packet_to_medium(packet, src_drone, dst_drone,
                                                                curr_step + config.DP_DELAY)
        else:
            self.simulator.network_dispatcher.send_packet_to_medium(packet, src_drone, dst_drone,
                                                                    curr_step + config.LIL_DELTA)

    def gaussian_success_handler(self, drones_distance):
        """ get the probability of the drone bucket """
        bucket_id = int(drones_distance / self.radius_corona) * self.radius_corona
        return self.buckets_probability[bucket_id] * config.GUASSIAN_SCALE

    def transfer_to_depot(self, depot, cur_step):
        """ self.entity is close enough to depot and offloads its buffer to it, restarting the monitoring
                mission from where it left it
        """
        depot.transfer_notified_packets(self.entity, cur_step)
        self.entity.empty_buffer()
        self.entity.move_routing = False

    # --- PRIVATE ---
    def __init_guassian(self, mu=0, sigma_wrt_range=1.15, bucket_width_wrt_range=.5):

        # bucket width is 0.5 times the communication radius by default
        self.radius_corona = int(self.entity.communication_range * bucket_width_wrt_range)

        # sigma is 1.15 times the communication radius by default
        sigma = self.entity.communication_range * sigma_wrt_range

        max_prob = norm.cdf(mu + self.radius_corona, loc=mu, scale=sigma) - norm.cdf(0, loc=mu, scale=sigma)

        # maps a bucket starter to its probability of gaussian success
        buckets_probability = {}
        for bk in range(0, self.entity.communication_range, self.radius_corona):
            prob_leq = norm.cdf(bk, loc=mu, scale=sigma)
            prob_leq_plus = norm.cdf(bk + self.radius_corona, loc=mu, scale=sigma)
            prob = (prob_leq_plus - prob_leq) / max_prob
            buckets_probability[bk] = prob

        return buckets_probability
