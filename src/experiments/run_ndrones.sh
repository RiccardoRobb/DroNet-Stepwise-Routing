#-----------------------------------------------------------#
#           _  _ ___  ___  ___  _  _ ___ ___                #
#          | \| |   \| _ \/ _ \| \| | __/ __|               #
#          | .` | |) |   / (_) | .` | _|\__ \               #
#          |_|\_|___/|_|_\\___/|_|\_|___|___/               #
#                                                           #
#-----------------------------------------------------------#


#test others algorithms
for nd in "25" "30";
do
    for alg in "RND" "GEO";
    do
        echo "run: ${alg} - ndrones ${nd} "
        /bin/python3.7 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 0 -e_s 10 -alg ${alg} &
        /bin/python3.7 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 10 -e_s 20 -alg ${alg} &
        /bin/python3.7 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 20 -e_s 30 -alg ${alg} &
    done;
done;
wait


