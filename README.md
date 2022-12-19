# Running the code
The command to run experiments has been changed to be able to specify high-level and low-level algorithms

--highlevel determines the high-level algorithm (CBS, ICTS)

--lowlevel determines the low-level algorithm (A*, LA*, AL*, IDA*)

Example command:
python run_experiments.py --instance instances/exp4.txt --highlevel CBS --lowlevel A*

If confused do:
python run_experiments.py --help

# Generating new maps
The generate_map.py script has been added to generate 25 new instances of random maps. These maps are saved in ./test_instances
The command to generate scripts takes in 3 arguments 
 
 --agents determines the number of agents in the map

 --length determines the length of the square map

 --density determines the percentage of map covered in obstacles

 Example command:
 python generate_map.py --agents 5 --length 10 --density 0.1
 creates 25 maps, each being a 10x10 square with 5 agents and an obstacle density of 10%
