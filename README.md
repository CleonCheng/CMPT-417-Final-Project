# Running the code
The command to run experiments has been changed to be able to specify high-level and low-level algorithms

--highlevel determines the high-level algorithm (CBS, ICTS)

--lowlevel determines the low-level algorithm (A*, LA*, AL*, IDA*)

Example command:
python run_experiments.py --instance instances/exp4.txt --highlevel CBS --lowlevel A*

If confused do:
python run_experiments.py --help
