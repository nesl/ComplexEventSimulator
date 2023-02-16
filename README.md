# Complex Event Simulator
Complex Event Simulator

## Setup

1. Run `git clone --recurse-submodules https://github.com/nesl/ComplexEventSimulator.git`
2. Install CARLA 0.9.10 with the Additional Maps. Extract the CARLA files and put them under a new directory with the name of **CARLA_0.9.10**.
3. Create virtual environment using the **requirements.txt** file.
4. Install [Poetry](https://python-poetry.org/) into your system
5. Install Scenic by getting into the Scenic folder and using `poetry install -E dev`
6. Clone [CARLA-2DBBox](https://github.com/MukhlasAdib/CARLA-2DBBox) and name it bbox_annotation.

## Run

1. Execute in one command line tab `./CARLA_0.9.10/CarlaUE4.sh`
2. Use the next command to generate the scenario `scenic pedestrian.scenic --simulate --param num_scenario 0 --param num_extra_pedestrians 0 --param output_dir camera_img --param bind_address '127.0.0.1' --param cameras_on "1,2"`
	* `num_scenario`: choose one of the predefined scenarios encoded in the file
	* `num_extra_pedestrians`: how many extra pedestrians to render, besides the ones needed for the scenario
	* `output_dir`: where to save images if a camera is instantiated
	* `bind_address`: which address to bind in order to allow for transmitting of camera frames
	* `cameras_on`: select the cameras to instantiate. These cameras are defined in **locations.txt**

You can use the **get_spectator_pos.py** to check the location of any place in the simulation.


