import os
import subprocess
import shutil

#  Your img_dir and output_dir should be the same, barring the output_dir also having a '/'
#  Change the scenario and cameras and domain shift as well with every experiment.

# scenic pedestrian.scenic --simulate --time 3000 --img_dir camera_img  --param num_scenario 0 --param num_extra_pedestrians 0 --param output_dir camera_img/ --param bind_address '' --param cameras_on "11,1" --param weather CloudySunset


# You can change weather with 
# world.set_weather(carla.WeatherParameters.ClearNoon)
# world.set_weather(carla.WeatherParameters.HardRainNoon)
# world.set_weather(carla.WeatherParameters.CloudySunset)
#  Or a dict of all parameters



# First, let's get all of parameters we should iterate through
cameras = ["11, 12, 13", "21, 22", "31, 32", "41, 42, 43", "51, 52"]
scenarios = [1, 2, 3, 4, 0]

# weather_night = {"cloudiness": 50, "precipitation": 0, "preciptation_deposits": 0, "wind_intensity": 0, "sun_azimuth_angle": 0, "sun_altitude_angle": -45, "fog_density": 0, "fog_distance": 0, "wetness": 0, "fog_falloff": 0}

domain_shifts = [
    "ClearNoon",
    "HardRainNoon",
    "night"
]


TOTAL_EXAMPLES_PER_SETTING = 10
save_directory = "/media/brianw/1511bdc1-b782-4302-9f3e-f6d90b91f857/home/brianw/ICRA_DATA/carla_data/"


# Begin iteration
for ce_index in range(len(scenarios)):

    current_ce = scenarios[ce_index]
    current_cameras = cameras[ce_index]

    # Iterate through each domain case
    for domain_case_i, domain_case in enumerate(domain_shifts):
        # Iterate through total examples
        for i in range(TOTAL_EXAMPLES_PER_SETTING):

            num_extras = 0

            # Skip ce1 domain case 0
            # if current_ce == 1:
            #     # num_extras = 1
            #     # continue

            folder_name = "ce_" + str(current_ce) + "_ds_" + str(domain_case_i) + "_it_" + str(i)
            save_path = save_directory + folder_name

            #  First off, ignore data in this path if it already exists
            #  And there's already files in it
            if os.path.exists(save_path) and os.listdir(save_path):
                # shutil.rmtree(save_path)
                continue

            # Now, format our function call
            # Now, let's get the function call
            command = "scenic pedestrian.scenic --simulate --time 3000 --img_dir {save_dir}  --param num_scenario {ce_num} --param num_extra_pedestrians {num_peds} --param output_dir  {save_dir}/ --param bind_address '' --param cameras_on '{cameras}' --param weather {ds}".format(save_dir = save_path, ce_num=current_ce, num_peds=num_extras, cameras=current_cameras, ds=domain_case)

            print(command)

            # Run the command
            process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
            process.wait()
            





# Timestep is 0.1, so 10fps throughput