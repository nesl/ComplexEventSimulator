import os
import imageio
from tqdm import tqdm

def create_video_from_imgs(img_filepath_list, out_filepath, fps):

    writer = imageio.get_writer(out_filepath, fps=fps)
    for im in tqdm(img_filepath_list):
        writer.append_data(imageio.imread(im))
    writer.close()

# Parent folder for all of our videos
image_dir = "/media/brianw/1511bdc1-b782-4302-9f3e-f6d90b91f857/home/brianw/ICRA_DATA/carla_data"


# Video parent dir
video_dir = "/media/brianw/1511bdc1-b782-4302-9f3e-f6d90b91f857/home/brianw/ICRA_DATA/carla_videos"
if not os.path.exists(video_dir):
    os.mkdir(video_dir)
IMAGE_FPS = 10


# Now go through every sub folder in the image dir
for experiment in os.listdir(image_dir):
    current_path = image_dir + "/" + experiment
    
    # Grab all the camera folders
    image_folders = ["tc1out_rgb", "tc2out_rgb", "tc3out_rgb"]
    out_files = ["cam0.mp4", "cam1.mp4", "cam2.mp4"]

    print(experiment)
    # Iterate through each folder, if it exists
    for f_i, image_folder in enumerate(image_folders):
        # print(image_dir)
        # print(experiment)
        # print(current_path)
        imgs_path = current_path + "/" + image_folder

        # Check if this image path exists - if it does not, it lacks that camera
        if not os.path.exists(imgs_path):
            continue

        current_video_folder = os.path.join(video_dir, experiment)
        if not os.path.exists(current_video_folder):
            os.mkdir(current_video_folder)

        out_filepath = os.path.join(current_video_folder, out_files[f_i])
        # Importantly, if this out filepath already exists, then skip
        if os.path.exists(out_filepath):
            continue


        # Make the video file
        img_filenames = sorted(os.listdir(imgs_path))
        # Get all the sorted image filepaths
        image_filepaths = [os.path.join(imgs_path, fname) for fname in img_filenames]

        create_video_from_imgs(image_filepaths, out_filepath, IMAGE_FPS)


