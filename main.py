import argparse

import scene_visualizer as sv

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description=
        "MultiLidar mesh creator. Please select one of the optional arguments")
    parser.add_argument("--input_folder",
                        default='../input/',
                        help="set input folder")
    parser.add_argument("--lidar",
                        action='store_true',
                        help="Recording rgbd stream from L515")

    args = parser.parse_args()

    window = sv.SceneVisualizer(args.lidar, args.input_folder)
    window.display()
    print("--- Main thread finished ---")
