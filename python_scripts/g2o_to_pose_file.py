import os
import argparse

#  G2O file format:
TX, TY, TZ, QX, QY, QZ, QW = range(7)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--g2o_folder', type=str,
                        help='path to g2o folder containing result with folders for every vertex', required=True)
    parser.add_argument('-o', '--output_file', type=str, help='absolute path to output file, if not set a file', required=False)

    args = parser.parse_args()

    # Get all the folder names containing only digits
    folder_names = [folder for folder in os.listdir(args.g2o_folder) if folder.isdigit()]
    folder_names = sorted(folder_names, key=lambda x: int(x))

    # The robot name is contained in the data file of the first folder
    own_robot_name = None
    id_stamp_pose = []
    with open(os.path.join(args.g2o_folder, 'graph.g2o'), 'r') as graph:
        graph_lines = graph.readlines()
        vertices = [line for line in graph_lines if line.startswith('VERTEX_SE3:QUAT')]
        for folder in folder_names:
            with open(os.path.join(args.g2o_folder, folder, 'data'), 'r') as data:
                lines = data.readlines()
                # Get the robot name from the first line
                robot_name_str = [line for line in lines if line.startswith('robot_name')]
                robot_name = robot_name_str[0].split(' ')[1]
                if own_robot_name is None:
                    own_robot_name = robot_name

                if own_robot_name != robot_name:
                    continue

                id_str = [line for line in lines if line.startswith('id')]
                id = int(id_str[0].split(' ')[1])

                stamp_str = next((line for line in lines if line.startswith('stamp')), None)
                stamp_secs = stamp_str.split(' ')[1]
                stamp_nanosecs = stamp_str.split(' ')[2]
                # remove new line character if present
                if stamp_nanosecs.endswith('\n'):
                    stamp_nanosecs = stamp_nanosecs[:-1]
                stamp_nanosecs = stamp_nanosecs.zfill(9)
                stamp = float(stamp_secs + '.' + stamp_nanosecs)
                # print(f'stamp nanosecs: {stamp_nanosecs} stamp secs: {stamp_secs} stamp: {stamp}')

                # Skip the vertices with negative accum_dist indicating loop closure vertices leading to jumps in the estimated trajectory file
                accum_dist_str = next((line for line in lines if line.startswith('accum_dist')), None)
                accum_dist = float(accum_dist_str.split(' ')[1])
                if accum_dist < 0:
                    continue

            # Get the pose estimates from the .g2o file according to the gathered ids
            vertex_line = next((line for line in vertices if line.split(' ')[1] == str(id)), None)
            if not vertex_line:
                print('No vertex found for id: {}'.format(id))
                continue
            pose_str = vertex_line.split(' ')[2:]
            pose = [pose_str[TX], pose_str[TY], pose_str[TZ], pose_str[QX], pose_str[QY], pose_str[QZ], pose_str[QW]]
            id_stamp_pose.append([id, stamp, pose])

    # Write the gathered data to a file
    if args.output_file is None:
        output_file = os.path.join(args.g2o_folder, 'stamped_traj_estimate.txt')
    else:
        output_file = args.output_file
    print('Writing poses to file: {}'.format(os.path.abspath(output_file)))
    if not os.path.exists(output_file):
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, 'w') as f:
        for id, stamp, pose in id_stamp_pose:
            f.write('{} {} {} {} {} {} {} {}\n'.format(
                stamp, pose[TX],  pose[TY], pose[TZ],
                pose[QX],  pose[QY],  pose[QZ], pose[QW]))


if __name__ == '__main__':
    main()
