import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Lokasi instalasi package ros_gz_sim
    gazebo_share = get_package_share_directory(
        'ros_gz_sim'
    )

    # Lokasi instalasi package simulasi milik kita
    simulation_share = get_package_share_directory(
        'rov_simulation'
    )

    # Lokasi world yang akan dibuka
    world_path = os.path.join(
        simulation_share,
        'worlds',
        'test_world.sdf'
    )

    # Launch file untuk menjalankan Gazebo
    gazebo_launch = os.path.join(
        gazebo_share,
        'launch',
        'gz_sim.launch.py'
    )
    





    # Direktori induk model yang akan dicari oleh Gazebo
    model_resource_path = os.path.join(
        simulation_share,
        'models'
    )

    # Pertahankan resource path yang sudah ada
    existing_resource_path = os.environ.get(
        'GZ_SIM_RESOURCE_PATH',
        ''
    )    
    
    if existing_resource_path:
        model_resource_path += (
            os.pathsep + existing_resource_path
        )    




    # Jalankan Gazebo menggunakan world milik kita
    return LaunchDescription([
        
        # Berikan lokasi model kepada proses Gazebo
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=model_resource_path,
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                gazebo_launch
            ),
            launch_arguments={
                'gz_args': f'-r {world_path}'
            }.items()
        )
    ])