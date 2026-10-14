import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ==========================================
    # 1. Mencari lokasi package
    # ==========================================

    gazebo_share = get_package_share_directory(
        'ros_gz_sim'
    )

    simulation_share = get_package_share_directory(
        'rov_simulation'
    )

    # ==========================================
    # 2. Menentukan lokasi world dan model
    # ==========================================

    world_path = os.path.join(
        simulation_share,
        'worlds',
        'test_world.sdf'
    )

    model_path = os.path.join(
        simulation_share,
        'models',
        'simple_box',
        'model.sdf'
    )

    gazebo_launch_path = os.path.join(
        gazebo_share,
        'launch',
        'gz_sim.launch.py'
    )

    # ==========================================
    # 3. Mengatur lokasi pencarian resource Gazebo
    # ==========================================

    model_resource_path = os.path.join(
        simulation_share,
        'models'
    )

    existing_resource_path = os.environ.get(
        'GZ_SIM_RESOURCE_PATH',
        ''
    )

    if existing_resource_path:
        model_resource_path += (
            os.pathsep + existing_resource_path
        )

    # ==========================================
    # 4. Menjalankan Gazebo
    # ==========================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazebo_launch_path
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}'
        }.items()
    )

    # ==========================================
    # 5. Memasukkan model ke Gazebo
    #    menggunakan executable create
    # ==========================================

    spawn_model = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'ros_gz_sim',
            'create',
            '-world',
            'test_world',
            '-file',
            model_path,
            '-name',
            'simple_box_instance',
            '-x','0',
            '-y','0',
            '-z','15',
            '-R','84',
            '-P','57',
            '-Y','69' 
        ],
        output='screen',
    )

    # ==========================================
    # 6. Menggabungkan semua action
    # ==========================================

    return LaunchDescription([

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=model_resource_path,
        ),

        # Mulai Gazebo
        gazebo,

        # Beri waktu bagi Gazebo untuk memulai world
        TimerAction(
            period=5.0,
            actions=[
                spawn_model
            ]
        ),

    ])