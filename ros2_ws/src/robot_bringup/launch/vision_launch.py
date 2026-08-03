# generate_launch_description()以下にこれ書くんご
stereonet_launch_file = os.path.join(
        get_package_share_directory('hobot_stereonet'),
        'launch',
        'stereonet_model_web_visual_v2.4_int16.launch.py'
    )
    
stereonet_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(stereonet_launch_file),
        launch_arguments={
            'mipi_rotation': '0.0', 
            'publish_visual_enabled': 'True',
        }.items()
    )
